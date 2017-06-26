#include <advr_humanoids_common_utils/idynutils.h>
#include <advr_humanoids_common_utils/test_utils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>


using namespace yarp::math;

typedef idynutils2 iDynUtils;
static void null_deleter(iDynUtils *) {}


std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

void getPointsFromConstraints(const yarp::sig::Matrix &A_ch,
                              const yarp::sig::Vector& b_ch,
                              std::vector<KDL::Vector>& points) {
    unsigned int nRects = A_ch.rows();

    std::cout << "Computing intersection points between " << nRects << " lines" << std::endl;
    std::cout << "A:" << A_ch.toString() << std::endl;
    std::cout << "b:" << b_ch.toString() << std::endl;
    for(unsigned int j = 0; j < nRects; ++j) {
        int i;

        // intersection from line i to line j
        if(j == 0) i = nRects-1;
        else       i = (j-1);

        std::cout << "Computing intersection between line " << i << " and line " << j << ": ";
        // get coefficients for i-th line
        double a_i = A_ch(i,0);
        double b_i = A_ch(i,1);
        double c_i = b_ch(i);

        // get coefficients for j-th line
        double a_j = A_ch(j,0);
        double b_j = A_ch(j,1);
        double c_j = b_ch(j);

        /** Kramer rule to find intersection between two lines */
        double x = (c_i*b_j-b_i*c_j)/(a_i*b_j-b_i*a_j);
        double y = (a_i*c_j-c_i*a_j)/(a_i*b_j-b_i*a_j);
        std::cout << "(" << x << "," << y << ")" << std::endl;
        points.push_back(KDL::Vector(x,y,0.0));
    }
}

bool solveQP(   const yarp::sig::Matrix &J0,
                const yarp::sig::Vector &e0,
                const yarp::sig::Matrix &J1,
                const yarp::sig::Vector &eq,
                qpOASES::HessianType t1HessianType,
                const yarp::sig::Vector &l,
                const yarp::sig::Vector &u,
                const yarp::sig::Vector &q,
                yarp::sig::Vector &dq_ref)
{
    int nj = q.size();

    static bool initial_guess = false;

    static yarp::sig::Vector dq0(nj, 0.0);
    static yarp::sig::Vector dq1(nj, 0.0);;
    static yarp::sig::Vector y0(nj, 0.0);
    static yarp::sig::Vector y1(nj, 0.0);

    static qpOASES::Bounds bounds0;
    static qpOASES::Bounds bounds1;
    static qpOASES::Constraints constraints0;
    static qpOASES::Constraints constraints1;


    /**
      We solve a single QP where the priority between
      different tasks is set by using a weight matrix Q

      min         (Ax - b)'Q(Ax - b)
      subj to     l <=   x <=  u

      QPOASES::Quadratic_program solves by default a quadratic problem in the form
      min         x'Hx + x'g
      subj to  Alb <= Ax <= Aub
                 l <=  x <= u
     **/

    int njTask0 = J0.rows();

    yarp::sig::Matrix H0 = J0.transposed()*J0; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g0 = -1.0*J0.transposed()*e0;

    yarp::sig::Matrix H1 = J1.transposed()*J1; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*eq;

    USING_NAMESPACE_QPOASES

    /** Setting up QProblem object. **/
    Options qpOasesOptionsqp0;
    qpOasesOptionsqp0.printLevel = PL_NONE;
    qpOasesOptionsqp0.setToReliable();
    qpOasesOptionsqp0.enableRegularisation = BT_TRUE;
    qpOasesOptionsqp0.epsRegularisation *= 2E2;
    QProblem qp0( nj, 0, HST_SEMIDEF);
    qp0.setOptions( qpOasesOptionsqp0 );

    Options qpOasesOptionsqp1;
    qpOasesOptionsqp1.printLevel = PL_NONE;
    qpOasesOptionsqp1.setToReliable();
    qpOasesOptionsqp1.enableRegularisation = BT_TRUE;
    qpOasesOptionsqp1.epsRegularisation *= 2E2;
    QProblem qp1( nj, njTask0, t1HessianType);
    qp1.setOptions( qpOasesOptionsqp1 );

    /** Solve zero QP. **/
    int nWSR = 132;
    if(initial_guess==true)
        qp0.init( H0.data(),g0.data(),
                  NULL,
                  l.data(), u.data(),
                  NULL, NULL,
                  nWSR,0,
                  dq0.data(), y0.data(),
                  &bounds0, &constraints0);
    else {
        qp0.init( H0.data(),g0.data(),
                  NULL,
                  l.data(), u.data(),
                  NULL, NULL,
                  nWSR,0);
        std::cout << GREEN << "Not using initial guess" << DEFAULT;
    }

    if(dq0.size() != qp0.getNV()) {
        dq0.resize(qp0.getNV());
        initial_guess = false;
    }
    if(y0.size() != qp0.getNV() + qp0.getNC()) {
        y0.resize(qp0.getNV()+ qp0.getNC());
        initial_guess = false;
    }

    int success0 = qp0.getPrimalSolution( dq0.data() );
    qp0.getDualSolution(y0.data());
    qp0.getBounds(bounds0);
    qp0.getConstraints(constraints0);

    if(success0== RET_QP_NOT_SOLVED ||
      (success0 != RET_QP_SOLVED && success0 != SUCCESSFUL_RETURN))
    {
        std::cout << GREEN <<
                     "ERROR OPTIMIZING ZERO TASK! ERROR #" <<
                     success0 <<
                     "Not using initial guess" << DEFAULT;

        initial_guess = false;
    }
    else
    {
        /** Solve first QP. **/
        yarp::sig::Matrix A1 = J0;
        yarp::sig::Vector b1 = J0*dq0;
        yarp::sig::Vector lA1 = b1;
        yarp::sig::Vector uA1 = b1;

        nWSR = 132;

        if(initial_guess == true)
            qp1.init( H1.data(),g1.data(),
                      A1.data(),
                      l.data(), u.data(),
                      lA1.data(), uA1.data(),
                      nWSR, 0,
                      dq1.data(), y1.data(),
                      &bounds1, &constraints1);
        else
            qp1.init( H1.data(),g1.data(),
                      A1.data(),
                      l.data(), u.data(),
                      lA1.data(), uA1.data(),
                      nWSR, 0);

        if(dq1.size() != qp1.getNV()) {
            dq1.resize(qp1.getNV());
            initial_guess = false;
        }
        if(y1.size() != qp1.getNV() + qp1.getNC()) {
            y1.resize(qp1.getNV() + qp1.getNC());
            initial_guess = false;
        }

        int success1 = qp1.getPrimalSolution( dq1.data() );
        qp1.getDualSolution(y1.data());
        qp1.getBounds(bounds1);
        qp1.getConstraints(constraints1);

        if(success1 == RET_QP_NOT_SOLVED ||
          (success1 != RET_QP_SOLVED && success1 != SUCCESSFUL_RETURN))
        {
            std::cout << GREEN <<
                         "ERROR OPTIMIZING POSTURE TASK! ERROR #" <<
                         success1 << DEFAULT;
            initial_guess = false;
        }
        else
        {
            dq_ref = dq1;
            initial_guess = true;
            return true;
       }
    }
    return false;
}

bool solveQPrefactor(   const yarp::sig::Matrix &J0,
                        const yarp::sig::Vector &e0,
                        const yarp::sig::Matrix &J1,
                        const yarp::sig::Vector &eq,
                        OpenSoT::HessianType t1HessianType,
                        const yarp::sig::Vector &u,
                        const yarp::sig::Vector &l,
                        const yarp::sig::Vector &q,
                        yarp::sig::Vector &dq_ref)
{
    int nj = q.size();

    int njTask0 = J0.rows();

    yarp::sig::Matrix H0 = J0.transposed()*J0; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g0 = -1.0*J0.transposed()*e0;

    yarp::sig::Matrix H1 = J1.transposed()*J1; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*eq;

    yarp::sig::Matrix A0(0,nj);
    yarp::sig::Vector lA0(0), uA0(0);

    USING_NAMESPACE_QPOASES

    static OpenSoT::solvers::QPOasesProblem qp0(nj, 0, OpenSoT::HST_SEMIDEF);
    qp0.setnWSR(127);
    static bool result0 = false;
    static bool isQProblemInitialized0 = false;
    if(!isQProblemInitialized0){
        result0 = qp0.initProblem(
                    conversion_utils_YARP::toEigen(H0),
                    conversion_utils_YARP::toEigen(g0),
                    conversion_utils_YARP::toEigen(A0),
                    conversion_utils_YARP::toEigen(lA0),
                    conversion_utils_YARP::toEigen(uA0),
                    conversion_utils_YARP::toEigen(l),
                    conversion_utils_YARP::toEigen(u));
        isQProblemInitialized0 = true;}
    else
    {
        qp0.updateProblem(conversion_utils_YARP::toEigen(H0),
                          conversion_utils_YARP::toEigen(g0),
                          conversion_utils_YARP::toEigen(A0),
                          conversion_utils_YARP::toEigen(lA0),
                          conversion_utils_YARP::toEigen(uA0),
                          conversion_utils_YARP::toEigen(l),
                          conversion_utils_YARP::toEigen(u));
        result0 = qp0.solve();
    }

    if(result0)
    {
        yarp::sig::Vector dq0 = conversion_utils_YARP::toYARP(qp0.getSolution());
        yarp::sig::Matrix A1 = J0;
        yarp::sig::Vector b1 = J0*dq0;
        yarp::sig::Vector lA1 = b1;
        yarp::sig::Vector uA1 = b1;

        static OpenSoT::solvers::QPOasesProblem qp1(nj, njTask0, t1HessianType);
        qp1.setnWSR(127);
        static bool result1 = false;
        static bool isQProblemInitialized1 = false;
        if(!isQProblemInitialized1){
            result1 = qp1.initProblem(
                        conversion_utils_YARP::toEigen(H1),
                        conversion_utils_YARP::toEigen(g1),
                        conversion_utils_YARP::toEigen(A1),
                        conversion_utils_YARP::toEigen(lA1),
                        conversion_utils_YARP::toEigen(uA1),
                        conversion_utils_YARP::toEigen(l),
                        conversion_utils_YARP::toEigen(u));
            isQProblemInitialized1 = true;}
        else
        {
            qp1.updateProblem(conversion_utils_YARP::toEigen(H1),
                              conversion_utils_YARP::toEigen(g1),
                              conversion_utils_YARP::toEigen(A1),
                              conversion_utils_YARP::toEigen(lA1),
                              conversion_utils_YARP::toEigen(uA1),
                              conversion_utils_YARP::toEigen(l),
                              conversion_utils_YARP::toEigen(u));
            result1 = qp1.solve();
        }
        if(result1)
        {
            dq_ref = conversion_utils_YARP::toYARP(qp1.getSolution());
            return true;
        }
        else
        {
            std::cout << GREEN << "ERROR OPTIMIZING POSTURE TASK" << DEFAULT;
            return false;
        }
    }
    else
    {
        std::cout << GREEN << "ERROR OPTIMIZING CARTESIAN TASK" << DEFAULT;
        return false;
    }
}

enum useFootTaskOrConstraint {
    USE_TASK = 1,
    USE_CONSTRAINT = 2
};

class testQPOases_ConvexHull:
        public ::testing::Test,
        public ::testing::WithParamInterface<useFootTaskOrConstraint>
{
protected:
    std::ofstream _log;

    testQPOases_ConvexHull()
    {
        _log.open("testQPOases_ConvexHull.m", std::ofstream::app);
    }

    virtual ~testQPOases_ConvexHull() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& _robot) {
    yarp::sig::Vector q(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("RShSag")] =  20.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RShLat")] = -10.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RElbj")] = -80.0*M_PI/180.0;


    std::cout << "Q_initial: " << q.toString() << std::endl;
    return q;
}

//#define TRY_ON_SIMULATOR
TEST_P(testQPOases_ConvexHull, tryFollowingBounds) {

    useFootTaskOrConstraint footStrategy = GetParam();

#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("tryFollowingBounds");
#endif

    iDynUtils idynutils_com("coman",
                            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr_com;
    _model_ptr_com = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr_com->loadModel(boost::shared_ptr<iDynUtils>(&idynutils_com, &null_deleter));

    if(_model_ptr_com)
        std::cout<<"pointer address: "<<_model_ptr_com.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr_com.get()<<std::endl;



    yarp::sig::Vector q = getGoodInitialPosition(idynutils_com);
    idynutils_com.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);
    idynutils_com.switchAnchorAndFloatingBase("l_sole");

#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    // BOUNDS

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits(
            new OpenSoT::constraints::velocity::JointLimits(
                    conversion_utils_YARP::toEigen(q),
                        idynutils_com.getJointBoundMax(),
                        idynutils_com.getJointBoundMin()));

    OpenSoT::constraints::Aggregated::ConstraintPtr velocityLimits(
            new OpenSoT::constraints::velocity::VelocityLimits(0.3,
                                                               3e-3,
                                                               q.size()));


    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    // NOTE without this things don't work!
    bounds_list.push_back(velocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

    OpenSoT::tasks::velocity::CoM::Ptr com_task(
                new OpenSoT::tasks::velocity::CoM(
                    conversion_utils_YARP::toEigen(q), *(_model_ptr_com.get())));
    com_task->setLambda(.6);

    yarp::sig::Matrix W(3,3);
    W.eye(); W(2,2) = .1;
    com_task->setWeight(conversion_utils_YARP::toEigen(W));

    OpenSoT::constraints::velocity::CoMVelocity::ConstraintPtr boundsCoMVelocity(
                new OpenSoT::constraints::velocity::CoMVelocity(
                    conversion_utils_YARP::toEigen(yarp::sig::Vector(3, 0.05)), 0.004 ,
                    conversion_utils_YARP::toEigen(q), *(_model_ptr_com.get())));
    com_task->getConstraints().push_back(boundsCoMVelocity);

    std::list<std::string> _links_in_contact;
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");



    OpenSoT::constraints::velocity::ConvexHull::Ptr boundsConvexHull(
                new OpenSoT::constraints::velocity::ConvexHull(
                    conversion_utils_YARP::toEigen(q), *(_model_ptr_com.get()),
                    _links_in_contact, 0.01));
    com_task->getConstraints().push_back(boundsConvexHull);

    OpenSoT::tasks::velocity::Cartesian::Ptr right_foot_task(
                new OpenSoT::tasks::velocity::Cartesian("world::right_foot",
                                                        conversion_utils_YARP::toEigen(q), *(_model_ptr_com.get()),
                                                        "r_sole",
                                                        "world"));
    right_foot_task->setLambda(.2);
    right_foot_task->setOrientationErrorGain(.1);

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(conversion_utils_YARP::toEigen(q)));

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    if(footStrategy == USE_TASK)
    {
        _log.close();
        _log.open("testQPOases_ConvexHull.m");

        stack_of_tasks.push_back(right_foot_task);
    }
    else if (footStrategy == USE_CONSTRAINT)
    {
        using namespace OpenSoT::constraints;
        TaskToConstraint::Ptr right_foot_constraint(new TaskToConstraint(right_foot_task));

        com_task->getConstraints().push_back(right_foot_constraint);
        postural_task->getConstraints().push_back(right_foot_constraint);

    }

    stack_of_tasks.push_back(com_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks,
                                          bounds));

    //SET SOME REFERENCES
    yarp::sig::Vector T_com_p_init = idynutils_com.iDynTree_model.getCOM();
    yarp::sig::Vector T_com_p_ref = T_com_p_init;
    T_com_p_ref[0] = 0.0;
    T_com_p_ref[1] = 0.0;

    std::cout << "Initial CoM position is " << T_com_p_init.toString() << std::endl;
    std::cout << "Moving to (0,0)" << std::endl;

    com_task->setReference(conversion_utils_YARP::toEigen(T_com_p_ref));

    yarp::sig::Vector dq(q.size(), 0.0);
    double e = norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition()));
    double previous_e = 0.0;

    unsigned int i = 0;
    const unsigned int n_iterations = 10000;
    if(footStrategy == USE_TASK)
        _log << "com_traj = [";
    else if(footStrategy == USE_CONSTRAINT)
        _log << "com_traj_constraint = [";

    yarp::sig::Matrix right_foot_pose = conversion_utils_YARP::toYARP(right_foot_task->getActualPose());

    for(i = 0; i < n_iterations; ++i)
    {
        idynutils_com.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

        right_foot_task->update(conversion_utils_YARP::toEigen(q));
        com_task->update(conversion_utils_YARP::toEigen(q));
        postural_task->update(conversion_utils_YARP::toEigen(q));
        bounds->update(conversion_utils_YARP::toEigen(q));

        _log << com_task->getActualPosition()[0] << ","
            << com_task->getActualPosition()[1] <<";";
        e = norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition()));

        if(fabs(previous_e - e) < 1e-13) {
            std::cout << "i: " << i << " e: " << norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition())) << " . Error not decreasing. CONVERGED." << std::endl;
            break;
        }
        previous_e = e;

        Eigen::VectorXd _dq(dq.size());
        _dq.setZero(dq.size());
        EXPECT_TRUE(sot->solve(_dq));
        dq = conversion_utils_YARP::toYARP(_dq);
        q += dq;

        yarp::sig::Matrix right_foot_pose_now = conversion_utils_YARP::toYARP(right_foot_task->getActualPose());
        for(unsigned int r = 0; r < 4; ++r)
            for(unsigned int c = 0; c < 4; ++c)
                ASSERT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-6);
        ASSERT_LT(norm(conversion_utils_YARP::toYARP(right_foot_task->getb())),1e-8);

#ifdef TRY_ON_SIMULATOR
        robot.move(q);
#endif
    }

    ASSERT_NEAR(norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition())),0,1E-9);

    idynutils_com.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);
    boundsConvexHull->update(conversion_utils_YARP::toEigen(q));
    std::vector<KDL::Vector> points;
    std::vector<KDL::Vector> points_inner;
    boundsConvexHull->getConvexHull(points);

    KDL::Vector point_old;
    Eigen::MatrixXd A_ch, A_ch_outer;
    Eigen::VectorXd b_ch, b_ch_outer;
    boundsConvexHull->getConstraints(points, A_ch, b_ch, 0.01);
    boundsConvexHull->getConstraints(points, A_ch_outer, b_ch_outer, 0.0);
    std::cout << std::endl << "A_ch: " << std::endl << A_ch << std::endl;
    std::cout << std::endl << "b_ch: " << std::endl << b_ch << std::endl;
    std::cout << std::endl << "A_ch_outer: " << std::endl << A_ch_outer << std::endl;
    std::cout << std::endl << "b_ch_outer: " << std::endl << b_ch_outer << std::endl;
    std::cout << std::endl << "@q: " << std::endl << q.toString() << std::endl << std::endl;
    getPointsFromConstraints(conversion_utils_YARP::toYARP(A_ch),
                             conversion_utils_YARP::toYARP(b_ch),
                             points_inner);


    points.push_back(points.front());
    points_inner.push_back(points_inner.front());
    typedef std::vector<KDL::Vector>::iterator it_p;
    for(it_p point = points.begin();
        point != points.end();
        ++point)
    {
        std::cout << std::endl
                  << "=================" << std::endl
                  << "Moving from ("
                  << point_old.x() << "," << point_old.y()
                  << ") to ("
                  << point->x() << "," << point->y()
                  <<")" << std::endl;

        T_com_p_ref[0] = point->x();
        T_com_p_ref[1] = point->y();

        com_task->setReference(conversion_utils_YARP::toEigen(T_com_p_ref));

        yarp::sig::Vector dq(q.size(), 0.0);
        double e = norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition()));
        double previous_e = 0.0;
        double oscillation_check_e = 0.0;
        for(i = 0; i < n_iterations; ++i)
        {
            idynutils_com.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

            right_foot_task->update(conversion_utils_YARP::toEigen(q));
            com_task->update(conversion_utils_YARP::toEigen(q));
            postural_task->update(conversion_utils_YARP::toEigen(q));
            bounds->update(conversion_utils_YARP::toEigen(q));

            _log << com_task->getActualPosition()[0] << ","
                << com_task->getActualPosition()[1] <<";";
            e = norm(T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition()));

            if(fabs(e - oscillation_check_e) < 1e-9)
            {
                std::cout << "i: " << i << " e: " << e << " -- Oscillation detected. Stopping." << std::endl;
                break;
            }

            //std::cout << "i: " << i << " e: " << e << std::endl;
            if(fabs(previous_e - e) < 1e-12) {
                std::cout << "i: " << i << " e: " << e << " -- Error not decreasing. CONVERGED." << std::endl;
                break;
            }
            oscillation_check_e = previous_e;
            previous_e = e;

            if(e < 1e-3) {  // what if we get too close?!?
                boundsConvexHull->getConstraints(points, A_ch, b_ch, 0.01);
                std::cout << "A_ch:" << A_ch << std::endl;
                std::cout << "b_ch:" << b_ch << std::endl;
            }



            Eigen::VectorXd _dq(dq.size());
            _dq.setZero(dq.size());
            EXPECT_TRUE(sot->solve(_dq));
            dq = conversion_utils_YARP::toYARP(_dq);
            q += dq;


            yarp::sig::Matrix right_foot_pose_now = conversion_utils_YARP::toYARP(right_foot_task->getActualPose());
            for(unsigned int r = 0; r < 4; ++r)
                for(unsigned int c = 0; c < 4; ++c)
                    EXPECT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-3) << "Error at iteration "
                                                                                    << i
                                                                                    << " at position "
                                                                                    << "(" << r
                                                                                    << "," << c << ")"
                                                                                    << " with cartesian error equal to "
                                                                                    << norm(conversion_utils_YARP::toYARP(right_foot_task->getb())) << std::endl;
            EXPECT_LT(norm(conversion_utils_YARP::toYARP(right_foot_task->getb())),5e-5);
            EXPECT_LT(norm(conversion_utils_YARP::toYARP(right_foot_task->getA())*dq),1E-6) << "Error at iteration "
                                                             << i
                                                             << " J_foot*dq = "
                                                             << (right_foot_task->getA()*conversion_utils_YARP::toEigen(dq))
                                                             << std::endl;

#ifdef TRY_ON_SIMULATOR
            robot.move(q);
#endif
        }



        if(i == n_iterations)
            std::cout << "i: " << i << " e: " << norm(T_com_p_ref -
                                                      conversion_utils_YARP::toYARP(com_task->getActualPosition())) << " -- Error not decreasing. STOPPING." << std::endl;

        yarp::sig::Vector distance = T_com_p_ref - conversion_utils_YARP::toYARP(com_task->getActualPosition());
        double d = norm(distance);
        yarp::sig::Vector exp_distance(2,0.01);
        double expected_d = norm(exp_distance);

        std::vector<KDL::Vector> points_check;
        boundsConvexHull->getConvexHull(points_check);
        Eigen::MatrixXd A_ch_check;
        Eigen::VectorXd b_ch_check;
        boundsConvexHull->getConstraints(points_check, A_ch_check, b_ch_check, 0.01);


        EXPECT_NEAR(d, expected_d, (expected_d-0.01)*1.01) << "Failed to reach point "
                                                           << *point
                                                           << " in the allocated threshold (0.01m)." << std::endl;
        //EXPECT_TRUE(A_ch == A_ch_check) << "Convex Hull changed!" << std::endl;


        std::cout<<"A_ch size is "<<A_ch.rows()<<"x"<<A_ch.cols()<<std::endl;
        std::cout<<"A_ch_check size is "<<A_ch_check.rows()<<"x"<<A_ch_check.cols()<<std::endl;

        if(! (A_ch.rows() == A_ch_check.rows()) )
        {
            std::cout << "Old A:" << std::endl << A_ch << std::endl;
            std::cout << "New A:" << std::endl << A_ch_check << std::endl;
            std::cout << "Had originally " << points.size() -1 << " points in the Convex Hull, "
                      << " now we have " << points_check.size() << std::endl;
        }
        point_old = *point;
    }



    _log << "];" << std::endl;

    if(footStrategy == USE_TASK)
    {
        typedef std::vector<KDL::Vector>::iterator it_p;
        _log << "points=[";
        for(it_p point = points.begin();
            point != points.end();
            ++point)
            _log << point->x() << "," << point->y() << ";";
        _log << "];" << std::endl;
        _log << "points_inner=[";
        for(it_p point = points_inner.begin();
            point != points_inner.end();
            ++point)
            _log << point->x() << "," << point->y() << ";";
        _log << "];" << std::endl;

        _log << "ch_figure = figure('Position',[0 0 1027 768]); hold on; ref=plot(points(:,1), points(:,2), 'r-.'); constr=plot(points_inner(:,1), points_inner(:,2), 'g'); axis equal;" << std::endl;
        _log << "ct = plot(com_traj(:,1), com_traj(:,2), 'b--');" << std::endl;
    }
    else if(footStrategy==USE_CONSTRAINT)
    {
        _log << "ctc = plot(com_traj_constraint(:,1), com_traj_constraint(:,2), 'm:');" << std::endl;
        _log << "legend([ref,constr,ct,ctc], 'CoM ref', 'Support Polygon', 'CoM Traj, r\\_sole ctrl as task', 'CoM Traj, r\\_sole ctrl as constraint','Location','best');" << std::endl;
    }
    _log << "print(ch_figure,'-depsc','testQPOases_ConvexHull_followingBounds');" << std::endl;

}

INSTANTIATE_TEST_CASE_P(tryDifferentFootTaskStrategies,
                        testQPOases_ConvexHull,
                        ::testing::Values(USE_TASK, USE_CONSTRAINT));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
