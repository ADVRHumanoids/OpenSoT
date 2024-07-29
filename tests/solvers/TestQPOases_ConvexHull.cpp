#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <OpenSoT/utils/AutoStack.h>
#include <matlogger2/matlogger2.h>
#include "../common.h"


#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

void getPointsFromConstraints(const Eigen::MatrixXd &A_ch,
                              const Eigen::VectorXd& b_ch,
                              std::vector<Eigen::Vector3d>& points) {
    unsigned int nRects = 0;
    for(unsigned int i = 0; i < A_ch.rows(); ++i)
    {
        double a_i = A_ch(i,0);
        double b_i = A_ch(i,1);
        if(fabs(a_i) < 1e-9 && fabs(b_i) < 1e-9)
            break;
        nRects++;
    }

    std::cout << "Computing intersection points between " << nRects << " lines" << std::endl;
    std::cout << "A:" << A_ch << std::endl;
    std::cout << "b:" << b_ch << std::endl;
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
        if(!isnan(x) && !isnan(y))
            points.push_back(Eigen::Vector3d(x,y,0.0));

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

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface& _robot) {
    Eigen::VectorXd q = _robot.getNeutralQ();

    q[_robot.getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[_robot.getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[_robot.getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.getQIndex("LShSag")] =  20.0*M_PI/180.0;
    q[_robot.getQIndex("LShLat")] = 10.0*M_PI/180.0;
    q[_robot.getQIndex("LElbj")] = -80.0*M_PI/180.0;

    q[_robot.getQIndex("RShSag")] =  20.0*M_PI/180.0;
    q[_robot.getQIndex("RShLat")] = -10.0*M_PI/180.0;
    q[_robot.getQIndex("RElbj")] = -80.0*M_PI/180.0;


    std::cout << "Q_initial: " << q << std::endl;
    return q;
}

//#define TRY_ON_SIMULATOR
TEST_P(testQPOases_ConvexHull, tryFollowingBounds) {



    useFootTaskOrConstraint footStrategy = GetParam();

    XBot::MatLogger2::Ptr logger;
    if(footStrategy == USE_TASK){
        logger = XBot::MatLogger2::MakeLogger("convex_hull_in_com"); // date-time automatically appended
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    }
    if(footStrategy == USE_CONSTRAINT){
        logger = XBot::MatLogger2::MakeLogger("convex_hull_constr");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    }



    XBot::ModelInterface::Ptr _model_ptr_com;
    _model_ptr_com = GetTestModel("coman_floating_base");
    if(_model_ptr_com)
        std::cout<<"pointer address: "<<_model_ptr_com.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr_com.get()<<std::endl;



    Eigen::VectorXd q = getGoodInitialPosition(*_model_ptr_com);
    _model_ptr_com->setJointPosition(q);
    _model_ptr_com->update();

    Eigen::Affine3d waist_T_lsole, fb_T_lsole;
    _model_ptr_com->getPose("Waist", "l_sole", waist_T_lsole);
    fb_T_lsole.setIdentity();
    fb_T_lsole.translation()[0] = waist_T_lsole.translation()[0];
    fb_T_lsole.translation()[2] = waist_T_lsole.translation()[2];
    std::cout<<"fb_T_lsole: "<<fb_T_lsole.matrix()<<std::endl;
    _model_ptr_com->setFloatingBasePose(fb_T_lsole);
    _model_ptr_com->update();
    _model_ptr_com->getJointPosition(q);

    // BOUNDS
    Eigen::VectorXd qmin, qmax;
    _model_ptr_com->getJointLimits(qmin, qmax);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits(
            new OpenSoT::constraints::velocity::JointLimits(*_model_ptr_com, qmax, qmin));

    Eigen::VectorXd qdotmax(_model_ptr_com->getNv()); qdotmax.setOnes();
    qdotmax *= 0.3;
    qdotmax.segment(0,6)<<100.,100.,100.,100.,100.,100.;
    OpenSoT::constraints::Aggregated::ConstraintPtr velocityLimits(
            new OpenSoT::constraints::velocity::VelocityLimits(*_model_ptr_com, qdotmax, 3e-3));


    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    // NOTE without this things don't work!
    bounds_list.push_back(velocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, _model_ptr_com->getNv()));

    OpenSoT::tasks::velocity::CoM::Ptr com_task(
                new OpenSoT::tasks::velocity::CoM(*(_model_ptr_com.get())));
    com_task->setLambda(.6);

    Eigen::Matrix3d W;
    W.setIdentity(); W(2,2) = .1;
    com_task->setWeight(W);

    OpenSoT::constraints::velocity::CartesianVelocity::ConstraintPtr boundsCoMVelocity(
                new OpenSoT::constraints::velocity::CartesianVelocity(
                    Eigen::Vector3d(0.05, 0.05, 0.05), 0.004,com_task));
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
                    *(_model_ptr_com.get()),
                    _links_in_contact, 0.01));
    com_task->getConstraints().push_back(boundsConvexHull);

    OpenSoT::tasks::velocity::Cartesian::Ptr right_foot_task(
                new OpenSoT::tasks::velocity::Cartesian("world::right_foot", *(_model_ptr_com.get()),
                                                        "r_sole",
                                                        "world"));
    right_foot_task->setLambda(.2);
    right_foot_task->setOrientationErrorGain(.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr left_foot_task(
                new OpenSoT::tasks::velocity::Cartesian("world::left_foot", *(_model_ptr_com.get()),
                                                        "l_sole",
                                                        "r_sole"));
    left_foot_task->setLambda(.2);
    left_foot_task->setOrientationErrorGain(.1);

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(*_model_ptr_com));

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    if(footStrategy == USE_TASK)
    {
        _log.close();
        _log.open("testQPOases_ConvexHull.m");

        stack_of_tasks.push_back(right_foot_task);
        stack_of_tasks.push_back(left_foot_task);
    }
    else if (footStrategy == USE_CONSTRAINT)
    {
        using namespace OpenSoT::constraints;
        TaskToConstraint::Ptr feet_constraint(new TaskToConstraint((right_foot_task + left_foot_task)));

        com_task->getConstraints().push_back(feet_constraint);
        postural_task->getConstraints().push_back(feet_constraint);

    }

    stack_of_tasks.push_back(com_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::iHQP::Ptr sot(new OpenSoT::solvers::iHQP(stack_of_tasks, bounds, 1));

    //SET SOME REFERENCES
    Eigen::Vector3d T_com_p_init;
    _model_ptr_com->getCOM(T_com_p_init);
    Eigen::Vector3d T_com_p_ref = T_com_p_init;
    T_com_p_ref[0] = 0.0;
    T_com_p_ref[1] = 0.0;

    std::cout << "Initial CoM position is " << T_com_p_init << std::endl;
    std::cout << "Moving to (0,0)" << std::endl;

    com_task->setReference(T_com_p_ref);

    Eigen::Vector3d tmp;
    tmp(0) = com_task->getActualPosition()[0];
    tmp(1) = com_task->getActualPosition()[1];
    tmp(2) = com_task->getActualPosition()[2];
    double e = (T_com_p_ref - tmp).norm();
    double previous_e = 0.0;

    unsigned int i = 0;
    const unsigned int n_iterations = 10000;
    if(footStrategy == USE_TASK)
        _log << "com_traj = [";
    else if(footStrategy == USE_CONSTRAINT)
        _log << "com_traj_constraint = [";

    Eigen::MatrixXd right_foot_pose = right_foot_task->getActualPose();

    for(i = 0; i < n_iterations; ++i)
    {
        _model_ptr_com->setJointPosition(q);
        _model_ptr_com->update();

        right_foot_task->update();
        std::cout<<"right_foot_task->getb().norm(): "<<right_foot_task->getb().norm()<<" @ iter: "<<i<<std::endl;
        left_foot_task->update();
        std::cout<<"left_foot_task->getb().norm(): "<<left_foot_task->getb().norm()<<" @ iter: "<<i<<std::endl;
        com_task->update();
        boundsConvexHull->log(logger);
        postural_task->update();
        bounds->update();

        _log << com_task->getActualPosition()[0] << ","
            << com_task->getActualPosition()[1] <<";";
        Eigen::Vector3d tmp;
        tmp(0) = com_task->getActualPosition()[0];
        tmp(1) = com_task->getActualPosition()[1];
        tmp(2) = com_task->getActualPosition()[2];
        e = (T_com_p_ref - tmp).norm();

        if(fabs(previous_e - e) < 1e-13) {
            std::cout << "i: " << i << " e: " << (T_com_p_ref - tmp).norm() << " . Error not decreasing. CONVERGED." << std::endl;
            break;
        }
        previous_e = e;

        Eigen::VectorXd dq(q.size());
        dq.setZero(dq.size());
        EXPECT_TRUE(sot->solve(dq));
        q = _model_ptr_com->sum(q, dq);

        Eigen::MatrixXd right_foot_pose_now = right_foot_task->getActualPose();
        for(unsigned int r = 0; r < 4; ++r)
            for(unsigned int c = 0; c < 4; ++c)
                ASSERT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-5)<<"iteration "<<i;
        ASSERT_LT(left_foot_task->getb().norm(),1e-8)<<"iteration "<<i;

    }

    tmp(0) = com_task->getActualPosition()[0];
    tmp(1) = com_task->getActualPosition()[1];
    tmp(2) = com_task->getActualPosition()[2];
    ASSERT_NEAR((T_com_p_ref - tmp).norm(),0,1E-9);

    _model_ptr_com->setJointPosition(q);
    _model_ptr_com->update();

    boundsConvexHull->update();
    boundsConvexHull->log(logger);
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> points_inner;
    boundsConvexHull->getConvexHull(points);

    Eigen::Vector3d point_old;
    Eigen::MatrixXd A_ch(_links_in_contact.size(),2), A_ch_outer(_links_in_contact.size(),2);
    Eigen::VectorXd b_ch(_links_in_contact.size()), b_ch_outer(_links_in_contact.size());
    boundsConvexHull->getConstraints(points, A_ch, b_ch, 0.01);
    boundsConvexHull->getConstraints(points, A_ch_outer, b_ch_outer, 0.0);
    std::cout << std::endl << "A_ch: " << std::endl << A_ch << std::endl;
    std::cout << std::endl << "b_ch: " << std::endl << b_ch << std::endl;
    std::cout << std::endl << "A_ch_outer: " << std::endl << A_ch_outer << std::endl;
    std::cout << std::endl << "b_ch_outer: " << std::endl << b_ch_outer << std::endl;
    std::cout << std::endl << "@q: " << std::endl << q << std::endl << std::endl;
    getPointsFromConstraints(A_ch, b_ch, points_inner);


    points.push_back(points.front());
    points_inner.push_back(points_inner.front());
    for(auto point : points)
    {
        std::cout << std::endl
                  << "=================" << std::endl
                  << "Moving from ("
                  << point_old[0] << "," << point_old[1]
                  << ") to ("
                  << point[0] << "," << point[1]
                  <<")" << std::endl;

        T_com_p_ref[0] = point[0];
        T_com_p_ref[1] = point[1];

        com_task->setReference(T_com_p_ref);

        Eigen::Vector3d tmp;
        tmp(0) = com_task->getActualPosition()[0];
        tmp(1) = com_task->getActualPosition()[1];
        tmp(2) = com_task->getActualPosition()[2];

        double e = (T_com_p_ref - tmp).norm();
        double previous_e = 0.0;
        double oscillation_check_e = 0.0;
        for(i = 0; i < n_iterations; ++i)
        {
            _model_ptr_com->setJointPosition(q);
            _model_ptr_com->update();

            right_foot_task->update();
            left_foot_task->update();
            com_task->update();
            boundsConvexHull->log(logger);
            postural_task->update();
            bounds->update();

            _log << com_task->getActualPosition()[0] << ","
                << com_task->getActualPosition()[1] <<";";
            Eigen::Vector3d tmp;
            tmp(0) = com_task->getActualPosition()[0];
            tmp(1) = com_task->getActualPosition()[1];
            tmp(2) = com_task->getActualPosition()[2];
            e = (T_com_p_ref - tmp).norm();

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



            Eigen::VectorXd dq(q.size());
            dq.setZero(dq.size());
            EXPECT_TRUE(sot->solve(dq));
            q = _model_ptr_com->sum(q, dq);


            Eigen::MatrixXd right_foot_pose_now = right_foot_task->getActualPose();
            for(unsigned int r = 0; r < 4; ++r)
                for(unsigned int c = 0; c < 4; ++c)
                    EXPECT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-3) << "Error at iteration "
                                                                                    << i
                                                                                    << " at position "
                                                                                    << "(" << r
                                                                                    << "," << c << ")"
                                                                                    << " with cartesian error equal to "
                                                                                    << right_foot_task->getb().norm() << std::endl;
            EXPECT_LT(right_foot_task->getb().norm(),5e-5);
            EXPECT_LT((right_foot_task->getA()*dq).norm(),1E-6) << "Error at iteration "
                                                             << i
                                                             << " J_foot*dq = "
                                                             << (right_foot_task->getA()*dq)
                                                             << std::endl;


        }


        tmp(0) = com_task->getActualPosition()[0];
        tmp(1) = com_task->getActualPosition()[1];
        tmp(2) = com_task->getActualPosition()[2];

        if(i == n_iterations)
            std::cout << "i: " << i << " e: " << (T_com_p_ref -
                                                      tmp).norm() << " -- Error not decreasing. STOPPING." << std::endl;

        Eigen::Vector3d distance = T_com_p_ref - tmp;
        double d = (distance).norm();
        Eigen::Vector2d exp_distance;
        exp_distance<<0.01,0.01;
        double expected_d = exp_distance.norm();

        std::vector<Eigen::Vector3d> points_check;
        boundsConvexHull->getConvexHull(points_check);
        Eigen::MatrixXd A_ch_check(_links_in_contact.size(), 2);
        Eigen::VectorXd b_ch_check(_links_in_contact.size());
        boundsConvexHull->getConstraints(points_check, A_ch_check, b_ch_check, 0.01);


        EXPECT_NEAR(d, expected_d, (expected_d-0.01)*1.01) << "Failed to reach point "
                                                           << point
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
        point_old = point;
    }



    _log << "];" << std::endl;

    if(footStrategy == USE_TASK)
    {
        _log << "points=[";
        for(auto point : points)
            _log << point[0] << "," << point[1] << ";";
        _log << "];" << std::endl;
        _log << "points_inner=[";
        for(auto point : points_inner)
            _log << point[0] << "," << point[1] << ";";
        _log << "];" << std::endl;

        _log << "ch_figure = figure('Position',[0 0 1027 768]); hold on; ref=plot(points(:,1), points(:,2), 'r-.'); constr=plot(points_inner(:,1), points_inner(:,2), 'g'); axis equal;" << std::endl;
        _log << "ct = plot(com_traj(:,1), com_traj(:,2), 'b--');" << std::endl;
    }
    else if(footStrategy==USE_CONSTRAINT)
    {
        _log << "ctc = plot(com_traj_constraint(:,1), com_traj_constraint(:,2), 'm:');" << std::endl;
        _log << "legend([ref,constr,ct,ctc], 'CoM ref', 'Support Polygon', 'CoM Traj, r\\_sole ctrl as task', 'CoM Traj, r\\_sole ctrl as constraint','Location','best');" << std::endl;
    }


}

INSTANTIATE_TEST_CASE_P(tryDifferentFootTaskStrategies,
                        testQPOases_ConvexHull,
                        ::testing::Values(USE_TASK, USE_CONSTRAINT));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
