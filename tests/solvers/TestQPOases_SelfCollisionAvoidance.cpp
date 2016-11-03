#include <iCub/iDynTree/yarp_kdl.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/utilities/error.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/utils/VelocityAllocation.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/SubTask.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>
#include <sstream>

using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"
#define TEST_SCA_FILE               "testQPOases_SCA.py"
/* WE WILL TRY SCA PERFORMANCE WHILE TUNING CARTESIAN TASK PARAMETERS, BOUND SCALING, SMOOTHING */
#define TEST_SCA_CT1_FILE           "testQPOases_SCA_CartesianTuning1.py"
#define TEST_SCA_COM_FILE           "testQPOases_SCA_CoMTuning.py"
#define TEST_SCA_BST_FILE           "testQPOases_SCA_BoundScalingTuning.py"
#define TEST_SCA_EPS_FILE           "testQPOases_SCA_EpsTuning.py"
#define TEST_SCA_SS_FILE            "testQPOases_SCA_SmallerStack.py"
#define TEST_SCA_PT_FILE            "testQPOases_SCA_PosturalTuning.py"
#define TEST_SCA_DS_FILE            "testQPOases_SCA_DistanceSmoothingTuning.py"
/* for each teas, we want to save:
   Cartesian error (x,y,z,theta,phy,psi)
   2-norm of task error ofr each task in the stack

   we also want to save distance vector for the closest pair of capsules, and the distance

   we won't save (though we will compute) the variance of the Cartesian error - it will be a measure of vibration
   and it will be used as an indicator of the success for the constraint
   */
#define TEST_SCA_CT1_ERRORS_FILE    "testQPOases_SCA_CT1_Errors"
#define TEST_SCA_CT1_DISTANCES_FILE "testQPOases_SCA_CT1_Distances"
#define TEST_SCA_COM_ERRORS_FILE    "testQPOases_SCA_COM_Errors"
#define TEST_SCA_COM_DISTANCES_FILE "testQPOases_SCA_COM_Distances"
#define TEST_SCA_BST_ERRORS_FILE    "testQPOases_SCA_BST_Errors"
#define TEST_SCA_BST_DISTANCES_FILE "testQPOases_SCA_BST_Distances"
#define TEST_SCA_EPS_ERRORS_FILE    "testQPOases_SCA_EPS_Errors"
#define TEST_SCA_EPS_DISTANCES_FILE "testQPOases_SCA_EPS_Distances"
#define TEST_SCA_SS_ERRORS_FILE     "testQPOases_SCA_SS_Errors"
#define TEST_SCA_SS_DISTANCES_FILE  "testQPOases_SCA_SS_Distances"
#define TEST_SCA_PT_ERRORS_FILE     "testQPOases_SCA_PT_Errors"
#define TEST_SCA_PT_DISTANCES_FILE  "testQPOases_SCA_PT_Distances"
#define TEST_SCA_DS_ERRORS_FILE     "testQPOases_SCA_DS_Errors"
#define TEST_SCA_DS_DISTANCES_FILE  "testQPOases_SCA_DS_Distances"
#define dT 25e-3

namespace {

enum SCA_SMOOTHING_STRATEGY { STRATEGY_CARTESIAN_TUNING_1,
                              STRATEGY_COM_TUNING,
                              STRATEGY_BOUNDSCALING_TUNING,
                              STRATEGY_EPS_TUNING,
                              STRATEGY_SMALLER_STACK,
                              STRATEGY_POSTURAL_TUNING,
                              STRATEGY_DISTANCE_SMOOTHING };

struct smoothing_parameter
{
    SCA_SMOOTHING_STRATEGY strategy;
    double param_1;
    double param_2;

    smoothing_parameter(SCA_SMOOTHING_STRATEGY strategy_,
                        double param_1_,
                        double param_2_) : strategy(strategy_), param_1(param_1_), param_2(param_2_)
    {
        ;
    }

    smoothing_parameter(SCA_SMOOTHING_STRATEGY strategy_,
                        double param_1_) : strategy(strategy_), param_1(param_1_)
    {
        ;
    }

    smoothing_parameter(SCA_SMOOTHING_STRATEGY strategy_) : strategy(strategy_)
    {
        ;
    }
};

static std::map<SCA_SMOOTHING_STRATEGY, int> must_append;

std::string getPlotFilename(SCA_SMOOTHING_STRATEGY strategy, std::string filename)
{
    std::stringstream filename_stream;
    filename_stream << filename << must_append[strategy] << ".eps";
    return filename_stream.str();
}

std::list<std::pair<std::string,std::string> > getWalkmanSCAWhiteList() {
    std::list<std::pair<std::string,std::string> > whiteList;

    // lower body - arms collision whitelist for WalkMan (for upper-body manipulation tasks - i.e. not crouching)
    whiteList.push_back(std::pair<std::string,std::string>("LLowLeg","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LHipMot","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RLowLeg","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RHipMot","RSoftHandLink"));

    // torso - arms collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RWrMot2"));

    // arm - am collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RWrMot2"));

    return whiteList;
}

yarp::sig::Vector getStablePosition(iDynUtils& model) {
    double q_vec[31] =                              {-0.052231, -0.001525, -0.507339,  0.994823, -0.500385,
                                                      0.054478,  0.047959,  0.001114, -0.504303,  0.989594,
                                                     -0.498290, -0.045896,  0.000282, -0.030130, -0.000038,
                                                      0.304843,  0.323796, -0.335870, -1.594352,  0.290507,
                                                     -0.138132, -0.544013, -0.000115,  0.001027,  0.284300,
                                                     -0.248960, -0.007307, -1.156816, -0.006277, -0.024351,
                                                      0.003562};
    yarp::sig::Vector q(31, q_vec);
    return q;
}

yarp::sig::Vector getShakingPosition(iDynUtils& model, yarp::sig::Matrix& l_arm_ref) {
    double q_vec[31] =                              {-0.015077,  0.007937, -0.518198,  1.413155, -0.791772,
                                                      0.049212,  0.096468, -0.002291, -0.514733,  1.402080,
                                                     -0.783909, -0.062951, -0.006270, -0.029317,  0.000176,
                                                      0.212405,  0.228882, -0.321196, -0.730445,  0.160591,
                                                     -0.580431, -1.130814,  0.007054, -0.014495,  0.451303,
                                                     -0.282880,  0.132477, -1.586318, -0.086197,  0.351271,
                                                     -0.010837};
    yarp::sig::Vector q(31, q_vec);
    yarp::sig::Vector q_old = model.iDyn3_model.getAng();
    model.updateiDyn3Model(q, true);
    l_arm_ref = model.iDyn3_model.getPosition(model.left_arm.end_effector_index);
    model.updateiDyn3Model(q_old, true);
    return q;
}

void setupIK(OpenSoT::AutoStack::Ptr& stack,
             OpenSoT::DefaultHumanoidStack& DHS,
             OpenSoT::solvers::QPOases_sot::Ptr& solver,
             iDynUtils &model,
             double eps,
             bool threeTasks = false)
{
    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftLeg and rightLeg,
    // task of priority two is leftArm and rightArm,
    // and the stack is subject to bounds jointLimits and velocityLimits
    if(!threeTasks)
        stack =
            ( DHS.rightLeg ) /
            ( (DHS.com_XY) << DHS.selfCollisionAvoidance ) /
            ( (DHS.leftArm + DHS.rightArm ) << DHS.selfCollisionAvoidance ) /
            ( (DHS.postural) << DHS.selfCollisionAvoidance );
    else
        stack =
            ( (DHS.rightLeg + DHS.com_XY) << DHS.selfCollisionAvoidance ) /
            ( (DHS.leftArm + DHS.rightArm ) << DHS.selfCollisionAvoidance ) /
            ( (DHS.postural) << DHS.selfCollisionAvoidance );
    stack << DHS.jointLimits; // << DHS.velocityLimits; commented since we are using VelocityALlocation


    /*                            */
    /*      CONFIGURING STACK     */
    /*                            */

    DHS.rightLeg->setLambda(0.6);   DHS.rightLeg->setOrientationErrorGain(1.0);
    DHS.leftLeg->setLambda(0.6);    DHS.leftLeg->setOrientationErrorGain(1.0);
    DHS.rightArm->setLambda(0.2);   DHS.rightArm->setOrientationErrorGain(0.3);
    DHS.leftArm->setLambda(0.2);    DHS.leftArm->setOrientationErrorGain(0.3);

    DHS.com_XY->setLambda(1.0);
    Eigen::VectorXd tmp(3);
    tmp<<0.1,0.1,0.1;
    DHS.comVelocity->setVelocityLimits(tmp);
    DHS.velocityLimits->setVelocityLimits(0.3);

    Eigen::MatrixXd pW = DHS.postural->getWeight();
    for(unsigned int i_t = 0; i_t < 3; ++i_t)
        pW(model.torso.joint_numbers[i_t],
            model.torso.joint_numbers[i_t]) *= 1e3;
    for(unsigned int i_t = 0; i_t < 6; ++i_t)
    {
        double amt = 7.5e1;
        if(i_t == 3 || i_t == 4)
            amt = 3;
        pW(model.left_leg.joint_numbers[i_t],
           model.left_leg.joint_numbers[i_t]) *= amt;
        pW(model.right_leg.joint_numbers[i_t],
           model.right_leg.joint_numbers[i_t]) *= amt;
    }
    DHS.postural->setWeight(pW);

    std::list<std::pair<std::string,std::string> > whiteList = getWalkmanSCAWhiteList();

    DHS.selfCollisionAvoidance->setCollisionWhiteList(whiteList);

    OpenSoT::VelocityAllocation(stack,
                                dT,
                                0.3,
                                0.6);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<Eigen::MatrixXd,Eigen::VectorXd>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<Eigen::MatrixXd,Eigen::VectorXd>::TaskPtr lastTask = stack->getStack()[stack->getStack().size()-1];
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(.9);
    }

    solver.reset(
        new OpenSoT::solvers::QPOases_sot(stack->getStack(),
                                          stack->getBounds(),
                                          eps));

}

class testQPOases_SCA:
        public ::testing::Test, 
        public ::testing::WithParamInterface<smoothing_parameter>
{
protected:
    std::ofstream   _log;
    double eps, epsns;
    int stack_off;      // if using 4 tasks, stack_off is 1, otherwise it's 0

    void openOrAppend(SCA_SMOOTHING_STRATEGY strategy, std::string filename)
    {
        if(must_append.count(strategy) > 0)
        {
            _log.open(filename.c_str(), std::fstream::app);
            must_append[strategy]++;
        }
        else
        {
            _log.open(filename.c_str());
            must_append[strategy] = 1;
        }
    }

    testQPOases_SCA()
    {
        _log.open(TEST_SCA_FILE);
        _log << "#! /usr/bin/env python" << std::endl
         << std::endl;
        _log << "execfile('" << TEST_SCA_CT1_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_COM_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_BST_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_EPS_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_SS_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_PT_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_SCA_DS_FILE << "')" << std::endl;

        eps = 5e10;
        epsns = eps;
        stack_off = 1;
    }

    virtual ~testQPOases_SCA() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

//#define TRY_ON_SIMULATOR
// will try script on the simulator, without the prescribed smoothing technique
//#define TRY_NVS

TEST_P(testQPOases_SCA, trySCASmoothing) {

    smoothing_parameter params = GetParam();

#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    WalkmanUtils robot("testSCA");
#endif

    iDynUtils model("bigman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");

    yarp::sig::Vector q = getStablePosition(model);
    yarp::sig::Vector qns = q;

    model.setFloatingBaseLink(model.left_leg.end_effector_name);
    model.updateiDyn3Model(q, true);

    yarp::sig::Matrix l_arm_ref;
    getShakingPosition(model, l_arm_ref);


#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    OpenSoT::AutoStack::Ptr stack, stackns;
    OpenSoT::solvers::QPOases_sot::Ptr sot, sotns;
    OpenSoT::DefaultHumanoidStack DHS(model, dT, cartesian_utils::toEigen(q));
    OpenSoT::DefaultHumanoidStack DHSns(model, dT, cartesian_utils::toEigen(qns));

    // we need to change the eps before creating the solver
    if(params.strategy == STRATEGY_EPS_TUNING)
        eps = params.param_1;

    bool smaller_stack_flag = false;  // do not use smaller stack by default
    if(params.strategy == STRATEGY_SMALLER_STACK)
    {
        stack_off = 0;
        smaller_stack_flag = true; // switch on the smaller stack flag
    }


    setupIK(stack, DHS, sot, model, eps, smaller_stack_flag); setupIK(stackns, DHSns, sotns, model, epsns);

    _log.close();

    std::stringstream smoothing_strategy_stream;
    std::stringstream smoothing_params_stream;
    std::stringstream regular_params_stream;

    // all the other changes to the stack need to be done after calling setupIK or the DHS will be reconfigured
    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1)
    {
        DHS.leftArm->setLambda(params.param_1);
        DHS.leftArm->setOrientationErrorGain(params.param_2);

        smoothing_params_stream << "lambda: " << params.param_1
                                << " , oe: " << params.param_2;
        regular_params_stream   << "lambda: " << DHSns.leftArm->getLambda()
                                << " , oe: " << DHSns.leftArm->getOrientationErrorGain();
        smoothing_strategy_stream << "Tuning of l_arm lambda and oe";

        this->openOrAppend(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_FILE);
    }
    else if(params.strategy == STRATEGY_COM_TUNING)
    {
        DHS.com_XY->setLambda(params.param_1);

        smoothing_params_stream << "lambda: " << params.param_1;
        regular_params_stream   << "lambda: " << DHSns.com_XY->getLambda();
        smoothing_strategy_stream << "Tuning of com_xy lambda";

        this->openOrAppend(STRATEGY_COM_TUNING, TEST_SCA_COM_FILE);
    }
    else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING)
    {
        DHS.selfCollisionAvoidance->setBoundScaling(params.param_1);

        smoothing_params_stream << "boundScaling: " << params.param_1;
        regular_params_stream   << "boundScaling: " << 1.0;
        smoothing_strategy_stream << "Tuning of SCA BoundScaling";

        this->openOrAppend(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_FILE);
    }
    else if(params.strategy == STRATEGY_EPS_TUNING)
    {
        smoothing_params_stream << "eps: " << eps;
        regular_params_stream   << "eps: " << epsns;
        smoothing_strategy_stream << "Tuning of solver eps";

        this->openOrAppend(STRATEGY_EPS_TUNING, TEST_SCA_EPS_FILE);
    }
    else if(params.strategy == STRATEGY_SMALLER_STACK)
    {
        smoothing_params_stream << "3 tasks";
        regular_params_stream   << "4 tasks";
        smoothing_strategy_stream << "CoM task has highest priority";

        this->openOrAppend(STRATEGY_SMALLER_STACK, TEST_SCA_SS_FILE);
    }
    else if(params.strategy == STRATEGY_POSTURAL_TUNING)
    {
        DHS.postural->setLambda(params.param_1);

        smoothing_params_stream << "lambda:" << params.param_1;
        regular_params_stream   << "lambda:" << DHSns.postural->getLambda();
        smoothing_strategy_stream << "Tuning of postural task lambda ";

        this->openOrAppend(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_FILE);
    }
    else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING)
    {
        _log.open(TEST_SCA_DS_FILE);
        /* @TODO implement */
    }
    else
    {
        std::cerr << "Error: Unknown Strategy";
        exit(1);
    }


    //SET SOME REFERENCES
    yarp::sig::Matrix desired_pose = l_arm_ref;
    yarp::sig::Vector dq(q.size(), 0.0);
    yarp::sig::Vector dqns(q.size(), 0.0);
    double e_com, e_com_ns, e_arms, e_arms_ns, e_post, e_post_ns;

    _log << "#! /usr/bin/env python" << std::endl
         << std::endl
         << "import numpy as np" << std::endl
         << "import matplotlib" << std::endl
         << "from matplotlib.pyplot import *" << std::endl;
    _log << "#t, x, y, z, r, p, y,"                 // 0-6
         << " xns, yns, zns, rns, pns, yns,"        // 7-12
         << " xref, yref, zref, rref, pref, yref,"  // 13-18
         << " e_com, e_com_ns,"                     // 19-20
         << " e_arms, e_arms_ns,"                   // 21-22
         << " e_post, e_post_ns,"                   // 23-24
         << " t_loop, t_loop_nva" << std::endl;     // 25-26
    _log << "test_data = np.array((";

    double t_loop = 0.0;
    double t_loopns = 0.0;
    bool settled = false;
    double settling_counter = 1.0;
    bool converged_event = false;

    DHS.leftArm->setReference(cartesian_utils::toEigen(desired_pose));
    DHSns.leftArm->setReference(cartesian_utils::toEigen(desired_pose));

#ifdef TRY_ON_SIMULATOR
    double t_test = yarp::os::Time::now();
#else
    double t_test = yarp::os::SystemClock::nowSystem();
#endif
    do
    {
#ifdef TRY_ON_SIMULATOR
        double t_begin = yarp::os::Time::now();
#else
        double t_begin = yarp::os::SystemClock::nowSystem();
#endif

        model.updateiDyn3Model(q, true);
        stack->update(cartesian_utils::toEigen(q));

        e_com = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stack->getStack()[stack_off  + 0]->getb()));
        e_arms = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stack->getStack()[stack_off + 1]->getb()));
        e_post = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stack->getStack()[stack_off + 2]->getb()));
        Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
        EXPECT_TRUE(sot->solve(_dq));
        dq = cartesian_utils::fromEigentoYarp(_dq);
        q += dq;

#ifdef TRY_ON_SIMULATOR
#ifndef TRY_NVS
        robot.move(q);
        yarp::os::Time::delay(0.005);
#endif

        t_loop = yarp::os::Time::now() - t_begin;
#else
        t_loop = yarp::os::SystemClock::nowSystem() - t_begin;
#endif

#ifdef TRY_ON_SIMULATOR
        t_begin = yarp::os::Time::now();
#else
        t_begin = yarp::os::SystemClock::nowSystem();
#endif

        model.updateiDyn3Model(qns, true);
        stackns->update(cartesian_utils::toEigen(qns));

        e_com_ns = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stackns->getStack()[1]->getb()));
        e_arms_ns = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stackns->getStack()[2]->getb()));
        e_post_ns = yarp::math::norm(
                    cartesian_utils::fromEigentoYarp(stackns->getStack()[3]->getb()));
        Eigen::VectorXd _dqns(dqns.size()); _dqns.setZero(dqns.size());
        EXPECT_TRUE(sotns->solve(_dqns));
        dqns = cartesian_utils::fromEigentoYarp(_dqns);
        qns+=dqns;

#ifdef TRY_ON_SIMULATOR
#ifdef TRY_NVS
        robot.move(qns);
        yarp::os::Time::delay(0.005);
#endif

        t_loopns = yarp::os::Time::now() - t_begin;
#else
        t_loopns = yarp::os::SystemClock::nowSystem() - t_begin;
#endif


        double r, p, y,         r_ns, p_ns, y_ns,                      r_ref, p_ref, y_ref;
        yarp::sig::Matrix F =     cartesian_utils::fromEigentoYarp(DHS.leftArm->getActualPose());
        yarp::sig::Matrix F_ns =  cartesian_utils::fromEigentoYarp(DHSns.leftArm->getActualPose());
        yarp::sig::Matrix F_ref = cartesian_utils::fromEigentoYarp(DHS.leftArm->getReference());
        KDL::Frame F_kdl,        F_ns_kdl,                             F_ref_kdl;
        YarptoKDL(F, F_kdl);     YarptoKDL(F_ns, F_ns_kdl);            YarptoKDL(F_ref, F_ref_kdl);
        F_kdl.M.GetRPY(r, p, y); F_ns_kdl.M.GetRPY(r_ns, p_ns, y_ns);  F_ref_kdl.M.GetRPY(r_ref, p_ref, y_ref);

#ifdef TRY_ON_SIMULATOR
        _log << "(" << yarp::os::Time::now() - t_test << ","
#else
        _log << "(" << yarp::os::SystemClock::nowSystem() - t_test << ","
#endif
            << (DHS.leftArm->getActualPose())(0,3) << ","       // 1
            << (DHS.leftArm->getActualPose())(1,3) << ","
            << (DHS.leftArm->getActualPose())(2,3) << ","
            << (DHSns.leftArm->getActualPose())(0,3) << ","     // 4
            << (DHSns.leftArm->getActualPose())(1,3) << ","
            << (DHSns.leftArm->getActualPose())(2,3) << ","
            << (DHS.leftArm->getReference())(0,3) << ","        // 7
            << (DHS.leftArm->getReference())(1,3) << ","
            << (DHS.leftArm->getReference())(2,3) << ","
            << r  << ","                                        // 10
            << p  << ","
            << y  << ","
            << r_ns  << ","                                     // 13
            << p_ns  << ","
            << y_ns  << ","
            << r_ref  << ","                                    // 16
            << p_ref  << ","
            << y_ref  << ","
            << e_com << ","                                     // 19
            << e_com_ns << ","
            << e_arms << ","                                    // 21
            << e_arms_ns << ","
            << e_post << ","                                    // 23
            << e_post_ns << ","
            << t_loop << ","                                    // 25
            << t_loopns << ")," << std::endl;                   // 26

        if(e_com < 1.5e-3 && e_com_ns < 1.5e-3 && !converged_event)
        {
            converged_event = true;
            std::cout << "settling";
            std::cout.flush();
        }

        if(converged_event)
        {
            settling_counter -= 3e-3;
            std::cout << ".";
            std::cout.flush();
        }

        if(settling_counter <= 0.0)
        {
            settled = true;
            std::cout << std::endl;
            std::cout.flush();
        }

    } while(e_com > 1.5e-3 || e_com_ns > 1.5e-3 || !settled);

    _log << "));" << std::endl;

    _log << "se = figure('"<< smoothing_strategy_stream.str() << "- Cartesian Errors',figsize=(10.27,7.68));" << std::endl;
    std::string smoothing = smoothing_params_stream.str();
    std::string no_smoothing = regular_params_stream.str();

    _log << "subplot(3,2,1); p = plot(test_data[:,0], test_data[:,(1,4,7)]); title('l_arm x');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,2); p = plot(test_data[:,0], test_data[:,(2,5,8)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,3); p = plot(test_data[:,0], test_data[:,(3,6,9)]); title('l_arm z');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,4); p = plot(test_data[:,0], test_data[:,(10,13,16)]); title('l_arm r');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,5); p = plot(test_data[:,0], test_data[:,(11,14,17)]); title('l_arm p');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,6); p = plot(test_data[:,0], test_data[:,(12,15,18)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "et = figure('"<< smoothing_strategy_stream.str() << "- Task Errors',figsize=(8,6));" << std::endl << std::endl;

    _log << "subplot(2,2,1); p = plot(test_data[:,0],test_data[:,(25, 26)]);" << std::endl;
    _log << "title('Computation Time');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('Solve Time [s]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,2); p = plot(test_data[:,0],test_data[:,(19, 20)]);" << std::endl;
    _log << "title('CoM_XY Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,3); p = plot(test_data[:,0],test_data[:,(21, 22)]);" << std::endl;
    _log << "title('l_arm + r_arm Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,4); p = plot(test_data[:,0],test_data[:,(23, 24)]);" << std::endl;
    _log << "title('Postural Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_COM_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_EPS_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_SMALLER_STACK) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_POSTURAL_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING) {
        _log << "se.savefig('" << TEST_SCA_BST_DISTANCES_FILE << ".eps', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << TEST_SCA_BST_ERRORS_FILE << ".eps',format='eps',transparent=True);" << std::endl;
    } else {
        std::cerr << "Unhandled exception at line " << __LINE__ << std::endl;
        exit(1);
    }
    _log << "show(block=True)" << std::endl;
}

INSTANTIATE_TEST_CASE_P(trySCASmoothingWith_CT_BST,
                        testQPOases_SCA,
                        ::testing::Values(smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.2),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.1),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.05),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.4),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.5),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.1),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.05),
                                          smoothing_parameter(STRATEGY_COM_TUNING,0.01),
                                          smoothing_parameter(STRATEGY_COM_TUNING,0.1),
                                          smoothing_parameter(STRATEGY_COM_TUNING,0.3),
                                          smoothing_parameter(STRATEGY_COM_TUNING,0.7),
                                          smoothing_parameter(STRATEGY_COM_TUNING,1.3),
                                          smoothing_parameter(STRATEGY_COM_TUNING,1.6),
                                          smoothing_parameter(STRATEGY_COM_TUNING,2.0),
                                          smoothing_parameter(STRATEGY_BOUNDSCALING_TUNING, 0.6),
                                          smoothing_parameter(STRATEGY_BOUNDSCALING_TUNING, 0.3),
                                          smoothing_parameter(STRATEGY_BOUNDSCALING_TUNING, 1.3),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,2e10),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e10),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e9),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e8),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e7),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e6),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e5),
                                          smoothing_parameter(STRATEGY_EPS_TUNING,1e2),
                                          smoothing_parameter(STRATEGY_SMALLER_STACK),
                                          smoothing_parameter(STRATEGY_POSTURAL_TUNING, 0.6),
                                          smoothing_parameter(STRATEGY_POSTURAL_TUNING, 0.3)));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
