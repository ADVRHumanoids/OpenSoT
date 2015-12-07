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
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"
#define TEST_PI_FILE               "testQPOases_PI.py"
/* WE WILL INVESTIGATE PRIORITY INVERSIONE WHILE TUNING CARTESIAN TASK PARAMETERS, BOUND SCALING, SMOOTHING */
#define TEST_PI_CT1_FILE           "testQPOases_PI_CartesianTuning1.py"
#define TEST_PI_SVT_FILE           "testQPOases_PI_SubTaskVsTask.py"
#define TEST_PI_BST_FILE           "testQPOases_PI_BoundScalingTuning.py"
#define TEST_PI_EPS_FILE           "testQPOases_PI_EpsTuning.py"
#define TEST_PI_SS_FILE            "testQPOases_PI_SmallerStack.py"
#define TEST_PI_PT_FILE            "testQPOases_PI_PosturalTuning.py"
#define TEST_PI_DS_FILE            "testQPOases_PI_DistanceSmoothingTuning.py"
/* for each teas, we want to save:
   Cartesian error (x,y,z,theta,phy,psi)
   2-norm of task error ofr each task in the stack

   we also want to save distance vector for the closest pair of capsules, and the distance

   we won't save (though we will compute) the variance of the Cartesian error - it will be a measure of vibration
   and it will be used as an indicator of the success for the constraint
   */
#define TEST_PI_CT1_ERRORS_FILE    "testQPOases_PI_CT1_Errors"
#define TEST_PI_SVT_ERRORS_FILE    "testQPOases_PI_SVT_Errors"
#define TEST_PI_BST_ERRORS_FILE    "testQPOases_PI_BST_Errors"
#define TEST_PI_EPS_ERRORS_FILE    "testQPOases_PI_EPS_Errors"
#define TEST_PI_SS_ERRORS_FILE     "testQPOases_PI_SS_Errors"
#define TEST_PI_PT_ERRORS_FILE     "testQPOases_PI_PT_Errors"
#define TEST_PI_DS_ERRORS_FILE     "testQPOases_PI_DS_Errors"
#define dT 25e-3

#define N_COM_PARAMS 4

namespace {

double CoM_lambda_weights[N_COM_PARAMS] = {0.01, 0.1, 0.7, 1.7};

enum PI_SMOOTHING_STRATEGY { STRATEGY_CARTESIAN_TUNING_1,
                             STRATEGY_SUBTASK_VS_TASK_LAMBDA,
                             STRATEGY_BOUNDSCALING_TUNING,
                             STRATEGY_EPS_TUNING,
                             STRATEGY_SMALLER_STACK,
                             STRATEGY_POSTURAL_TUNING,
                             STRATEGY_DISTANCE_SMOOTHING };

struct smoothing_parameter
{
    PI_SMOOTHING_STRATEGY strategy;
    double param_1;
    double param_2;

    smoothing_parameter(PI_SMOOTHING_STRATEGY strategy_,
                        double param_1_,
                        double param_2_) : strategy(strategy_), param_1(param_1_), param_2(param_2_)
    {
        ;
    }

    smoothing_parameter(PI_SMOOTHING_STRATEGY strategy_,
                        double param_1_) : strategy(strategy_), param_1(param_1_)
    {
        ;
    }

    smoothing_parameter(PI_SMOOTHING_STRATEGY strategy_) : strategy(strategy_)
    {
        ;
    }
};

static std::map<PI_SMOOTHING_STRATEGY, int> must_append;

std::string getPlotFilename(PI_SMOOTHING_STRATEGY strategy, std::string filename)
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
             boost::shared_ptr<OpenSoT::DefaultHumanoidStack>& DHS,
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
            ( DHS->rightLeg ) /
            ( (DHS->com_XY) << DHS->selfCollisionAvoidance ) /
            ( (DHS->leftArm + DHS->rightArm ) << DHS->selfCollisionAvoidance ) /
            ( (DHS->postural) << DHS->selfCollisionAvoidance );
    else
        stack =
            ( (DHS->rightLeg + DHS->com_XY) << DHS->selfCollisionAvoidance ) /
            ( (DHS->leftArm + DHS->rightArm ) << DHS->selfCollisionAvoidance ) /
            ( (DHS->postural) << DHS->selfCollisionAvoidance );
    stack << DHS->jointLimits; // << DHS->velocityLimits; commented since we are using VelocityALlocation


    /*                            */
    /*      CONFIGURING STACK     */
    /*                            */

    DHS->rightLeg->setLambda(0.6);   DHS->rightLeg->setOrientationErrorGain(1.0);
    DHS->leftLeg->setLambda(0.6);    DHS->leftLeg->setOrientationErrorGain(1.0);
    DHS->rightArm->setLambda(0.2);   DHS->rightArm->setOrientationErrorGain(0.3);
    DHS->leftArm->setLambda(0.2);    DHS->leftArm->setOrientationErrorGain(0.3);

    DHS->com_XY->setLambda(1.0);
    DHS->comVelocity->setVelocityLimits(yarp::sig::Vector(0.1,3));
    DHS->velocityLimits->setVelocityLimits(0.3);

    yarp::sig::Matrix pW = DHS->postural->getWeight();
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
    DHS->postural->setWeight(pW);

    std::list<std::pair<std::string,std::string> > whiteList = getWalkmanSCAWhiteList();

    DHS->selfCollisionAvoidance->setCollisionWhiteList(whiteList);

    OpenSoT::VelocityAllocation(stack,
                                dT,
                                0.3,
                                0.6);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = stack->getStack()[stack->getStack().size()-1];
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

class testQPOases_PI:
        public ::testing::Test,
        public ::testing::WithParamInterface<smoothing_parameter>
{
protected:
    std::ofstream   _log;
    double eps, epsns;
    int stack_off;      // if using 4 tasks, stack_off is 1, otherwise it's 0

    void openOrAppend(PI_SMOOTHING_STRATEGY strategy, std::string filename)
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

    testQPOases_PI()
    {
        _log.open(TEST_PI_FILE);
        _log << "#! /usr/bin/env python" << std::endl
         << std::endl;
        _log << "execfile('" << TEST_PI_CT1_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_SVT_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_BST_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_EPS_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_SS_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_PT_FILE << "')" << std::endl;
        _log << "execfile('" << TEST_PI_DS_FILE << "')" << std::endl;

        eps = 5e10;
        epsns = eps;
        stack_off = 1;
    }

    virtual ~testQPOases_PI() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_P(testQPOases_PI, tryPISmoothing) {
    smoothing_parameter params = GetParam();

    iDynUtils model("bigman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");

    std::vector<yarp::sig::Vector> q_vec(N_COM_PARAMS, getStablePosition(model));
    std::vector<yarp::sig::Vector> qns_vec(N_COM_PARAMS, getStablePosition(model));

    model.setFloatingBaseLink(model.left_leg.end_effector_name);
    model.updateiDyn3Model(q_vec[0], true);

    yarp::sig::Matrix l_arm_ref;
    getShakingPosition(model, l_arm_ref);

    std::vector<OpenSoT::AutoStack::Ptr> stack_vec(N_COM_PARAMS);
    std::vector<OpenSoT::AutoStack::Ptr> stackns_vec(N_COM_PARAMS);

    std::vector<OpenSoT::solvers::QPOases_sot::Ptr> sot_vec(N_COM_PARAMS);
    std::vector<OpenSoT::solvers::QPOases_sot::Ptr> sotns_vec(N_COM_PARAMS);

    std::vector< boost::shared_ptr<OpenSoT::DefaultHumanoidStack> > DHS_vec(N_COM_PARAMS);
    std::vector< boost::shared_ptr<OpenSoT::DefaultHumanoidStack> > DHSns_vec(N_COM_PARAMS);
    for(unsigned int i=0; i < N_COM_PARAMS; ++i)
    {
        DHS_vec[i].reset(new OpenSoT::DefaultHumanoidStack(model, dT, q_vec[i]));
        DHSns_vec[i].reset(new OpenSoT::DefaultHumanoidStack(model, dT, q_vec[i]));
    }

    // we need to change the eps before creating the solver
    if(params.strategy == STRATEGY_EPS_TUNING)
        eps = params.param_1;

    bool smaller_stack_flag = false;  // do not use smaller stack by default
    if(params.strategy == STRATEGY_SMALLER_STACK)
    {
        stack_off = 0;
        smaller_stack_flag = true; // switch on the smaller stack flag
    }


    for(unsigned int i=0; i < N_COM_PARAMS; ++i)
    {
        setupIK(stack_vec[i], DHS_vec[i], sot_vec[i], model, eps, smaller_stack_flag);
        setupIK(stackns_vec[i], DHSns_vec[i], sotns_vec[i], model, epsns);

        DHSns_vec[i]->com_XY->setLambda(CoM_lambda_weights[i]);

        if(params.strategy == STRATEGY_SUBTASK_VS_TASK_LAMBDA)
            DHS_vec[i]->com->setLambda(CoM_lambda_weights[i]);
        else
            DHS_vec[i]->com_XY->setLambda(CoM_lambda_weights[i]);
    }

    _log.close();

    std::stringstream smoothing_strategy_stream;
    std::stringstream smoothing_params_stream;
    std::stringstream regular_params_stream;

    // all the other changes to the stack need to be done after calling setupIK or the DHS will be reconfigured
    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1)
    {
        for(unsigned int i=0; i < N_COM_PARAMS; ++i)
        {
            DHS_vec[i]->leftArm->setLambda(params.param_1);
            DHS_vec[i]->leftArm->setOrientationErrorGain(params.param_2);
        }

        smoothing_params_stream << "lambda: " << params.param_1
                                << " , oe: " << params.param_2;
        regular_params_stream   << "lambda: " << DHSns_vec[0]->leftArm->getLambda()
                                << " , oe: " << DHSns_vec[0]->leftArm->getOrientationErrorGain();
        smoothing_strategy_stream << "Tuning of l_arm lambda and oe";

        this->openOrAppend(STRATEGY_CARTESIAN_TUNING_1, TEST_PI_CT1_FILE);
    }
    else if(params.strategy == STRATEGY_SUBTASK_VS_TASK_LAMBDA)
    {
        smoothing_params_stream << "subtask lambda tuning";
        regular_params_stream   << "subtask lambda tuning";
        smoothing_strategy_stream << "Tuning of com_XY vs com lambda";

        this->openOrAppend(STRATEGY_SUBTASK_VS_TASK_LAMBDA, TEST_PI_SVT_FILE);
    }
    else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING)
    {
        for(unsigned int i=0; i < N_COM_PARAMS; ++i)
            DHS_vec[i]->selfCollisionAvoidance->setBoundScaling(params.param_1);

        smoothing_params_stream << "boundScaling: " << params.param_1;
        regular_params_stream   << "boundScaling: " << 1.0;
        smoothing_strategy_stream << "Tuning of SCA BoundScaling";

        this->openOrAppend(STRATEGY_BOUNDSCALING_TUNING, TEST_PI_BST_FILE);
    }
    else if(params.strategy == STRATEGY_EPS_TUNING)
    {
        smoothing_params_stream << "eps: " << eps;
        regular_params_stream   << "eps: " << epsns;
        smoothing_strategy_stream << "Tuning of solver eps";

        this->openOrAppend(STRATEGY_EPS_TUNING, TEST_PI_EPS_FILE);
    }
    else if(params.strategy == STRATEGY_SMALLER_STACK)
    {
        smoothing_params_stream << "3 tasks";
        regular_params_stream   << "4 tasks";
        smoothing_strategy_stream << "CoM task has highest priority";

        this->openOrAppend(STRATEGY_SMALLER_STACK, TEST_PI_SS_FILE);
    }
    else if(params.strategy == STRATEGY_POSTURAL_TUNING)
    {
        for(unsigned int i=0; i < N_COM_PARAMS; ++i)
            DHS_vec[i]->postural->setLambda(params.param_1);

        smoothing_params_stream << "lambda:" << params.param_1;
        regular_params_stream   << "lambda:" << DHSns_vec[0]->postural->getLambda();
        smoothing_strategy_stream << "Tuning of postural task lambda ";

        this->openOrAppend(STRATEGY_POSTURAL_TUNING, TEST_PI_PT_FILE);
    }
    else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING)
    {
        _log.open(TEST_PI_DS_FILE);
        /* @TODO implement */
    }
    else
    {
        std::cerr << "Error: Unknown Strategy";
        exit(1);
    }


    //SET SOME REFERENCES
    yarp::sig::Matrix desired_pose = l_arm_ref;
    yarp::sig::Vector dq(q_vec[0].size(), 0.0);
    yarp::sig::Vector dqns(dq);
    std::vector<double> e_com_vec(N_COM_PARAMS);
    std::vector<double> e_post_vec(N_COM_PARAMS);
    std::vector<double> e_com_ns_vec(N_COM_PARAMS);
    std::vector<double> e_post_ns_vec(N_COM_PARAMS);

    _log << "#! /usr/bin/env python" << std::endl
         << std::endl
         << "import numpy as np" << std::endl
         << "import matplotlib" << std::endl
         << "from matplotlib.pyplot import *" << std::endl;
    _log << "#t"                                    // 0
         << " e_com,"                               // 1-(N_COM_PARAMS-1)
         << " e_com_ns,"                            // N_COM_PARAMS-(2*N_COM_PARAMS-1)
         << " e_post,"                              // 2*N_COM_PARAMS-(3*N_COM_PARAMS-1)
         << " e_post_ns"                            // 3*N_COM_PARAMS-(4*N_COM_PARAMS-1)
         << std::endl;
    _log << "test_data = np.array((";

    bool settled = false;
    double settling_counter = 1.0;
    bool converged_event = false;

    for(unsigned int i=0; i < N_COM_PARAMS; ++i)
    {
        DHS_vec[i]->leftArm->setReference(desired_pose);
        DHSns_vec[i]->leftArm->setReference(desired_pose);
    }

    double t_test = yarp::os::SystemClock::nowSystem();
    do
    {
        for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
        {
            model.updateiDyn3Model(q_vec[i], true);
            stack_vec[i]->update(q_vec[i]);

            e_com_vec[i] = yarp::math::norm(stack_vec[i]->getStack()[stack_off  + 0]->getb());
            e_post_vec[i] = yarp::math::norm(stack_vec[i]->getStack()[stack_off + 2]->getb());

            EXPECT_TRUE(sot_vec[i]->solve(dq));
            q_vec[i] += dq;

            model.updateiDyn3Model(qns_vec[i], true);
            stackns_vec[i]->update(qns_vec[i]);

            e_com_ns_vec[i] = yarp::math::norm(stackns_vec[i]->getStack()[1]->getb());
            e_post_ns_vec[i] = yarp::math::norm(stackns_vec[i]->getStack()[3]->getb());

            EXPECT_TRUE(sotns_vec[i]->solve(dqns));
            qns_vec[i]+=dqns;
        }


        _log << "(" << yarp::os::SystemClock::nowSystem() - t_test << ",";
        for(unsigned int i = 0; i < N_COM_PARAMS; ++i) // 1-(N_COM_PARAMS)
            _log << e_com_vec[i]     << ",";
        for(unsigned int i = 0; i < N_COM_PARAMS; ++i) // (N_COM_PARAMS+1)-2*N_COM_PARAMS
            _log << e_com_ns_vec[i]     << ",";
        for(unsigned int i = 0; i < N_COM_PARAMS; ++i) // (2*N_COM_PARAMS+1)-3*N_COM_PARAMS
            _log << e_post_vec[i]     << ",";
        for(unsigned int i = 0; i < N_COM_PARAMS; ++i) // (3*N_COM_PARAMS+1)-4*N_COM_PARAMS
            _log << e_post_ns_vec[i]     << ",";
        _log << ")," << std::endl;

        if(*std::max_element(e_com_vec.begin(),
                             e_com_vec.end()) < 1.5e-3 &&
           *std::max_element(e_com_ns_vec.begin(),
                             e_com_ns_vec.end()) < 1.5e-3 && !converged_event)
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

    } while(*std::max_element(e_com_vec.begin(),
                              e_com_vec.end()) > 1.5e-3 ||
            *std::max_element(e_com_ns_vec.begin(),
                              e_com_ns_vec.end()) > 1.5e-3 || !settled);

    _log << "));" << std::endl;

    _log << "et = figure('"<< smoothing_strategy_stream.str() << "- Task Errors',figsize=(8,6));" << std::endl << std::endl;

    _log << "subplot(1,2,1); p = plot(test_data[:,0],test_data[:,(";
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << 1+i;
        _log << cp.str() <<",";
    }
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << N_COM_PARAMS+1+i;
        _log << cp.str() <<",";
    }
    _log << ")]);" << std::endl;
    _log << "title('CoM_XY Task Error');" << std::endl;
    _log << "legend(p,(";
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << "'" << smoothing_params_stream.str()
           << "; lambda: " << CoM_lambda_weights[i] << "',";
        _log << cp.str();
    }
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << "'" << regular_params_stream.str()
           << "; lambda: " << CoM_lambda_weights[i] << "',";
        _log << cp.str();
    }
    _log << "));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(1,2,2); p = plot(test_data[:,0],test_data[:,(";
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << 2*N_COM_PARAMS+1+i;
        _log << cp.str() <<",";
    }
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << 3*N_COM_PARAMS+1+i;
        _log << cp.str() <<",";
    }
    _log << ")]);" << std::endl;
    _log << "title('Postural Task Error');" << std::endl;
    _log << "legend(p,(";
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << "'" << smoothing_params_stream.str()
           << "; lambda: " << CoM_lambda_weights[i] << "',";
        _log << cp.str();
    }
    for(unsigned int i = 0; i < N_COM_PARAMS; ++i)
    {
        std::stringstream cp;
        cp << "'" << regular_params_stream.str()
           << "; lambda: " << CoM_lambda_weights[i] << "',";
        _log << cp.str();
    }
    _log << "));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_PI_CT1_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_SUBTASK_VS_TASK_LAMBDA) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_SUBTASK_VS_TASK_LAMBDA, TEST_PI_SVT_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_PI_BST_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_EPS_TUNING) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_PI_EPS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_SMALLER_STACK) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_PI_SS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_POSTURAL_TUNING) {
        _log << "et.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_PI_PT_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING) {
        _log << "et.savefig('" << TEST_PI_BST_ERRORS_FILE << ".eps',format='eps',transparent=True);" << std::endl;
    } else {
        std::cerr << "Unhandled exception at line " << __LINE__ << std::endl;
        exit(1);
    }
    _log << "show(block=True)" << std::endl;
}

INSTANTIATE_TEST_CASE_P(trySCASmoothingWith_CT_BST,
                        testQPOases_PI,
                        ::testing::Values(smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.2),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.1),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.05),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.2, 0.4),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.5),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.1),
                                          smoothing_parameter(STRATEGY_CARTESIAN_TUNING_1,0.5, 0.05),
                                          smoothing_parameter(STRATEGY_SUBTASK_VS_TASK_LAMBDA, 0.1),
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
