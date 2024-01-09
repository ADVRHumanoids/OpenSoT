#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <qpOASES.hpp>
#include <fstream>

#include <xbot2_interface/xbotinterface2.h>
#include <chrono>
#include <ctime>
#include <thread>



#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

typedef std::pair<KDL::Path::IdentifierType,bool> testType;

class testQPOases_TestFF: public ::testing::Test
{
protected:
    typedef std::shared_ptr<KDL::Trajectory> TrajPtr;
    typedef std::shared_ptr<KDL::Path> PathPtr;
    typedef std::shared_ptr<KDL::VelocityProfile> VelProfPtr;
    typedef std::shared_ptr<KDL::RotationalInterpolation> RotIntPtr;
    std::ofstream _log;

    RotIntPtr rotationInterpolationMethod;
    PathPtr path;
    VelProfPtr velocityProfile;
    TrajPtr trajectory;

    testQPOases_TestFF() :
        rotationInterpolationMethod(
            new KDL::RotationalInterpolation_SingleAxis())
    {
    }

    virtual ~testQPOases_TestFF() {
    }


    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void get5cmFwdLinearTraj(KDL::Frame& start) {

        // 5 cm forward
        KDL::Frame end = start;
        end.p[0] += .05;

        // construct an equivalent radius so that the length
        // of the path along the arc of .1rad will be .05m
        // .05 = (eqRad*.1);
        double eqRad = .05/.1;

        path = PathPtr( new KDL::Path_Line(start, end,
                                           rotationInterpolationMethod.get()->Clone(),
                                           eqRad));

        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.01,.005));
        velocityProfile->SetProfile(0,path->PathLength());
        trajectory = TrajPtr( new KDL::Trajectory_Segment(path.get()->Clone(),
                                                          velocityProfile.get()->Clone()));
    }

    void get1radTraj(const double q0) {

        // 1 rad clockwise
        KDL::Frame start;
        start.p[0] = q0;
        KDL::Frame end = start;
        end.p[0] += 1;

        // construct an equivalent radius so that the length
        // of the path along the arc of .1rad will be .05m
        // .05 = (eqRad*1);
        double eqRad = .05/1;

        path = PathPtr( new KDL::Path_Line(start, end,
                                           rotationInterpolationMethod.get()->Clone(),
                                           eqRad));

        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.2,.1));
        velocityProfile->SetProfile(0,path->PathLength());
        trajectory = TrajPtr( new KDL::Trajectory_Segment(path.get()->Clone(),
                                                          velocityProfile.get()->Clone()));
    }

    void get1radCircularTraj(KDL::Frame& start) {
        // 1rad clocwise rotation
        KDL::Frame end = start;
        end.M.DoRotX(1);

        // construct an equivalent radius so that the length
        // of the path along the arc of .1rad will be .05m
        // .05 = (eqRad*.1); .. but make it faster! (nd Alessio)
        double eqRad = .05/1;
        path = PathPtr( new KDL::Path_Line(start, end,
                                           rotationInterpolationMethod.get()->Clone(),
                                           eqRad));
        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.01,.1));
        velocityProfile->SetProfile(0,path->PathLength());
        trajectory = TrajPtr( new KDL::Trajectory_Segment(path.get()->Clone(),
                                                          velocityProfile.get()->Clone()));
    }
};

class testQPOases_CartesianFF: public testQPOases_TestFF,
        public ::testing::WithParamInterface<testType>
{
protected:

    testQPOases_CartesianFF()
    {
        _log.open("testQPOases_CartesianFF.m");
        _log << "testQPOases_FF_Cartesian_5cmfw_noerr" << std::endl;
        _log << "testQPOases_FF_Cartesian_5cmfw_1cmerr" << std::endl;
        _log << "testQPOases_FF_Cartesian_1rad_noerr" << std::endl;
        _log << "testQPOases_FF_Cartesian_1rad_1draderr" << std::endl;
    }

    virtual ~testQPOases_CartesianFF() {
        _log.close();
    }


    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testQPOases_CoMAndPosturalFF: public testQPOases_TestFF,
        public ::testing::WithParamInterface<bool>
{
protected:

    testQPOases_CoMAndPosturalFF()
    {
        _log.open("testQPOases_CoMAndPosturalFF.m");
        _log << "testQPOases_FF_CoM_5cmfw_noerr" << std::endl;
        _log << "testQPOases_FF_CoM_5cmfw_1cmerr" << std::endl;
        _log << "testQPOases_FF_Postural_1rad_noerr" << std::endl;
        _log << "testQPOases_FF_Postural_1rad_1draderr" << std::endl;
    }

    virtual ~testQPOases_CoMAndPosturalFF() {
        _log.close();
    }


    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _robot) {

    Eigen::VectorXd _q(_robot->getJointNum());
    _q.setZero(_q.size());

    _q[_robot->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_robot->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_robot->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_robot->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_robot->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_robot->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_robot->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_robot->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_robot->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_robot->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_robot->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_robot->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;

}



TEST_P(testQPOases_CartesianFF, testCartesianFF)
{

    KDL::Path::IdentifierType  trajType = GetParam().first;
    bool hasInitialError = GetParam().second;



    std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";

    std::string _path_to_cfg = relative_path;

    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;


    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();



    // BOUNDS
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(q,qmax,qmin);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsVelocityLimits =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>( 0.6,3e-3,q.size());

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    bounds_list.push_back(boundsVelocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds =
            std::make_shared<OpenSoT::constraints::Aggregated>(bounds_list, q.size());

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm_task =
            std::make_shared<OpenSoT::tasks::velocity::Cartesian>("l_arm",q, *(_model_ptr.get()),
                                                        "l_wrist",
                                                        "world");

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(q);

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(l_arm_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::iHQP::Ptr sot =
            std::make_shared<OpenSoT::solvers::iHQP>(stack_of_tasks, bounds,1e8);



    Eigen::VectorXd dq(q.size()); dq.setZero(q.size());

    double dt=3e-3;

    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist twist_estimate, previous_twist_estimate, desired_twist;
    Eigen::MatrixXd current_pose_y;
    double R, Rdes, Rprev, P, Pdes, Pprev, Y, Ydes, Yprev;


    q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    l_arm_task->update(q);
    postural_task->update(q);
    bounds->update(q);

    current_pose_y = l_arm_task->getActualPose();
    l_arm_task->getActualPose(current_pose);

    if(!hasInitialError) {

        if(trajType == KDL::Path::ID_LINE) {
            /************************************************
             * COMMANDING 5cm FORWARD FROM CURRENT POSITION
             * meaning zero error at trajectory begin
             ***********************************************/

            _log.close();
            _log.open("testQPOases_FF_Cartesian_5cmfw_noerr.m");


            get5cmFwdLinearTraj(current_pose);

            l_arm_task->setOrientationErrorGain(.1);//0.1
        } else {
            /************************************************
             * COMMANDING 1rad CLOCKWISE FROM CURRENT POSITION
             * meaning zero error at trajectory begin
             ***********************************************/

            _log.close();
            _log.open("testQPOases_FF_Cartesian_1rad_noerr.m");

            get1radCircularTraj(current_pose);

            l_arm_task->setOrientationErrorGain(.5);//0.5
        }

        desired_pose = trajectory->Pos(0.0);

        l_arm_task->setLambda(.6);//0.6

    } else {
        if(trajType == KDL::Path::ID_LINE) {
            /*************************************************
             * COMMANDING 5cm FORWARD FROM PERTURBED POSITION
             * 1cm error at trajectory startup
             *************************************************/

            _log.close();
            _log.open("testQPOases_FF_Cartesian_5cmfw_1cmerr.m");

            desired_pose = current_pose;
            desired_pose.p[0] = current_pose.p[0] + .01;
            get5cmFwdLinearTraj(desired_pose);
            desired_pose = trajectory->Pos(0.0);

            l_arm_task->setOrientationErrorGain(.1);
            /* setting lambda lower than this can cause tracking problems
             * along the trajectory on secondary variables (e.g. the one that
             * we want fixed at 0) */
            l_arm_task->setLambda(.1);
        } else {
            /*************************************************
             * COMMANDING 1rad CLOCKWISE FROM PERTURBED POSITION
             * 1e-1rad error at trajectory startup
             *************************************************/

            _log.close();
            _log.open("testQPOases_FF_Cartesian_1rad_1draderr.m");

            desired_pose = current_pose;
            desired_pose.M.DoRotX(.1);
            get1radCircularTraj(desired_pose);
            desired_pose = trajectory->Pos(0.0);

            l_arm_task->setOrientationErrorGain(.4);
            /* setting lambda lower than this can cause tracking problems
             * along the trajectory on secondary variables (e.g. the one that
             * we want fixed at 0) */
            l_arm_task->setLambda(.3);
        }
    }

    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    std::chrono::duration<double> t_loop = std::chrono::duration<double>(dt);
    double t_compute = 0;

    double previous_norm = -1;
    double current_norm;
    double previous_error;
    if(trajType == KDL::Path::ID_LINE) {
        previous_error = desired_pose.p[0] - current_pose.p[0];
    } else {
        desired_pose.M.GetRPY(Rdes,Pdes,Ydes);
        current_pose.M.GetRPY(R,P,Y);
        previous_error = Rdes - R;
    }

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(333Hz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> t_begin;
    for (double t=0.0; t <= trajectory->Duration(); t+= t_loop.count())
    {
        t_begin = std::chrono::system_clock::now();
        Eigen::MatrixXd desired_pose_y(4,4); desired_pose_y.setZero(4,4);
        Eigen::VectorXd desired_twist_y(6); desired_twist_y.setZero(6);
        desired_pose = trajectory->Pos(t);
        desired_twist = trajectory->Vel(t);
        l_arm_task->setReference(desired_pose, desired_twist*t_loop.count());


        // initializing previous norm
        if(previous_norm < 0){

            Eigen::VectorXd twist(6);
            twist<<Eigen::Vector3d(desired_twist.vel.data), Eigen::Vector3d(desired_twist.rot.data);

            Eigen::VectorXd b = twist*t_loop.count() + l_arm_task->getLambda()*l_arm_task->getError();
            previous_norm = sqrt(b.squaredNorm());
            //previous_norm = sqrt(l_arm_task->getb().squaredNorm());
        }

        // checking variation of gain during trajectory following
        if(t>=6)
            l_arm_task->setLambda(.6);

        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        l_arm_task->update(q);
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

        current_pose_y = l_arm_task->getActualPose();
        l_arm_task->getActualPose(current_pose);


        std::chrono::duration<double> t_compute = std::chrono::system_clock::now() - t_begin;

        std::chrono::duration<double> time_for_sleep = std::chrono::duration<double>(dt) - t_compute;

        std::this_thread::sleep_for(time_for_sleep);
        t_loop = std::chrono::system_clock::now() - t_begin;

        /* first order fading filter -> to implement in Matlab
        double beta = 0.3; double G = 1-beta;
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_estimate[0] + G*(twist_measure - twist_estimate[0]);
        */
        double twist_measure;
        if(trajType == KDL::Path::ID_LINE) {
            twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop.count();
        } else {
            current_pose.M.GetRPY(R,P,Y);
            previous_pose.M.GetRPY(Rprev,Pprev,Yprev);
            twist_measure = (R - Rprev)/t_loop.count();
        }

        twist_estimate[0] = twist_measure;

        current_norm = sqrt(l_arm_task->getb().squaredNorm());

        if(trajType == KDL::Path::ID_LINE) {
            _log << t << ",\t"
                 << twist_estimate[0] << ",\t"
                 << desired_twist[0]   << ",\t"
                 << current_pose.p[0]  << ",\t"
                 << desired_pose.p[0]  << ",\t"
                 << t_compute.count()          << ",\t"
                 << t_loop.count()             << ",\t"
                 << current_norm       << ";" << std::endl;
        } else {
            desired_pose.M.GetRPY(Rdes,Pdes,Ydes);
            _log << t << ",\t"
                 << twist_estimate[0] << ",\t"
                 << desired_twist.rot.x()   << ",\t"
                 << R  << ",\t"
                 << Rdes  << ",\t"
                 << t_compute.count()          << ",\t"
                 << t_loop.count()             << ",\t"
                 << current_norm       << ";" << std::endl;
        }

        // also velocities and accelerations are available !
        previous_pose = current_pose;
        previous_twist_estimate[0] = twist_estimate[0];

        if(!hasInitialError) {
            if(trajType == KDL::Path::ID_LINE) {
                EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4) << " @t= " << t;;
                EXPECT_NEAR(sqrt(l_arm_task->getb().squaredNorm()), 0, 5e-4) << " @t= " << t;;
            } else {
                EXPECT_NEAR(R, Rdes,3e-3) << " @t= " << t;;
                EXPECT_NEAR(sqrt(l_arm_task->getb().squaredNorm()), 0, 1.5e-2) << " @t= " << t;;
            }
        } else {
            if(t<=1.3) {
                double current_error;

                if(trajType == KDL::Path::ID_LINE) {
                    current_error = desired_pose.p[0] - current_pose.p[0];

                    /* error should always decrease, or at least accept
                     * a local increment of 1e-4 */
                    EXPECT_GE(previous_error - current_error, -1e-4) << " @t= " << t;
                    EXPECT_GE(previous_norm - current_norm, -8e-4) << " @t= " << t;

                } else {
                    desired_pose.M.GetRPY(Rdes,Pdes,Ydes);
                    current_pose.M.GetRPY(R,P,Y);
                    current_error = Rdes - R;

                    /* error should always decrease, or at least accept
                     * a local increment of 1e-4 */
                    EXPECT_GE(previous_error - current_error, -3e-3) << " @t= " << t;
                    EXPECT_GE(previous_norm - current_norm, -2.4e-2) << " @t= " << t;
                }

                previous_error = current_error;
                previous_norm = current_norm;
            } else {
                if(trajType == KDL::Path::ID_LINE) {
                    EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1.5e-4) << " @t= " << t;
                    EXPECT_NEAR(sqrt(l_arm_task->getb().squaredNorm()), 0, 1.5e-3) << " @t= " << t;
                } else {
                    EXPECT_NEAR(R, Rdes,2e-3);
                    EXPECT_NEAR(sqrt(l_arm_task->getb().squaredNorm()), 0, 1e-2);
                }
            }
        }
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "%moving average filter" << std::endl;
    _log << "filt_window = 25;" << std::endl;
    _log << "a = 1; b = 1/filt_window*ones(1,filt_window);" << std::endl;
    _log << "twist_estimate = filter(b,a,pos_des_x(:,2));" << std::endl;
    _log << "plot(pos_des_x(:,1),[twist_estimate, pos_des_x(:,3)]);" << std::endl;
    _log << "legend('Actual Velocity Profile','Desired Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Actual Position','Desired Position','Location','SouthEast');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,6:7)); title('Computation time'); legend('Solve time','loop time (333Hz)');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,8)); title('Tracking Error'); legend('Cartesian 6d tracking error');" << std::endl;

}











TEST_P(testQPOases_CoMAndPosturalFF, testCoMFF)
{
    bool hasInitialError = GetParam();

    std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_floating_base.yaml";

    std::string _path_to_cfg = relative_path;

    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;


    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

Eigen::Affine3d floating_base_pose;
_model_ptr->getFloatingBasePose(floating_base_pose);
std::cout<<"floating_base_pose:\n"<<floating_base_pose.matrix()<<std::endl;


    // BOUNDS
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(q, qmax, qmin);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsVelocityLimits =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>( 0.9,3e-3,q.size());

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    bounds_list.push_back(boundsVelocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds =
            std::make_shared<OpenSoT::constraints::Aggregated>(bounds_list, q.size());

    OpenSoT::tasks::velocity::CoM::Ptr com =
            std::make_shared<OpenSoT::tasks::velocity::CoM>(q, *(_model_ptr));

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(q);

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(com);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::iHQP::Ptr sot = std::make_shared<OpenSoT::solvers::iHQP>(stack_of_tasks, bounds,1e8);



    Eigen::VectorXd dq(q.size()); dq.setZero(q.size());

    double dt=3e-3;

    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist twist_estimate, previous_twist_estimate, desired_twist;
    Eigen::Vector3d current_position_y;


    q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    com->update(q);
    postural_task->update(q);
    bounds->update(q);


    if(!hasInitialError) {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM CURRENT POSITION
         * meaning zero error at trajectory begin
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_CoM_5cmfw_noerr.m");

        current_position_y = com->getActualPosition();
        current_pose.p.x(current_position_y(0));
        current_pose.p.y(current_position_y(1));
        current_pose.p.z(current_position_y(2));
        get5cmFwdLinearTraj(current_pose);
        desired_pose = trajectory->Pos(0.0);

        com->setLambda(.6);

    } else {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM PERTURBED POSITION
         * 1cm error at trajectory startup
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_CoM_5cmfw_1cmerr.m");

        current_position_y = com->getActualPosition();
        current_pose.p.x(current_position_y(0));
        current_pose.p.y(current_position_y(1));
        current_pose.p.z(current_position_y(2));
        desired_pose = current_pose;
        desired_pose.p[0] = current_pose.p[0] + .01;
        get5cmFwdLinearTraj(desired_pose);
        desired_pose = trajectory->Pos(0.0);

        /* setting lambda lower than this can cause tracking problems
         * along the trajectory on secondary variables (e.g. the one that
         * we want fixed at 0) */
        com->setLambda(.2);
    }

    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    std::chrono::duration<double> t_loop = std::chrono::duration<double>(dt);
    double t_compute = 0;

    double previous_norm = -1;
    double current_norm;
    double previous_error = desired_pose.p[0] - current_pose.p[0];

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(333Hz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> t_begin;
    for (double t=0.0; t <= trajectory->Duration(); t+= t_loop.count())
    {
        t_begin = std::chrono::system_clock::now();
        Eigen::Vector3d desired_position_y; desired_position_y.setZero();
        Eigen::Vector3d desired_twist_y; desired_twist_y.setZero();
        desired_pose = trajectory->Pos(t);
        desired_twist = trajectory->Vel(t);
        desired_position_y(0) = desired_pose.p.x();
        desired_position_y(1) = desired_pose.p.y();
        desired_position_y(2) = desired_pose.p.z();
        desired_twist_y(0) = desired_twist.vel.x();
        desired_twist_y(1) = desired_twist.vel.y();
        desired_twist_y(2) = desired_twist.vel.z();
        com->setReference(desired_pose.p, desired_twist.vel*t_loop.count());

        // initializing previous norm
        if(previous_norm < 0)
        {
            com->update(Eigen::VectorXd(1));
            previous_norm = sqrt(com->getb().squaredNorm());
        }

        // checking variation of gain during trajectory following
        if(t>=6)
            com->setLambda(.6);

        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        com->update(q);
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

        current_position_y = com->getActualPosition();
        current_pose.p.x(current_position_y(0));
        current_pose.p.y(current_position_y(1));
        current_pose.p.z(current_position_y(2));


        std::chrono::duration<double> t_compute = std::chrono::system_clock::now() - t_begin;

        std::chrono::duration<double> time_for_sleep = std::chrono::duration<double>(dt) - t_compute;

        std::this_thread::sleep_for(time_for_sleep);
        t_loop = std::chrono::system_clock::now() - t_begin;

        /* first order fading filter -> to implement in Matlab
        double beta = 0.3; double G = 1-beta;
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_estimate[0] + G*(twist_measure - twist_estimate[0]);
        */
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop.count();
        twist_estimate[0] = twist_measure;

        current_norm = sqrt(com->getb().squaredNorm());

        _log << t << ",\t"
             << twist_estimate[0] << ",\t"
             << desired_twist[0]   << ",\t"
             << current_pose.p[0]  << ",\t"
             << desired_pose.p[0]  << ",\t"
             << t_compute.count()          << ",\t"
             << t_loop.count()             << ",\t"
             << current_norm       << ";" << std::endl;
        // also velocities and accelerations are available !
        previous_pose = current_pose;
        previous_twist_estimate[0] = twist_estimate[0];

        if(!hasInitialError) {
            EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4);
            EXPECT_NEAR(current_norm, 0, 5e-4);
        } else {
            if(t<=1.3) {
                double current_error = desired_pose.p[0] - current_pose.p[0];

                /* error should always decrease, or at least accept
                 * a local increment of 1e-4 */
                EXPECT_GE(previous_error - current_error, -1e-4) << " @t= " << t;
                EXPECT_GE(previous_norm - current_norm, -8e-4) << " @t= " << t;

                previous_error = current_error;
                previous_norm = current_norm;
            } else {

                EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4) << " @t= " << t;
                EXPECT_NEAR(current_norm, 0, 1.3e-3) << " @t= " << t;
            }
        }
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "%moving average filter" << std::endl;
    _log << "filt_window = 25;" << std::endl;
    _log << "a = 1; b = 1/filt_window*ones(1,filt_window);" << std::endl;
    _log << "twist_estimate = filter(b,a,pos_des_x(:,2));" << std::endl;
    _log << "plot(pos_des_x(:,1),[twist_estimate, pos_des_x(:,3)]);" << std::endl;
    _log << "legend('Actual Velocity Profile','Desired Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Actual Position','Desired Position','Location','SouthEast');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,6:7)); title('Computation time'); legend('Solve time','loop time (333Hz)');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,8)); title('Tracking Error'); legend('CoM 3d tracking error');" << std::endl;

}



















TEST_P(testQPOases_CoMAndPosturalFF, testPosturalFF)
{
    bool hasInitialError = GetParam();

    std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";

    std::string _path_to_cfg = relative_path;

    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;


    double j_index = _model_ptr->getDofIndex("LShLat");
    std::cout << "Applying trajectory to joint "
              << "LShLat" << std::endl;

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    q[j_index] = .0;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    // BOUNDS
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(q,qmax,qmin);

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsVelocityLimits =
            std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>( 1.5,3e-3,q.size());

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    bounds_list.push_back(boundsVelocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds =
            std::make_shared<OpenSoT::constraints::Aggregated>(bounds_list, q.size());

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task =
            std::make_shared<OpenSoT::tasks::velocity::Postural>(q);

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::iHQP::Ptr sot =
            std::make_shared<OpenSoT::solvers::iHQP>(stack_of_tasks, bounds,1e9);


    Eigen::VectorXd dq(q.size()); dq.setZero(q.size());

    double dt=3e-3;

    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist twist_estimate, previous_twist_estimate, desired_twist;

    q = getGoodInitialPosition(_model_ptr);
    q[j_index] = .0;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    postural_task->update(q);
    bounds->update(q);

    if(!hasInitialError) {
        /**************************************************
         * COMMANDING 1rad CLOCKWISE FROM CURRENT POSITION
         * meaning zero error at trajectory begin
         *************************************************/

        _log.close();
        _log.open("testQPOases_FF_Postural_1rad_noerr.m");

        current_pose.p[0] = q[j_index];
        get1radTraj(current_pose.p[0]);
        desired_pose = trajectory->Pos(0.0);

        postural_task->setLambda(.99);

    } else {
        /****************************************************
         * COMMANDING 1rad CLOCKWISE FROM PERTURBED POSITION
         * 1cm error at trajectory startup
         ***************************************************/

        _log.close();
        _log.open("testQPOases_FF_Postural_1rad_1draderr.m");

        current_pose.p[0] = q[j_index];
        desired_pose = current_pose;
        desired_pose.p[0] = current_pose.p[0] + .1;
        get1radTraj(desired_pose.p[0]);
        desired_pose = trajectory->Pos(0.0);

        /* setting lambda lower than this can cause tracking problems
         * along the trajectory on secondary variables (e.g. the one that
         * we want fixed at 0) */
        postural_task->setLambda(.8);
    }

    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    std::chrono::duration<double> t_loop = std::chrono::duration<double>(dt);
    double t_compute = 0;

    double previous_norm = -1;
    double current_norm;
    double previous_error = desired_pose.p[0] - current_pose.p[0];

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(333Hz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> t_begin;
    for (double t=0.0; t <= trajectory->Duration(); t+= t_loop.count())
    {
        t_begin = std::chrono::system_clock::now();

        Eigen::VectorXd desired_q(q.size()); desired_q.setZero(desired_q.size());
        Eigen::VectorXd desired_qdot(q.size()); desired_qdot.setZero(desired_q.size());
        desired_pose = trajectory->Pos(t);
        desired_twist = trajectory->Vel(t);
        desired_q = q;
        desired_q[j_index] = desired_pose.p[0];
        desired_qdot[j_index] = desired_twist.vel[0];
        postural_task->setReference(desired_q,desired_qdot*t_loop.count());

        // initializing previous norm
        if(previous_norm < 0)
            previous_norm = sqrt(postural_task->getb().squaredNorm());

        // checking variation of gain during trajectory following
        if(t>=6)
            postural_task->setLambda(.99);

        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot->solve(dq));

        current_pose.p[0] = q[j_index];

        q += dq;

        std::chrono::duration<double> t_compute = std::chrono::system_clock::now() - t_begin;

        std::chrono::duration<double> time_for_sleep = std::chrono::duration<double>(dt) - t_compute;

        std::this_thread::sleep_for(time_for_sleep);
        t_loop = std::chrono::system_clock::now() - t_begin;

        /* first order fading filter -> to implement in Matlab
        double beta = 0.3; double G = 1-beta;
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_estimate[0] + G*(twist_measure - twist_estimate[0]);
        */
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop.count();
        twist_estimate[0] = twist_measure;

        current_norm = sqrt(postural_task->getb().squaredNorm());

        _log << t << ",\t"
             << twist_estimate[0] << ",\t"
             << desired_twist[0]   << ",\t"
             << current_pose.p[0]  << ",\t"
             << desired_pose.p[0]  << ",\t"
             << t_compute.count()          << ",\t"
             << t_loop.count()             << ",\t"
             << current_norm       << ";" << std::endl;
        // also velocities and accelerations are available !
        previous_pose = current_pose;
        previous_twist_estimate[0] = twist_estimate[0];

        if(!hasInitialError) {
            EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],2e-3);
            EXPECT_NEAR(current_norm, 0, 3e-3);
        } else {
            if(t<=1.3) {
                double current_error = desired_pose.p[0] - current_pose.p[0];

                /* error should always decrease, or at least accept
                 * a local increment of 1e-4 */
                EXPECT_GE(previous_error - current_error, -2e-3) << " @t= " << t;
                EXPECT_GE(previous_norm - current_norm, -8e-3) << " @t= " << t;

                previous_error = current_error;
                previous_norm = current_norm;
            } else {

                EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],2e-3) << " @t= " << t;
                EXPECT_NEAR(current_norm, 0, 4e-3) << " @t= " << t;
            }
        }
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "%moving average filter" << std::endl;
    _log << "filt_window = 25;" << std::endl;
    _log << "a = 1; b = 1/filt_window*ones(1,filt_window);" << std::endl;
    _log << "twist_estimate = filter(b,a,pos_des_x(:,2));" << std::endl;
    _log << "plot(pos_des_x(:,1),[twist_estimate, pos_des_x(:,3)]);" << std::endl;
    _log << "legend('Actual Velocity Profile','Desired Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Actual Position','Desired Position','Location','SouthEast');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,6:7)); title('Computation time'); legend('Solve time','loop time (333Hz)');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,8)); title('Tracking Error'); legend('Postural 29d tracking error');" << std::endl;

}

INSTANTIATE_TEST_CASE_P(FFTests,
                        testQPOases_CartesianFF,
                        ::testing::Values(
    std::make_pair(KDL::Path::ID_LINE,false),
    std::make_pair(KDL::Path::ID_LINE, true),
    std::make_pair(KDL::Path::ID_CIRCLE, true),
    std::make_pair(KDL::Path::ID_CIRCLE, false)));

INSTANTIATE_TEST_CASE_P(FFTests,
                        testQPOases_CoMAndPosturalFF,
                        ::testing::Values(false, true));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
