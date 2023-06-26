#include <trajectory_utils/trajectory_utils.h>
#include <gtest/gtest.h>
#include <trajectory_utils/utils/ros_trj_publisher.h>
#include <tf/transform_broadcaster.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Gaze.h>
#include <ros/master.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <qpOASES/Options.hpp>
#include <XBotInterface/ModelInterface.h>
#include <sensor_msgs/JointState.h>

#include <OpenSoT/solvers/eHQP.h>

#include <XBotInterface/Logger.hpp>
#include <matlogger2/matlogger2.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

bool IS_ROSCORE_RUNNING;

XBot::MatLogger2::Ptr logger;

#define CHECK_JOINT_LIMITS false
#define CHECK_CARTESIAN_ERROR true
#define USE_WRONG_COM_REFERENCE false
#define USE_INERTIA_MATRIX false
#define USE_COM_AS_CONSTR false

namespace{

/**
 * @brief The walking_pattern_generator class generate Cartesian trajectories
 * for COM, left/right feet for a "static walk" (the CoM is always inside the
 * support polygon)
 * We consider 3 steps (hardcoded) and we always start with the right foot.
 */
class walking_pattern_generator{
public:
    walking_pattern_generator(const KDL::Frame& com_init,
                              const KDL::Frame& l_sole_init,
                              const KDL::Frame& r_sole_init):
        com_trj(0.01, "world", "com"),
        l_sole_trj(0.01, "world", "l_sole"),
        r_sole_trj(0.01, "world", "r_sole")
    {
        T_com = 3.;
        T_foot = 1.;

        step_lenght = 0.1;

        KDL::Vector plane_normal; plane_normal.Zero();
        plane_normal.y(1.0);

        KDL::Frame com_wp = com_init;

        KDL::Frame r_sole_wp;
        KDL::Frame l_sole_wp;

        std::vector<double> com_time;
        std::vector<KDL::Frame> com_waypoints;
        com_waypoints.push_back(com_init);

        //1. We assume the CoM is in the middle of the feet and it
        //moves on top of the left feet (keeping the same height),
        //left and right feet keep the same pose:
        com_wp.p.x(l_sole_init.p.x());
        com_wp.p.y(l_sole_init.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_init, r_sole_init, T_com);
        //2. Now the CoM is on the left foot, the right foot move forward
        // while the left foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_foot);

        KDL::Vector arc_center = r_sole_init.p;
        arc_center.x(arc_center.x() + step_lenght/2.);
        r_sole_trj.addArcTrj(r_sole_init, r_sole_init.M, M_PI, arc_center, plane_normal, T_foot);
        //3. CoM pass from left to right foot, feet remains in the
        // same position
        r_sole_wp = r_sole_trj.Pos(r_sole_trj.Duration());
        com_wp.p.x(r_sole_wp.p.x());
        com_wp.p.y(r_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_init, l_sole_init, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //4. Now the CoM is on the right foot, the left foot move forward
        // while the right foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_foot);

        arc_center = l_sole_init.p;
        arc_center.x(arc_center.x() + step_lenght);
        l_sole_trj.addArcTrj(l_sole_init, l_sole_init.M, M_PI, arc_center, plane_normal, T_foot);
        //5. CoM pass from right to left foot, feet remains in the
        // same position
        l_sole_wp = l_sole_trj.Pos(l_sole_trj.Duration());
        com_wp.p.x(l_sole_wp.p.x());
        com_wp.p.y(l_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //6. Now the CoM is on the left foot, the right foot move forward
        // while the left foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_foot);

        arc_center = r_sole_wp.p;
        arc_center.x(arc_center.x() + step_lenght);
        r_sole_trj.addArcTrj(r_sole_wp, r_sole_wp.M, M_PI, arc_center, plane_normal, T_foot);
        //7. CoM pass from left to right foot, feet remains in the
        // same position
        r_sole_wp = r_sole_trj.Pos(r_sole_trj.Duration());
        com_wp.p.x(r_sole_wp.p.x());
        com_wp.p.y(r_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //8. Now the CoM is on the right foot, the left foot move forward
        // while the right foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_foot);

        arc_center = l_sole_wp.p;
        arc_center.x(arc_center.x() + step_lenght/2.);
        l_sole_trj.addArcTrj(l_sole_wp, l_sole_wp.M, M_PI, arc_center, plane_normal, T_foot);
        //9. The CoM goes back in the middle of the feet. Feet remain
        // in the same position
        com_wp.p.y(0.0);
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_wp = l_sole_trj.Pos(l_sole_trj.Duration());
        l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);


        com_trj.addMinJerkTrj(com_waypoints,com_time);

    }

    /**
     * @brief T_COM trajectory time for the CoM
     */
    double T_com;
    /**
     * @brief T_foot trajectory time for the feet
     */
    double T_foot;

    /**
     * @brief step_lenght
     */
    double step_lenght;

    trajectory_utils::trajectory_generator com_trj;
    trajectory_utils::trajectory_generator l_sole_trj;
    trajectory_utils::trajectory_generator r_sole_trj;
};


class manipulation_trajectories{
public:
    manipulation_trajectories(const KDL::Frame& com_init,
                              const KDL::Frame& r_wrist_init):
        com_trj(0.01, "world", "com"),
        r_wrist_trj(0.01, "DWYTorso", "r_wrist")
    {
        T_trj = 1.;
#if USE_WRONG_COM_REFERENCE
        double h_com = 0.2;//0.1;
#else
        double h_com = 0.1;
#endif
        double arm_forward = 0.1;


        KDL::Frame com_wp = com_init;
        KDL::Frame r_wrist_wp = r_wrist_init;

        std::vector<KDL::Frame> com_waypoints;
        std::vector<KDL::Frame> r_wrist_waypoints;
        //1. CoM goes down a little
        com_waypoints.push_back(com_wp);
        com_wp.p.z(com_wp.p.z()-h_com);
        com_waypoints.push_back(com_wp);

        r_wrist_waypoints.push_back(r_wrist_wp);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //2. right arm move forward
        com_waypoints.push_back(com_wp);

        r_wrist_wp.p.x(r_wrist_wp.p.x()+arm_forward);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //3. right arm move backward
        com_waypoints.push_back(com_wp);

        r_wrist_wp.p.x(r_wrist_wp.p.x()-arm_forward);
        r_wrist_waypoints.push_back(r_wrist_wp);
        //4. com goes back
        com_wp.p.z(com_wp.p.z()+h_com);
        com_waypoints.push_back(com_wp);
        r_wrist_waypoints.push_back(r_wrist_wp);

        com_trj.addMinJerkTrj(com_waypoints, T_trj);
        r_wrist_trj.addMinJerkTrj(r_wrist_waypoints, T_trj);
    }

    trajectory_utils::trajectory_generator com_trj;
    trajectory_utils::trajectory_generator r_wrist_trj;
    double T_trj;
};

class theWalkingStack
{
public:
    void printAb(OpenSoT::Task<Eigen::MatrixXd,Eigen::VectorXd>& task)
    {
        std::cout<<"Task: "<<task.getTaskID()<<std::endl;
        std::cout<<"A: "<<task.getA()<<std::endl;
        std::cout<<"size of A: "<<task.getA().rows()<<"x"<<task.getA().cols()<<std::endl;
        std::cout<<"b: "<<task.getb()<<std::endl;
        std::cout<<std::endl;
    }

    theWalkingStack(XBot::ModelInterface& _model,
                    const Eigen::VectorXd& q):
        model_ref(_model),
        I(q.size(), q.size())
    {

        I.setIdentity(q.size(), q.size());

        l_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_wrist", q,
            model_ref, "l_wrist","DWYTorso"));
        r_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_wrist", q,
            model_ref, "r_wrist","DWYTorso"));
        l_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_sole", q,
            model_ref, "l_sole","world"));
        r_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_sole", q,
            model_ref, "r_sole","world"));
        com.reset(new OpenSoT::tasks::velocity::CoM(q, model_ref));
        gaze.reset(new OpenSoT::tasks::velocity::Gaze("Cartesian::Gaze",q,
                        model_ref, "world"));
        std::vector<bool> ajm = gaze->getActiveJointsMask();
        for(unsigned int i = 0; i < ajm.size(); ++i)
            ajm[i] = false;
        ajm[model_ref.getDofIndex("WaistYaw")] = true;
        ajm[model_ref.getDofIndex("WaistSag")] = true;
        ajm[model_ref.getDofIndex("WaistLat")] = true;
        gaze->setActiveJointsMask(ajm);


        postural.reset(new OpenSoT::tasks::velocity::Postural(q));

        Eigen::VectorXd qmin, qmax;
        model_ref.getJointLimits(qmin, qmax);
        joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(q, qmax, qmin));

        vel_limits.reset(new OpenSoT::constraints::velocity::VelocityLimits(2.*M_PI, 0.01, q.size()));





            auto_stack = (l_sole + r_sole)/
                    (com)/
                    (l_wrist + r_wrist + gaze)/
                    (postural)<<joint_limits<<vel_limits;


        auto_stack->update(q);

        solver.reset(new OpenSoT::solvers::eHQP(auto_stack->getStack()));


    }

    void setInertiaPostureTask()
    {
        Eigen::MatrixXd M;
        model_ref.getInertiaMatrix(M);

        postural->setWeight(M+I);
        postural->setLambda(0.);
    }

    void update(const Eigen::VectorXd& q)
    {
#if USE_INERTIA_MATRIX
        setInertiaPostureTask();
#endif
        auto_stack->update(q);

    }

    bool solve(Eigen::VectorXd& dq)
    {
        return solver->solve(dq);
    }

    OpenSoT::tasks::velocity::Cartesian::Ptr l_wrist;
    OpenSoT::tasks::velocity::Cartesian::Ptr r_wrist;
    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole;
    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole;
    OpenSoT::tasks::velocity::CoM::Ptr    com;
    OpenSoT::tasks::velocity::Gaze::Ptr gaze;
    OpenSoT::tasks::velocity::Postural::Ptr postural;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_limits;

    OpenSoT::AutoStack::Ptr auto_stack;

    XBot::ModelInterface& model_ref;

    OpenSoT::solvers::eHQP::Ptr solver;


    Eigen::MatrixXd I;



};

class testStaticWalkFloatingBase: public ::testing::Test{
public:
    void printKDLFrame(const KDL::Frame& F)
    {
        std::cout<<"    pose: ["<<F.p.x()<<", "<<F.p.y()<<", "<<F.p.z()<<"]"<<std::endl;
        double qx, qy,qz,qw;
        F.M.GetQuaternion(qx,qy,qz,qw);
        std::cout<<"    quat: ["<<qx<<", "<<qy<<", "<<qz<<", "<<qw<<"]"<<std::endl;
    }

    testStaticWalkFloatingBase()
    {

        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

        int number_of_dofs = _model_ptr->getJointNum();
        std::cout<<"#DoFs: "<<number_of_dofs<<std::endl;

        _q.resize(number_of_dofs);
        _q.setZero(number_of_dofs);

        _model_ptr->setJointPosition(_q);
        _model_ptr->update();

        KDL::Frame world_T_bl;
        _model_ptr->getPose("Waist",world_T_bl);

        std::cout<<"world_T_bl:"<<std::endl;
        printKDLFrame(world_T_bl);





        if(IS_ROSCORE_RUNNING){

            _n.reset(new ros::NodeHandle());
            world_broadcaster.reset(new tf::TransformBroadcaster());

        }
    }

    ~testStaticWalkFloatingBase(){}
    virtual void SetUp(){}
    virtual void TearDown(){}
    void initTrj(const KDL::Frame& com_init,
            const KDL::Frame& l_sole_init,
            const KDL::Frame& r_sole_init)
    {
        walk_trj.reset(new walking_pattern_generator(com_init,
                                                     l_sole_init,
                                                     r_sole_init));
    }


    void initManipTrj(const KDL::Frame& com_init,
            const KDL::Frame& r_wrist_init)
    {
        manip_trj.reset(new manipulation_trajectories(com_init,r_wrist_init));
    }

    void initTrjPublisher()
    {
        if(IS_ROSCORE_RUNNING){
            com_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("com_trj"));
            com_trj_pub->setTrj(walk_trj->com_trj.getTrajectory(), "world", "com");

            l_sole_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("l_sole_trj"));
            l_sole_trj_pub->setTrj(walk_trj->l_sole_trj.getTrajectory(), "world", "l_sole");

            r_sole_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("r_sole_trj"));
            r_sole_trj_pub->setTrj(walk_trj->r_sole_trj.getTrajectory(), "world", "r_sole");

            visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/com_feet_visual_marker"));


            joint_state_pub = _n->advertise<sensor_msgs::JointState>("joint_states", 1000);
        }
    }

    void initManipTrjPublisher()
    {
        if(IS_ROSCORE_RUNNING)
        {
            visual_tools->deleteAllMarkers();


            com_trj_pub->deleteAllMarkersAndTrj();
            com_trj_pub->setTrj(manip_trj->com_trj.getTrajectory(), "world", "com");

            l_sole_trj_pub->deleteAllMarkersAndTrj();
            r_sole_trj_pub->deleteAllMarkersAndTrj();

            r_wrist_trj_pub.reset(
                        new trajectory_utils::trajectory_publisher("r_wrist_trj"));
            r_wrist_trj_pub->setTrj(manip_trj->r_wrist_trj.getTrajectory(), "DWYTorso", "r_wrist");
        }
    }


    void publishCoMAndFeet(const KDL::Frame& com,
                           const KDL::Frame& l_foot,
                           const KDL::Frame& r_foot)
    {
        if(IS_ROSCORE_RUNNING)
        {
            visual_tools->deleteAllMarkers();

            geometry_msgs::PoseStamped _com;
            _com.pose.position.x = com.p.x();
            _com.pose.position.y = com.p.y();
            _com.pose.position.z = com.p.z();
            double x,y,z,w;
            com.M.GetQuaternion(x,y,z,w);
            _com.pose.orientation.x = x;
            _com.pose.orientation.y = y;
            _com.pose.orientation.z = z;
            _com.pose.orientation.w = w;

            Eigen::Affine3d _l_foot;
            _l_foot(0,3) = l_foot.p.x()+0.02;
            _l_foot(1,3) = l_foot.p.y();
            _l_foot(2,3) = l_foot.p.z();
            for(unsigned int i = 0; i < 3; ++i)
                for(unsigned j = 0; j < 3; ++j)
                    _l_foot(i,j) = l_foot.M(i,j);


            Eigen::Affine3d _r_foot;
            _r_foot(0,3) = r_foot.p.x()+0.02;
            _r_foot(1,3) = r_foot.p.y();
            _r_foot(2,3) = r_foot.p.z();
            for(unsigned int i = 0; i < 3; ++i)
                for(unsigned j = 0; j < 3; ++j)
                    _r_foot(i,j) = r_foot.M(i,j);

            _com.header.frame_id="world";
            _com.header.stamp = ros::Time::now();


            geometry_msgs::Vector3 scale;
            scale.x = .02; scale.y = .02; scale.z = .02;
            visual_tools->publishSphere(_com,rviz_visual_tools::colors::GREEN, scale);


            Eigen::Isometry3d tmp, tmp2;
            tmp.translation() = _l_foot.translation();
            tmp.linear() = _l_foot.rotation();
            tmp2.translation() = _r_foot.translation();
            tmp2.linear() = _r_foot.rotation();
            visual_tools->publishWireframeRectangle(tmp, 0.05, 0.1);
            visual_tools->publishWireframeRectangle(tmp2, 0.05, 0.1);



        }
    }

    void publishRobotState()
    {
        if(IS_ROSCORE_RUNNING)
        {
            sensor_msgs::JointState joint_msg;
            joint_msg.name = _model_ptr->getEnabledJointNames();


            for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
                joint_msg.position.push_back(0.0);

            for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
            {
                int id = _model_ptr->getDofIndex(joint_msg.name[i]);
                joint_msg.position[id] = _q[i];
            }

            joint_msg.header.stamp = ros::Time::now();


            KDL::Frame world_T_bl;
            _model_ptr->getPose("Waist",world_T_bl);

            tf::Transform anchor_T_world;
            anchor_T_world.setOrigin(tf::Vector3(world_T_bl.p.x(),
                world_T_bl.p.y(), world_T_bl.p.z()));
            double x,y,z,w;
            world_T_bl.M.GetQuaternion(x,y,z,w);
            anchor_T_world.setRotation(tf::Quaternion(x,y,z,w));

            world_broadcaster->sendTransform(tf::StampedTransform(
                anchor_T_world.inverse(), joint_msg.header.stamp,
                "Waist", "world"));


            joint_state_pub.publish(joint_msg);
        }

    }

    void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q)
    {
        this->_model_ptr->setFloatingBasePose(l_sole_T_Waist);

        this->_model_ptr->getJointPosition(q);
    }

    void update(Eigen::VectorXd& q)
    {
        this->_model_ptr->setJointPosition(q);
        this->_model_ptr->update();
    }


    std::shared_ptr<manipulation_trajectories> manip_trj;
    std::shared_ptr<walking_pattern_generator> walk_trj;
    std::shared_ptr<trajectory_utils::trajectory_publisher> com_trj_pub;
    std::shared_ptr<trajectory_utils::trajectory_publisher> l_sole_trj_pub;
    std::shared_ptr<trajectory_utils::trajectory_publisher> r_sole_trj_pub;
    std::shared_ptr<trajectory_utils::trajectory_publisher> r_wrist_trj_pub;

    ros::Publisher joint_state_pub;
    std::shared_ptr<tf::TransformBroadcaster> world_broadcaster;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools;

    XBot::ModelInterface::Ptr _model_ptr;
    Eigen::VectorXd _q;

    std::shared_ptr<ros::NodeHandle> _n;

    void setGoodInitialPosition() {
        _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    }

};

static inline void KDLFramesAreEqual(const KDL::Frame& a, const KDL::Frame& b,
                                     const double near = 1e-10)
{
    EXPECT_NEAR(a.p.x(), b.p.x(), near);
    EXPECT_NEAR(a.p.y(), b.p.y(), near);
    EXPECT_NEAR(a.p.z(), b.p.z(), near);

    double x,y,z,w; a.M.GetQuaternion(x,y,z,w);
    double xx,yy,zz,ww; b.M.GetQuaternion(xx,yy,zz,ww);

    EXPECT_NEAR(x,xx, near);
    EXPECT_NEAR(y,yy, near);
    EXPECT_NEAR(z,zz, near);
    EXPECT_NEAR(w,ww, near);
}

TEST_F(testStaticWalkFloatingBase, testStaticWalkFloatingBase_)
{
    this->setGoodInitialPosition();

    this->_model_ptr->setJointPosition(this->_q);
    this->_model_ptr->update();


    //Update world according this new configuration:
    KDL::Frame l_sole_T_Waist;
    this->_model_ptr->getPose("Waist", "l_sole", l_sole_T_Waist);
    std::cout<<"l_sole_T_Waist:"<<std::endl;
    this->printKDLFrame(l_sole_T_Waist);

    l_sole_T_Waist.p.x(0.0);
    l_sole_T_Waist.p.y(0.0);

    this->setWorld(l_sole_T_Waist, this->_q);
    this->_model_ptr->setJointPosition(this->_q);
    this->_model_ptr->update();


    KDL::Frame world_T_bl;
    _model_ptr->getPose("Waist",world_T_bl);

    std::cout<<"world_T_bl:"<<std::endl;
    printKDLFrame(world_T_bl);
    //

    //Walking
    KDL::Vector CoM;
    this->_model_ptr->getCOM(CoM);
    KDL::Frame CoM_frame; CoM_frame.p = CoM;
    std::cout<<"CoM init:"<<std::endl;
    this->printKDLFrame(CoM_frame);

    KDL::Frame l_foot_init;
    this->_model_ptr->getPose("l_sole", l_foot_init);
    std::cout<<"l_sole init:"<<std::endl;
    this->printKDLFrame(l_foot_init);

    KDL::Frame r_foot_init;
    this->_model_ptr->getPose("r_sole", r_foot_init);
    std::cout<<"r_sole init:"<<std::endl;
    this->printKDLFrame(r_foot_init);

    this->initTrj(CoM_frame, l_foot_init, r_foot_init);
    this->initTrjPublisher();

    //Initialize Walking Stack
    theWalkingStack ws(*(this->_model_ptr.get()), this->_q);

    double t = 0.;
    Eigen::VectorXd dq(this->_q.size()); dq.setZero(dq.size());
    std::vector<double> loop_time;
    for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
    {
        //log
//        logger->add("q", this->_q);
//        logger->add("dq", dq);
        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);
//        logger->add("qmin", qmin);
//        logger->add("qmax", qmax);
        //


        KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
        KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
        KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.l_sole->setReference(l_sole_d);
        ws.r_sole->setReference(r_sole_d);

        ws.update(this->_q);

        uint tic = 0.0;
        if(IS_ROSCORE_RUNNING)
            tic = ros::Time::now().nsec;

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero(dq.size());}
        this->_q += dq;

        uint toc = 0.0;
        if(IS_ROSCORE_RUNNING)
            toc = ros::Time::now().nsec;

        this->update(this->_q);

#if CHECK_CARTESIAN_ERROR
        KDL::Frame tmp; KDL::Vector tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        tmp.p = tmp_vector;
        KDLFramesAreEqual(com_d, tmp, 1e-3);
        KDL::Frame l_sole;
        this->_model_ptr->getPose("l_sole", l_sole);
        KDLFramesAreEqual(l_sole_d, l_sole,1e-3);
        KDL::Frame r_sole;
        this->_model_ptr->getPose("r_sole", r_sole);
        KDLFramesAreEqual(r_sole_d, r_sole,1e-3);
#endif

#if CHECK_JOINT_LIMITS
        for(unsigned int i = 0; i < this->_q.size(); ++i){
            EXPECT_LE(this->_q[i]-qmax[i], 1e-10)<<"i: "<<i<<"\n qmax: "<<qmax[i]<<"\n q: "<<this->_q[i]<<std::endl;
            EXPECT_LE(qmin[i]-this->_q[i], 1e-10)<<"i: "<<i<<"\n qmin: "<<qmin[i]<<"\n q: "<<this->_q[i]<<std::endl;;}
#endif


        if(IS_ROSCORE_RUNNING){
            loop_time.push_back((toc-tic)/1e6);

            this->com_trj_pub->publish();
            this->l_sole_trj_pub->publish();
            this->r_sole_trj_pub->publish();}

        this->publishCoMAndFeet(com_d,l_sole_d,r_sole_d);
        this->publishRobotState();

        if(IS_ROSCORE_RUNNING)
            ros::spinOnce();

        t+=0.01;
        usleep(10000);
    }




    //Manipulation

    this->_model_ptr->getCOM(CoM);
    CoM_frame; CoM_frame.p = CoM;

    KDL::Frame r_wrist_init;
    this->_model_ptr->getPose("r_wrist","DWYTorso",r_wrist_init);
    std::cout<<"r_wrist init:"<<std::endl;
    this->printKDLFrame(r_wrist_init);

    this->initManipTrj(CoM_frame,  r_wrist_init);
    this->initManipTrjPublisher();


    dq.setZero(dq.size());
    t = 0.0;
    std::cout<<"Starting whole-body manipulation"<<std::endl;
    for(unsigned int i = 0; i < int(this->manip_trj->com_trj.Duration()) * 100; ++i)
    {
        //log
        logger->add("Eigq", this->_q);
        logger->add("Eigdq", dq);
        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);
        logger->add("Eigqmin", qmin);
        logger->add("Eigqmax", qmax);
        //



        KDL::Frame com_d = this->manip_trj->com_trj.Pos(t);
        KDL::Frame r_wrist_d = this->manip_trj->r_wrist_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.r_wrist->setReference(r_wrist_d);

        ws.update(this->_q);

        uint tic = 0.0;
        if(IS_ROSCORE_RUNNING)
            tic = ros::Time::now().nsec;

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero(dq.size());}
        this->_q += dq;

        uint toc = 0.0;
        if(IS_ROSCORE_RUNNING)
            toc = ros::Time::now().nsec;

        this->update(this->_q);
        KDL::Frame tmp; KDL::Vector tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        tmp.p = tmp_vector;
#if CHECK_CARTESIAN_ERROR
        KDLFramesAreEqual(com_d, tmp, 1e-3);
        KDL::Frame r_wrist;
        this->_model_ptr->getPose("r_wrist","DWYTorso",r_wrist);
        KDLFramesAreEqual(r_wrist_d, r_wrist, 1e-3);
#endif
        //log
        Eigen::VectorXd com(3); com[0] = tmp.p.x(); com[1] = tmp.p.y(); com[2] = tmp.p.z();
        logger->add("EigCoM", com);
        com[0] = com_d.p.x(); com[1] = com_d.p.y(); com[2] = com_d.p.z();
        logger->add("EigCoMd", com);
        //


#if CHECK_JOINT_LIMITS
        for(unsigned int i = 0; i < this->_q.size(); ++i){
            EXPECT_LE(this->_q[i]-qmax[i], 1e-10)<<"i: "<<i<<"\n qmax: "<<qmax[i]<<"\n q: "<<this->_q[i]<<std::endl;
            EXPECT_LE(qmin[i]-this->_q[i], 1e-10)<<"i: "<<i<<"\n qmin: "<<qmin[i]<<"\n q: "<<this->_q[i]<<std::endl;;}
#endif

        if(IS_ROSCORE_RUNNING){
            loop_time.push_back((toc-tic)/1e6);

            this->com_trj_pub->publish();
            this->r_wrist_trj_pub->publish(true);}

        this->publishRobotState();

        if(IS_ROSCORE_RUNNING)
            ros::spinOnce();


        t+=0.01;
        usleep(10000);
    }


    //Walking
    t = 0.0;
    dq.setZero(dq.size());
    this->update(this->_q);

    this->_model_ptr->getCOM(CoM);
    CoM_frame.p = CoM;

    this->_model_ptr->getPose("l_sole", l_foot_init);

    this->_model_ptr->getPose("r_sole", r_foot_init);

    this->initTrj(CoM_frame, l_foot_init, r_foot_init);
    this->initTrjPublisher();

    for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
    {
        //log
//        logger->add("q", this->_q);
//        logger->add("dq", dq);
        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);
//        logger->add("qmin", qmin);
//        logger->add("qmax", qmax);
        //


        KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
        KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
        KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.l_sole->setReference(l_sole_d);
        ws.r_sole->setReference(r_sole_d);

        ws.update(this->_q);

        uint tic = 0.0;
        if(IS_ROSCORE_RUNNING)
            tic = ros::Time::now().nsec;

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero(dq.size());}
        this->_q += dq;

        uint toc = 0.0;
        if(IS_ROSCORE_RUNNING)
            toc = ros::Time::now().nsec;

        this->update(this->_q);

#if CHECK_CARTESIAN_ERROR
        KDL::Frame tmp; KDL::Vector tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        tmp.p = tmp_vector;
        KDLFramesAreEqual(com_d, tmp, 1e-3);
        KDL::Frame l_sole;
        this->_model_ptr->getPose("l_sole", l_sole);
        KDLFramesAreEqual(l_sole_d, l_sole,1e-3);
        KDL::Frame r_sole;
        this->_model_ptr->getPose("r_sole", r_sole);
        KDLFramesAreEqual(r_sole_d, r_sole,1e-3);
#endif

#if CHECK_JOINT_LIMITS
        for(unsigned int i = 0; i < this->_q.size(); ++i){
            EXPECT_LE(this->_q[i]-qmax[i], 1e-10)<<"i: "<<i<<"\n qmax: "<<qmax[i]<<"\n q: "<<this->_q[i]<<std::endl;
            EXPECT_LE(qmin[i]-this->_q[i], 1e-10)<<"i: "<<i<<"\n qmin: "<<qmin[i]<<"\n q: "<<this->_q[i]<<std::endl;;}
#endif

        if(IS_ROSCORE_RUNNING){
            loop_time.push_back((toc-tic)/1e6);

            this->com_trj_pub->publish();
            this->l_sole_trj_pub->publish();
            this->r_sole_trj_pub->publish();}

        this->publishCoMAndFeet(com_d,l_sole_d,r_sole_d);
        this->publishRobotState();

        if(IS_ROSCORE_RUNNING)
            ros::spinOnce();

        t+=0.01;
        usleep(10000);
    }



    if(IS_ROSCORE_RUNNING){
        double acc = 0.;
        for(unsigned int i = 0; i < loop_time.size(); ++i)
            acc += loop_time[i];
        std::cout<<"Medium time per solve: "<<acc/double(loop_time.size())<<" ms"<<std::endl;
    }



}

}

XBot::MatLogger2::Ptr getLogger(const std::string& name)
{
    XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger(name); // date-time automatically appended
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    return logger;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testStaticWalkFloatingBaseFloatingBase_node");
  logger = getLogger("EigenSVD_StaticWalk_FloatingBase");
  IS_ROSCORE_RUNNING = ros::master::check();
  ::testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  logger.reset();
  return ret;
}
