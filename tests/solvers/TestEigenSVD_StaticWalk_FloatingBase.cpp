#include "trajectory_utils.h"
#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Gaze.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <qpOASES/Options.hpp>
#include <xbot2_interface/xbotinterface2.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <OpenSoT/solvers/eHQP.h>

#include <xbot2_interface/logger.h>
#include <matlogger2/matlogger2.h>
#include "../common.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


bool ENABLE_ROS = false;


#define USE_INERTIA_MATRIX false

namespace{


KDL::Frame poseEigenToKDL(const Eigen::Affine3d& T_Eigen)
{
    KDL::Frame T_KDL;
    tf2::transformEigenToKDL(T_Eigen, T_KDL);
    return T_KDL;
}

KDL::Vector vectorEigenToKDL(const Eigen::Vector3d& T)
{
    KDL::Vector tmp;
    tmp.x(T[0]);
    tmp.y(T[1]);
    tmp.z(T[2]);
    return tmp;
}


/**
 * @brief The walking_pattern_generator class generate Cartesian trajectories
 * for COM, left/right feet for a "static walk" (the CoM is always inside the
 * support polygon)
 * We consider 3 steps (hardcoded) and we always start with the right foot.
 */
class walking_pattern_generator{
public:
    walking_pattern_generator(const Eigen::Affine3d& com_init,
                              const Eigen::Affine3d& l_sole_init,
                              const Eigen::Affine3d& r_sole_init):
        com_trj(0.01, "world", "com"),
        l_sole_trj(0.01, "world", "l_sole"),
        r_sole_trj(0.01, "world", "r_sole")
    {
        T_com = 3.;
        T_foot = 1.;

        step_lenght = 0.1;

        KDL::Vector plane_normal; plane_normal.Zero();
        plane_normal.y(1.0);

        KDL::Frame com_wp = poseEigenToKDL(com_init);

        KDL::Frame r_sole_wp;
        KDL::Frame l_sole_wp;

        std::vector<double> com_time;
        std::vector<KDL::Frame> com_waypoints;
        com_waypoints.push_back(com_wp);

        //1. We assume the CoM is in the middle of the feet and it
        //moves on top of the left feet (keeping the same height),
        //left and right feet keep the same pose:
        com_wp.p.x(poseEigenToKDL(l_sole_init).p.x());
        com_wp.p.y(poseEigenToKDL(l_sole_init).p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(poseEigenToKDL(l_sole_init), poseEigenToKDL(l_sole_init), T_com);
        r_sole_trj.addMinJerkTrj(poseEigenToKDL(r_sole_init), poseEigenToKDL(r_sole_init), T_com);
        //2. Now the CoM is on the left foot, the right foot move forward
        // while the left foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        l_sole_trj.addMinJerkTrj(poseEigenToKDL(l_sole_init), poseEigenToKDL(l_sole_init), T_foot);

        KDL::Vector arc_center = poseEigenToKDL(r_sole_init).p;
        arc_center.x(arc_center.x() + step_lenght/2.);
        r_sole_trj.addArcTrj(poseEigenToKDL(r_sole_init), poseEigenToKDL(r_sole_init).M, M_PI, arc_center, plane_normal, T_foot);
        //3. CoM pass from left to right foot, feet remains in the
        // same position
        r_sole_wp = r_sole_trj.Pos(r_sole_trj.Duration());
        com_wp.p.x(r_sole_wp.p.x());
        com_wp.p.y(r_sole_wp.p.y());
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_com);

        l_sole_trj.addMinJerkTrj(poseEigenToKDL(l_sole_init), poseEigenToKDL(l_sole_init), T_com);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);
        //4. Now the CoM is on the right foot, the left foot move forward
        // while the right foot keep the position:
        com_waypoints.push_back(com_wp);
        com_time.push_back(T_foot);
        r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_foot);

        arc_center = poseEigenToKDL(l_sole_init).p;
        arc_center.x(arc_center.x() + step_lenght);
        l_sole_trj.addArcTrj(poseEigenToKDL(l_sole_init), poseEigenToKDL(l_sole_init).M, M_PI, arc_center, plane_normal, T_foot);
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
    manipulation_trajectories(const Eigen::Affine3d& com_init,
                              const Eigen::Affine3d& r_wrist_init):
        com_trj(0.01, "world", "com"),
        r_wrist_trj(0.01, "DWYTorso", "r_wrist")
    {
        T_trj = 1.;
        double h_com = 0.1;
        double arm_forward = 0.1;


        KDL::Frame com_wp = poseEigenToKDL(com_init);
        KDL::Frame r_wrist_wp = poseEigenToKDL(r_wrist_init);

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

    theWalkingStack(XBot::ModelInterface& _model):
        model_ref(_model),
        I(_model.getNv(), _model.getNv())
    {

        I.setIdentity();

        l_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_wrist",
            model_ref, "l_wrist","DWYTorso"));
        r_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_wrist",
            model_ref, "r_wrist","DWYTorso"));
        l_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_sole",
            model_ref, "l_sole","world"));
        r_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_sole",
            model_ref, "r_sole","world"));
        com.reset(new OpenSoT::tasks::velocity::CoM(model_ref));
        gaze.reset(new OpenSoT::tasks::velocity::Gaze("Cartesian::Gaze",
                        model_ref, "world"));
        std::vector<bool> ajm = gaze->getActiveJointsMask();
        for(unsigned int i = 0; i < ajm.size(); ++i)
            ajm[i] = false;
        ajm[model_ref.getDofIndex("WaistYaw")] = true;
        ajm[model_ref.getDofIndex("WaistSag")] = true;
        ajm[model_ref.getDofIndex("WaistLat")] = true;
        gaze->setActiveJointsMask(ajm);


        postural.reset(new OpenSoT::tasks::velocity::Postural(model_ref));

        Eigen::VectorXd qmin, qmax;
        model_ref.getJointLimits(qmin, qmax);
        joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(model_ref, qmax, qmin));

        vel_limits.reset(new OpenSoT::constraints::velocity::VelocityLimits(model_ref, 2.*M_PI, 0.01));





            auto_stack = (l_sole + r_sole)/
                    (com)/
                    (l_wrist + r_wrist + gaze)/
                    (postural)<<joint_limits<<vel_limits;


        auto_stack->update();

        solver.reset(new OpenSoT::solvers::eHQP(auto_stack->getStack()));


    }

    void setInertiaPostureTask()
    {
        Eigen::MatrixXd M;
        model_ref.computeInertiaMatrix(M);

        postural->setWeight(M+I);
        postural->setLambda(0.);
    }

    void update()
    {
#if USE_INERTIA_MATRIX
        setInertiaPostureTask();
#endif
        auto_stack->update();

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

class ros2_node: public rclcpp::Node
{
public:
    ros2_node():
        Node("ros2_node")
    {
        world_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);
    }
    std::unique_ptr<tf2_ros::TransformBroadcaster> world_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
};

class testStaticWalkFloatingBase: public TestBase
{
public:

    testStaticWalkFloatingBase() : TestBase("coman_floating_base")
    {


        int number_of_dofs = _model_ptr->getNq();
        std::cout<<"#DoFs: "<<number_of_dofs<<std::endl;

        _q = _model_ptr->getNeutralQ();

        _model_ptr->setJointPosition(_q);
        _model_ptr->update();

        Eigen::Affine3d world_T_bl;
        _model_ptr->getPose("Waist",world_T_bl);

        std::cout<<"world_T_bl:"<<std::endl;
        std::cout<<world_T_bl.matrix()<<std::endl;


        if(ENABLE_ROS){
            _n.reset(new ros2_node());
        }
    }

    ~testStaticWalkFloatingBase(){}
    virtual void SetUp(){}
    virtual void TearDown(){}
    void initTrj(const Eigen::Affine3d& com_init,
            const Eigen::Affine3d& l_sole_init,
            const Eigen::Affine3d& r_sole_init)
    {
        walk_trj.reset(new walking_pattern_generator(com_init,
                                                     l_sole_init,
                                                     r_sole_init));
    }

    void initManipTrj(const Eigen::Affine3d& com_init,
                      const Eigen::Affine3d& r_wrist_init)
    {
        manip_trj.reset(new manipulation_trajectories(com_init,r_wrist_init));
    }

    void publishRobotState()
    {
        if(ENABLE_ROS)
        {
            sensor_msgs::msg::JointState joint_msg;
            for(unsigned int i = 1; i < _model_ptr->getJointNames().size(); ++i)
            {
                joint_msg.name.push_back(_model_ptr->getJointNames()[i]);
                joint_msg.position.push_back(_q[_model_ptr->getQIndex(_model_ptr->getJointNames()[i])]);
            }

            joint_msg.header.stamp = rclcpp::Clock().now();


            Eigen::Affine3d world_T_bl;
            _model_ptr->getPose("Waist",world_T_bl);

            Eigen::Affine3d bl_T_world = world_T_bl.inverse();


            geometry_msgs::msg::TransformStamped anchor_T_world = tf2::eigenToTransform(bl_T_world);
            anchor_T_world.header.frame_id = "Waist";
            anchor_T_world.child_frame_id = "world";
            anchor_T_world.header.stamp = joint_msg.header.stamp;

            _n->joint_state_pub->publish(joint_msg);
            _n->world_broadcaster->sendTransform(anchor_T_world);
        }
    }

    void setWorld(const Eigen::Affine3d& l_sole_T_Waist, Eigen::VectorXd& q)
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

    Eigen::VectorXd _q;

    std::shared_ptr<ros2_node> _n;

    void setGoodInitialPosition() {
        _q[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getQIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model_ptr->getQIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model_ptr->getQIndex("RElbj")] = -80.0*M_PI/180.0;

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

static inline void KDLVectorAreEqual(const KDL::Vector& a, const KDL::Vector& b,
                                     const double near = 1e-10)
{
    EXPECT_NEAR(a.x(), b.x(), near);
    EXPECT_NEAR(a.y(), b.y(), near);
    EXPECT_NEAR(a.z(), b.z(), near);

}

TEST_F(testStaticWalkFloatingBase, testStaticWalkFloatingBase_)
{
    this->setGoodInitialPosition();

    this->_model_ptr->setJointPosition(this->_q);
    this->_model_ptr->update();


    //Update world according this new configuration:
    Eigen::Affine3d l_sole_T_Waist;
    this->_model_ptr->getPose("Waist", "l_sole", l_sole_T_Waist);
    std::cout<<"l_sole_T_Waist:"<<std::endl;
    std::cout<<l_sole_T_Waist.matrix()<<std::endl;

    l_sole_T_Waist.translation()[0] = 0.;
    l_sole_T_Waist.translation()[1] = 0.;

    this->setWorld(l_sole_T_Waist, this->_q);
    this->_model_ptr->setJointPosition(this->_q);
    this->_model_ptr->update();


    Eigen::Affine3d world_T_bl;
    _model_ptr->getPose("Waist",world_T_bl);

    std::cout<<"world_T_bl:"<<std::endl;
    std::cout<<world_T_bl.matrix()<<std::endl;
    //

    //Walking
    Eigen::Vector3d CoM;
    this->_model_ptr->getCOM(CoM);
    Eigen::Affine3d CoM_frame;
    CoM_frame.translation() = CoM;
    std::cout<<"CoM init:"<<std::endl;
    std::cout<<CoM_frame.matrix()<<std::endl;

    Eigen::Affine3d l_foot_init;
    this->_model_ptr->getPose("l_sole", l_foot_init);
    std::cout<<"l_sole init:"<<std::endl;
    std::cout<<l_foot_init.matrix()<<std::endl;

    Eigen::Affine3d r_foot_init;
    this->_model_ptr->getPose("r_sole", r_foot_init);
    std::cout<<"r_sole init:"<<std::endl;
    std::cout<<r_foot_init.matrix()<<std::endl;

    this->initTrj(CoM_frame, l_foot_init, r_foot_init);

    //Initialize Walking Stack
    theWalkingStack ws(*(this->_model_ptr.get()));

    rclcpp::executors::SingleThreadedExecutor exec;

    double t = 0.;
    Eigen::VectorXd dq(this->_model_ptr->getNv()); dq.setZero();
    std::vector<double> loop_time;
    for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
    {

        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);


        KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
        KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
        KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.l_sole->setReference(l_sole_d);
        ws.r_sole->setReference(r_sole_d);

        ws.update();

        int64_t tic = 0.0;
        if(ENABLE_ROS)
            tic = rclcpp::Clock().now().nanoseconds();

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero();}
        this->_q = this->_model_ptr->sum(this->_q, dq);

        int64_t toc = 0.0;
        if(ENABLE_ROS)
            toc = rclcpp::Clock().now().nanoseconds();

        this->update(this->_q);

        Eigen::Vector3d tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        KDLVectorAreEqual(com_d.p, vectorEigenToKDL(tmp_vector), 1e-3);
        Eigen::Affine3d l_sole;
        this->_model_ptr->getPose("l_sole", l_sole);
        KDLFramesAreEqual(l_sole_d, poseEigenToKDL(l_sole),1e-3);
        Eigen::Affine3d r_sole;
        this->_model_ptr->getPose("r_sole", r_sole);
        KDLFramesAreEqual(r_sole_d, poseEigenToKDL(r_sole), 1e-3);



        if(ENABLE_ROS)
            loop_time.push_back((toc-tic)/1e6);


        this->publishRobotState();

        //if(ENABLE_ROS)
        //    exec.spin_node_once(_n);

        t+=0.01;
        usleep(10000);
    }




    //Manipulation

    this->_model_ptr->getCOM(CoM);
    CoM_frame.translation() = CoM;

    Eigen::Affine3d r_wrist_init;
    this->_model_ptr->getPose("r_wrist","DWYTorso",r_wrist_init);
    std::cout<<"r_wrist init:"<<std::endl;
    std::cout<<r_wrist_init.matrix()<<std::endl;

    this->initManipTrj(CoM_frame, r_wrist_init);

    dq.setZero();
    t = 0.0;
    std::cout<<"Starting whole-body manipulation"<<std::endl;
    for(unsigned int i = 0; i < int(this->manip_trj->com_trj.Duration()) * 100; ++i)
    {

        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);


        KDL::Frame com_d = this->manip_trj->com_trj.Pos(t);
        KDL::Frame r_wrist_d = this->manip_trj->r_wrist_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.r_wrist->setReference(r_wrist_d);

        ws.update();

        int64_t tic = 0.0;
        if(ENABLE_ROS)
            tic = rclcpp::Clock().now().nanoseconds();

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero();}
        this->_q = this->_model_ptr->sum(this->_q, dq);

        int64_t toc = 0.0;
        if(ENABLE_ROS)
            toc = rclcpp::Clock().now().nanoseconds();

        this->update(this->_q);
        Eigen::Vector3d tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        KDLVectorAreEqual(com_d.p, vectorEigenToKDL(tmp_vector), 1e-3);
        Eigen::Affine3d r_wrist;
        this->_model_ptr->getPose("r_wrist","DWYTorso",r_wrist);
        KDLFramesAreEqual(r_wrist_d, poseEigenToKDL(r_wrist), 1e-3);
        //log
        Eigen::VectorXd com(3);
        com[0] = vectorEigenToKDL(tmp_vector).x();
        com[1] = vectorEigenToKDL(tmp_vector).y();
        com[2] = vectorEigenToKDL(tmp_vector).z();
        com[0] = com_d.p.x(); com[1] = com_d.p.y(); com[2] = com_d.p.z();
        //



        if(ENABLE_ROS)
            loop_time.push_back((toc-tic)/1e6);


        this->publishRobotState();

        //if(ENABLE_ROS)
        //    exec.spin_node_once(_n);


        t+=0.01;
        usleep(10000);
    }


    //Walking
    t = 0.0;
    dq.setZero();
    this->update(this->_q);

    this->_model_ptr->getCOM(CoM);
    CoM_frame.translation() = CoM;

    this->_model_ptr->getPose("l_sole", l_foot_init);

    this->_model_ptr->getPose("r_sole", r_foot_init);

    this->initTrj(CoM_frame, l_foot_init, r_foot_init);

    for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
    {

        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);


        KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
        KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
        KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);

        this->update(this->_q);

        ws.com->setReference(com_d.p);
        ws.l_sole->setReference(l_sole_d);
        ws.r_sole->setReference(r_sole_d);

        ws.update();

        int64_t tic = 0.0;
        if(ENABLE_ROS)
            tic = rclcpp::Clock().now().nanoseconds();

        if(!ws.solve(dq)){
            std::cout<<"SOLVER ERROR!"<<std::endl;
            dq.setZero();}
        this->_q = this->_model_ptr->sum(this->_q, dq);

        int64_t toc = 0.0;
        if(ENABLE_ROS)
            toc = rclcpp::Clock().now().nanoseconds();


        this->update(this->_q);

        Eigen::Vector3d tmp_vector;
        this->_model_ptr->getCOM(tmp_vector);
        KDLVectorAreEqual(com_d.p, vectorEigenToKDL(tmp_vector), 1e-3);
        Eigen::Affine3d l_sole;
        this->_model_ptr->getPose("l_sole", l_sole);
        KDLFramesAreEqual(l_sole_d, poseEigenToKDL(l_sole), 1e-3);
        Eigen::Affine3d r_sole;
        this->_model_ptr->getPose("r_sole", r_sole);
        KDLFramesAreEqual(r_sole_d, poseEigenToKDL(r_sole), 1e-3);


        if(ENABLE_ROS)
            loop_time.push_back((toc-tic)/1e6);

        this->publishRobotState();

        //if(ENABLE_ROS)
        //    exec.spin_node_once(_n);

        t+=0.01;
        usleep(10000);
    }



    if(ENABLE_ROS){
        double acc = 0.;
        for(unsigned int i = 0; i < loop_time.size(); ++i)
            acc += loop_time[i];
        std::cout<<"Medium time per solve: "<<acc/double(loop_time.size())<<" ms"<<std::endl;
    }



}

}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    auto ret = RUN_ALL_TESTS();
    return ret;
}
