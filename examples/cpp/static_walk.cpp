#include "StaticWalkUtils.hpp"
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
#include "qp_estimation.h"
#include "../../tests/common.h"
#include <xbot2_interface/xbotinterface2.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>



/**
 * @brief The example3 shows the usage of the OpenSoT library for open-loop resolved-rate IK.
 * The execution of the example can be shown in rviz by launching the launch file coman_ik.launch in \examples\launch folder,
 * which depends on the package
 *      https://github.com/ADVRHumanoids/iit-coman-ros-pkg/tree/master
 * for the COMAN model and meshes.
 */

namespace{

    /**
     * @brief The theWalkingStack class setup the stack for the walking
     */
    class theWalkingStack
    {
    public:
        /**
         * @brief printAb print A matrix and b vector for a task
         * @param task input task
         */
        void printAb(OpenSoT::Task<Eigen::MatrixXd,Eigen::VectorXd>& task)
        {
            std::cout<<"Task: "<<task.getTaskID()<<std::endl;
            std::cout<<"A: "<<task.getA()<<std::endl;
            std::cout<<"size of A: "<<task.getA().rows()<<"x"<<task.getA().cols()<<std::endl;
            std::cout<<"b: "<<task.getb()<<std::endl;
            std::cout<<std::endl;
        }

        /**
         * @brief theWalkingStack constructor
         * @param _model of the robot
         * @param q configuration
         */
        theWalkingStack(XBot::ModelInterface& _model):
            model_ref(_model)
        {
            using namespace OpenSoT::tasks::velocity;
            /**
              * @brief Creates Cartesian tasks for l_wrist and r_wrist frames w.r.t. DWYTorso frame,
              * and l_sole and r_sole w.r.t. world frame
              **/
            l_wrist = std::make_shared<Cartesian>("Cartesian::l_wrist", model_ref, "l_wrist","DWYTorso");
            r_wrist = std::make_shared<Cartesian>("Cartesian::r_wrist", model_ref, "r_wrist","DWYTorso");
            l_sole = std::make_shared<Cartesian>("Cartesian::l_sole", model_ref, "l_sole","world");
            r_sole = std::make_shared<Cartesian>("Cartesian::r_sole", model_ref, "r_sole","world");

            /**
              * @brief Creates CoM task always defined in world frame
              **/
            com = std::make_shared<CoM>(model_ref);

            /**
              * @brief Creates gase task in world frame. The active joint mask is used to set columns of the Jacobian to 0.
              * The active joint mask is initialized to false, the Waist joints are the only one set to true.
              **/
            gaze = std::make_shared<Gaze>("Cartesian::Gaze", model_ref, "world");
            std::vector<bool> ajm = gaze->getActiveJointsMask();
            for(unsigned int i = 0; i < ajm.size(); ++i)
                ajm[i] = false;
            ajm[model_ref.getDofIndex("WaistYaw")] = true;
            ajm[model_ref.getDofIndex("WaistSag")] = true;
            ajm[model_ref.getDofIndex("WaistLat")] = true;
            gaze->setActiveJointsMask(ajm);

            /**
              * @brief Creates postural task
              **/
            postural = std::make_shared<Postural>(model_ref);
            postural->setLambda(0.1);

            /**
             * @brief Retrieves joint limits from the model and creates joint limits constraint
             */
            Eigen::VectorXd qmin, qmax;
            model_ref.getJointLimits(qmin, qmax);
            joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(model_ref, qmax, qmin);

            /**
              * @brief creates joint velocity limits
              **/
            vel_limits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(model_ref, 2.*M_PI, 0.01);

            /**
              * @brief transform CoM tasks into equality constraint
              **/
            com_constr = std::make_shared<OpenSoT::constraints::TaskToConstraint>(com);

            /**
              * @brief creation of a stack with 3 tasks priorities (0 higest, 2 lowest)
              * 0. contacts
              * 1. arms + gaze
              * 2. postural[6:end]
              * and the following constrains: joint position and velocity limits, CoM tracking
              **/
            std::list<unsigned int> indices;
            for(unsigned int i = 6; i < model_ref.getNv(); ++i)
                indices.push_back(i);
            auto_stack = (l_sole + r_sole)/
                    (l_wrist + r_wrist + gaze)/
                    (postural%indices)<<joint_limits<<vel_limits<<com_constr;

            /**
              * @brief updates of the autostack
              **/
            auto_stack->update();

            /**
              * @brief creates solver inserting the stack with qpOASES by default
              **/
            solver = std::make_shared<OpenSoT::solvers::iHQP>(auto_stack->getStack(), auto_stack->getBounds(), 1e6);

            /**
             * @brief Setting some options to the qpOases Solver
             */
            qpOASES::Options opt;
            boost::any any_opt;
            solver->getOptions(0, any_opt);
            opt = boost::any_cast<qpOASES::Options>(any_opt);
            opt.numRefinementSteps = 0;
            opt.numRegularisationSteps = 1;
            for(unsigned int i = 0; i < 3; ++i)
                solver->setOptions(i, opt);

        }

        /**
         * @brief setInertiaPostureTask set Inertia matrix as weight for the posture task
         * @note Despite we are using a SubTask in the stack, we set the full Inertia matrix as weight to the task,
         * the SubTask will use only the required part of the task
         */
        void setInertiaPostureTask()
        {
            Eigen::MatrixXd M;
            model_ref.computeInertiaMatrix(M);

            postural->setWeight(M);
            postural->setLambda(0.);
        }

        /**
         * @brief update set the weight to the posture task and updates the stack
         * @param q actual state of the robot
         */
        void update()
        {
            setInertiaPostureTask();
            auto_stack->update();
        }

        /**
         * @brief solve call the solver
         * @param dq result
         * @return true if solution is found
         */
        bool solve(Eigen::VectorXd& dq)
        {
            return solver->solve(dq);
        }

        OpenSoT::tasks::velocity::Cartesian::Ptr l_wrist, r_wrist, l_sole, r_sole;
        OpenSoT::tasks::velocity::CoM::Ptr  com;
        OpenSoT::tasks::velocity::Gaze::Ptr gaze;
        OpenSoT::tasks::velocity::Postural::Ptr postural;
        OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits;
        OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_limits;
        OpenSoT::constraints::TaskToConstraint::Ptr com_constr;

        OpenSoT::AutoStack::Ptr auto_stack;

        XBot::ModelInterface& model_ref;

        OpenSoT::solvers::iHQP::Ptr solver;

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


    /**
     * @brief The StaticWalk class contains publisher helpers and main execution loop
     */
    class StaticWalk
    {
    public:
        StaticWalk()
        {
            /**
              * @brief Retrieve model from config file
              **/
            _model_ptr = GetTestModel("coman_floating_base");
            /**
              * @brief Set and update moedl with zero config
              **/
            _q = _model_ptr->getNeutralQ();
            _model_ptr->setJointPosition(_q);
            _model_ptr->update();

            /**
              * @brief Initialize kinematic estimation
              **/
            _fb.reset(new OpenSoT::floating_base_estimation::kinematic_estimation(_model_ptr,"r_sole"));
            _fb->update();

            /**
              * @brief if ROSCORE is running the nodehandle is created with some classes to visualize the execution
              * in rviz
              **/
            _n.reset(new ros2_node());
        }

        std::shared_ptr<ros2_node> _n;

        ~StaticWalk(){}

        void initTrj(const KDL::Frame& com_init, const KDL::Frame& l_sole_init, const KDL::Frame& r_sole_init)
        {
            walk_trj.reset(new walking_pattern_generator(com_init, l_sole_init, r_sole_init));
        }

        void initManipTrj(const KDL::Frame& com_init, const KDL::Frame& r_wrist_init)
        {
            manip_trj.reset(new manipulation_trajectories(com_init,r_wrist_init));
        }


        void publishRobotState()
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


        std::shared_ptr<manipulation_trajectories> manip_trj;
        std::shared_ptr<walking_pattern_generator> walk_trj;


        OpenSoT::floating_base_estimation::kinematic_estimation::Ptr _fb;
        XBot::ModelInterface::Ptr _model_ptr;
        Eigen::VectorXd _q;

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

        void static_walk()
        {
            /**
              * @brief Initialize and update the robot with a home position
              **/
            this->setGoodInitialPosition();
            this->_model_ptr->setJointPosition(_q);
            this->_model_ptr->update();

            /**
              * @brief Set anchor foot for the fb estimation and updates
              **/
            this->_fb->setAnchor("r_sole");
            this->_fb->update();

        //1. WALKING Phase
            /**
              * @brief Get CoM and feet state from model
              **/
            Eigen::Vector3d com_vector; this->_model_ptr->getCOM(com_vector);
            Eigen::Affine3d com_init; com_init.translation() = com_vector;
            Eigen::Affine3d l_foot_init; this->_model_ptr->getPose("l_sole", l_foot_init);
            Eigen::Affine3d r_foot_init; this->_model_ptr->getPose("r_sole", r_foot_init);

            /**
              * @brief Initialize trajectories and walking stack
              **/
            KDL::Frame com_init_kdl, l_foot_init_kdl, r_foot_init_kdl;
            tf2::transformEigenToKDL(com_init, com_init_kdl);
            tf2::transformEigenToKDL(l_foot_init, l_foot_init_kdl);
            tf2::transformEigenToKDL(r_foot_init, r_foot_init_kdl);
            this->initTrj(com_init_kdl, l_foot_init_kdl, r_foot_init_kdl);
            theWalkingStack ws(*_model_ptr);


            double t = 0.;
            Eigen::VectorXd dq(this->_model_ptr->getNv());
            dq.setZero();
            std::vector<double> loop_time;
            for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
            {
                /**
                  * @brief Get trajectory and anchor at time t
                  **/
                KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
                KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
                KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);
                std::string anchor_d = this->walk_trj->getAnchor(t);

                /**
                  * @brief Update model and fb estimation
                  **/
                _model_ptr->setJointPosition(_q);
                _model_ptr->update();
                _fb->setAnchor(anchor_d);
                _fb->update();

                /**
                  * @brief Set references to tasks
                  **/
                ws.com->setReference(com_d.p);
                ws.l_sole->setReference(l_sole_d);
                ws.r_sole->setReference(r_sole_d);

                /**
                  * @brief Stack update
                  **/
                ws.update();

                uint tic = 0.0;
                tic = rclcpp::Clock().now().nanoseconds();

                /**
                  * @brief Solve and integrate state
                  **/
                if(!ws.solve(dq))
                    dq.setZero();
                this->_q = _model_ptr->sum(this->_q, dq);

                uint toc = 0.0;
                toc = rclcpp::Clock().now().nanoseconds();
                loop_time.push_back((toc-tic)/1e6);


                this->publishRobotState();


                t+=0.01;
                usleep(10000);
            }


        //2 MANIPULATION phase
            /**
              * @brief Same steps as before
              **/
            _model_ptr->getCOM(com_vector);
            com_init.translation() = com_vector;
            Eigen::Affine3d r_wrist_init; _model_ptr->getPose("r_wrist","DWYTorso",r_wrist_init);

            KDL::Frame r_wrist_init_kdl;
            tf2::transformEigenToKDL(r_wrist_init, r_wrist_init_kdl);
            tf2::transformEigenToKDL(com_init, com_init_kdl);
            this->initManipTrj(com_init_kdl,  r_wrist_init_kdl);


            t = 0.0;
            for(unsigned int i = 0; i < int(this->manip_trj->com_trj.Duration()) * 100; ++i)
            {
                KDL::Frame com_d = this->manip_trj->com_trj.Pos(t);
                KDL::Frame r_wrist_d = this->manip_trj->r_wrist_trj.Pos(t);

                _model_ptr->setJointPosition(_q);
                _model_ptr->update();
                _fb->update();

                ws.com->setReference(com_d.p);
                ws.r_wrist->setReference(r_wrist_d);

                ws.update();

                uint tic = 0.0;
                tic = rclcpp::Clock().now().nanoseconds();

                if(!ws.solve(dq))
                    dq.setZero();
                this->_q = _model_ptr->sum(this->_q, dq);

                uint toc = 0.0;
                toc = rclcpp::Clock().now().nanoseconds();

                loop_time.push_back((toc-tic)/1e6);

                this->publishRobotState();


                t+=0.01;
                usleep(10000);
            }


        //3 WALKING (AGAIN) Phase
        /**
         * @brief Same steps as before, we want to be sure to start with the left foot again so we first set the anchor
         **/
        _fb->setAnchor("l_sole");
        _model_ptr->setJointPosition(_q);
        _model_ptr->update();
        _fb->update();

        ws.update();

        this->_model_ptr->getCOM(com_vector);
        com_init.translation() = com_vector;
        this->_model_ptr->getPose("l_sole", l_foot_init);
        this->_model_ptr->getPose("r_sole", r_foot_init);


        tf2::transformEigenToKDL(com_init, com_init_kdl);
        tf2::transformEigenToKDL(l_foot_init, l_foot_init_kdl);
        tf2::transformEigenToKDL(r_foot_init, r_foot_init_kdl);
        this->initTrj(com_init_kdl, l_foot_init_kdl, r_foot_init_kdl);

        t = 0.;
        for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
        {
            KDL::Frame com_d = this->walk_trj->com_trj.Pos(t);
            KDL::Frame l_sole_d = this->walk_trj->l_sole_trj.Pos(t);
            KDL::Frame r_sole_d = this->walk_trj->r_sole_trj.Pos(t);
            std::string anchor_d = this->walk_trj->getAnchor(t);

            _model_ptr->setJointPosition(_q);
            _model_ptr->update();
            _fb->setAnchor(anchor_d);
            _fb->update();

            ws.com->setReference(com_d.p);
            ws.l_sole->setReference(l_sole_d);
            ws.r_sole->setReference(r_sole_d);

            ws.update();


            uint tic = 0.0;
            tic = rclcpp::Clock().now().nanoseconds();

            if(!ws.solve(dq))
                dq.setZero();
            this->_q = _model_ptr->sum(this->_q, dq);

            uint toc = 0.0;
            toc = rclcpp::Clock().now().nanoseconds();
            loop_time.push_back((toc-tic)/1e6);


            this->publishRobotState();


            t+=0.01;
            usleep(10000);
        }



        double acc = 0.;
        for(unsigned int i = 0; i < loop_time.size(); ++i)
            acc += loop_time[i];
        std::cout<<"Medium time per solve: "<<acc/double(loop_time.size())<<" ms"<<std::endl;

    }

};


}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    StaticWalk static_walk;
    static_walk.static_walk();
    return 0;
}
