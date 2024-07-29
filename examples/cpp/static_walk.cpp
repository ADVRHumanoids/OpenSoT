#include "../../tests/trajectory_utils.h"
#include "../../tests/ros_trj_publisher.h"
#include <tf/transform_broadcaster.h>
#include "StaticWalkUtils.hpp"
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
#include "qp_estimation.h"
#include <sensor_msgs/JointState.h>
#include "../../tests/common.h"
#include <eigen_conversions/eigen_kdl.h>


bool IS_ROSCORE_RUNNING;


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
            if(IS_ROSCORE_RUNNING){
                _n.reset(new ros::NodeHandle());
                world_broadcaster.reset(new tf::TransformBroadcaster());
            }
        }

        ~StaticWalk(){}

        void initTrj(const KDL::Frame& com_init, const KDL::Frame& l_sole_init, const KDL::Frame& r_sole_init)
        {
            walk_trj.reset(new walking_pattern_generator(com_init, l_sole_init, r_sole_init));
        }

        void initManipTrj(const KDL::Frame& com_init, const KDL::Frame& r_wrist_init)
        {
            manip_trj.reset(new manipulation_trajectories(com_init,r_wrist_init));
        }

        void initTrjPublisher()
        {
            if(IS_ROSCORE_RUNNING){
                com_trj_pub.reset(new trajectory_utils::trajectory_publisher("com_trj"));
                com_trj_pub->setTrj(walk_trj->com_trj.getTrajectory(), "world", "com");

                l_sole_trj_pub.reset(new trajectory_utils::trajectory_publisher("l_sole_trj"));
                l_sole_trj_pub->setTrj(walk_trj->l_sole_trj.getTrajectory(), "world", "l_sole");

                r_sole_trj_pub.reset(new trajectory_utils::trajectory_publisher("r_sole_trj"));
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

                r_wrist_trj_pub.reset(new trajectory_utils::trajectory_publisher("r_wrist_trj"));
                r_wrist_trj_pub->setTrj(manip_trj->r_wrist_trj.getTrajectory(), "DWYTorso", "r_wrist");
            }
        }

        void publishCoMAndFeet(const KDL::Frame& com, const KDL::Frame& l_foot, const KDL::Frame& r_foot, const std::string& anchor)
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

                Eigen::Affine3d _l_foot, _r_foot;
                _l_foot(0,3) = l_foot.p.x()+0.02;
                _l_foot(1,3) = l_foot.p.y();
                _l_foot(2,3) = l_foot.p.z();
                for(unsigned int i = 0; i < 3; ++i)
                    for(unsigned j = 0; j < 3; ++j)
                        _l_foot(i,j) = l_foot.M(i,j);

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

                Eigen::Isometry3d text_pose; text_pose.Identity();
                text_pose(1,3) = -0.4;
                std::string text = "Anchor: "+anchor+"\n";
                text += "Robot Anchor: "+ _fb->getAnchor()+"\n";
                std::string floating_base;
                _model_ptr->getFloatingBaseLink(floating_base);
                text += "Robot Floating Base: "+ floating_base+"\n";
                visual_tools->publishText(text_pose,text);
            }
        }


        void publishRobotState()
        {

            if(IS_ROSCORE_RUNNING)
            {
                sensor_msgs::JointState joint_msg;
                for(unsigned int i = 1; i < _model_ptr->getJointNames().size(); ++i)
                {
                    joint_msg.name.push_back(_model_ptr->getJointNames()[i]);
                    joint_msg.position.push_back(_q[_model_ptr->getQIndex(_model_ptr->getJointNames()[i])]);
                }

                joint_msg.header.stamp = ros::Time::now();


                Eigen::Affine3d world_T_bl;
                _model_ptr->getPose("Waist",world_T_bl);

                tf::Transform anchor_T_world;
                tf::transformEigenToTF(world_T_bl, anchor_T_world);

                world_broadcaster->sendTransform(tf::StampedTransform(
                    anchor_T_world.inverse(), joint_msg.header.stamp,
                    "Waist", "world"));


                joint_state_pub.publish(joint_msg);
            }

        }


        std::shared_ptr<manipulation_trajectories> manip_trj;
        std::shared_ptr<walking_pattern_generator> walk_trj;
        std::shared_ptr<trajectory_utils::trajectory_publisher> com_trj_pub, l_sole_trj_pub, r_sole_trj_pub, r_wrist_trj_pub;

        ros::Publisher joint_state_pub;
        std::shared_ptr<tf::TransformBroadcaster> world_broadcaster;

        rviz_visual_tools::RvizVisualToolsPtr visual_tools;

        OpenSoT::floating_base_estimation::kinematic_estimation::Ptr _fb;
        XBot::ModelInterface::Ptr _model_ptr;
        Eigen::VectorXd _q;

        std::shared_ptr<ros::NodeHandle> _n;

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
              * @brief Initialize trajectories, publiahers and walking stack
              **/
            KDL::Frame com_init_kdl, l_foot_init_kdl, r_foot_init_kdl;
            tf::transformEigenToKDL(com_init, com_init_kdl);
            tf::transformEigenToKDL(l_foot_init, l_foot_init_kdl);
            tf::transformEigenToKDL(r_foot_init, r_foot_init_kdl);
            this->initTrj(com_init_kdl, l_foot_init_kdl, r_foot_init_kdl);
            this->initTrjPublisher();
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
                if(IS_ROSCORE_RUNNING)
                    tic = ros::Time::now().nsec;

                /**
                  * @brief Solve and integrate state
                  **/
                if(!ws.solve(dq))
                    dq.setZero();
                this->_q = _model_ptr->sum(this->_q, dq);

                uint toc = 0.0;
                if(IS_ROSCORE_RUNNING){
                    toc = ros::Time::now().nsec;
                    loop_time.push_back((toc-tic)/1e6);

                    this->com_trj_pub->publish();
                    this->l_sole_trj_pub->publish();
                    this->r_sole_trj_pub->publish();
                }

                this->publishCoMAndFeet(com_d,l_sole_d,r_sole_d,anchor_d);
                this->publishRobotState();

                if(IS_ROSCORE_RUNNING)
                    ros::spinOnce();

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
            tf::transformEigenToKDL(r_wrist_init, r_wrist_init_kdl);
            tf::transformEigenToKDL(com_init, com_init_kdl);
            this->initManipTrj(com_init_kdl,  r_wrist_init_kdl);
            this->initManipTrjPublisher();


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
                if(IS_ROSCORE_RUNNING)
                    tic = ros::Time::now().nsec;

                if(!ws.solve(dq))
                    dq.setZero();
                this->_q = _model_ptr->sum(this->_q, dq);

                uint toc = 0.0;
                if(IS_ROSCORE_RUNNING){
                    toc = ros::Time::now().nsec;

                    loop_time.push_back((toc-tic)/1e6);

                    this->com_trj_pub->publish();
                    //this->r_wrist_trj_pub->setTrj(
                    //            this->manip_trj->r_wrist_trj.getTrajectory(), "DWYTorso");
                    this->r_wrist_trj_pub->publish(true);
                }

                this->publishRobotState();

                if(IS_ROSCORE_RUNNING)
                    ros::spinOnce();


                t+=0.01;
                usleep(10000);
            }

            if(IS_ROSCORE_RUNNING){
                this->com_trj_pub->deleteAllMarkersAndTrj();
                this->r_wrist_trj_pub->deleteAllMarkersAndTrj();}

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


        tf::transformEigenToKDL(com_init, com_init_kdl);
        tf::transformEigenToKDL(l_foot_init, l_foot_init_kdl);
        tf::transformEigenToKDL(r_foot_init, r_foot_init_kdl);
        this->initTrj(com_init_kdl, l_foot_init_kdl, r_foot_init_kdl);
        this->initTrjPublisher();

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
            if(IS_ROSCORE_RUNNING)
                tic = ros::Time::now().nsec;

            if(!ws.solve(dq))
                dq.setZero();
            this->_q = _model_ptr->sum(this->_q, dq);

            uint toc = 0.0;
            if(IS_ROSCORE_RUNNING){
                toc = ros::Time::now().nsec;
                loop_time.push_back((toc-tic)/1e6);

                this->com_trj_pub->publish();
                this->l_sole_trj_pub->publish();
                this->r_sole_trj_pub->publish();
            }

            this->publishCoMAndFeet(com_d,l_sole_d,r_sole_d,anchor_d);
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

};


}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testStaticWalk_node");
  IS_ROSCORE_RUNNING = ros::master::check();
  StaticWalk static_walk;
  static_walk.static_walk();
  return 0;
}
