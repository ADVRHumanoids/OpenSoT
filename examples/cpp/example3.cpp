#include "../../tests/trajectory_utils.h"
#include "../../tests/ros_trj_publisher.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
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
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <sensor_msgs/JointState.h>

std::string _path_to_cfg = OPENSOT_EXAMPLE_PATH "configs/coman/configs/config_coman_floating_base.yaml";

bool IS_ROSCORE_RUNNING;


/**
 * @brief The example3 shows the usage of the OpenSoT library for open-loop resolved-rate IK.
 * The execution of the example can be shown in rviz by launching the launch file example3.launch in \examples\launch folder,
 * which depends on the package
 *      https://github.com/ADVRHumanoids/iit-coman-ros-pkg/tree/master
 * for the COMAN model and meshes.
 */

namespace{
    /**
     * @brief The manipulation_trajectories class creates a simple linear trajectory for the arm and the com during
     * simple manipulation task
     */
    class manipulation_trajectories{
    public:
        manipulation_trajectories(const KDL::Frame& com_init,
                                  const KDL::Frame& r_wrist_init):
            com_trj(0.01, "world", "com"),
            r_wrist_trj(0.01, "DWYTorso", "r_wrist")
        {
            T_trj = 1.;
            double h_com = 0.1;
            double arm_forward = 0.1;

            KDL::Frame com_wp = com_init;
            KDL::Frame r_wrist_wp = r_wrist_init;

            std::vector<KDL::Frame> com_waypoints, r_wrist_waypoints;
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

        trajectory_utils::trajectory_generator com_trj, r_wrist_trj;
        double T_trj;
    };


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

            KDL::Frame r_sole_wp, l_sole_wp;

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
            com_wp.p.y(com_init.p.y());
            com_waypoints.push_back(com_wp);
            com_time.push_back(T_com);

            l_sole_wp = l_sole_trj.Pos(l_sole_trj.Duration());
            l_sole_trj.addMinJerkTrj(l_sole_wp, l_sole_wp, T_com);
            r_sole_trj.addMinJerkTrj(r_sole_wp, r_sole_wp, T_com);

            com_trj.addMinJerkTrj(com_waypoints,com_time);
        }

        /**
         * @brief getAnchor return the hardcoded anchor foot based on the time
         * @param t time
         * @return anchor string
         */
        std::string getAnchor(const double t)
        {
            double phase = T_com+T_foot;
            if(t < phase)
                return "l_sole";
            else if(t < 2.*phase)
                return "r_sole";
            else if (t < 3*phase)
                return "l_sole";
            else
                return "r_sole";
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

        trajectory_utils::trajectory_generator com_trj, l_sole_trj, r_sole_trj;
    };

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
        theWalkingStack(XBot::ModelInterface& _model,
                        const Eigen::VectorXd& q):
            model_ref(_model),
            I(q.size(), q.size())
        {
            I.setIdentity(q.size(), q.size());

            /**
              * @brief Creates Cartesian tasks for l_wrist and r_wrist frames w.r.t. DWYTorso frame,
              * and l_sole and r_sole w.r.t. world frame
              **/
            l_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_wrist", q,
                model_ref, "l_wrist","DWYTorso"));
            r_wrist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_wrist", q,
                model_ref, "r_wrist","DWYTorso"));
            l_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_sole", q,
                model_ref, "l_sole","world"));
            r_sole.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_sole", q,
                model_ref, "r_sole","world"));

            /**
              * @brief Creates CoM task always defined in world frame
              **/
            com.reset(new OpenSoT::tasks::velocity::CoM(q, model_ref));

            /**
              * @brief Creates gase task in world frame. The active joint mask is used to set columns of the Jacobian to 0.
              * The active joint mask is initialized to false, the Waist joints are the only one set to true.
              **/
            gaze.reset(new OpenSoT::tasks::velocity::Gaze("Cartesian::Gaze",q,
                            model_ref, "world"));
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
            postural.reset(new OpenSoT::tasks::velocity::Postural(q));
            postural->setLambda(0.1);

            /**
             * @brief Retrieves joint limits from the model and creates joint limits constraint
             */
            Eigen::VectorXd qmin, qmax;
            model_ref.getJointLimits(qmin, qmax);
            joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(q, qmax, qmin));

            /**
              * @brief creates joint velocity limits
              **/
            vel_limits.reset(new OpenSoT::constraints::velocity::VelocityLimits(2.*M_PI, 0.01, q.size()));

            /**
              * @brief transform CoM tasks into equality constraint
              **/
            com_constr.reset(new OpenSoT::constraints::TaskToConstraint(com));

            /**
              * @brief creation of a stack with 3 tasks priorities (0 higest, 2 lowest)
              * 0. contacts
              * 1. arms + gaze
              * 2. postural[6:end]
              * and the following constrains: joint position and velocity limits, CoM tracking
              **/
            std::list<unsigned int> indices;
            for(unsigned int i = 6; i < q.size(); ++i)
                indices.push_back(i);
            auto_stack = (l_sole + r_sole)/
                    (l_wrist + r_wrist + gaze)/
                    (postural%indices)<<joint_limits<<vel_limits<<com_constr;

            /**
              * @brief updates of the autostack
              **/
            auto_stack->update(q);

            /**
              * @brief creates solver inserting the stack with qpOASES by default
              **/
            solver.reset(new OpenSoT::solvers::iHQP(auto_stack->getStack(),
                        auto_stack->getBounds(), 1e6));

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
            model_ref.getInertiaMatrix(M);

            postural->setWeight(M);
            postural->setLambda(0.);
        }

        /**
         * @brief update set the weight to the posture task and updates the stack
         * @param q actual state of the robot
         */
        void update(const Eigen::VectorXd& q)
        {
            setInertiaPostureTask();
            auto_stack->update(q);
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

        Eigen::MatrixXd I;

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
            _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

            /**
              * @brief Set and update moedl with zero config
              **/
            _q.setZero(_model_ptr->getJointNum());
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
                joint_msg.name = _model_ptr->getEnabledJointNames();

                for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
                    joint_msg.position.push_back(0.0);

                for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
                {
                    int id = _model_ptr->getDofIndex(joint_msg.name[i]);
                    joint_msg.position[id] = _q[i];
                }

                joint_msg.header.stamp = ros::Time::now();

                tf::Transform anchor_T_world;
                Eigen::Affine3d world_T_anchor = _fb->getAnchorPose();
                KDL::Frame anchor_T_world_KDL;
                tf::transformEigenToKDL(world_T_anchor.inverse(), anchor_T_world_KDL);

                anchor_T_world.setOrigin(tf::Vector3(anchor_T_world_KDL.p.x(),
                    anchor_T_world_KDL.p.y(), anchor_T_world_KDL.p.z()));
                double x,y,z,w;
                anchor_T_world_KDL.M.GetQuaternion(x,y,z,w);
                anchor_T_world.setRotation(tf::Quaternion(x,y,z,w));

                world_broadcaster->sendTransform(tf::StampedTransform(
                    anchor_T_world, joint_msg.header.stamp,
                    _fb->getAnchor(), "world"));

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
            KDL::Vector com_vector; this->_model_ptr->getCOM(com_vector);
            KDL::Frame com_init; com_init.p = com_vector;
            KDL::Frame l_foot_init; this->_model_ptr->getPose("l_sole", l_foot_init);
            KDL::Frame r_foot_init; this->_model_ptr->getPose("r_sole", r_foot_init);

            /**
              * @brief Initialize trajectories, publiahers and walking stack
              **/
            this->initTrj(com_init, l_foot_init, r_foot_init);
            this->initTrjPublisher();
            theWalkingStack ws(*(this->_model_ptr.get()), this->_q);


            double t = 0.;
            Eigen::VectorXd dq(this->_q.size());
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
                ws.update(this->_q);

                uint tic = 0.0;
                if(IS_ROSCORE_RUNNING)
                    tic = ros::Time::now().nsec;

                /**
                  * @brief Solve and integrate state
                  **/
                if(!ws.solve(dq))
                    dq.setZero(dq.size());
                this->_q += dq;

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
            com_init.p = com_vector;
            KDL::Frame r_wrist_init; _model_ptr->getPose("r_wrist","DWYTorso",r_wrist_init);

            this->initManipTrj(com_init,  r_wrist_init);
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

                ws.update(this->_q);

                uint tic = 0.0;
                if(IS_ROSCORE_RUNNING)
                    tic = ros::Time::now().nsec;

                if(!ws.solve(dq))
                    dq.setZero(dq.size());
                this->_q += dq;

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

        ws.update(this->_q);

        this->_model_ptr->getCOM(com_vector);
        com_init.p = com_vector;
        this->_model_ptr->getPose("l_sole", l_foot_init);
        this->_model_ptr->getPose("r_sole", r_foot_init);

        this->initTrj(com_init, l_foot_init, r_foot_init);
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

            ws.update(this->_q);


            uint tic = 0.0;
            if(IS_ROSCORE_RUNNING)
                tic = ros::Time::now().nsec;

            if(!ws.solve(dq))
                dq.setZero(dq.size());
            this->_q += dq;

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
