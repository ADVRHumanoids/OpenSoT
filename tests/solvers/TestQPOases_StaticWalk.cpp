#include <advr_humanoids_common_utils/idynutils.h>
#include <advr_humanoids_common_utils/test_utils.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <trajectory_utils/trajectory_utils.h>
#include <gtest/gtest.h>
#include <trajectory_utils/utils/ros_trj_publisher.h>
#include <tf/transform_broadcaster.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

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
            com_trj(0.001, "world", "com"),
            l_sole_trj(0.001, "world", "l_sole"),
            r_sole_trj(0.001, "world", "r_sole")
        {
            T_com = 4.;
            T_foot = 2.;

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

        trajectory_utils::trajectory_generator com_trj;
        trajectory_utils::trajectory_generator l_sole_trj;
        trajectory_utils::trajectory_generator r_sole_trj;
    };

    class testStaticWalk: public ::testing::Test{
    public:
        static void null_deleter(idynutils2 *) {}

        testStaticWalk():
            _robot("coman",
                   std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                   std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
            _q(_robot.iDynTree_model.getNrOfDOFs())
        {
            _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
                    (XBot::ModelInterface::getModel(_path_to_cfg));
            _model_ptr->loadModel(boost::shared_ptr<idynutils2>(&_robot, &null_deleter));

            if(_model_ptr)
                std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
            else
                std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;



            _q.setZero(_robot.iDynTree_model.getNrOfDOFs());
            _robot.updateiDynTreeModel(_q,true);

            int argc = 0;
            char* argv[] = {};
            ros::init(argc, argv, "testStaticWalk_node");

            _n.reset(new ros::NodeHandle());
            world_broadcaster.reset(new tf::TransformBroadcaster());
        }

        ~testStaticWalk(){}
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

        void initTrjPublisher()
        {
            visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/com_feet_visual_marker"));

            com_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("com_trj"));
            com_trj_pub->setTrj(walk_trj->com_trj.getTrajectory(), "world");

            l_sole_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("l_sole_trj"));
            l_sole_trj_pub->setTrj(walk_trj->l_sole_trj.getTrajectory(), "world");

            r_sole_trj_pub.reset(
                new trajectory_utils::trajectory_publisher("r_sole_trj"));
            r_sole_trj_pub->setTrj(walk_trj->r_sole_trj.getTrajectory(), "world");

            joint_state_pub = _n->advertise<sensor_msgs::JointState>("joint_states", 1000);
        }

        void publishCoMAndFeet(const KDL::Frame& com,
                               const KDL::Frame& l_foot,
                               const KDL::Frame& r_foot,
                               const std::string& anchor)
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

            visual_tools->publishWireframeRectangle(_l_foot, 0.05, 0.1);
            visual_tools->publishWireframeRectangle(_r_foot, 0.05, 0.1);

            Eigen::Affine3d text_pose; text_pose.Identity();
            text_pose(1,3) = -0.4;
            std::string text = "Anchor: "+anchor+"\n";
            text += "Robot Anchor: "+ _robot.getAnchorName()+"\n";
            std::string floating_base;
            _robot.iDynTree_model.getLinkName(_robot.iDynTree_model.getFloatingBaseLink(),
                                              floating_base);
            text += "Robot Floating Base: "+ floating_base+"\n";
            visual_tools->publishText(text_pose,text);
        }


        void publishRobotState()
        {
            sensor_msgs::JointState joint_msg;
            joint_msg.name = _robot.getJointNames();


            for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
                joint_msg.position.push_back(0.0);

            for(unsigned int i = 0; i < joint_msg.name.size(); ++i)
            {
                int id = _robot.iDynTree_model.getDOFIndex(joint_msg.name[i]);
                joint_msg.position[id] = _q[i];
            }

            joint_msg.header.stamp = ros::Time::now();


            tf::Transform anchor_T_world;
            KDL::Frame anchor_T_world_KDL = _robot.getAnchor_T_World();
            anchor_T_world.setOrigin(tf::Vector3(anchor_T_world_KDL.p.x(),
                anchor_T_world_KDL.p.y(), anchor_T_world_KDL.p.z()));
            double x,y,z,w;
            anchor_T_world_KDL.M.GetQuaternion(x,y,z,w);
            anchor_T_world.setRotation(tf::Quaternion(x,y,z,w));

            world_broadcaster->sendTransform(tf::StampedTransform(
                anchor_T_world, joint_msg.header.stamp,
                _robot.getAnchorName(), "world"));


            joint_state_pub.publish(joint_msg);

        }


        boost::shared_ptr<walking_pattern_generator> walk_trj;
        boost::shared_ptr<trajectory_utils::trajectory_publisher> com_trj_pub;
        boost::shared_ptr<trajectory_utils::trajectory_publisher> l_sole_trj_pub;
        boost::shared_ptr<trajectory_utils::trajectory_publisher> r_sole_trj_pub;

        ros::Publisher joint_state_pub;
        boost::shared_ptr<tf::TransformBroadcaster> world_broadcaster;

        rviz_visual_tools::RvizVisualToolsPtr visual_tools;

        idynutils2 _robot;
        XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
        Eigen::VectorXd _q;

        boost::shared_ptr<ros::NodeHandle> _n;

        void setGoodInitialPosition() {
            _q[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

            _q[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

            _q[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("LShYaw")] = -15.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

            _q[_robot.iDynTree_model.getDOFIndex("RShSag")] =  20.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("RShLat")] = -10.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("RShYaw")] = 15.0*M_PI/180.0;
            _q[_robot.iDynTree_model.getDOFIndex("RElbj")] = -80.0*M_PI/180.0;

        }

    };


    TEST_F(testStaticWalk, testStaticWalk)
    {
        this->setGoodInitialPosition();
        this->_robot.updateiDynTreeModel(this->_q, true);

        //We assume the world between the feet
        KDL::Vector com_vector = this->_robot.iDynTree_model.getCOMKDL();
        KDL::Frame com_init; com_init.p = com_vector;
        KDL::Frame l_foot_init = this->_robot.iDynTree_model.
                getPositionKDL(
                    this->_robot.iDynTree_model.getLinkIndex("l_sole"));
        KDL::Frame r_foot_init = this->_robot.iDynTree_model.
                getPositionKDL(
                    this->_robot.iDynTree_model.getLinkIndex("r_sole"));

        this->initTrj(com_init, l_foot_init, r_foot_init);
        this->initTrjPublisher();

        double t = 0.;
        for(unsigned int i = 0; i < int(this->walk_trj->com_trj.Duration()) * 100; ++i)
        {
            this->com_trj_pub->publish();
            this->l_sole_trj_pub->publish();
            this->r_sole_trj_pub->publish();

            this->publishCoMAndFeet(this->walk_trj->com_trj.Pos(t),
                                    this->walk_trj->l_sole_trj.Pos(t),
                                    this->walk_trj->r_sole_trj.Pos(t),
                                    this->walk_trj->getAnchor(t));
            this->publishRobotState();

            ros::spinOnce();

            t+=0.01;
            usleep(10000);
        }
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
