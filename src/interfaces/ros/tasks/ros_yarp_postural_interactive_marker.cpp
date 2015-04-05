#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <boost/phoenix/bind/bind_member_function.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_position_joint_msg.h>
#include <yarp/os/BufferedPort.h>
#include <idynutils/idynutils.h>
#include <idynutils/cartesian_utils.h>


using namespace visualization_msgs;
using namespace moveit::core;

class SmallSphereMarker1DoF
{
public:
    SmallSphereMarker1DoF(iDynUtils& model, const std::vector<std::string>& joint_names,
                          const std::string& server_name,
                          const std::string& port_name, yarp::os::Network& yarp_network):
        _model(model),
        _joint_names(joint_names),
        server( new interactive_markers::InteractiveMarkerServer(server_name, server_name+"_id") ),
        _yarp_network(yarp_network),
        _q_init()
    {
        _base_link = _model.moveit_robot_model->getLinkModelNames()[1];

        for(unsigned int i = 0; i < _joint_names.size(); ++i)
        {
            boost::shared_ptr<const urdf::Joint> joint = _model.urdf_model->getJoint(_joint_names[i]);
            if(joint != NULL)
            {
                std::string distal_link = joint->child_link_name;

                tf::StampedTransform T;
                ros::Time t = ros::Time(0);
                _tl.waitForTransform(_base_link, distal_link, t, ros::Duration(3.0));
                _tl.lookupTransform(_base_link, distal_link, t, T);

                //Here I create a marker
                make1DofMarker(false, T, true, joint);
            }
            else
                ROS_ERROR("Joint %s does not exists! Skipped.", _joint_names[i].c_str());
        }

        _port.open(server_name);
        yarp_network.connect(_port.getName(), port_name+"/set_ref:i");

        _rpc.open("/"+server_name+"/rpc");
        yarp_network.connect("/"+server_name+"/rpc", port_name+"/rpc");

        server->applyChanges();
    }

    void update()
    {
        boost::lock_guard<boost::mutex> guard(_mtx);
        for(unsigned int i = 0; i < _joint_names.size(); ++i)
        {
            boost::shared_ptr<const urdf::Joint> joint = _model.urdf_model->getJoint(_joint_names[i]);
            if(joint != NULL)
            {
                std::string distal_link = joint->child_link_name;

                tf::StampedTransform T;
                ros::Time t = ros::Time(0);
                _tl.waitForTransform(_base_link, distal_link, t, ros::Duration(3.0));
                _tl.lookupTransform(_base_link, distal_link, t, T);

                geometry_msgs::Pose pose;
                pose.position.x = T.getOrigin().getX();
                pose.position.y = T.getOrigin().getY();
                pose.position.z = T.getOrigin().getZ();
                pose.orientation.x = T.getRotation().getX();
                pose.orientation.y = T.getRotation().getY();
                pose.orientation.z = T.getRotation().getZ();
                pose.orientation.w = T.getRotation().getW();

                server->setPose(joint->name, pose);

                server->applyChanges();
            }
            else
                ROS_ERROR("Joint %s does not exists! Skipped.", _joint_names[i].c_str());
        }
    }

    ~SmallSphereMarker1DoF()
    {
        _port.close();
        _rpc.close();
    }

private:
    iDynUtils& _model;
    std::string _base_link;
    std::vector<std::string> _joint_names;
public: boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    interactive_markers::MenuHandler _menu_handler;
    yarp::os::BufferedPort<OpenSoT::interfaces::yarp::msgs::yarp_position_joint_msg_portable> _port;
    yarp::os::Network& _yarp_network;
    yarp::os::RpcClient _rpc;
    tf::TransformListener _tl;
    quaternion _q_init;
    boost::mutex _mtx;

    Marker makeSphere( InteractiveMarker &msg )
    {
      Marker marker;
      marker.type = Marker::SPHERE;
      marker.scale.x = msg.scale * 0.1;
      marker.scale.y = msg.scale * 0.1;
      marker.scale.z = msg.scale * 0.1;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;
      return marker;
    }

    InteractiveMarkerControl& makeSphereControl( InteractiveMarker &msg )
    {
      InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back( makeSphere(msg) );
      msg.controls.push_back( control );
      return msg.controls.back();
    }

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        boost::lock_guard<boost::mutex> guard(_mtx);
        switch ( feedback->event_type )
        {
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
                _q_init.w = feedback->pose.orientation.w;
                _q_init.x = feedback->pose.orientation.x;
                _q_init.y = feedback->pose.orientation.y;
                _q_init.z = feedback->pose.orientation.z;

                break;
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                quaternion q;
                q.w = feedback->pose.orientation.w;
                q.x = feedback->pose.orientation.x;
                q.y = feedback->pose.orientation.y;
                q.z = feedback->pose.orientation.z;

                KDL::Vector e = -2.0*quaternion::error(_q_init, q);
                boost::shared_ptr<const urdf::Joint> joint = _model.urdf_model->getJoint(feedback->marker_name);
                KDL::Vector vers(joint->axis.x, joint->axis.y, joint->axis.z);
                double dq = e[0]*vers[0]+e[1]*vers[1]+e[2]*vers[2];

                yarp::os::Bottle b_in, b_out;
                b_out.addString("get actual_pose");
                _rpc.write(b_out, b_in);

                double q_ref = b_in.get(_model.iDyn3_model.getDOFIndex(feedback->marker_name)).asDouble() + dq;

                std::map<std::string, double> joint_map;
                joint_map[feedback->marker_name] = q_ref;

                OpenSoT::interfaces::yarp::msgs::yarp_position_joint_msg_portable& joint_position_msg =
                        _port.prepare();
                joint_position_msg.joints = joint_map;
                _port.write();

                break;
        }

        server->applyChanges();
    }

    void make1DofMarker( bool fixed, const tf::StampedTransform& T, bool show_6dof,
                         const boost::shared_ptr<const urdf::Joint>& joint)
    {
      InteractiveMarker int_marker;
      int_marker.header.frame_id = _base_link;
      tf::pointTFToMsg(T.getOrigin(), int_marker.pose.position);
      tf::quaternionTFToMsg(T.getRotation(), int_marker.pose.orientation);
      int_marker.scale = 0.2;

      int_marker.name = joint->name;
      int_marker.description = joint->name;

      // insert a sphere + menu
      _menu_handler.insert( "Reset pose", boost::bind(&SmallSphereMarker1DoF::processFeedback, this, _1) );
      makeSphereControl(int_marker);
      int_marker.controls[0].interaction_mode = InteractiveMarkerControl::MENU;

      InteractiveMarkerControl control;

      if ( fixed )
      {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
      }

      if(show_6dof)
      {
        tf::Vector3 x(1.0, 0.0, 0.0);
        tf::Vector3 v(joint->axis.x, joint->axis.y, joint->axis.z);

        tf::Vector3 rot_axis = v.cross(x);
        rot_axis.normalized();

        tf::Quaternion q(rot_axis, -1.0*acos(v.dot(x)));

        control.orientation.w = q.w();
        control.orientation.x = q.x();
        control.orientation.y = q.y();
        control.orientation.z = q.z();
        control.name = "rotation_axis";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
      }

      server->insert(int_marker);
      server->setCallback(int_marker.name, boost::bind(&SmallSphereMarker1DoF::processFeedback, this, _1));
      _menu_handler.apply( *server, int_marker.name );
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "postural_marker");
    ros::NodeHandle n("~");

    ros::Rate yarp_check_network_rate(1);
    yarp::os::Network yarp_network;
    while(ros::ok())
    {
        if(!yarp_network.checkNetwork()){
        ROS_WARN("Yarp Network not available... run a yarpserver...");
        yarp_check_network_rate.sleep();}
        else
            break;
    }

    std::string urdf_path;
    n.getParam("urdf_path", urdf_path);
    if(urdf_path.empty()){
        ROS_ERROR("urdf_path param not provided!");
        return 0;}

    std::string srdf_path;
    n.getParam("srdf_path", srdf_path);
    if(srdf_path.empty()){
        ROS_ERROR("srdf_path param not provided!");
        return 0;}

    std::string port_name;
    n.getParam("port_name", port_name);
    if(port_name.empty()){
        ROS_ERROR("port_name param not provided!");
        return 0;}

    XmlRpc::XmlRpcValue joint_names;
    n.param("joint_names", joint_names, joint_names);
    if(joint_names.getType() == XmlRpc::XmlRpcValue::TypeInvalid){
        ROS_ERROR("NO joint_names provided!");
        return 0;}

    std::vector<std::string> joint_names_;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        joint_names_.push_back(std::string(joint_names[i]));

    std::string node_name = ros::this_node::getName();
    iDynUtils model("robot", urdf_path, srdf_path);

    SmallSphereMarker1DoF marker(model, joint_names_, node_name, port_name, yarp_network);

    double hz;
    n.param("rate", hz, 50.0);
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        marker.update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    marker.server.reset();
}

