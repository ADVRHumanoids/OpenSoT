#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <boost/phoenix/bind/bind_member_function.hpp>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_pose_msg.h>
#include <yarp/os/BufferedPort.h>

using namespace visualization_msgs;
/**
 * @brief The SPhereMarker6DoF class implements a 6DoF interactive marker with a menu for reset the position of
 * the marker. The marker works from base_link to distal_link sending position references to a port opened by a
 * YCartesian interface.
 */
class SphereMarker6DoF
{
public:
    SphereMarker6DoF(const std::string& base_link, const std::string& distal_link, const std::string& server_name,
                     const std::string& port_name, yarp::os::Network& yarp_network):
        _base_link(base_link),
        _distal_link(distal_link),
        server( new interactive_markers::InteractiveMarkerServer(server_name, server_name+"_id") ),
        _yarp_network(yarp_network)
    {
        _rpc.open("/"+server_name+"/rpc");
        yarp_network.connect("/"+server_name+"/rpc", port_name+"/rpc");
        yarp::os::Bottle b_in, b_out;
        b_out.addString("get actual_pose");
        _rpc.write(b_out, b_in);

        tf::StampedTransform base_link_T_distal_link_0 = fromBottleToTFStampedTransform(b_in);

        ROS_INFO("Creating Cartesian Interactive Marker Server for:\n    base_link: %s\n    distal_link: %s",
                 base_link.c_str(), distal_link.c_str());

        make6DofMarker(false, base_link_T_distal_link_0, base_link, distal_link, true);

        _port.open(server_name);
        yarp_network.connect(_port.getName(), port_name+"/set_ref:i");

        server->applyChanges();
    }

    tf::StampedTransform fromBottleToTFStampedTransform(const yarp::os::Bottle& bot)
    {
        yarp::sig::Matrix T(4,4); T.eye();
        for(unsigned int i = 0; i < 4; ++i)
        {
            T(0,i) = bot.get(i).asDouble();
            T(1,i) = bot.get(i+4).asDouble();
            T(2,i) = bot.get(i+8).asDouble();
            T(3,i) = bot.get(i+12).asDouble();
        }
        KDL::Frame T_KDL;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T, T_KDL);

        tf::StampedTransform T_TF;
        T_TF.setOrigin(tf::Vector3(T_KDL.p[0], T_KDL.p[1], T_KDL.p[2]));

        double qx, qy, qz, qw;
        T_KDL.M.GetQuaternion(qx, qy, qz, qw);
        T_TF.setRotation(tf::Quaternion(qx, qy, qz, qw));
        return T_TF;
    }

    ~SphereMarker6DoF()
    {
        _port.close();
        _rpc.close();
    }

private:
    std::string _base_link;
    std::string _distal_link;
public: boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    interactive_markers::MenuHandler _menu_handler;
    yarp::os::BufferedPort<OpenSoT::interfaces::yarp::msgs::yarp_pose_msg_portable> _port;
    yarp::os::Network& _yarp_network;
    yarp::os::RpcClient _rpc;

    Marker makeSphere( InteractiveMarker &msg )
    {
      Marker marker;
      marker.type = Marker::SPHERE;
      marker.scale.x = msg.scale * 0.45;
      marker.scale.y = msg.scale * 0.45;
      marker.scale.z = msg.scale * 0.45;
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
      tf::StampedTransform T;
      geometry_msgs::Pose pose;

      OpenSoT::interfaces::yarp::msgs::yarp_pose_msg_portable& pose_msg = _port.prepare();
      std::string tmp = _base_link; tmp.erase(0,1);
      pose_msg.base_frame = tmp;
      tmp = _distal_link; tmp.erase(0,1);
      pose_msg.distal_frame = tmp;

      switch ( feedback->event_type )
      {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
          if(feedback->menu_entry_id){
              yarp::os::Bottle b_in, b_out;
              b_out.addString("get actual_pose");
              _rpc.write(b_out, b_in);

              T = fromBottleToTFStampedTransform(b_in);
              tf::poseTFToMsg(T, pose);
              server->setPose(_distal_link, pose);
//              pose_msg.pose.M = KDL::Rotation::Quaternion(T.getRotation().x(), T.getRotation().y(),T.getRotation().z(),T.getRotation().w());
//              pose_msg.pose.p = KDL::Vector(T.getOrigin().x(), T.getOrigin().y(), T.getOrigin().z());
//              _port.write();
          }
          break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
          tf::poseMsgToTF(feedback->pose, T);
          pose_msg.pose.M = KDL::Rotation::Quaternion(T.getRotation().x(), T.getRotation().y(),T.getRotation().z(),T.getRotation().w());
          pose_msg.pose.p = KDL::Vector(T.getOrigin().x(), T.getOrigin().y(), T.getOrigin().z());
          cartesian_utils::printKDLFrame(pose_msg.pose); std::cout<<std::endl;
          _port.write();
          break;
      }

      server->applyChanges();
    }

    void make6DofMarker( bool fixed, const tf::StampedTransform& T, const std::string& base_link,  const std::string& distal_link, bool show_6dof )
    {
      InteractiveMarker int_marker;
      int_marker.header.frame_id = base_link;
      tf::pointTFToMsg(T.getOrigin(), int_marker.pose.position);
      tf::quaternionTFToMsg(T.getRotation(), int_marker.pose.orientation);
      int_marker.scale = 0.25;

      int_marker.name = distal_link;
      int_marker.description = distal_link;

      // insert a sphere + menu
      _menu_handler.insert( "Reset pose", boost::bind(&SphereMarker6DoF::processFeedback, this, _1) );
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
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
      }

      server->insert(int_marker);
      server->setCallback(int_marker.name, boost::bind(&SphereMarker6DoF::processFeedback, this, _1));
      _menu_handler.apply( *server, int_marker.name );
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Cartesian_marker");
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

    std::string base_link;
    n.getParam("base_link", base_link);
    if(base_link.empty()){
        ROS_ERROR("base_link param not provided!");
        return 0;}

    std::string distal_link;
    n.getParam("distal_link", distal_link);
    if(distal_link.empty()){
        ROS_ERROR("distal_link param not provided!");
        return 0;}

    std::string port_name;
    n.getParam("port_name", port_name);
    if(port_name.empty()){
        ROS_ERROR("port_name param not provided!");
        return 0;}

    std::string node_name = ros::this_node::getName();
    SphereMarker6DoF marker(base_link, distal_link, node_name, port_name, yarp_network);

    ros::spin();

    marker.server.reset();
}
