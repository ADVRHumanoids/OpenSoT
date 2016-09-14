#ifndef __INTERFACES_YARP_TRJ_GENERATOR_MSG_H__
#define __INTERFACES_YARP_TRJ_GENERATOR_MSG_H__

#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_pose_msg.h>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            class yarp_trj_generator_msg
            {
            public:
                
                yarp_trj_generator_msg() {}

                // Line and Min Jerk trajectory
                yarp_trj_generator_msg(std::string traj_type, 
                                       std::vector<yarp_pose_msg> way_points,
                                       std::vector<double> sub_traj_time ) :
                    traj_type(traj_type),
                    way_points(way_points),
                    sub_traj_time(sub_traj_time),
                    way_points_num(way_points.size())
                {
                    bottle_msg_size = 1 + 
                                      1 + 
                                      way_points.size() * yarp_pose_msg::bottle_msg_size + 
                                      sub_traj_time.size();
                }

                // Arc trajectory
                yarp_trj_generator_msg(std::string traj_type, 
                                       yarp_pose_msg start_pose,
                                       yarp_pose_msg final_rotation,
                                       double angle_of_rotation,
                                       yarp_pose_msg circle_center,
                                       yarp_pose_msg plane_normal,
                                       double arc_traj_time ) :
                    traj_type(traj_type),
                    start_pose(start_pose),
                    final_rotation(final_rotation),
                    angle_of_rotation(angle_of_rotation),
                    circle_center(circle_center),
                    plane_normal(plane_normal),
                    arc_traj_time(arc_traj_time)
                {
                    bottle_msg_size = 1 +
                                      4 * yarp_pose_msg::bottle_msg_size + 
                                      2;
                }
                
                // trajectory type
                std::string traj_type;
                
                // Line and Min Jerk trajectory
                int way_points_num;
                std::vector<yarp_pose_msg> way_points;
                std::vector<double> sub_traj_time; 
                
                // Arc trajectory
                yarp_pose_msg start_pose;
                yarp_pose_msg final_rotation;
                double angle_of_rotation;
                yarp_pose_msg circle_center;
                yarp_pose_msg plane_normal;
                double arc_traj_time;
                
                // size of the bottle changes based on the traj type
                int bottle_msg_size;
                
                
                void serializeMsg(::yarp::os::Bottle& tmp_b)
                {
                    tmp_b.addString(traj_type);
                    if(traj_type == "LineTrj" || traj_type == "MinJerkTrj") {
                        tmp_b.addInt(way_points_num);
                        for(int i = 0; i < way_points_num; i++) {
                            way_points[i].serializeMsg(tmp_b);
                            tmp_b.addDouble(sub_traj_time[i]);
                        }
                    }
                    else if(traj_type == "ArcTrj") {
                        start_pose.serializeMsg(tmp_b);
                        final_rotation.serializeMsg(tmp_b);
                        tmp_b.addDouble(angle_of_rotation);
                        circle_center.serializeMsg(tmp_b);
                        plane_normal.serializeMsg(tmp_b);
                        tmp_b.addDouble(arc_traj_time);
                    }                    
                }
                
                void deserializeMsg(::yarp::os::Bottle& tmp_b, unsigned int cont = 0)
                {

                    // traj_type
                    traj_type = tmp_b.get(cont++).asString();
                    
                    // LINEAR or MIN JERK TRAJECTORY
                    if(traj_type == "LineTrj" || traj_type == "MinJerkTrj") {
                        way_points_num = tmp_b.get(cont++).asInt();
                        // resize proper arrays
                        way_points.resize(way_points_num);
                        sub_traj_time.resize(way_points_num);
                        
                        for(int i = 0; i < way_points_num; i++) {
                            way_points[i].deserializeMsg(tmp_b, cont);
                            cont += way_points[i].bottle_msg_size;
                            sub_traj_time[i] = tmp_b.get(cont++).asDouble();
                        }
                        // updated bottle msg size
                        bottle_msg_size =   1 + 
                                            1 + 
                                            way_points.size() * yarp_pose_msg::bottle_msg_size + 
                                            sub_traj_time.size();                        
                    }
                    else if(traj_type == "ArcTrj") {
                                      
                        start_pose.deserializeMsg(tmp_b, cont);
                        cont += start_pose.bottle_msg_size;
                        
                        final_rotation.deserializeMsg(tmp_b, cont);
                        cont += final_rotation.bottle_msg_size;
                        
                        angle_of_rotation = tmp_b.get(cont++).asDouble();
                        
                        circle_center.deserializeMsg(tmp_b, cont);
                        cont += circle_center.bottle_msg_size;
                        
                        plane_normal.deserializeMsg(tmp_b, cont);
                        cont += plane_normal.bottle_msg_size;
                        
                        arc_traj_time = tmp_b.get(cont++).asDouble();
                        
                        // updated bottle msg size
                        bottle_msg_size =   1 +
                                            4 * yarp_pose_msg::bottle_msg_size + 
                                            2;                        
                    }

                    
                }

            };

            class yarp_trj_generator_msg_portable : public yarp_trj_generator_msg, 
                                                    public ::yarp::os::Portable
            {
            public:
                
                yarp_trj_generator_msg_portable():
                    yarp_trj_generator_msg()
                {

                }

                yarp_trj_generator_msg_portable(std::string traj_type, 
                                                std::vector<yarp_pose_msg> way_points,
                                                std::vector<double> sub_traj_time ) :
                    yarp_trj_generator_msg(traj_type, way_points, sub_traj_time)
                {

                }
                
                yarp_trj_generator_msg_portable(std::string traj_type, 
                                                yarp_pose_msg start_pose,
                                                yarp_pose_msg final_rotation,
                                                double angle_of_rotation,
                                                yarp_pose_msg circle_center,
                                                yarp_pose_msg plane_normal,
                                                double arc_traj_time ) :
                    yarp_trj_generator_msg(traj_type, start_pose, final_rotation,
                                           angle_of_rotation, circle_center, plane_normal, arc_traj_time)
                {

                }

                bool read(::yarp::os::ConnectionReader& connection)
                {
                    ::yarp::os::Bottle tmp_bot;
                    if(!tmp_bot.read(connection))
                        return false;

                    deserializeMsg(tmp_bot);
                    return true;
                }

                bool write(::yarp::os::ConnectionWriter& connection)
                {
                    ::yarp::os::Bottle tmp_bot;
                    serializeMsg(tmp_bot);

                    if(tmp_bot.write(connection))
                        return true;
                    return false;
                }
            };

            }
        }
    }
}

#endif //__INTERFACES_YARP_TRJ_GENERATOR_MSG_H__
