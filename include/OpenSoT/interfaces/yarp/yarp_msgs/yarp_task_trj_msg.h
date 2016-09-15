#ifndef __INTERFACES_YARP_TASK_TRJ_MSG_H__
#define __INTERFACES_YARP_TASK_TRJ_MSG_H__

#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_trj_generator_msg.h>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_pose_msg.h>

#include <vector>
#include <kdl/frames.hpp>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            class yarp_task_trj_msg
            {
            private:
                // Task ID
                std::string task_id;
                // number of /*traj*/ in the composite task traj
                int traj_num;
                // vector of traj
                std::vector<yarp_trj_generator_msg> trajs;
                
                // base link
                std::string base_link;
                // distal link
                std::string distal_link;
                
                // size of the bottle 
                int bottle_msg_size;
                
                void addTrj(const yarp_trj_generator_msg& trj) 
                {
                    trajs.push_back(trj);
                    traj_num = trajs.size();
                    bottle_msg_size += trj.bottle_msg_size;
                }
                
                
                bool addLineOrMinJerkTrj(const std::string& trj_type,
                                         const std::vector<KDL::Frame>& way_points, 
                                         const std::vector<double> Ts ) 
                {
                    std::vector<yarp_pose_msg> pose_vector;
                    for(int i = 0; i < way_points.size(); i++) {
                        yarp_pose_msg pose(way_points[i],
                                           base_link,
                                           distal_link);
                        pose_vector.push_back(pose);
                    }
                    
                    yarp_trj_generator_msg trj(trj_type, 
                                               pose_vector,
                                               Ts);
                    addTrj(trj);
                    return true;
                }
                
            public:
                
                yarp_task_trj_msg() 
                {
                    // empty trajs
                    trajs.clear();
                    traj_num = 0;
                    bottle_msg_size = 2;
                }

                // Line and Min Jerk trajectory
                yarp_task_trj_msg(const std::string& task_id, 
                                  const std::vector<yarp_trj_generator_msg>& trajs ) :
                    task_id(task_id),
                    trajs(trajs),
                    traj_num(trajs.size())
                {
                    bottle_msg_size = 2;
                    for(int i = 0; i < traj_num; i++) {
                        bottle_msg_size += trajs[i].bottle_msg_size;
                    }
                }
                
                
                int get_bottle_msg_size()
                {
                    return bottle_msg_size;
                }
                
                // NOTE in python is easier
                ::yarp::os::Bottle& serializeMsg(::yarp::os::Bottle& tmp_b)
                {
                    tmp_b.addString(task_id);
                    tmp_b.addInt(traj_num);
                    for(int i = 0; i < traj_num; i++) {
                        trajs[i].serializeMsg(tmp_b);
                    }
                    return tmp_b;
                }
                
                void deserializeMsg(const ::yarp::os::Bottle& tmp_b, unsigned int cont = 0)
                {
                    task_id = tmp_b.get(cont++).asString();
                    traj_num = tmp_b.get(cont++).asInt();
                    // resize array
                    trajs.resize(traj_num);
                    for(int i = 0; i < traj_num; i++) {
                        trajs[i].deserializeMsg(tmp_b, cont);
                        cont += trajs[i].bottle_msg_size;
                    }
                }
                
                const yarp_task_trj_msg& assign( const yarp_task_trj_msg &right )
                {
                    *this = right;
                    return *this; 
                }
                

                
                //***********//
                // Utils API //
                //***********//
                
                yarp_task_trj_msg( const std::string& task_id,
                                   const std::string& base_link,
                                   const std::string& distal_link ) :
                    task_id(task_id),
                    base_link(base_link),
                    distal_link(distal_link)
                {
                    // empty trajs
                    trajs.clear();
                    traj_num = 0;
                    bottle_msg_size = 2;
                }                

                
                bool addLineTrj(const std::vector<KDL::Frame>& way_points, 
                                const std::vector<double> Ts ) 
                {
                    return addLineOrMinJerkTrj("LineTrj", way_points, Ts);
                }
                
                bool addMinJerkTrj(const std::vector<KDL::Frame>& way_points, 
                                   const std::vector<double> Ts ) 
                {
                    return addLineOrMinJerkTrj("MinJerkTrj", way_points, Ts);
                }
                
                bool addArcTrj( const KDL::Frame& start_pose, 
                                const KDL::Rotation& final_rotation,
                                const double angle_of_rotation,
                                const KDL::Vector& circle_center, 
                                const KDL::Vector& plane_normal,
                                const double T ) 
                {
                    KDL::Frame aux_frame;
                    
                    //start pose frame
                    yarp_pose_msg yarp_start_pose(  start_pose,
                                                    base_link,
                                                    distal_link);
                    // final rotation orientation rotation matrix
                    aux_frame = aux_frame.Identity();
                    aux_frame.M = final_rotation;
                    
                    yarp_pose_msg yarp_final_rotation(  aux_frame,
                                                        base_link,
                                                        distal_link);
                    // circle center position vector
                    aux_frame = aux_frame.Identity();
                    aux_frame.p = circle_center;
                    
                    yarp_pose_msg yarp_circle_center(  aux_frame,
                                                       base_link,
                                                       distal_link);
                    // plane normal position vector
                    aux_frame = aux_frame.Identity();
                    aux_frame.p = plane_normal;
                    
                    yarp_pose_msg yarp_plane_normal(  aux_frame,
                                                      base_link,
                                                      distal_link);
                    
                    
                    yarp_trj_generator_msg trj( "ArcTrj",
                                                yarp_start_pose,
                                                yarp_final_rotation,
                                                angle_of_rotation,
                                                yarp_circle_center,
                                                yarp_plane_normal,
                                                T);
                    addTrj(trj);
                    return true;
                }
                
                bool resetTrj()
                {
                    trajs.clear();
                    traj_num = 0;
                    bottle_msg_size = 2;  // TBD do it properly
                }
                
                std::vector<yarp_trj_generator_msg> get_trajs()
                {
                    return trajs;
                }
                
                int get_traj_num()
                {
                    return traj_num;
                }
                
                std::string get_task_id()
                {
                    return task_id;
                }
                
                void set_task_id(std::string task_id)
                {
                    this->task_id = task_id;
                }
                
                std::string get_base_link()
                {
                    return base_link;
                }
                
                void set_base_link(std::string base_link)
                {
                    this-> base_link = base_link;
                }
                
                std::string get_distal_link()
                {
                    return distal_link;
                }
                
                void set_distal_link(std::string distal_link)
                {
                    this->distal_link = distal_link;
                }
                

            };

            class yarp_task_trj_msg_portable : public yarp_task_trj_msg, 
                                               public ::yarp::os::Portable
            {
            public:
                
                yarp_task_trj_msg_portable():
                    yarp_task_trj_msg()
                {

                }

                yarp_task_trj_msg_portable(std::string task_id, 
                                           std::vector<yarp_trj_generator_msg> trajs ) :
                    yarp_task_trj_msg(task_id, trajs)
                {

                }
                
                yarp_task_trj_msg_portable( const std::string& task_id,
                                            const std::string& base_link,
                                            const std::string& distal_link ) :
                    yarp_task_trj_msg(task_id, base_link, distal_link)
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

#endif //__INTERFACES_YARP_TASK_TRJ_MSG_H__
