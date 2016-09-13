#ifndef __INTERFACES_YARP_TASK_TRJ_MSG_H__
#define __INTERFACES_YARP_TASK_TRJ_MSG_H__

#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_trj_generator_msg.h>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            class yarp_task_trj_msg
            {
            public:
                
                yarp_task_trj_msg() {}

                // Line and Min Jerk trajectory
                yarp_task_trj_msg(std::string task_id, 
                                  std::vector<yarp_trj_generator_msg> trajs ) :
                    task_id(task_id),
                    trajs(trajs),
                    traj_num(trajs.size())
                {
                    bottle_msg_size = 2;
                    for(int i = 0; i < traj_num; i++) {
                        bottle_msg_size += trajs[i].bottle_msg_size;
                    }
                }

                // Task ID
                std::string task_id;
                // number of traj in the composite task traj
                int traj_num;
                // vector of traj
                std::vector<yarp_trj_generator_msg> trajs;
                
                // size of the bottle 
                int bottle_msg_size;
                
                
                void serializeMsg(::yarp::os::Bottle& tmp_b)
                {
                    tmp_b.addString(task_id);
                    tmp_b.addInt(traj_num);
                    for(int i = 0; i < traj_num; i++) {
                        trajs[i].serializeMsg(tmp_b);
                    }
                }
                
                void deserializeMsg(::yarp::os::Bottle& tmp_b, unsigned int cont = 0)
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
