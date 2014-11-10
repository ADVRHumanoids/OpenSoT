#ifndef __INTERFACES_YARP_TRJ_MSG_H__
#define __INTERFACES_YARP_TRJ_MSG_H__

#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_pose_msg.h>
#include <OpenSoT/interfaces/yarp/yarp_msgs/yarp_twist_msg.h>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            class yarp_trj_msg: public yarp_pose_msg, public yarp_twist_msg
            {
            public:
                yarp_trj_msg(const KDL::Frame& p, const KDL::Twist& v, const std::string& base, const std::string& distal):
                yarp_pose_msg(p, base, distal),
                yarp_twist_msg()
                {
                    twist = v;
                }


                yarp_trj_msg(const KDL::Frame& p, const std::string& base, const std::string& distal):
                yarp_pose_msg(p, base, distal),
                yarp_twist_msg()
                {

                }

                yarp_trj_msg():
                yarp_pose_msg(),
                yarp_twist_msg()
                {

                }

                using yarp_pose_msg::base_frame;
                using yarp_pose_msg::distal_frame;

                static const int bottle_msg_size = yarp_pose_msg::bottle_msg_size + yarp_twist_msg::bottle_msg_size;

            };

            class yarp_trj_msg_portable : public yarp_trj_msg, public ::yarp::os::Portable
            {
            public:
                yarp_trj_msg_portable():
                    yarp_trj_msg()
                {

                }

                yarp_trj_msg_portable(const KDL::Frame& p, const std::string& base, const std::string& distal):
                    yarp_trj_msg(p, base, distal)
                {

                }

                yarp_trj_msg_portable(const KDL::Frame& p, const KDL::Twist& v, const std::string& base, const std::string& distal):
                    yarp_trj_msg(p, v, base, distal)
                {

                }

                bool read(::yarp::os::ConnectionReader& connection)
                {
                    ::yarp::os::Bottle tmp_bot;
                    if(!tmp_bot.read(connection))
                        return false;

                    /**
                     * If a pose_msg is recived then the twist is automatically set to zero
                     */
                    yarp_pose_msg::deserializeMsg(tmp_bot);
                    if(!(tmp_bot.size() == yarp_pose_msg::bottle_msg_size))
                        yarp_twist_msg::deserializeMsg(tmp_bot, yarp_pose_msg::bottle_msg_size);
                    else
                        twist = KDL::Twist::Zero();

                    return true;
                }

                bool write(::yarp::os::ConnectionWriter& connection)
                {
                    ::yarp::os::Bottle tmp_bot;
                    yarp_pose_msg::serializeMsg(tmp_bot);
                    yarp_twist_msg::serializeMsg(tmp_bot);

                    if(tmp_bot.write(connection))
                        return true;
                    return false;
                }
            };

            }
        }
    }
}

#endif
