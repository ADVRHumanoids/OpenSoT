#ifndef __INTERFACES_YARP_TWIST_MSG_H__
#define __INTERFACES_YARP_TWIST_MSG_H__

#include <kdl/frames.hpp>
#include <string>
#include <yarp/os/all.h>
#include <drc_shared/cartesian_utils.h>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            /**
             * @brief The yarp_twist_msg class define a twist from base_link to distal_link expressed in base_link
             */
            class yarp_twist_msg
            {
            public:
                /**
                 * @brief yarp_twist_msg set unknown base_frame, unknown distal_frame and twist to Zero
                 */
                yarp_twist_msg():
                twist(),
                base_frame(""),
                distal_frame("")
                {
                    twist = twist.Zero();
                }

                yarp_twist_msg(const KDL::Twist& v, const std::string& base, const std::string& distal):
                twist(v),
                base_frame(base),
                distal_frame(distal)
                {

                }

                /**
                 * @brief twist of distal_link wrt base_link expressed in base_link uses SI ()
                 */
                KDL::Twist twist;

                /**
                 * @brief base_frame of reference
                 */
                std::string base_frame;

                /**
                 * @brief distal_frame of reference
                 */
                std::string distal_frame;

                /**
                 * @brief bottle_msg_size contains the lenght of the bottle created by the serializePoseMsg.
                 */
                static const unsigned int bottle_msg_size = 8;

                /**
                 * @brief serializeMsg push in a bottle data contained in a KDL::Twist. Data are serialized as:
                 * [dx,dy,dz] + [wx,wy,wz] + base_frame + distal_frame
                 * @param tmp_b bottle
                 */
                void serializeMsg(::yarp::os::Bottle& tmp_b)
                {
                    //1) linear velocity
                    tmp_b.addDouble(twist.vel.x());
                    tmp_b.addDouble(twist.vel.y());
                    tmp_b.addDouble(twist.vel.z());

                    //2) angular velocity
                    tmp_b.addDouble(twist.rot.x());
                    tmp_b.addDouble(twist.rot.y());
                    tmp_b.addDouble(twist.rot.z());

                    //3) base_frame & distal_frame
                    tmp_b.addString(base_frame);
                    tmp_b.addString(distal_frame);
                }

                /**
                 * @brief deserializeMsg write on the KDL::Twist the data contained in the bottle
                 * @param tmp_b bottle
                 * @param i index from which data are started to read
                 */
                void deserializeMsg(::yarp::os::Bottle& tmp_b, unsigned int i = 0)
                {
                    //1) linear velocity
                    twist.vel[0] = tmp_b.get(i+0).asDouble();
                    twist.vel[1] = tmp_b.get(i+1).asDouble();
                    twist.vel[2] = tmp_b.get(i+2).asDouble();

                    //2) angular velocity
                    twist.rot[0] = tmp_b.get(i+3).asDouble();
                    twist.rot[1] = tmp_b.get(i+4).asDouble();
                    twist.rot[2] = tmp_b.get(i+5).asDouble();

                    //3) base_frame & distal_frame
                    base_frame = tmp_b.get(i+6).asString().c_str();
                    distal_frame = tmp_b.get(i+7).asString().c_str();
                }

            };

            /**
             * @brief The yarp_twist_msg_portable class implement a portable from yarp_twist_msg
             */
            class yarp_twist_msg_portable: public yarp_twist_msg, public ::yarp::os::Portable
            {
            public:
                yarp_twist_msg_portable():
                yarp_twist_msg()
                {

                }

                yarp_twist_msg_portable(const KDL::Twist& v, const std::string& base, const std::string& distal):
                yarp_twist_msg(v, base, distal)
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

#endif
