#ifndef __INTERFACES_YARP_POSE_MSG_H__
#define __INTERFACES_YARP_POSE_MSG_H__

#include <kdl/frames.hpp>
#include <string>
#include <yarp/os/all.h>
#include <drc_shared/cartesian_utils.h>

namespace OpenSoT {
    namespace interfaces {
        namespace yarp {
            namespace msgs{

            /**
             * @brief The yarp_pose_msg class define an Homogeneous Transformation from base_link to distal_link expressed in base_link
             */
            class yarp_pose_msg
            {
            public:
                /**
                 * @brief yarp_pose_msg set unknown base_frame, unknown distal_frame and pose to Identity
                 */
                yarp_pose_msg():
                pose(),
                base_frame(""),
                distal_frame("")
                {
                    pose = pose.Identity();
                }

                yarp_pose_msg(const KDL::Frame& p, const std::string& base, const std::string& distal):
                pose(p),
                base_frame(base),
                distal_frame(distal)
                {

                }

                /**
                 * @brief pose from base_link to distal_link expressed in base_link uses SI ()
                 */
                KDL::Frame pose;

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
                static const unsigned int bottle_msg_size = 9;
                /**
                 * @brief serializePoseMsg push in a bottle data contained in a KDL::Frame. Data are serialized as:
                 * [x,y,z] + [qx,qy,qz,qw] + base_frame + distal_frame
                 * @param tmp_b bottle
                 */
                void serializeMsg(::yarp::os::Bottle& tmp_b)
                {
                    //1) pose
                    tmp_b.addDouble(pose.p.x());
                    tmp_b.addDouble(pose.p.y());
                    tmp_b.addDouble(pose.p.z());

                    //2) quaternion
                    double qx, qy, qz, qw;
                    pose.M.GetQuaternion(qx, qy, qz, qw);
                    tmp_b.addDouble(qx);
                    tmp_b.addDouble(qy);
                    tmp_b.addDouble(qz);
                    tmp_b.addDouble(qw);

                    //3) base_frame & distal_frame
                    tmp_b.addString(base_frame);
                    tmp_b.addString(distal_frame);
                }

                /**
                 * @brief deserializePoseMsg write on the KDL::Frame the data contained in the bottle
                 * @param tmp_b bottle
                 * @param i index from which data are started to read
                 */
                void deserializeMsg(::yarp::os::Bottle& tmp_b, unsigned int i = 0)
                {
                    double qx,qy,qz,qw;

                    //1) pose
                    pose.p[0] = tmp_b.get(i+0).asDouble();
                    pose.p[1] = tmp_b.get(i+1).asDouble();
                    pose.p[2] = tmp_b.get(i+2).asDouble();

                    //2)quaternion
                    qx = tmp_b.get(i+3).asDouble();
                    qy = tmp_b.get(i+4).asDouble();
                    qz = tmp_b.get(i+5).asDouble();
                    qw = tmp_b.get(i+6).asDouble();
                    pose.M = pose.M.Quaternion(qx, qy, qz, qw);

                    //3) base_frame & distal_frame
                    base_frame = tmp_b.get(i+7).asString().c_str();
                    distal_frame = tmp_b.get(i+8).asString().c_str();
                }

            };

            /**
             * @brief The yarp_pose_msg_portable class implement a portable from yarp_pose_msg
             */
            class yarp_pose_msg_portable: public yarp_pose_msg, public ::yarp::os::Portable
            {
            public:
                yarp_pose_msg_portable():
                yarp_pose_msg()
                {

                }

                yarp_pose_msg_portable(const KDL::Frame& p, const std::string& base, const std::string& distal):
                yarp_pose_msg(p, base, distal)
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
