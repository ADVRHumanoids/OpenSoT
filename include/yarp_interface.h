#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/iDynTree/DynTree.h>
#include "sot_VelKinCon_constants.h"
#include <ros/ros.h>
#include <drc_shared/yarp_single_chain_interface.h>
#include <drc_shared/cartesian_utils.h>

#define LOCAL_FRAME_UPPER_BODY "world"
#define LOCAL_FRAME_LOWER_BODY "world"
#define LOCAL_FRAME_COM "l_sole"

class yarp_interface
{
public:
    yarp_interface();
    ~yarp_interface();

    void getLeftArmCartesianRef(yarp::sig::Matrix& left_arm_ref);
    void getRightArmCartesianRef(yarp::sig::Matrix& right_arm_ref);
    void getCoMCartesianRef(yarp::sig::Vector& com_ref);
    void tic();
    double toc();

    void sendWorldToBaseLinkPose(const yarp::sig::Matrix& T_world_base_link)
    {
        //Here the reference frame is the same that in updateIdyn3Model !!!
        sendCartesianRef(world_to_base_link_pose_port, "r_sole", T_world_base_link);
    }

    void sendLSoleToCoMPose(const yarp::sig::Vector& p_l_sole_CoM)
    {
        yarp::sig::Matrix T(4,4);
        T.eye();
        T(0,3) = p_l_sole_CoM[0];
        T(1,3) = p_l_sole_CoM[1];
        T(2,3) = p_l_sole_CoM[2];
        sendCartesianRef(l_sole_to_CoM_pose_port, "l_sole", T);
    }

    void cleanPorts();


    walkman::drc::yarp_single_chain_interface left_arm,right_arm,left_leg,right_leg,torso;
    
    /// Remember every time you put a port here to clean it as done in function cleanPorts()!!!
    yarp::os::BufferedPort<yarp::os::Bottle> right_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> left_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> com_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> world_to_base_link_pose_port;
    yarp::os::BufferedPort<yarp::os::Bottle> l_sole_to_CoM_pose_port;
    ///



private:
    double time_tic;
    bool checkRefFrame(const std::string& ref_frame, const std::string& local_frame)
    {
        if(ref_frame.compare(local_frame.c_str()) == 0)
            return true;
        return false;
    }
    bool getCartesianRef(yarp::sig::Matrix &ref, yarp::os::Bottle *bot, const std::string& local_frame);
    bool sendCartesianRef(yarp::os::BufferedPort<yarp::os::Bottle>& port, const std::string& ref_frame,
                            const yarp::sig::Matrix& T);
};

#endif
