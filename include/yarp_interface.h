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
#define LOCAL_FRAME_SWING_FOOT "l_sole"

class yarp_interface
{
public:
    yarp_interface();
    ~yarp_interface();

    void getLeftArmCartesianRef(yarp::sig::Matrix& left_arm_ref);
    void getRightArmCartesianRef(yarp::sig::Matrix& right_arm_ref);
    void getCoMCartesianRef(yarp::sig::Vector& com_ref);
    void getSwingFootCartesianRef(yarp::sig::Matrix& swing_foot_ref);
    void tic();
    double toc();

    /**
     * @brief sendCH sends a list of points through the CH port.
     * @param A_ch a Matrix of coefficients for the CH rects, contains a_i,b_i
     * @param b_ch a vector of coefficients for the CH rects, containts -c_i
     */
    void sendCH(std::vector<KDL::Vector> ch);

    void sendWorldToBaseLinkPose(const yarp::sig::Matrix& T_world_base_link)
    {
        //Here the reference frame is the same that in updateIdyn3Model !!!
        sendCartesianRef(world_to_base_link_pose_port, "l_sole", T_world_base_link);
    }

    void cleanPorts();


    walkman::drc::yarp_single_chain_interface left_arm,right_arm,left_leg,right_leg,torso;
    
    /// Remember every time you put a port here to clean it as done in function cleanPorts()!!!
    yarp::os::BufferedPort<yarp::os::Bottle> right_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> left_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> com_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> world_to_base_link_pose_port;
    yarp::os::BufferedPort<yarp::os::Bottle> com_to_ch_pos_port;
    yarp::os::BufferedPort<yarp::os::Bottle> swing_foot_pos_ref_port;
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
