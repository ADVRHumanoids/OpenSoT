#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/iDynTree/DynTree.h>
#include "sot_VelKinCon_constants.h"
#include <ros/ros.h>

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
    void getSetClik(bool& is_clik);
    void tic();
    double toc();

    int getDofLeftArm()
    {
        int dofs = 0;
        encodersMotor_left_arm->getAxes(&dofs);
        return dofs;
    }
    int getDofRightArm()
    {
        int dofs = 0;
        encodersMotor_right_arm->getAxes(&dofs);
        return dofs;
    }
    int getDofRightLeg()
    {
        int dofs = 0;
        encodersMotor_right_leg->getAxes(&dofs);
        return dofs;
    }
    int getDofLeftLeg()
    {
        int dofs = 0;
        encodersMotor_left_leg->getAxes(&dofs);
        return dofs;
    }
    int getDofTorso()
    {
        int dofs = 0;
        encodersMotor_torso->getAxes(&dofs);
        return dofs;
    }

    void sendWorldToBaseLinkPose(const yarp::sig::Matrix& T_world_base_link)
    {
        //Here the reference frame is the same that in updateIdyn3Model !!!
        sendCartesianRef(world_to_base_link_pose_port, "r_sole", T_world_base_link);
    }

    void cleanPorts();


    yarp::dev::PolyDriver polyDriver_left_arm;
    yarp::dev::PolyDriver polyDriver_right_arm;
    yarp::dev::PolyDriver polyDriver_left_leg;
    yarp::dev::PolyDriver polyDriver_right_leg;
    yarp::dev::PolyDriver polyDriver_torso;

    yarp::dev::IEncodersTimed *encodersMotor_left_arm;
    yarp::dev::IEncodersTimed *encodersMotor_right_arm;
    yarp::dev::IEncodersTimed *encodersMotor_left_leg;
    yarp::dev::IEncodersTimed *encodersMotor_right_leg;
    yarp::dev::IEncodersTimed *encodersMotor_torso;

    yarp::dev::IControlMode *controlMode_left_arm;
    yarp::dev::IControlMode *controlMode_right_arm;
    yarp::dev::IControlMode *controlMode_torso;
    yarp::dev::IControlMode *controlMode_left_leg;
    yarp::dev::IControlMode *controlMode_right_leg;

    yarp::dev::IPositionDirect *directControl_left_arm;
    yarp::dev::IPositionDirect *directControl_right_arm;
    yarp::dev::IPositionDirect *directControl_torso;
    yarp::dev::IPositionDirect *directControl_left_leg;
    yarp::dev::IPositionDirect *directControl_right_leg;

    yarp::dev::IImpedanceControl *impedanceCtrl_left_arm;
    yarp::dev::IImpedanceControl *impedanceCtrl_right_arm;
    yarp::dev::IImpedanceControl *impedanceCtrl_torso;

    /// Remember every time you put a port here to clean it as done in function cleanPorts()!!!
    yarp::os::BufferedPort<yarp::os::Bottle> right_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> left_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> com_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> clik_port;
    yarp::os::BufferedPort<yarp::os::Bottle> world_to_base_link_pose_port;
    ///



private:
    double time_tic;
    bool createPolyDriver(const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver);
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
