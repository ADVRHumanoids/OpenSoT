#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

class yarp_interface
{
public:
    yarp_interface();
    ~yarp_interface();


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
    yarp::dev::IPositionDirect *directControl_left_arm;
    yarp::dev::IPositionDirect *directControl_right_arm;
    yarp::dev::IPositionDirect *directControl_torso;

private:
    bool createPolyDriver(const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver);
};

#endif
