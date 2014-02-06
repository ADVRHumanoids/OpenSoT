#include "yarp_interface.h"

using namespace yarp::os;
using namespace yarp::sig;

#define toRad(X) (X*M_PI/180.0)

yarp_interface::yarp_interface()
{
    if(createPolyDriver("left_arm", polyDriver_left_arm))
    {
        polyDriver_left_arm.view(encodersMotor_left_arm);
        polyDriver_left_arm.view(directControl_left_arm);
        polyDriver_left_arm.view(controlMode_left_arm);
    }
    if(createPolyDriver("right_arm", polyDriver_right_arm))
    {
        polyDriver_right_arm.view(encodersMotor_right_arm);
        polyDriver_right_arm.view(directControl_right_arm);
        polyDriver_right_arm.view(controlMode_right_arm);
    }
    if(createPolyDriver("left_leg", polyDriver_left_leg))
    {
        polyDriver_left_leg.view(encodersMotor_left_leg);
    }
    if(createPolyDriver("right_leg", polyDriver_right_leg))
    {
        polyDriver_right_leg.view(encodersMotor_right_leg);
    }
    if(createPolyDriver("torso", polyDriver_torso))
    {
        polyDriver_torso.view(encodersMotor_torso);
        polyDriver_right_arm.view(directControl_torso);
        polyDriver_left_arm.view(controlMode_torso);
    }
}

yarp_interface::~yarp_interface()
{
    polyDriver_left_arm.close();
    polyDriver_right_arm.close();
    polyDriver_left_leg.close();
    polyDriver_right_leg.close();
    polyDriver_torso.close();
}

bool yarp_interface::createPolyDriver(const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/sot_VelKinCon/coman/" + kinematic_chain;
    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/coman/" + kinematic_chain;
    options.put("remote", ss.c_str());

    polyDriver.open(options);
    if (!polyDriver.isValid()){
        std::cout<<"Device "<<kinematic_chain<<" not available."<<std::endl;
        return false;
    }
    else{
        std::cout<<"Device "<<kinematic_chain<<" available."<<std::endl;
        return true;
    }
}
