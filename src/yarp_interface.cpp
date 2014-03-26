#include "yarp_interface.h"
#include "cartesian_utils.h"

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
        polyDriver_left_leg.view(directControl_left_leg);
        polyDriver_left_leg.view(controlMode_left_leg);
    }
    if(createPolyDriver("right_leg", polyDriver_right_leg))
    {
        polyDriver_right_leg.view(encodersMotor_right_leg);
        polyDriver_right_leg.view(directControl_right_leg);
        polyDriver_right_leg.view(controlMode_right_leg);
    }
    if(createPolyDriver("torso", polyDriver_torso))
    {
        polyDriver_torso.view(encodersMotor_torso);
        polyDriver_torso.view(directControl_torso);
        polyDriver_torso.view(controlMode_torso);
    }

    left_arm_pos_ref_port.open("/sot_VelKinCon/left_arm/set_ref:i");
    right_arm_pos_ref_port.open("/sot_VelKinCon/right_arm/set_ref:i");
    clik_port.open("/sot_VelKinCon/set_clik:i");
}

yarp_interface::~yarp_interface()
{
    left_arm_pos_ref_port.close();
    right_arm_pos_ref_port.close();

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

void yarp_interface::getLeftArmCartesianRef(Matrix &left_arm_ref)
{
    yarp::os::Bottle *bot = left_arm_pos_ref_port.read(false);

    if(!bot == NULL)
    {
        std::cout<<"Left Arm:"<<std::endl;
        getArmCartesianRef(left_arm_ref, bot);
        std::cout<<std::endl;
    }
}

void yarp_interface::getRightArmCartesianRef(Matrix &right_arm_ref)
{
    yarp::os::Bottle *bot = right_arm_pos_ref_port.read(false);

    if(!bot == NULL)
    {
        std::cout<<"Right Arm:"<<std::endl;
        getArmCartesianRef(right_arm_ref, bot);
        std::cout<<std::endl;
    }
}

void yarp_interface::getSetClik(bool &is_clik)
{
    yarp::os::Bottle *bot = clik_port.read(false);

    if(!bot == NULL){
        is_clik = (bool)bot->get(0).asInt();
        if(is_clik)
            std::cout<<"CLIK activated!"<<std::endl;
        else
            std::cout<<"CLIK NOT activated!"<<std::endl;
    }
}

bool yarp_interface::getArmCartesianRef(Matrix &arm_ref, yarp::os::Bottle *bot)
{
    Bottle& frame = bot->findGroup("frame");
    if(!frame.isNull())
    {
        if(checkRefFrame(frame.get(1).asString()))
        {
            Bottle& data = bot->findGroup("data");
            if(!data.isNull())
            {
                if(data.size() == 6+1)
                {
                    cartesian_utils::homogeneousMatrixFromRPY(arm_ref,
                                                              data.get(1).asDouble(), data.get(2).asDouble(), data.get(3).asDouble(),
                                                              toRad(data.get(4).asDouble()), toRad(data.get(5).asDouble()), toRad(data.get(6).asDouble()));
                    std::cout<<"    New reference in base_link is "<<std::endl;
                    std::cout<<arm_ref.toString()<<std::endl;
                    return true;
                }
                else
                    std::cout<<"    ERROR: wrong size of reference! Should be 6!"<<std::endl;
            }
            else
                std::cout<<"    ERROR: no data section in port"<<std::endl;
        }
        else
            std::cout<<"    ERROR: wrong Reference Frame, should be base_link!"<<std::endl;
    }
    else
        std::cout<<"    ERROR: no frame section in port"<<std::endl;
    return false;
}

