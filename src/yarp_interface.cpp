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

void yarp_interface::getLeftArmCartesianRef(Vector &left_arm_ref, const iCub::iDynTree::DynTree &coman_idyn3)
{
    yarp::os::Bottle *bot = left_arm_pos_ref_port.read(false);

    if(!bot == NULL)
    {
        Bottle& frame = bot->findGroup("frame");
        if(!frame.isNull())
        {
            std::string ref_frame = frame.get(1).asString();
            //if(!ref_frame.compare("base_link"))
                //transformRefFrame();

            Bottle& data = bot->findGroup("data");
            if(!data.isNull())
            {
                if(data.size() == left_arm_ref.size()+1)
                {
                    for(unsigned int i = 1; i < left_arm_ref.size()+1; ++i)
                        left_arm_ref[i-1] = data.get(i).asDouble();
                    std::cout<<"New reference for left_arm is "<<left_arm_ref.toString()<<"in base_link"<<std::endl;
                }
                else
                    std::cout<<"Wrong size of reference! Should be "<<left_arm_ref.size()<<std::endl;
            }
            else
                std::cout << "error: no data section" << std::endl;
        }
        else
            std::cout << "error: no frame section" <<std::endl;
    }
}

void yarp_interface::getRightArmCartesianRef(Vector &right_arm_ref, const iCub::iDynTree::DynTree &coman_idyn3)
{
    yarp::os::Bottle *bot = right_arm_pos_ref_port.read(false);

    if(!bot == NULL)
    {
        Bottle& frame = bot->findGroup("frame");
        if(!frame.isNull())
        {
            std::string ref_frame = frame.get(1).asString();
            //if(!ref_frame.compare("base_link"))
                //transformRefFrame();

            Bottle& data = bot->findGroup("data");
            if(!data.isNull())
            {
                if(data.size() == right_arm_ref.size()+1)
                {
                    for(unsigned int i = 1; i < right_arm_ref.size()+1; ++i)
                        right_arm_ref[i-1] = data.get(i).asDouble();
                    std::cout<<"New reference for right_arm is "<<right_arm_ref.toString()<<"in base_link"<<std::endl;
                }
                else
                    std::cout<<"Wrong size of reference! Should be "<<right_arm_ref.size()<<std::endl;
            }
            else
                std::cout << "error: no data section inside valve data" << std::endl;
        }
        else
            std::cout << "error: no frame section" <<std::endl;
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
