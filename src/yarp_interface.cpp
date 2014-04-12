/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo, Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "yarp_interface.h"
#include "cartesian_utils.h"
#include <yarp/os/Time.h>
#include <iCub/iDynTree/yarp_kdl.h>

using namespace yarp::os;
using namespace yarp::sig;

#define toRad(X) (X*M_PI/180.0)
#define toDeg(X) (X*180.0/M_PI)

void yarp_interface::tic()
{
    time_tic = yarp::os::Time::now();
}

double yarp_interface::toc()
{
    return yarp::os::Time::now() - time_tic;
}

yarp_interface::yarp_interface()
{
    time_tic = 0.0;

    if(createPolyDriver("left_arm", polyDriver_left_arm))
    {
        polyDriver_left_arm.view(encodersMotor_left_arm);
        polyDriver_left_arm.view(directControl_left_arm);
        polyDriver_left_arm.view(controlMode_left_arm);
        polyDriver_left_arm.view(impedanceCtrl_left_arm);
    }
    if(createPolyDriver("right_arm", polyDriver_right_arm))
    {
        polyDriver_right_arm.view(encodersMotor_right_arm);
        polyDriver_right_arm.view(directControl_right_arm);
        polyDriver_right_arm.view(controlMode_right_arm);
        polyDriver_right_arm.view(impedanceCtrl_right_arm);
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
    com_pos_ref_port.open("/sot_VelKinCon/com/set_ref:i");
    clik_port.open("/sot_VelKinCon/set_clik:i");
    world_to_base_link_pose_port.open("/sot_VelKinCon/world_to_base_link_pose:o");
}

yarp_interface::~yarp_interface()
{
    left_arm_pos_ref_port.close();
    right_arm_pos_ref_port.close();
    com_pos_ref_port.close();

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
        getCartesianRef(left_arm_ref, bot, LOCAL_FRAME_UPPER_BODY);
        std::cout<<std::endl;
    }
}

void yarp_interface::getRightArmCartesianRef(Matrix &right_arm_ref)
{
    yarp::os::Bottle *bot = right_arm_pos_ref_port.read(false);

    if(!bot == NULL)
    {
        std::cout<<"Right Arm:"<<std::endl;
        getCartesianRef(right_arm_ref, bot, LOCAL_FRAME_UPPER_BODY);
        std::cout<<std::endl;
    }
}

void yarp_interface::getCoMCartesianRef(Vector &com_ref)
{
    yarp::os::Bottle *bot = com_pos_ref_port.read(false);
    Matrix com_ref_T;
    if(!bot == NULL)
    {
        std::cout<<"CoM::"<<std::endl;
        if(getCartesianRef(com_ref_T, bot, LOCAL_FRAME_COM))
        {
            KDL::Frame com_ref_T_KDL;
            YarptoKDL(com_ref_T, com_ref_T_KDL);
            com_ref = KDLtoYarp(com_ref_T_KDL.p);
            std::cout<<std::endl;
        }
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

bool yarp_interface::sendCartesianRef(BufferedPort<Bottle> &port, const std::string &ref_frame , const Matrix &T)
{
    yarp::os::Bottle &temp = port.prepare();
    yarp::os::Bottle &tf_list  = temp.addList();

    tf_list.addString("frame");
    tf_list.addString(ref_frame);

    yarp::os::Bottle &pose_list  = temp.addList();

    KDL::Frame T_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T, T_kdl);
    pose_list.addString("data");

    pose_list.add(T_kdl.p.x());
    pose_list.add(T_kdl.p.y());
    pose_list.add(T_kdl.p.z());

    double R = 0.0;
    double P = 0.0;
    double Y = 0.0;
    T_kdl.M.GetRPY(R,P,Y);
    pose_list.add(toDeg(R));
    pose_list.add(toDeg(P));
    pose_list.add(toDeg(Y));

    port.write();
}

bool yarp_interface::getCartesianRef(Matrix &ref, yarp::os::Bottle *bot, const std::string &local_frame)
{
    Bottle& frame = bot->findGroup("frame");
    if(!frame.isNull())
    {
        if(checkRefFrame(frame.get(1).asString(), local_frame.c_str()))
        {
            Bottle& data = bot->findGroup("data");
            if(!data.isNull())
            {
                if(data.size() == 6+1)
                {
                    cartesian_utils::homogeneousMatrixFromRPY(ref,
                                                              data.get(1).asDouble(), data.get(2).asDouble(), data.get(3).asDouble(),
                                                              toRad(data.get(4).asDouble()), toRad(data.get(5).asDouble()), toRad(data.get(6).asDouble()));
                    std::cout<<"    New reference in "<< local_frame.c_str()<< " is "<<std::endl;
                    std::cout<<ref.toString()<<std::endl;
                    return true;
                }
                else if(data.size() == 7+1)
                {
                    cartesian_utils::homogeneousMatrixFromQuaternion(ref,
                                                                     data.get(1).asDouble(), data.get(2).asDouble(), data.get(3).asDouble(),
                                                                     data.get(4).asDouble(), data.get(5).asDouble(), data.get(6).asDouble(), data.get(7).asDouble());
                    std::cout<<"    New reference in "<< local_frame.c_str()<< " is "<<std::endl;
                    std::cout<<ref.toString()<<std::endl;
                    return true;
                }
                else
                    std::cout<<"    ERROR: wrong size of reference! Should be 6 if RPY are used or 7 if Quaternions are used!"<<std::endl;
            }
            else
                std::cout<<"    ERROR: no data section in port"<<std::endl;
        }
        else
            std::cout<<"    ERROR: wrong Reference Frame, should be "<< local_frame.c_str() <<std::endl;
    }
    else
        std::cout<<"    ERROR: no frame section in port"<<std::endl;
    return false;
}

void yarp_interface::cleanPorts()
{
    yarp::os::Bottle* foo;

    int pendings = left_arm_pos_ref_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        left_arm_pos_ref_port.read(foo);

    pendings = right_arm_pos_ref_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        right_arm_pos_ref_port.read(foo);

    pendings = com_pos_ref_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        com_pos_ref_port.read(foo);

    pendings = clik_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        clik_port.read(foo);

    pendings = world_to_base_link_pose_port.getPendingReads();
    for(unsigned int i = 0; i < pendings; ++i)
        world_to_base_link_pose_port.read(foo);
}
