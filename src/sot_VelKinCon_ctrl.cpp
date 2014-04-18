/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo, Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "sot_VelKinCon_ctrl.h"
#include <boost/foreach.hpp>
#include "task_solver.h"
#include "cartesian_utils.h"
#include "sot_VelKinCon_constants.h"


#define toRad(X) (X*M_PI/180.0)
#define toDeg(X) (X*180.0/M_PI)
#define MilliSecToSec(X) (X/1000.0)

/** ******************************************* **/

using namespace iCub::iDynTree;
using namespace yarp::math;
using namespace wb_sot;

sot_VelKinCon_ctrl::sot_VelKinCon_ctrl(const double period, int argc, char *argv[], paramHelp::ParamHelperServer* _ph):
    RateThread(int(period*1000.0)),
    IYarp(),
    q_ref(1),
    dq_ref(1),
    ddq_ref(1),
    tau_gravity(1),
    q(1),
    q_left_arm(1),
    q_right_arm(1),
    q_left_leg(1),
    q_right_leg(1),
    q_torso(1),
    right_arm_pos_ref(3, 0.0),
    left_arm_pos_ref(3, 0.0),
    worldT(4,4),
    t_elapsed(0.0),
    eRWrist_p(3, 0.0),
    eRWrist_o(3, 0.0),
    eLWrist_p(3, 0.0),
    eLWrist_o(3, 0.0),
    eSwingFoot_p(3, 0.0),
    eSwingFoot_o(3, 0.0),
    eCoM(3, 0.0),
    paramHelper(_ph)
{
    iDyn3Model();
    setJointNames();
    setControlledKinematicChainsLinkIndex();
    setControlledKinematicChainsJointNumbers();

    int nJ = coman_iDyn3.getNrOfDOFs();

    Q_postural.resize(nJ, nJ);
    Q_postural.eye();
//    yarp::sig::Vector qMax = coman_iDyn3.getJointBoundMax();
//    yarp::sig::Vector qMin = coman_iDyn3.getJointBoundMin();
//    Q_postural.diagonal(computeW(qMin, qMax, right_arm_joint_numbers,
//                                 left_arm_joint_numbers, waist_joint_numbers));
    zero.resize(1);
    zero.zero();

    q.resize(nJ, 0.0);
    q_ref.resize(nJ, 0.0);
    dq_ref.resize(nJ,0.0);
    ddq_ref.resize(nJ, 0.0);

    tau_gravity.resize(nJ, 0.0);

    IYarp.encodersMotor_left_arm->getAxes(&nJ);
    q_left_arm.resize(nJ, 0.0);
    IYarp.encodersMotor_right_arm->getAxes(&nJ);
    q_right_arm.resize(nJ, 0.0);
    IYarp.encodersMotor_left_leg->getAxes(&nJ);
    q_left_leg.resize(nJ, 0.0);
    IYarp.encodersMotor_right_leg->getAxes(&nJ);
    q_right_leg.resize(nJ, 0.0);
    IYarp.encodersMotor_torso->getAxes(&nJ);
    q_torso.resize(nJ, 0.0);

    worldT.eye();

    is_clik = false;
}

void sot_VelKinCon_ctrl::parameterUpdated(const ParamProxyInterface *pd)
{
    return;
}

//Qui devo prendere la configurazione iniziale del robot!
bool sot_VelKinCon_ctrl::threadInit()
{
    IYarp.cleanPorts();

    dq_ref.zero();
    getFeedBack();

    //Here we set as initial reference the measured value: this will be the postural task
    q_ref = q;

    updateiDyn3Model(true);

    support_foot_LinkIndex = left_leg_LinkIndex;
    swing_foot_LinkIndex = right_leg_LinkIndex;

    right_arm_pos_ref = coman_iDyn3.getPosition(right_arm_LinkIndex);
    left_arm_pos_ref = coman_iDyn3.getPosition(left_arm_LinkIndex);

    com_pos_ref = coman_iDyn3.getCOM("",support_foot_LinkIndex);

    swing_foot_pos_ref = coman_iDyn3.getPosition(support_foot_LinkIndex, swing_foot_LinkIndex);

    ROS_INFO("Initial Pose Ref left_arm:");   cartesian_utils::printHomogeneousTransform(left_arm_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref right_arm:");  cartesian_utils::printHomogeneousTransform(right_arm_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref swing_foot:"); cartesian_utils::printHomogeneousTransform(swing_foot_pos_ref);std::cout<<std::endl;
    ROS_INFO("Initial Position Ref CoM: [ %s ]", com_pos_ref.toString().c_str());

#if RIGHT_ARM_IMPEDANCE
    ROS_INFO("Setting Impedance Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.controlMode_right_arm->setImpedancePositionMode(i);
#else
    ROS_INFO("Setting Position Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.controlMode_right_arm->setPositionMode(i);
#endif

#if LEFT_ARM_IMPEDANCE
    ROS_INFO("Setting Impedance Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.controlMode_left_arm->setImpedancePositionMode(i);
#else
    ROS_INFO("Setting Position Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.controlMode_left_arm->setPositionMode(i);
#endif

#if TORSO_IMPEDANCE
    ROS_INFO("Setting Impedance Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.controlMode_torso->setImpedancePositionMode(i);
#else
    ROS_INFO("Setting Position Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.controlMode_torso->setPositionMode(i);
#endif

    ROS_INFO("Setting Position Mode for q_right_leg:");
    for(unsigned int i = 0; i < q_right_leg.size(); ++i)
        IYarp.controlMode_right_leg->setPositionMode(i);

    ROS_INFO("Setting Position Mode for q_left_leg:");
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
        IYarp.controlMode_left_leg->setPositionMode(i);

    if(is_clik)
        ROS_WARN("SoT is running as CLIK");
    else
        ROS_WARN("SoT is NOT running as CLIK");

    ROS_INFO("sot_VelKinCon START!!!");

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COMPUTATION_TIME,             &t_elapsed));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_POSITION_ERROR,      eLWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,   eLWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_POSITION_ERROR,     eRWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,  eRWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_POSITION_ERROR,    eSwingFoot_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_ORIENTATION_ERROR, eSwingFoot_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COM_POSITION_ERROR,           eCoM.data()));

    return true;
}

void sot_VelKinCon_ctrl::run()
{
#ifdef DEBUG
    paramHelper->lock();
    paramHelper->readStreamParams();
#endif

    IYarp.tic();

    checkInput();

    if(is_clik)
        getFeedBack();
    else
        q += dq_ref;

    updateiDyn3Model(true);

    if(controlLaw())
        move();

    IYarp.sendWorldToBaseLinkPose(worldT);

    t_elapsed = IYarp.toc();

#ifdef DEBUG
    paramHelper->sendStreamParams();
    paramHelper->unlock();
#endif
}

void sot_VelKinCon_ctrl::iDyn3Model()
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_sensor_names;
    joint_sensor_names.push_back("l_ankle_joint");
    joint_sensor_names.push_back("r_ankle_joint");
    waist_link_name = "Waist";


    std::string coman_model_folder = ros::package::getPath("coman_urdf") + "/urdf/coman.urdf";
    coman_model.reset(new urdf::Model());
    if (!coman_model->initFile(coman_model_folder))
      ROS_ERROR("Failed to parse URDF robot model!");
    else
    {
        std::string path_to_srdf = ros::package::getPath("coman_srdf") + "/srdf/coman.srdf";
        coman_srdf.reset(new srdf::Model());
        if(!coman_srdf->initFile(*coman_model.get(), path_to_srdf))
            ROS_ERROR("Failed to parse SRDF robot model!");
        else
        {
            coman_robot_model.reset(new robot_model::RobotModel(coman_model, coman_srdf));
            std::ostringstream robot_info;
            coman_robot_model->printModelInfo(robot_info);
            ROS_INFO(robot_info.str().c_str());
        }
    }


    if (!kdl_parser::treeFromUrdfModel(*coman_model.get(), coman_tree))
      ROS_ERROR("Failed to construct kdl tree!");

    // Here the iDyn3 model of the robot is generated
    coman_iDyn3.constructor(coman_tree, joint_sensor_names, waist_link_name);

    int nJ = coman_iDyn3.getNrOfDOFs(); //29
    yarp::sig::Vector qMax; qMax.resize(nJ,0.0);
    yarp::sig::Vector qMin; qMin.resize(nJ,0.0);

    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator i;
    for(i = coman_model->joints_.begin(); i != coman_model->joints_.end(); ++i) {
        int jIndex = coman_iDyn3.getDOFIndex(i->first);
        if(jIndex != -1) {
            qMax[jIndex] = i->second->limits->upper;
            qMin[jIndex] = i->second->limits->lower;
        }
    }

    coman_iDyn3.setJointBoundMax(qMax);
    coman_iDyn3.setJointBoundMin(qMin);

    yarp::sig::Vector tauMax; tauMax.resize(nJ,1.0);
    for(i = coman_model->joints_.begin(); i != coman_model->joints_.end(); ++i) {
        int jIndex = coman_iDyn3.getDOFIndex(i->first);
        if(jIndex != -1) {
            tauMax[jIndex] = i->second->limits->effort;
        }
    }

    coman_iDyn3.setJointTorqueBoundMax(tauMax);

    yarp::sig::Vector a; a = coman_iDyn3.getJointTorqueMax();

    ROS_INFO("Loaded COMAN in iDyn3!");
}

void sot_VelKinCon_ctrl::setControlledKinematicChainsLinkIndex()
{
    right_arm_name = "r_wrist";
    left_arm_name = "l_wrist";
    right_leg_name = "r_sole";
    left_leg_name = "l_sole";
    waist_LinkIndex = coman_iDyn3.getLinkIndex(waist_link_name);
    right_arm_LinkIndex = coman_iDyn3.getLinkIndex(right_arm_name);
    left_arm_LinkIndex = coman_iDyn3.getLinkIndex(left_arm_name);
    right_leg_LinkIndex = coman_iDyn3.getLinkIndex(right_leg_name);
    left_leg_LinkIndex = coman_iDyn3.getLinkIndex(left_leg_name);
    if(right_arm_LinkIndex == -1)
        ROS_ERROR("Failed to get link index for right arm");
    if(left_arm_LinkIndex == -1)
        ROS_ERROR("Failed to get link index for left arm");
    if(waist_LinkIndex == -1)
        ROS_ERROR("Failed to get link index for Waist");
    if(right_leg_LinkIndex == -1)
        ROS_ERROR("Failed to get link index for right leg");
    if(left_leg_LinkIndex == -1)
        ROS_ERROR("Failed to get link index for left leg");
}

void sot_VelKinCon_ctrl::setControlledKinematicChainsJointNumbers()
{
    ROS_INFO("Right Arm joint indices:");
    BOOST_FOREACH(std::string joint_name, right_arm_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        right_arm_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;

    ROS_INFO("Left Arm joint indices:");
    BOOST_FOREACH(std::string joint_name, left_arm_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        left_arm_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;

    ROS_INFO("Waist joint indices:");
    BOOST_FOREACH(std::string joint_name, torso_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        waist_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;

    ROS_INFO("Right Leg joint indices:");
    BOOST_FOREACH(std::string joint_name, right_leg_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        right_leg_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;

    ROS_INFO("Left Leg joint indices:");
    BOOST_FOREACH(std::string joint_name, left_leg_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        left_leg_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;
}

yarp::sig::Vector sot_VelKinCon_ctrl::computeW(const yarp::sig::Vector &qMin,
                                               const yarp::sig::Vector &qMax,
                                               const std::vector<unsigned int>& right_arm_joint_numbers,
                                               const std::vector<unsigned int>& left_arm_joint_numbers,
                                               const std::vector<unsigned int>& waist_joint_numbers)
{
    yarp::sig::Vector w(qMax.size(), 1.0);

    std::vector<unsigned int> waist_left_arm_joint_numbers = waist_joint_numbers;
    std::vector<unsigned int> waist_right_arm_joint_numbers = waist_joint_numbers;
    waist_left_arm_joint_numbers.insert(waist_left_arm_joint_numbers.end(), left_arm_joint_numbers.begin(), left_arm_joint_numbers.end());
    waist_right_arm_joint_numbers.insert(waist_right_arm_joint_numbers.end(), right_arm_joint_numbers.begin(), right_arm_joint_numbers.end());

    for(unsigned int i = 0; i < waist_left_arm_joint_numbers.size(); ++i)
        w[waist_left_arm_joint_numbers[i]]  *= (double)(waist_left_arm_joint_numbers.size() - i);

    for(unsigned int i = 0; i < waist_right_arm_joint_numbers.size(); ++i)
        w[waist_right_arm_joint_numbers[i]] *= (double)(waist_right_arm_joint_numbers.size() - i);

    for(unsigned int i = 0; i < waist_joint_numbers.size(); ++i) {
        w[waist_joint_numbers[i]] = sqrt(w[waist_joint_numbers[i]]);
        w[waist_joint_numbers[i]] *= TORSO_WEIGHT;
    }

    w = w/(qMax-qMin);
    for(unsigned int j = 0; j < w.size(); ++j)
        w[j] = fabs(w[j]);

    ROS_INFO("W: [ %s ]", w.toString().c_str());
    return w;
}

void sot_VelKinCon_ctrl::updateiDyn3Model(const bool set_world_pose)
{
    // Here we set these values in our internal model
    coman_iDyn3.setAng(q);
    coman_iDyn3.setDAng(ddq_ref); //We use the accelerations here because we want a vector of zeros
    coman_iDyn3.setD2Ang(ddq_ref);
    // This is the fake Inertial Measure
    yarp::sig::Vector g(3);
    g[0] = 0; g[1] = 0; g[2] = 9.81;
    yarp::sig::Vector o(3);
    o[0] = 0; o[1] = 0; o[2] = 0;
    coman_iDyn3.setInertialMeasure(o, o, g);

    coman_iDyn3.kinematicRNEA();
#if (LEFT_ARM_IMPEDANCE || RIGHT_ARM_IMPEDANCE || TORSO_IMPEDANCE)
    coman_iDyn3.dynamicRNEA();
    tau_gravity = coman_iDyn3.getTorques();
#endif
    coman_iDyn3.computePositions();

    // Set World Pose we do it at the beginning
    if(set_world_pose)
    {
        //yarp::sig::Matrix worldT(4,4);
        worldT.eye();
        coman_iDyn3.setWorldBasePose(worldT);
//        yarp::sig::Vector foot_pose(3);
//        foot_pose = coman_iDyn3.getPosition(support_foot_LinkIndex).getCol(3).subVector(0,2);
//        worldT(2,3) = -foot_pose(2);

        worldT = yarp::math::luinv(coman_iDyn3.getPosition(coman_iDyn3.getLinkIndex("l_sole")));
        worldT(0,3) = 0.0;
        worldT(1,3) = 0.0;

        //ROS_INFO("World Base Pose: "); cartesian_utils::printHomogeneousTransform(worldT);std::cout<<std::endl;
        coman_iDyn3.setWorldBasePose(worldT);
        coman_iDyn3.computePositions();
    }
}

//Also here the configurations come in deg so we need to convert to rad!
void sot_VelKinCon_ctrl::getFeedBack()
{
    IYarp.encodersMotor_left_arm->getEncoders(q_left_arm.data());
    IYarp.encodersMotor_right_arm->getEncoders(q_right_arm.data());
    IYarp.encodersMotor_left_leg->getEncoders(q_left_leg.data());
    IYarp.encodersMotor_right_leg->getEncoders(q_right_leg.data());
    IYarp.encodersMotor_torso->getEncoders(q_torso.data());

    //To make things faster: we suppose that arms has same number of dofs
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
    {
        q_left_arm[i] = toRad(q_left_arm[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(left_arm_joint_names[i])] = q_left_arm[i];
        q_right_arm[i] = toRad(q_right_arm[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(right_arm_joint_names[i])] = q_right_arm[i];
    }
    //To make things faster: we suppose that legs has same number of dofs
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
    {
        q_left_leg[i] = toRad(q_left_leg[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(left_leg_joint_names[i])] = q_left_leg[i];
        q_right_leg[i] = toRad(q_right_leg[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(right_leg_joint_names[i])] = q_right_leg[i];
    }
    for(unsigned int i = 0; i < q_torso.size(); ++i)
    {
        q_torso[i] = toRad(q_torso[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(torso_joint_names[i])] = q_torso[i];
    }
}

void sot_VelKinCon_ctrl::checkInput()
{
    IYarp.getLeftArmCartesianRef(left_arm_pos_ref);
    IYarp.getRightArmCartesianRef(right_arm_pos_ref);
    IYarp.getCoMCartesianRef(com_pos_ref);
    IYarp.getSetClik(is_clik);
}

/** Here we convert from rad to deg!
    The implemented control is like a velocity control: q_d = q + dq
**/
void sot_VelKinCon_ctrl::move()
{
    yarp::sig::Vector torso(q_torso.size(), 0.0);
    yarp::sig::Vector left_arm(q_left_arm.size(), 0.0);
    yarp::sig::Vector right_arm(q_right_arm.size(), 0.0);
    yarp::sig::Vector left_leg(q_left_leg.size(), 0.0);
    yarp::sig::Vector right_leg(q_right_leg.size(), 0.0);

    double q_sent = 0.0;
    for(unsigned int i = 0; i < torso.size(); ++i){
        q_sent = q[waist_joint_numbers[i]] + dq_ref[waist_joint_numbers[i]];
        torso[i] = toDeg( q_sent );}
    //Here we assumes that left and right arm has the same number of joints!
    for(unsigned int i = 0; i < left_arm.size(); ++i){
        q_sent = q[left_arm_joint_numbers[i]] + dq_ref[left_arm_joint_numbers[i]];
        left_arm[i] = toDeg( q_sent );
        q_sent = q[right_arm_joint_numbers[i]] + dq_ref[right_arm_joint_numbers[i]];
        right_arm[i] = toDeg( q_sent );
    }
    //Here we assumes that left and right leg has the same number of joints!
    for(unsigned int i = 0; i < left_leg.size(); ++i){
        q_sent = q[left_leg_joint_numbers[i]] + dq_ref[left_leg_joint_numbers[i]];
        left_leg[i] = toDeg( q_sent );
        q_sent = q[right_leg_joint_numbers[i]] + dq_ref[right_leg_joint_numbers[i]];
        right_leg[i] = toDeg( q_sent );
    }

    IYarp.directControl_torso->setPositions(torso.data());
    IYarp.directControl_left_arm->setPositions(left_arm.data());
    IYarp.directControl_right_arm->setPositions(right_arm.data());
    IYarp.directControl_left_leg->setPositions(left_leg.data());
    IYarp.directControl_right_leg->setPositions(right_leg.data());

#if LEFT_ARM_IMPEDANCE
    yarp::sig::Vector tau_gravity_left_arm = getGravityCompensationTorque(left_arm_joint_names);
    for(unsigned int i = 0; i < left_arm_joint_names.size(); ++i)
        IYarp.impedanceCtrl_left_arm->setImpedanceOffset(i, tau_gravity_left_arm[i]);
#endif
#if RIGHT_ARM_IMPEDANCE
    yarp::sig::Vector tau_gravity_right_arm = getGravityCompensationTorque(right_arm_joint_names);
    for(unsigned int i = 0; i < right_arm_joint_names.size(); ++i)
        IYarp.impedanceCtrl_right_arm->setImpedanceOffset(i, tau_gravity_right_arm[i]);
#endif
#if TORSO_IMPEDANCE
    yarp::sig::Vector tau_gravity_torso = getGravityCompensationTorque(torso_joint_names);
    for(unsigned int i = 0; i < torso_joint_names.size(); ++i)
        IYarp.impedanceCtrl_torso->setImpedanceOffset(i, tau_gravity_torso[i]);
#endif
}

bool sot_VelKinCon_ctrl::controlLaw()
{
    yarp::sig::Matrix pos_wrist_R = coman_iDyn3.getPosition(right_arm_LinkIndex);
    yarp::sig::Matrix pos_wrist_L = coman_iDyn3.getPosition(left_arm_LinkIndex);

    yarp::sig::Vector pos_CoM = coman_iDyn3.getCOM("",support_foot_LinkIndex);

    yarp::sig::Matrix pos_foot_swing = coman_iDyn3.getPosition(support_foot_LinkIndex,swing_foot_LinkIndex);

    yarp::sig::Matrix JRWrist;
    if(!coman_iDyn3.getJacobian(right_arm_LinkIndex,JRWrist))
        ROS_ERROR("Error computing Jacobian for Right Wrist");
    JRWrist = JRWrist.removeCols(0,6);    // removing unactuated joints (floating base)

    yarp::sig::Matrix JLWrist;
    if(!coman_iDyn3.getJacobian(left_arm_LinkIndex,JLWrist))
        ROS_ERROR("Error computing Jacobian for Left Wrist");
    JLWrist = JLWrist.removeCols(0,6);    // removing unactuated joints (floating base)

    yarp::sig::Matrix JSwingFoot; // for now, SwingFoot is Left
    if(!coman_iDyn3.getRelativeJacobian(swing_foot_LinkIndex,support_foot_LinkIndex,JSwingFoot,true))
        ROS_ERROR("Error computing Jacobian for Left Foot");

    yarp::sig::Matrix JCoM;
    //
    coman_iDyn3.setFloatingBaseLink(support_foot_LinkIndex);
    if(!coman_iDyn3.getCOMJacobian(JCoM))
        ROS_ERROR("Error computing CoM Jacobian");
    coman_iDyn3.setFloatingBaseLink(waist_LinkIndex);
    JCoM = JCoM.removeCols(0,6);    // remove floating base
    JCoM = JCoM.removeRows(3,3);    // remove orientation

    extractJacobians(JRWrist, JLWrist);

    cartesian_utils::computeCartesianError(pos_wrist_R, right_arm_pos_ref,
                                           eRWrist_p, eRWrist_o);
    cartesian_utils::computeCartesianError(pos_wrist_L, left_arm_pos_ref,
                                           eLWrist_p, eLWrist_o);
    cartesian_utils::computeCartesianError(pos_foot_swing, swing_foot_pos_ref,
                                           eSwingFoot_p, eSwingFoot_o);
    eCoM = com_pos_ref-pos_CoM;

    yarp::sig::Vector eRWrist = yarp::math::cat(eRWrist_p, -ORIENTATION_ERROR_GAIN*eRWrist_o);
    yarp::sig::Vector eLWrist = yarp::math::cat(eLWrist_p, -ORIENTATION_ERROR_GAIN*eLWrist_o);
    yarp::sig::Vector eSwingFoot = yarp::math::cat(eSwingFoot_p, -ORIENTATION_ERROR_GAIN*eSwingFoot_o);

//    std::cout<<"eRWrist: "<<eRWrist.toString()<<std::endl;
//    std::cout<<"eLWrist: "<<eLWrist.toString()<<std::endl;
//    std::cout<<"eSwingFoot: "<<eSwingFoot.toString()<<std::endl;
//    std::cout<<"eCoM: "<<eCoM.toString()<<std::endl;
//    std::cout<<"com_pos_ref: "<<com_pos_ref.toString()<<std::endl;
//    std::cout<<"pos_CoM: "<<pos_CoM.toString()<<std::endl;

#if SET_3_TASKS
    yarp::sig::Matrix JEe = yarp::math::pile(JRWrist, JLWrist);
    JEe = yarp::math::pile(JEe, JSwingFoot);
    yarp::sig::Vector eEe = yarp::math::cat(eRWrist, eLWrist);
    eEe = yarp::math::cat(eEe, eSwingFoot);
#else
    yarp::sig::Matrix JEe = yarp::math::pile(JRWrist, JLWrist);
    JEe = yarp::math::pile(JEe, JSwingFoot);
    JEe = yarp::math::pile(JEe, JCoM);
    yarp::sig::Vector eEe = yarp::math::cat(eRWrist, eLWrist);
    eEe = yarp::math::cat(eEe, eSwingFoot);
    eEe = yarp::math::cat(eEe, eCoM);
#endif

    /** Set of last tasks **/
    /**
    *  Here we stack the last set of tasks:
    *
    *  Qdq + (q_ref - q) <-- Postural task
    *  (-grad(g(q))/tau_max)dq <-- Min effort
    *
    **/
    yarp::sig::Vector eq = (q_ref - q);
    yarp::sig::Matrix gGradient(1, eq.size());
    gGradient.setRow(0, getGravityCompensationGradient());
    for(unsigned int i = 0; i < gGradient.cols(); ++i)
        gGradient(0,i) = -1.0*gGradient(0,i)/coman_iDyn3.getJointTorqueMax()[i];
    yarp::sig::Matrix F = yarp::math::pile(Q_postural, gGradient);
    yarp::sig::Vector f = yarp::math::cat(eq, zero);

    bool control_computed = false;
#if SET_3_TASKS
    control_computed = task_solver::computeControlHQP(JCoM,eCoM,
                                                      JEe, eEe,
                                                      Q_postural, gGradient,
                                                      coman_iDyn3.getJointBoundMax(),
                                                      coman_iDyn3.getJointBoundMin(),
                                                      q, MAX_JOINT_VELOCITY,
                                                      MilliSecToSec(getRate()),
                                                      dq_ref);
#else
    control_computed = task_solver::computeControlHQP(JEe, eEe,
                                                     F, f,
                                                     coman_iDyn3.getJointBoundMax(),
                                                     coman_iDyn3.getJointBoundMin(),
                                                     q, MAX_JOINT_VELOCITY,
                                                     MilliSecToSec(getRate()),
                                                     dq_ref);
#endif

    if(!control_computed) {
        ROS_ERROR("Error computing control");
    }
    return control_computed;
}

yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationTorque(const std::vector<string> &joint_names)
{
    yarp::sig::Vector tau(joint_names.size(), 0.0);
    int j = 0;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
        j = coman_iDyn3.getDOFIndex(joint_names[i]);
        tau[i] = tau_gravity[j];
    }
    return tau;
}

yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationTorque(const yarp::sig::Vector q)
{
    static yarp::sig::Vector zeroes(q.size(),0.0);
    static yarp::sig::Vector tau(q.size(),0.0);

    coman_iDyn3.setAng(q);
    coman_iDyn3.setDAng(zeroes); //We use the accelerations here because we want a vector of zeros
    coman_iDyn3.setD2Ang(zeroes);
    // This is the fake Inertial Measure
    yarp::sig::Vector g(3);
    g[0] = 0; g[1] = 0; g[2] = 9.81;
    yarp::sig::Vector o(3);
    o[0] = 0; o[1] = 0; o[2] = 0;
    coman_iDyn3.setInertialMeasure(o, o, g);

    coman_iDyn3.kinematicRNEA();
    coman_iDyn3.dynamicRNEA();
    tau = coman_iDyn3.getTorques();

    return tau;
}

/** compute gradient of an effort (due to gravity) cost function */
yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationGradient()
{
    //double start = yarp::os::Time::now();
    /// cost function is tau_g^t*tau_g
    double C_g_q = yarp::math::dot(tau_gravity,tau_gravity);
    static yarp::sig::Vector gradient(coman_iDyn3.getNrOfDOFs(),0.0);
    static yarp::sig::Vector deltas(coman_iDyn3.getNrOfDOFs(),0.0);
    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        // forward method gradient computation, milligrad
        const double h = 1E-3;
        deltas[i] = h;
        yarp::sig::Vector tau_gravity_q = getGravityCompensationTorque(q+deltas);
        double C_g_q_h = yarp::math::dot(tau_gravity_q,tau_gravity_q);
        gradient[i] = (C_g_q - C_g_q_h)/h;
        deltas[i] = 0;
    }

    //double elapsed = yarp::os::Time::now() - start;
    //ROS_WARN(" took %f ms", elapsed);
    return gradient;
}
