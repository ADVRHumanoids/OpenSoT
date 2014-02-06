/*
 * WALK-MAN Project
 *
 * PD+Gravity Compensation Tutorial/Test
 *
 * Author: Enrico Mingo
 * 2013
 */


#ifndef _PDGRAVITY_H_
#define _PDGRAVITY_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDynTree/DynTree.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <qpOASES.hpp>



class sot_VelKinCon : public yarp::os::RateThread
{

public:
    KDL::Tree coman_tree; // A KDL Tree
    urdf::Model coman_model; // A URDF Model
    iCub::iDynTree::DynTree coman_iDyn3; // iDyn3 Model
    std::vector<unsigned int> arm_joints_number; // ID of right_arm
    yarp::dev::PolyDriver right_arm_polyDriver; // PolyDriver to connect to the robot/simulator (right arm)
    yarp::dev::PolyDriver left_arm_polyDriver; // PolyDriver to connect to the robot/simulator (left arm)
    yarp::dev::PolyDriver waist_polyDriver; // PolyDriver to connect to the robot/simulator (waist)
    yarp::dev::IControlMode *left_arm_controlMode; // To change control mode
    yarp::dev::IControlMode *right_arm_controlMode; // To change control mode
    yarp::dev::IControlMode *waist_controlMode; // To change control mode
    yarp::dev::IPositionControl2 *left_arm_posControl; // To control in velocity the robot
    yarp::dev::IPositionControl2 *right_arm_posControl; // To control in velocity the robot
    yarp::dev::IPositionControl2 *waist_posControl; // To control in velocity the robot
    yarp::dev::IEncoders *left_arm_encodersMotor;   // encoders for left arm
    yarp::dev::IEncoders *right_arm_encodersMotor;   // encoders for right arm
    yarp::dev::IEncoders *waist_encodersMotor;   // encoders for right arm

    int waist_LinkIndex;
    int right_arm_LinkIndex;
    int left_arm_LinkIndex;

    yarp::sig::Vector q; // Vector of measured joint position [1x23]
    yarp::sig::Vector right_arm_pos_ref; // Vector of desired position for right arm [1x3]
    yarp::sig::Vector left_arm_pos_ref; // Vector of desired position for left arm [1x3]
    yarp::sig::Vector q_ref; // Vector of desired joint configurations [1x23]
    yarp::sig::Vector dq_ref; // Vector of desired joint velocities [1x23]
    yarp::sig::Vector ddq_ref; // Vector of desired joint accelerations [1x23]
    std::vector<std::string> right_arm_joint_names;
    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> waist_joint_names;
    yarp::sig::Matrix Q_postural; //Matrix of weights for the postural task

    // Joint ids for right arm and left arm
    std::vector<unsigned int> right_arm_joint_numbers;
    std::vector<unsigned int> left_arm_joint_numbers;
    std::vector<unsigned int> waist_joint_numbers;

    /// Input Ports: We define some input port to interact with the controller
    // Defining Port to set end effector position reference
    yarp::os::BufferedPort<yarp::os::Bottle> right_arm_pos_ref_port;
    yarp::os::BufferedPort<yarp::os::Bottle> left_arm_pos_ref_port;

    /// Output Ports
    // Outuput ports with configuration computed by sot
    yarp::os::Port right_arm_configuration_ref_port;
    yarp::os::Port left_arm_configuration_ref_port;
    yarp::os::Port waist_configuration_ref_port;

    sot_VelKinCon();
    bool computeControlHQP(  const yarp::sig::Matrix& J1,
                                            const yarp::sig::Vector& e1,
                                            const yarp::sig::Matrix& J2,
                                            const yarp::sig::Vector& eq,
                                            yarp::sig::Vector& dq_ref);
    ~sot_VelKinCon(){}
    bool threadInit();
    void run();
    bool onStop();
    void threadRelease();
    void suspend();

private:
    void printModelInformation();
    void setQ_postural();
    yarp::os::Network yarp;
};

#endif
