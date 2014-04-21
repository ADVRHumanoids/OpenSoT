#ifndef _SOT_VELKINCON_CTRL_H_
#define _SOT_VELKINCON_CTRL_H_

#include <yarp/os/RateThread.h>
#include <urdf/model.h>
#include <iCub/iDynTree/DynTree.h>
#include <kdl_parser/kdl_parser.hpp>
#include "yarp_interface.h"
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <paramHelp/paramHelperServer.h>
#include "sot_VelKinCon_constants.h"

namespace wb_sot {
    class sot_VelKinCon_ctrl : public yarp::os::RateThread, public paramHelp::ParamValueObserver
     {
     public:
         sot_VelKinCon_ctrl(const int period, const bool _LEFT_ARM_IMPEDANCE,
                                              const bool _RIGHT_ARM_IMPEDANCE,
                                              const bool _TORSO_IMPEDANCE,
                                              paramHelp::ParamHelperServer *_ph);

         virtual bool threadInit();
         virtual void run();

         static yarp::sig::Vector computeW(const yarp::sig::Vector& qMin, const yarp::sig::Vector& qMax,
                                    const std::vector<unsigned int>& right_arm_joint_numbers,
                                    const std::vector<unsigned int>& left_arm_joint_numbers,
                                    const std::vector<unsigned int>& waist_joint_numbers);
         yarp::sig::Vector getGravityCompensationTorque(const std::vector<std::string>& joint_names);
         yarp::sig::Vector getGravityCompensationTorque(const yarp::sig::Vector q);
         yarp::sig::Vector getGravityCompensationGradient();

     private:
         paramHelp::ParamHelperServer   *paramHelper;
         void parameterUpdated(const ParamProxyInterface *pd);

         const bool LEFT_ARM_IMPEDANCE;
         const bool RIGHT_ARM_IMPEDANCE;
         const bool TORSO_IMPEDANCE;

         KDL::Tree coman_tree; // A KDL Tree
         urdf::Model coman_model; // A URDF Model
         iCub::iDynTree::DynTree coman_iDyn3; // iDyn3 Model
         std::vector<std::string> left_arm_joint_names;
         std::vector<std::string> right_arm_joint_names;
         std::vector<std::string> left_leg_joint_names;
         std::vector<std::string> right_leg_joint_names;
         std::vector<std::string> torso_joint_names;

         bool is_clik;

         int waist_LinkIndex;
         int right_arm_LinkIndex;
         int left_arm_LinkIndex;
         int right_leg_LinkIndex;
         int left_leg_LinkIndex;
         int support_foot_LinkIndex;
         int swing_foot_LinkIndex;
         std::string right_arm_name;
         std::string left_arm_name;
         std::string right_leg_name;
         std::string left_leg_name;
         std::string waist_link_name;
         // Joint ids for right arm and left arm
         std::vector<unsigned int> right_arm_joint_numbers;
         std::vector<unsigned int> left_arm_joint_numbers;
         std::vector<unsigned int> right_leg_joint_numbers;
         std::vector<unsigned int> left_leg_joint_numbers;
         std::vector<unsigned int> waist_joint_numbers;

         yarp::sig::Vector q_ref; // Vector of desired joint configurations [1x29]
         yarp::sig::Vector dq_ref; // Vector of desired joint velocities [1x29]
         yarp::sig::Vector ddq_ref; // Vector of desired joint accelerations [1x29]
         yarp::sig::Vector tau_gravity; // Vector of toques for gravity compensation
         yarp::sig::Vector q; // Vector of measured joint angles
         yarp::sig::Vector q_left_arm; // Vector of measured joint angles
         yarp::sig::Vector q_left_leg; // Vector of measured joint angles
         yarp::sig::Vector q_right_arm; // Vector of measured joint angles
         yarp::sig::Vector q_right_leg; // Vector of measured joint angles
         yarp::sig::Vector q_torso; // Vector of measured joint angles

         yarp::sig::Matrix right_arm_pos_ref; // Homogeneous Matrix of desired position for right arm
         yarp::sig::Matrix left_arm_pos_ref; // Homogeneous Matrix of desired position for left arm
         yarp::sig::Matrix worldT;
         yarp::sig::Vector com_pos_ref;
         yarp::sig::Matrix swing_foot_pos_ref;

         // how much time did it take to execute run()?
         double t_elapsed;
         yarp::sig::Vector eRWrist_p;
         yarp::sig::Vector eRWrist_o;
         yarp::sig::Vector eLWrist_p;
         yarp::sig::Vector eLWrist_o;
         yarp::sig::Vector eSwingFoot_p;
         yarp::sig::Vector eSwingFoot_o;
         yarp::sig::Vector eCoM;

         /** Some Theory: **/
         /**
            We are considering the optimization problem:
                (Ax-b)'Q(Ax-b) = ... = x'Hx + x'g
            where:
                H = A'QA
                g = -2A'Qb

            For the inverse kinematic problem:
                x = dq
                Q = general weights
                A = J
                b = dx (desired Cartesian velocity to the goal)
         **/
         yarp::sig::Matrix Q_postural; //Matrix of weights for the postural task
         yarp::sig::Vector zero;

         yarp_interface IYarp;

         void iDyn3Model();
         void updateiDyn3Model(const bool set_world_pose = false);
         void setControlledKinematicChainsLinkIndex();
         void setControlledKinematicChainsJointNumbers();
         void getFeedBack();
         void checkInput();
         void move();
         bool controlLaw();
         void setJointNames()
         {
             right_arm_joint_names.push_back("RShSag");
             right_arm_joint_names.push_back("RShLat");
             right_arm_joint_names.push_back("RShYaw");
             right_arm_joint_names.push_back("RElbj");
             right_arm_joint_names.push_back("RForearmPlate");
             right_arm_joint_names.push_back("RWrj1");
             right_arm_joint_names.push_back("RWrj2");

             left_arm_joint_names.push_back("LShSag");
             left_arm_joint_names.push_back("LShLat");
             left_arm_joint_names.push_back("LShYaw");
             left_arm_joint_names.push_back("LElbj");
             left_arm_joint_names.push_back("LForearmPlate");
             left_arm_joint_names.push_back("LWrj1");
             left_arm_joint_names.push_back("LWrj2");

             right_leg_joint_names.push_back("RHipSag");
             right_leg_joint_names.push_back("RHipLat");
             right_leg_joint_names.push_back("RHipYaw");
             right_leg_joint_names.push_back("RKneeSag");
             right_leg_joint_names.push_back("RAnkLat");
             right_leg_joint_names.push_back("RAnkSag");

             left_leg_joint_names.push_back("LHipSag");
             left_leg_joint_names.push_back("LHipLat");
             left_leg_joint_names.push_back("LHipYaw");
             left_leg_joint_names.push_back("LKneeSag");
             left_leg_joint_names.push_back("LAnkLat");
             left_leg_joint_names.push_back("LAnkSag");

             torso_joint_names.push_back("WaistSag");
             torso_joint_names.push_back("WaistLat");
             torso_joint_names.push_back("WaistYaw");
         }


         /**
           We use this function to set to zero all the part of the Jacobians that we are not
           controlling (basically the legs).
           Each Jacobian contains only waist + arm.
           **/
         void extractJacobians(yarp::sig::Matrix& JRWrist, yarp::sig::Matrix& JLWrist)
         {
            for(unsigned int i = 0; i < JLWrist.cols(); ++i)
            {
                 bool set_zero = true;
                 for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j){
                     if(i == right_arm_joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 for(unsigned int j = 0; j < torso_joint_names.size(); ++j){
                     if(i == waist_joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 if(set_zero){
                     for(unsigned int k = 0; k < 6; ++k)
                         JRWrist(k,i) = 0.0;
                 }

                 set_zero = true;
                 for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j){
                     if(i == left_arm_joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 for(unsigned int j = 0; j < torso_joint_names.size(); ++j){
                     if(i == waist_joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 if(set_zero){
                     for(unsigned int k = 0; k < 6; ++k)
                         JLWrist(k,i) = 0.0;
                 }
            }
         }
    };
}

#endif
