#ifndef _SOT_VELKINCON_CTRL_H_
#define _SOT_VELKINCON_CTRL_H_

#include <yarp/os/RateThread.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <iCub/iDynTree/DynTree.h>
#include <kdl_parser/kdl_parser.hpp>
#include "yarp_interface.h"
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <paramHelp/paramHelperServer.h>
#include "sot_VelKinCon_constants.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <drc_shared/idynutils.h>


namespace wb_sot {
    class sot_VelKinCon_ctrl :  public yarp::os::RateThread,
                                public paramHelp::ParamValueObserver,
                                public paramHelp::CommandObserver
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
                                    const std::vector<unsigned int>& waist_joint_numbers,
                                    const double w_torso_weight);
         yarp::sig::Vector getGravityCompensationTorque(const std::vector<std::string>& joint_names);
         yarp::sig::Vector getGravityCompensationTorque(const yarp::sig::Vector q);
         yarp::sig::Vector getGravityCompensationGradient(const yarp::sig::Matrix& W);

     private:
         paramHelp::ParamHelperServer   *paramHelper;

         /** Callback function for parameter updates. */
         void parameterUpdated(const ParamProxyInterface *pd);
         /** Callback function for rpc commands. */
         void commandReceived(const CommandDescription &cd,
                              const yarp::os::Bottle &params,
                              yarp::os::Bottle &reply);

         const bool LEFT_ARM_IMPEDANCE;
         const bool RIGHT_ARM_IMPEDANCE;
         const bool TORSO_IMPEDANCE;

//          boost::shared_ptr<urdf::Model> coman_model; // A URDF Model
//          boost::shared_ptr<srdf::Model> coman_srdf; // A SRDF description
//          robot_model::RobotModelPtr coman_robot_model; // A robot model

         bool is_clik;

         int support_foot_LinkIndex;
         int swing_foot_LinkIndex;

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

         // the gradient of the gravity vector
         yarp::sig::Vector gradientGq;

         yarp::sig::Vector eRWrist_p;
         yarp::sig::Vector eRWrist_o;
         yarp::sig::Vector eLWrist_p;
         yarp::sig::Vector eLWrist_o;
         yarp::sig::Vector eSwingFoot_p;
         yarp::sig::Vector eSwingFoot_o;
         yarp::sig::Vector eCoM;

         bool use_3_stacks;
         double max_joint_velocity;
         double orientation_error_gain;
         unsigned int last_stack_type;
         unsigned int postural_weight_strategy;
         double postural_weight_coefficient;
         double mineffort_weight_coefficient;
         double velocity_bounds_scale;
         unsigned int qpOASES_NWSR0;
         unsigned int qpOASES_NWSR1;
         unsigned int qpOASES_NWSR2;
         bool qpOASES_enableRegularisation0;
         bool qpOASES_enableRegularisation1;
         bool qpOASES_enableRegularisation2;
         double qpOASES_eps0;
         double qpOASES_eps1;
         double qpOASES_eps2;

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
         iDynUtils idynutils,gravity_compensator_idynutils;
         void updateiDyn3Model(const bool set_world_pose = false);
         void getFeedBack();
         void checkInput();
         void move();
         bool controlLaw();
         void computeLastTaskType();
         void computePosturalWeight();
         void computeMinEffort();

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
                 for(unsigned int j = 0; j < idynutils.right_arm.joint_names.size(); ++j){
                     if(i == idynutils.right_arm.joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 for(unsigned int j = 0; j < idynutils.torso.joint_names.size(); ++j){
                     if(i == idynutils.torso.joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 if(set_zero){
                     for(unsigned int k = 0; k < 6; ++k)
                         JRWrist(k,i) = 0.0;
                 }

                 set_zero = true;
                 for(unsigned int j = 0; j < idynutils.left_arm.joint_names.size(); ++j){
                     if(i == idynutils.left_arm.joint_numbers[j]){
                         set_zero = false;
                         break;}
                 }
                 for(unsigned int j = 0; j < idynutils.torso.joint_names.size(); ++j){
                     if(i == idynutils.torso.joint_numbers[j]){
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
