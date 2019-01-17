#ifndef __TASKS_VELOCITY_JOINT_ADMITTANCE_H__
#define __TASKS_VELOCITY_JOINT_ADMITTANCE_H__

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {
       /**
        * @brief The JointAdmittance class implements a simple admittance controller in velocity at the joints.
        */
       class JointAdmittance: public Postural {
        public:
           typedef boost::shared_ptr<JointAdmittance> Ptr;

           /**
            * @brief JointAdmittance constructor
            * @param model is updated with the integrated state from the robot used to perform open-loop IK and
            * measured joint torques from the robot
            */
           JointAdmittance(XBot::ModelInterface &model, const Eigen::VectorXd& x);

           /**
            * @brief setJointCompliance set the Compliance matrix
            * @param C a SPD matrix
            */
           void setJointCompliance(const Eigen::MatrixXd& C);

           /**
            * @brief setJointCompliance set the same compiance to all the joints
            * @param C a positive or equal zero number
            */
           void setJointCompliance(const double C);

           /**
            * @brief getJointCompliance
            * @return the actual Compliance matrix
            */
           const Eigen::MatrixXd& getJointCompliance();

           /**
            * @brief getJointCompliance
            * @param C the actual Compliance matrix
            */
           void getJointCompliance(Eigen::MatrixXd& C);

           /**
            * @brief setFilterParams set all the parameters of the internal second order filter used to filter the
            * \f$ \boldsymbol{\Delta \tau} \f$
            * @param time_step of the filter
            * @param damping of the filter
            * @param omega of the filter
            */
           void setFilterParams(const double time_step, const double damping, const double omega);

           /**
            * @brief setFilterTimeStep
            * @param time_step of the filter
            */
           void setFilterTimeStep(const double time_step);

           /**
            * @brief setFilterDamping
            * @param damping of the filter
            */
           void setFilterDamping(const double damping);

           /**
            * @brief setFilterOmega
            * @param omega rembember that: \f$ \omega = \frac{f}{2\pi} \f$ where \f$ f\f$ is the cut-off frequency.
            */
           void setFilterOmega(const double omega);

           /**
            * @brief isJointAdmittance
            * @param task a generic task pointer
            * @return true if task is a JointAdmittance task
            */
           static bool isJointAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief asJointAdmittance dynamic cast a generic task pointer to JointAdmittance task
            * @param task a generic task pointer
            * @return a pointer to a JointAdmittance task
            */
           static OpenSoT::tasks::velocity::JointAdmittance::Ptr asJointAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief setTorqueReference
            * @param tau_ref
            */
           void setTorqueReference(const Eigen::VectorXd& tau_ref);

           /**
            * @brief getTorqueReference
            * @return vector of tau_ref
            */
           const Eigen::VectorXd& getTorqueReference();

           /**
            * @brief getTorqueReference
            * @param tau_ref
            */
           void getTorqueReference(Eigen::VectorXd& tau_ref);

           /**
            * @brief reset
            * @return true if succeed
            */
           bool reset();



        private:
           XBot::ModelInterface& _model;

           void _update(const Eigen::VectorXd& x);

           XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _filter;

           Eigen::MatrixXd _C;

           Eigen::VectorXd _tau, _tau_filt, _q, _tau_error;
           Eigen::VectorXd _tau_ref;

           Eigen::VectorXd _deadzone;
           void apply_deadzone(Eigen::VectorXd& data);

       };

       }
   }
}

#endif
