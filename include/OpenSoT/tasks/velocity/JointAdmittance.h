#ifndef __TASKS_VELOCITY_JOINT_ADMITTANCE_H__
#define __TASKS_VELOCITY_JOINT_ADMITTANCE_H__

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {

       class JointAdmittance: public Postural {
        public:
           typedef boost::shared_ptr<JointAdmittance> Ptr;

           /**
            * @brief JointAdmittance
            * @param robot is updated with the actual measurements from the robot
            * @param model is updated with the integrated state from the robot
            */
           JointAdmittance(XBot::ModelInterface &robot, XBot::ModelInterface &model, const Eigen::VectorXd& x);

           void setJointCompliance(const Eigen::MatrixXd& C);
           void setJointCompliance(const double C);
           const Eigen::MatrixXd& getJointCompliance();
           void getJointCompliance(Eigen::MatrixXd& C);

           void setFilterParams(const double time_step, const double damping, const double omega);
           void setFilterTimeStep(const double time_step);
           void setFilterDamping(const double damping);
           void setFilterOmega(const double omega);


        private:
           XBot::ModelInterface& _robot;
           XBot::ModelInterface& _model;

           void _update(const Eigen::VectorXd& x);

           XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _filter;

           Eigen::MatrixXd _C;

           Eigen::VectorXd _tau, _tau_filt, _h, _q;

       };

       }
   }
}

#endif
