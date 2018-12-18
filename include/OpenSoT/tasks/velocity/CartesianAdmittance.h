#ifndef __TASKS_VELOCITY_CARTESIAN_ADMITTANCE_H__
#define __TASKS_VELOCITY_CARTESIAN_ADMITTANCE_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {

       class CartesianAdmittance: public Cartesian {
         public:
            typedef boost::shared_ptr<CartesianAdmittance> Ptr;

            CartesianAdmittance(std::string task_id,
                                const Eigen::VectorXd& x,
                                XBot::ModelInterface &robot,
                                std::string base_link,
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor);

           void setCartesianCompliance(const Eigen::Matrix6d& C);
           void setCartesianCompliance(const double C_linear, const double C_angular);
           const Eigen::Matrix6d& getCartesianCompliance();
           void getCartesianCompliance(Eigen::MatrixXd& C);

           void setFilterParams(const double time_step, const double damping, const double omega);
           void setFilterTimeStep(const double time_step);
           void setFilterDamping(const double damping);
           void setFilterOmega(const double omega);

           void setWrenchReference(const Eigen::Vector6d& wrench);
           const Eigen::Vector6d& getWrenchReference();
           void getWrenchReference(Eigen::Vector6d& wrench_reference);

           static bool isCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
           static OpenSoT::tasks::velocity::CartesianAdmittance::Ptr asCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           bool reset();

         private:
           Eigen::Vector6d _wrench_reference;
           Eigen::Vector6d _wrench_measured;
           Eigen::Vector6d _wrench_filt;
           Eigen::Vector6d _wrench_error;

           void _update(const Eigen::VectorXd& x);

           XBot::Utils::SecondOrderFilter<Eigen::Vector6d> _filter;
           Eigen::Matrix6d _C;

           XBot::ForceTorqueSensor::ConstPtr _ft_sensor;

           Eigen::Affine3d _bl_T_ft;


       };

       }
   }
}

#endif
