#ifndef __TASKS_VELOCITY_CARTESIAN_ADMITTANCE_H__
#define __TASKS_VELOCITY_CARTESIAN_ADMITTANCE_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>

using namespace XBot::Utils;

namespace OpenSoT {
   namespace tasks {
       namespace velocity {
       /**
        * @brief The CartesianAdmittance class implements a simple admittance controller in velocity at the end-effectors.
        * The implemented scheme is the following:
        *
        *  \f$ \boldsymbol{\Delta}\mathbf{x}_r = \mathbf{C}\boldsymbol{\Delta}\mathbf{F} \f$
        *
        *  where \f$ \mathbf{C} \in \mathbb{R}^{6 \times 6}\f$ is the Compliance matrix.
        *  The torque error is computed as:
        *
        *  \f$ \boldsymbol{\Delta}\mathbf{F} = \mathbf{F}_d - \mathbf{F}_m \f$
        *
        *  with \f$ \mathbf{F}_m \f$ transformed in the base frame of the Cartesian task.
        *  The computed Cartesian velocity reference is plugged in the Cartesian task as a feed-forward desired Cartesian velocity:
        *
        *  \f$ \boldsymbol{\Delta}\mathbf{x}_d = \boldsymbol{\Delta}\mathbf{x}_r + \lambda \left( \mathbf{x}_d  - \mathbf{x}\right) \\ \f$
        *
        *  NOTE: the task distal_link is the same link of the force/torque sensor
        */
       class CartesianAdmittance: public Cartesian {
         public:
            typedef boost::shared_ptr<CartesianAdmittance> Ptr;

           /**
             * @brief CartesianAdmittance constructor
             * @param task_id name of the task
             * @param x the robot configuration
             * @param robot model
             * @param base_link base link of the task
             * @param ft_sensor used to get measured contact forces and retrieve the distal_link
             */
            CartesianAdmittance(std::string task_id,
                                const Eigen::VectorXd& x,
                                XBot::ModelInterface &robot,
                                std::string base_link,
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor);

            /**
            * @brief setCartesianCompliance set the Compliance matrix
            * @param C a SPD matrix
            */
           void setCartesianCompliance(const Eigen::Matrix6d& C);

           /**
            * @brief setCartesianCompliance set the same compiance to linear and angular the Cartesian direction
            * respectively
            * @param C_linear the Compliance in th linear part
            * @param C_angular the Compliance in th angular part
            */
           void setCartesianCompliance(const double C_linear, const double C_angular);

           /**
            * @brief getCartesianCompliance
            * @return the actual Compliance matrix
            */
           const Eigen::Matrix6d& getCartesianCompliance();

           /**
            * @brief getCartesianCompliance
            * @param C the actual Compliance matrix
            */
           void getCartesianCompliance(Eigen::MatrixXd& C);

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
            * @brief setFilterOmega set the same omega to all the filter channels
            * @param omega rembember that: \f$ \omega = \frac{f}{2\pi} \f$ where \f$ f\f$ is the cut-off frequency.
            */
           void setFilterOmega(const double omega);

           /**
            * @brief setFilterOmega to a certain filter channel
            * @param omega rembember that: \f$ \omega = \frac{f}{2\pi} \f$ where \f$ f\f$ is the cut-off frequency.
            * @param channel
            * @return false if channel does not exists
            */
           bool setFilterOmega(const double omega, const int channel);

           /**
            * @brief setWrenchReference set the desired wrench at the controlled distal_link
            * @param wrench
            */
           void setWrenchReference(const Eigen::Vector6d& wrench);

           /**
            * @brief getWrenchReference
            * @return the actual reference wrench
            */
           const Eigen::Vector6d& getWrenchReference();

           /**
            * @brief getWrenchReference
            * @param wrench_reference the actual reference wrench
            */
           void getWrenchReference(Eigen::Vector6d& wrench_reference);

           /**
            * @brief isCartesianAdmittance
            * @param task a generic task pointer
            * @return true if task is a JointAdmittance task
            */
           static bool isCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief asCartesianAdmittance
            * @param task a generic task pointer
            * @return a pointer to a JointAdmittance task
            */
           static OpenSoT::tasks::velocity::CartesianAdmittance::Ptr asCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

           /**
            * @brief reset is used to reset the internal references of the Task (and Cartesian Task)
            * The reference wrench is reset to the one measured by the Force/Torque sensors
            * @return true
            */
           bool reset();

           void _log(XBot::MatLogger::Ptr logger);

         private:
           Eigen::Vector6d _wrench_reference;
           Eigen::Vector6d _wrench_measured;
           Eigen::Vector6d _wrench_filt;
           Eigen::Vector6d _wrench_error;
           std::vector<double> _tmp;

           void _update(const Eigen::VectorXd& x);

           //XBot::Utils::SecondOrderFilter<Eigen::Vector6d> _filter;
           XBot::Utils::SecondOrderFilterArray<double> _filter;

           Eigen::Matrix6d _C;

           XBot::ForceTorqueSensor::ConstPtr _ft_sensor;

           Eigen::Affine3d _bl_T_ft;


       };

       }
   }
}

#endif
