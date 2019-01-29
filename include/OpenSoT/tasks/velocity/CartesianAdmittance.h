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
            * @brief getCartesianCompliance
            * @return the actual Compliance matrix
            */
           const Eigen::Matrix6d& getCartesianCompliance();

           /**
            * @brief getCartesianCompliance
            * @param C the actual Compliance matrix
            */
           void getCartesianCompliance(Eigen::Matrix6d& C);

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

           /**
            * @brief getStiffness
            * @return actual Cartesian Stiffness
            */
           const Eigen::Matrix6d getStiffness();
           
           /**
            * @brief getStiffness
            * @return actual Cartesian Stiffness
            */
           const Eigen::Matrix6d getInertia();

           /**
            * @brief getDamping
            * @return actual Cartesian Damping
            */
           const Eigen::Matrix6d getDamping();
           
           double getFilterTimeStep();
           
           void setFilterDamping(const double damping);
           
           /**
           * @brief Set impedance parameters to be emulated via admittance control.
           * Stiffness and lambda MUST BE POSITIVE!
           * Higher values of lambda will make the robot feel lighter, but 
           * will also destabilize the system.
           * 
           * @param K Stiffness
           * @param D Damping
           * @param lambda Lambda
           * @param dt Control period
           */
           void setImpedanceParams(const Eigen::Vector6d& K, 
                                   const Eigen::Vector6d& D, 
                                   const double lambda,
                                   const double dt);
           
           /**
           * @brief Set internal controller parameters implemeting
           * the law: 
           * 
           *    dx = lambda*(xd-x)+C*F_filt*dt
           *    F_filt = 1 / (1 + s/omega) * F
           * 
           * @param C compliance 
           * @param omega filter cutoff
           * @param lambda position feedback
           * @param dt control period
           */
           bool setRawParams(const Eigen::Vector6d& C, 
                             const Eigen::Vector6d& omega, 
                             const double lambda,
                             const double dt);
           
           bool setDeadZone(const Eigen::Vector6d& dead_zone_amplitude);

           /**
            * @brief computeParameters given user's K, D and lambda, computes M and w
            * @param K user desired Cartesian Stiffness
            * @param D user desired Cartesian Damping
            * @param lambda user desired lambda
            * @param dt filter time step
            * @param C Cartesian Compliance \f$ \mathbf{C} = \lambda\mathbf{K}^{-1} \f$
            * @param M Cartesian Mass \f$ \mathbf{M} = \frac{dt}{\lambda}\left( \mathbf{D} - \mathbf{D}dt \right) \f$
            * @param w filter params \f$ \mathbf{w} = dt\left( \mathbf{CM} \right)^{-1} \f$
            * @return true
            */
           bool computeParameters(const Eigen::Vector6d& K, 
                                  const Eigen::Vector6d& D, 
                                  const double lambda, 
                                  const double dt,
                                  Eigen::Vector6d& C, 
                                  Eigen::Vector6d& M, 
                                  Eigen::Vector6d& w);

           /**
            * @brief setLambda do nothing in CartesianAdmittance, use instead the setRawParams() or
            * the setImpedanceParams()
            * @param lambda
            */
           void setLambda(double lambda){XBot::Logger::warning("setLambda do nothing in CartesianAdmittance, use instead the setRawParams() or the setImpedanceParams()");}


         private:
             
           void apply_deadzone(Eigen::Vector6d& data);
             
           Eigen::Vector6d _wrench_reference;
           Eigen::Vector6d _wrench_measured;
           Eigen::Vector6d _wrench_filt;
           Eigen::Vector6d _wrench_error;
           Eigen::Vector6d _deadzone;
           std::vector<double> _tmp;

           void _update(const Eigen::VectorXd& x);

           //XBot::Utils::SecondOrderFilter<Eigen::Vector6d> _filter;
           XBot::Utils::SecondOrderFilterArray<double> _filter;

           Eigen::Vector6d _C;

           /**
            * @brief _K equivalent Cartesian Stiffness (diagonal)
            * \f$ \mathbf{K} = \lambda\mathbf{C}^{-1} \f$
            */
           Eigen::Vector6d _K;

           /**
            * @brief _M equivalent Cartesian Inertia (diagonal)
            * \f$ \mathbf{M} = dt\left( \mathbf{Cw} \right)^{-1} \f$
            * where \f$ \mathbf{w} \f$ is the diagonal matrix of the filter's \f$ w \f$
            */
           Eigen::Vector6d _M;

           /**
            * @brief _D equivalent Cartesian Damping (diagonal)
            * \f$ \mathbf{D} = \lambda\left( \mathbf{Cw} \right)^{-1} + \mathbf{C}dt = \frac{\lambda}{dt}\mathbf{M} + \mathbf{C}dt  \f$
            */
           Eigen::Vector6d _D;

           /**
            * @brief _w diagonal matrix containing the omega params of the filters
            */
           Eigen::Vector6d _w;

           Eigen::Vector6d _tmp_vec6;
           Eigen::Matrix6d _tmp_mat6;


           XBot::ForceTorqueSensor::ConstPtr _ft_sensor;

           Eigen::Affine3d _bl_T_ft;

           /**
            * @brief _dt filter tim_stamp
            */
           double _dt;

           void setFilterOmega(const Eigen::Vector6d& w);


       };

       }
   }
}

#endif
