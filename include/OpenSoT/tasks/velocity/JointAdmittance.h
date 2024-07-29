#ifndef __TASKS_VELOCITY_JOINT_ADMITTANCE_H__
#define __TASKS_VELOCITY_JOINT_ADMITTANCE_H__

#include <OpenSoT/tasks/velocity/Postural.h>

namespace OpenSoT {
   namespace tasks {
       namespace velocity {
       template <typename SignalType>
       /**
        * @brief SecondOrderFilter implements a canonical continuous-time
        * second order filter with transfer function
        *                   1
        * P(s) =  -----------------------,  w = natural frequency, eps = damping ratio
        *         (s/w)^2 + 2*eps/w*s + 1
        *
        * and discretized according to a trapezoidal (aka Tustin) scheme. This yields
        * a difference equation of the following form:
        *
        *      a0*y + a1*yd + a2*ydd = u + b1*ud + b2*udd
        *
        * where yd = y(k-1), ydd = y(k-2) and so on (d = delayed).
        */
       class SecondOrderFilter {

       public:

           typedef std::shared_ptr<SecondOrderFilter<SignalType>> Ptr;

           SecondOrderFilter():
               _omega(1.0),
               _eps(0.8),
               _ts(0.01),
               _reset_has_been_called(false)
           {
               computeCoeff();
           }

           SecondOrderFilter(double omega, double eps, double ts, const SignalType& initial_state):
               _omega(omega),
               _eps(eps),
               _ts(ts),
               _reset_has_been_called(false)
           {
               computeCoeff();
               reset(initial_state);
           }

           void reset(const SignalType& initial_state){
               _reset_has_been_called = true;
               _u = initial_state;
               _y = initial_state;
               _yd = initial_state;
               _ydd = initial_state;
               _udd = initial_state;
               _ud = initial_state;
           }

           const SignalType& process(const SignalType& input){

               if(!_reset_has_been_called) reset(input*0);


               _ydd = _yd;
               _yd = _y;
               _udd = _ud;
               _ud = _u;


               _u = input;
               _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

               return _y;
           }

           const SignalType& getOutput() const {
               return _y;
           }

           void setOmega(double omega){
               _omega = omega;
               computeCoeff();
           }

           double getOmega()
           {
               return _omega;
           }

           void setDamping(double eps){
               _eps = eps;
               computeCoeff();
           }

           double getDamping()
           {
               return _eps;
           }

           void setTimeStep(double ts){
               _ts = ts;
               computeCoeff();
           }

           double getTimeStep()
           {
               return _ts;
           }

       private:

           void computeCoeff()
           {
               _b1 = 2.0;
               _b2 = 1.0;

               _a0 = 1.0 + 4.0*_eps/(_omega*_ts) + 4.0/std::pow(_omega*_ts, 2.0);
               _a1 = 2 - 8.0/std::pow(_omega*_ts, 2.0);
               _a2 = 1.0 + 4.0/std::pow(_omega*_ts, 2.0) - 4.0*_eps/(_omega*_ts);

           }

           double _omega;
           double _eps;
           double _ts;

           double _b1, _b2;
           double _a0, _a1, _a2;

           bool _reset_has_been_called;

           SignalType _y, _yd, _ydd, _u, _ud, _udd;

       };

       /**
        * @brief The JointAdmittance class implements a simple admittance controller in velocity at the joints.
        * The implemented scheme is the following:
        *
        *  \f$ \boldsymbol{\Delta}\mathbf{q}_r = \mathbf{C}\boldsymbol{\Delta \tau} \f$
        *
        *  where \f$ \mathbf{C} \in \mathbb{R}^{n \times n}\f$ is the Compliance matrix.
        *  The torque error is computed as:
        *
        *  \f$ \boldsymbol{\Delta \tau} = \mathbf{h} - \boldsymbol{\tau}_m \f$
        *
        *  with \f$ \mathbf{h} \f$ containing the torques due to gravity and Coriolis/Centrifugal forces, computed from the
        *  robot feedback.
        *  The computed joint velocity reference is plugged in the postural task as a feed-forward desired joint velocity:
        *
        *  \f$ \boldsymbol{\Delta}\mathbf{q}_d = \boldsymbol{\Delta}\mathbf{q}_r + \lambda \left( \mathbf{q}_d  - \mathbf{q}\right) \\ \f$
        *
        */
       class JointAdmittance: public Postural {
        public:
           typedef std::shared_ptr<JointAdmittance> Ptr;

           /**
            * @brief JointAdmittance constructor
            * @param robot is updated with the actual measurements from the robot (used to cpmuted the non-linear terms \f$ \mathbf{h} \f$)
            * @param model is updated with the integrated state from the robot used to perform open-loop IK
            */
           JointAdmittance(XBot::ModelInterface &robot, XBot::ModelInterface &model);

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


        private:
           XBot::ModelInterface& _robot;
           XBot::ModelInterface& _model;

           void _update();

           SecondOrderFilter<Eigen::VectorXd> _filter;

           Eigen::MatrixXd _C;

           Eigen::VectorXd _tau, _tau_filt, _h, _q, _tau_error, _qdot_desired;

       };

       }
   }
}

#endif
