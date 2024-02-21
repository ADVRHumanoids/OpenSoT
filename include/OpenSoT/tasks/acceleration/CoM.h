/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __OPENSOT_ACCELERATION_TASK_COM_H__
#define __OPENSOT_ACCELERATION_TASK_COM_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/common/utils.h>


namespace OpenSoT { namespace tasks { namespace acceleration {
/**
  * The CoM class implements a task that tries to control the acceleration
  * of the CoM w.r.t. world frame.
  */
    class CoM : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {

    public:

        typedef std::shared_ptr<CoM> Ptr;

        /**
         * @brief CoM
         * @param robot the robot model
         */
        CoM(const XBot::ModelInterface& robot);

        /**
         * @brief CoM
         * @param robot the robot model
         * @param qddot an affine structure variable
         */
        CoM(const XBot::ModelInterface& robot, const AffineHelper& qddot);

        /**
         * @brief getBaseLink
         * @return "world"
         */
        const std::string& getBaseLink() const;

        /**
         * @brief getDistalLink
         * @return "CoM
         */
        const std::string& getDistalLink() const;

        /**
         * @brief setReference sets a new reference for the CoM task.
         * The task error IS NOT recomputed immediately, you need to call the _update(x) function
         * Notice how the setReference(...) needs to be called before each _update(x) of the Cartesian task,
         * since THE _update() RESETS THE FEED-FORWARD VELOCITY and ACCELERATION TERMS for safety reasons.
         * @param pose_ref the \f$R^{3}\f$ desired position
         * @param vel_ref is a \f$R^{3}\f$ twist describing the desired trajectory velocity
         * @param acc_ref is a \f$R^{3}\f$ twist describing the desired trajectory acceleration, and it represents
         * a feed-forward term in the cartesian task computation.
         */
        void setReference(const Eigen::Vector3d& ref);
        void setReference(const Eigen::Vector3d& pose_ref,
                          const Eigen::Vector3d& vel_ref);
        void setReference(const Eigen::Vector3d& pose_ref,
                          const Eigen::Vector3d& vel_ref,
                          const Eigen::Vector3d& acc_ref);

        /**
         * @brief getReference
         * @param ref position reference
         */
        void getReference(Eigen::Vector3d& ref);

        /**
         * @brief getActualPose
         * @param actual pose of the CoM
         */
        void getActualPose(Eigen::Vector3d& actual);

        void getPosError(Eigen::Vector3d& error);

        bool reset() override;

        virtual void _update(const Eigen::VectorXd& x);

        virtual void _log(XBot::MatLogger2::Ptr logger);

        /**
         * @brief setLambda set position and velocity gains of the feedback errors
         * @param lambda1 postion gain
         * @param lambda2 velocity gain
         */
        void setLambda(double lambda1, double lambda2);

        /**
         * @brief setLambda set position gain of the feedback error
         * NOTE: velocity gain is set to 2*std::sqrt(lambda)
         * @param lambda postion gain
         */
        virtual void setLambda(double lambda);

        /**
         * @brief getLambda return both position and velocity convergence gains
         * @param lambda position gain
         * @param lambda2 velocity gain
         */
        void getLambda(double & lambda, double & lambda2);
        using Task::getLambda;

        /**
         * @brief getLambda2 return velocity convergence gain
         * @return lambda2 gain
         */
        const double getLambda2() const;

        /**
         * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal velcity reference
         */
        const Eigen::Vector3d& getCachedVelocityReference() const;

        /**
         * @brief getCachedAccelerationReference can be used to get Velocity reference after update(), it will reset
         * next update()
         * @return internal acceleration reference
         */
        const Eigen::Vector3d& getCachedAccelerationReference() const;

        const Eigen::Matrix3d& getKp() const;
        const Eigen::Matrix3d& getKd() const;
        void setKp(const Eigen::Matrix3d& Kp);
        void setKd(const Eigen::Matrix3d& Kd);
        void setGains(const Eigen::Matrix3d& Kp, const Eigen::Matrix3d& Kd);
        void getGains(Eigen::Matrix3d& Kp, Eigen::Matrix3d& Kd);

    private:

        static const std::string world_name;

        std::string _base_link, _distal_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _cartesian_task;

        Eigen::MatrixXd _J;
        Eigen::Vector3d _jdotqdot;

        Eigen::Matrix3d _Kp, _Kd;

        Eigen::Vector3d _pose_ref, _pose_current;
        Eigen::Vector3d _pose_error, _vel_ref, _vel_current, _acc_ref, _vel_ref_cached, _acc_ref_cached;

        double _lambda2;

        /**
         * @brief resetReference ste the actual position as the postion reference, set to zero
         * velocity and acceleration references
         */
        void resetReference();


    };

} } }






#endif
