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
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>


namespace OpenSoT { namespace tasks { namespace acceleration {
/**
  * The CoM class implements a task that tries to control the acceleration
  * of the CoM w.r.t. world frame.
  */
    class CoM : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {

    public:

        typedef boost::shared_ptr<CoM> Ptr;

        /**
         * @brief CoM
         * @param robot the robot model
         * @param x vector of size of joints of the robot (not used anymore)
         */
        CoM(const XBot::ModelInterface& robot, const Eigen::VectorXd& x);

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
         * @brief setReference set a position reference for the CoM
         * @param ref
         */
        void setReference(const Eigen::Vector3d& ref);

        /**
         * @brief setReference position and velocity reference for the CoM
         * @param pose_ref
         * @param vel_ref
         */
        void setReference(const Eigen::Vector3d& pose_ref,
                          const Eigen::Vector3d& vel_ref);

        /**
         * @brief setReference position, velocity and acceleration references for the CoM
         * @param pose_ref
         * @param vel_ref
         * @param acc_ref
         */
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

        /**
         * @brief resetReference ste the actual position as the postion reference, set to zero
         * velocity and acceleration references
         */
        void resetReference();

        virtual void _update(const Eigen::VectorXd& x);

        virtual void _log(XBot::MatLogger::Ptr logger);

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

    private:

        static const std::string world_name;

        std::string _base_link, _distal_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _cartesian_task;

        Eigen::MatrixXd _J;
        Eigen::Vector3d _jdotqdot;

        Eigen::Vector3d _pose_ref, _pose_current;
        Eigen::Vector3d _pose_error, _vel_ref, _vel_current, _acc_ref;

        double _lambda2;

    };

} } }






#endif
