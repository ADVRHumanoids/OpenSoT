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

#ifndef _OPENSOT_TASK_FLOATING_BASE_IMU_
#define _OPENSOT_TASK_FLOATING_BASE_IMU_

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT{
    namespace tasks{
        namespace floating_base{
        /**
         * @brief The IMU class estimates the orientation velocities of the floating base from the
         * angular velocities of the IMU.
         * NOTE: we consider the IMU attached to the floating_base link
         */
        class IMU: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{
            public:
                typedef std::shared_ptr<IMU> Ptr;
            /**
                 * @brief IMU accept a robot model and an imu, throws an error if the imu is
                 * not attached to the floating_base
                 * @param robot
                 * @param imu
                 */
                IMU(XBot::ModelInterface& robot, XBot::ImuSensor::ConstPtr imu);
                ~IMU();

                void _update();

            private:
                Eigen::MatrixXd _J;
                XBot::ModelInterface& _robot;
                std::string _fb_link;
                XBot::ImuSensor::ConstPtr _imu;
                Eigen::Vector3d _angular_velocity;
        };

        }
    }
}

#endif
