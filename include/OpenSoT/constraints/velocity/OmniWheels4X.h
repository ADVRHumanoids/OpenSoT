/*
 * Copyright (C) 2023
 * Author: Enrico Mingo Hoffman
 * email:  enricomingo@gmail.com
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

#ifndef __BOUNDS_OMNIWHEEL4X_H__
#define __BOUNDS_OMNIWHEEL4X_H__

 #include <OpenSoT/Constraint.h>
 #include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT {
   namespace constraints {
       namespace velocity {
       /**
        * @brief The OmniWheels4X class implements a constraint to map base velocities into (omni-)wheels for a 4 drive case.
        * The kinematic model is based on: "An admittance-controlled wheeled mobile manipulator for mobility assistance:
        * Human–robot interaction estimation and redundancy resolution for enhanced force exertion ability" by Hongjun Xing et al.,
        * Mechatronics, April 2021
        */
       class OmniWheels4X: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
       public:
           typedef std::shared_ptr<OmniWheels4X> Ptr;

           /**
            * @brief OmniWheel4X maps base velocities (XY-YAW) into wheels velocities
            *
            *                                 | x
            *                                 |
            *                            y____|
            *
            *                     l1
            *                   ------
            *            __          __
            *           |fl|________|fr|
            *           |__|        |__| |
            *              |        |    | l2
            *              |        |    |
            *              |        |
            *            __|        |__
            *           |hl|________|hr|
            *           |__|        |__|
            *
            *
            * @note we suppose the base_link at the center of the mobile base
            * @note we assume a floating base robot
            *
            * @param l1 y distance from the base frame to the center of the wheel (see figure)
            * @param l2 x distance from the base frame to the center of the wheel (see figure)
            * @param r wheel radius
            * @param joint_wheels_name ordered list of joint names representing the wheels [fl, fr, hl, hr]
            * @param base_link of the mobile base assumed to be at the center
            * @param x initial configuration of the robot when creating the constraint
            * @param robot the robot model
            */
           OmniWheels4X(const double l1, const double l2, const double r,
                       const std::vector<std::string> joint_wheels_name,
                       const std::string base_link,
                       XBot::ModelInterface& robot);
           /**
            * @brief update the constraint
            * @param x state vector
            */
           virtual void update();

           /**
            * @brief setIsGlobalVelocity set the flag to consider the velocity in the global frame
            * @param is_global_velocity default is false
            */
           void setIsGlobalVelocity(bool is_global_velocity) { _is_global_velocity = is_global_velocity; }

           /**
            * @brief getIsGlobalVelocity get the flag to consider the velocity in the global frame
            * @return true or false
            */
           bool getIsGlobalVelocity() const { return _is_global_velocity; }


       private:
           XBot::ModelInterface& _robot;
           Eigen::MatrixXd _J;
           Eigen::Affine3d _w_T_b;
           const std::string _base_link;
           bool _is_global_velocity;
       };

       }
   }
}

#endif
