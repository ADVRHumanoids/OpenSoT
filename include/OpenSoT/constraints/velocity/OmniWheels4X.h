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
 #include <XBotInterface/ModelInterface.h>

namespace OpenSoT {
   namespace constraints {
       namespace velocity {
       class OmniWheels4X: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
       public:
           typedef std::shared_ptr<OmniWheels4X> Ptr;

           /**
            * @brief OmniWheel4X maps base velocities (XY-YAW) into wheels velocities
            *
            *                 l1
            *               ----
            *           O------O |
            *           |      | | l2
            *           |      | |
            *           |      |
            *           |      |
            *           O------O      ^ x
            *                         |
            *                    y <--|
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
                       const Eigen::VectorXd& x,
                       XBot::ModelInterface& robot);

           virtual void update(const Eigen::VectorXd &x);


       private:
           XBot::ModelInterface& _robot;
           Eigen::MatrixXd _J;
           Eigen::Affine3d _w_T_b;
           const std::string _base_link;
       };

       }
   }
}

#endif
