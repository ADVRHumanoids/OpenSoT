/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#ifndef __BOUNDS_VELOCITY_COMVELOCITY_H__
#define __BOUNDS_VELOCITY_COMVELOCITY_H__

 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/CoM.h>
 #include <XBotInterface/ModelInterface.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            class CoMVelocity: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<CoMVelocity> Ptr;
            private:
                XBot::ModelInterface& _robot;
                Eigen::Vector3d _velocityLimits;
                double _dT;

                void generatebBounds();

            public:
                /**
                 * @brief CoMVelocity constructor
                 * @param velocityLimits a vector of 3 elements describing the maximum velocity along x,y,z of the CoM.
                 * @param dT the time constant at which we are performing velocity control [s]
                 * @param x initial configuration of the robot when creating the constraint
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoMVelocity(const Eigen::Vector3d velocityLimits,
                            const double dT,
                            const Eigen::VectorXd& x,
                            XBot::ModelInterface& robot);

                virtual void update(const Eigen::VectorXd &x);

                Eigen::Vector3d getVelocityLimits();
                void setVelocityLimits(const Eigen::Vector3d velocityLimits);
            };
        }
    }
 }

#endif
