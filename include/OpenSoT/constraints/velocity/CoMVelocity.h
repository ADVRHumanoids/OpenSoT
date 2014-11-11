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
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            class CoMVelocity: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<CoMVelocity> Ptr;
            private:
                iDynUtils& _robot;
                yarp::sig::Vector _velocityLimits;
                double _dT;

                void generatebBounds();

            public:
                /**
                 * @brief CoMVelocity constructor
                 * @param velocityLimits a vector of 3 elements describing the maximum velocity along x,y,z of the CoM.
                 * The CoM frame of reference is that of the support foot
                 * @param dT the time constant at which we are performing velocity control [s]
                 * @param x initial configuration of the robot when creating the constraint
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoMVelocity(const yarp::sig::Vector velocityLimits,
                            const double dT,
                            const yarp::sig::Vector& x,
                            iDynUtils& robot);

                virtual void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
