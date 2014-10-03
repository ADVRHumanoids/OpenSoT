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

 #include <yarp/sig/all.h>
 #include <drc_shared/idynutils.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            class CoMVelocity: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                iDynUtils _robot;
                yarp::sig::Vector _velocityLimits;
                int _support_foot_linkIndex;
                double _dT;
            public:
                /**
                 * @brief CoMVelocity constructor
                 * @param robot the robot model
                 * @param dT the time constant at which we are performing velocity control [s]
                 */
                CoMVelocity(const yarp::sig::Vector velocityLimits,
                            const iDynUtils &robot,
                            const double dT,
                            const unsigned int x_size);

                virtual void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
