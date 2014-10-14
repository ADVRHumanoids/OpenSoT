/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __BOUNDS_VELOCITY_VELOCITYLIMITS_H__
#define __BOUNDS_VELOCITY_VELOCITYLIMITS_H__

 #include <OpenSoT/Constraint.h>

 #include <yarp/sig/all.h>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The VelocityLimits class implements a bound on joint velocities
             */
            class VelocityLimits: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                double _qDotLimit;
                double _dT;
            public:
                /**
                 * @brief VelocityLimits constructor
                 * @param dT the time constant at which we are performing velocity control [s]
                 */
                VelocityLimits(const double qDotLimit,
                               const double dT,
                               const unsigned int x_size);
            };
        }
    }
 }

#endif
