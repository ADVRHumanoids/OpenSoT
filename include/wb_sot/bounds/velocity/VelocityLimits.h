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

 #include <wb_sot/Bounds.h>

 #include <yarp/sig/all.h>

 namespace wb_sot {
    namespace bounds {
        namespace velocity {
            class VelocityLimits: public Bounds<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                double _qDotLimit;
                double _dT;
            public:
                /**
                 * @brief VelocityLimits constructor
                 * @param robot the robot model which includes joint limits
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
