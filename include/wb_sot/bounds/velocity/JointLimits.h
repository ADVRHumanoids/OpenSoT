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

#ifndef __BOUNDS_VELOCITY_JOINTLIMITS_H__
#define __BOUNDS_VELOCITY_JOINTLIMITS_H__

 #include <wb_sot/Bounds.h>

 #include <yarp/sig/all.h>
 #include <iCub/iDynTree/DynTree.h>

 namespace wb_sot {
    namespace bounds {
        namespace velocity {
            class JointLimits: public Bounds<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                double _boundScaling;
                yarp::sig::Vector _jointLimitsMin;
                yarp::sig::Vector _jointLimitsMax;
            public:
                /**
                 * @brief JointLimits constructor
                 * @param q the configuration of the robot when
                 *          creating the joint limits constraint
                 * @param jointBoundMax upper bounds for joint limits
                 * @param jointBounMin lower bounds for joint limits
                 */
                JointLimits(const yarp::sig::Vector &q,
                            const yarp::sig::Vector &jointBoundMax,
                            const yarp::sig::Vector &jointBoundMin,
                            const double boundScaling = 1.0);

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
