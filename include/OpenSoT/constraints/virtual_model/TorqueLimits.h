/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo Hoffman
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

#ifndef __BOUNDS_VIRTUAL_MODEL_TORQUELIMITS_H__
#define __BOUNDS_VIRTUAL_MODEL_TORQUELIMITS_H__

 #include <OpenSoT/Constraint.h>

 #include <yarp/sig/all.h>

 namespace OpenSoT {
    namespace constraints {
        namespace virtual_model {

            class TorqueLimits: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<TorqueLimits> Ptr;
            private:

            public:
                TorqueLimits(const yarp::sig::Vector& torque_limits);


                void getTorqueLimits(yarp::sig::Vector& torque_limits);

                void setTorqueLimits(const yarp::sig::Vector& torque_limits);

            };
        }
    }
 }

#endif
