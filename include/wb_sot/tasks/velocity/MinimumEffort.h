/*
 * Copyright (C) 2014 Walkman
 * Authors:Alessio Rocchi, Enrico Mingo
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

#ifndef __TASKS_VELOCITY_MINIMUMEFFORT_H__
#define __TASKS_VELOCITY_MINIMUMEFFORT_H__

 #include <wb_sot/Task.h>
 #include <drc_shared/idynutils.h>
 #include <drc_shared/cartesian_utils.h>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>

 using namespace yarp::math;

 namespace wb_sot {
    namespace tasks {
        namespace velocity {
            class MinimumEffort : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            private:
                yarp::sig::Vector _x_desired;

                iDynUtils _robot;

                class ComputeGTauGradient : public cartesian_utils::CostFunction {
                    public:
                    iDynUtils _robot;
                    yarp::sig::Vector _q;
                    yarp::sig::Matrix _W;
                    ComputeGTauGradient(const yarp::sig::Vector& q) :
                        _q(q), _W(_q.size(),_q.size())
                    {;}
                    double compute(const yarp::sig::Vector &q) {
                        _robot.updateiDyn3Model(_q, true);
                        yarp::sig::Vector tau = _robot.coman_iDyn3.getTorques();
                        return yarp::math::dot(tau, _W * tau);
                    }

                    void setW(const yarp::sig::Matrix& W) { _W = W; }
                };

                ComputeGTauGradient _gTauGradientWorker;

            public:

                MinimumEffort(const yarp::sig::Vector& x);

                ~MinimumEffort();

                void update(const yarp::sig::Vector& x);
            };
        }
    }
 }

#endif
