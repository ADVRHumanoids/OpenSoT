/*
 * Copyright (C) 2016 Cogimon
 * Authors: Enrico Mingo Hoffman, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#ifndef __TASKS_VIRTUAL_MODEL_JOINT_SPRING_DAMPER_H__
#define __TASKS_VIRTUAL_MODEL_JOINT_SPRING_DAMPER_H__


 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 namespace OpenSoT {
    namespace tasks {
        namespace virtual_model {

            class JointSpringDamper : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<JointSpringDamper> Ptr;
            protected:
                yarp::sig::Vector _x_desired;
                yarp::sig::Vector _xdot_desired;
                yarp::sig::Vector _x;
                yarp::sig::Vector _x_dot;

                iDynUtils& _robot;

                yarp::sig::Matrix _M;

                bool _use_inertia_matrix;

                yarp::sig::Matrix _K;
                yarp::sig::Matrix _D;

                void update_b();

            public:

                JointSpringDamper(const yarp::sig::Vector& x, iDynUtils &robot);

                ~JointSpringDamper();

                void _update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Vector& x_desired);

                void setReference(const yarp::sig::Vector& x_desired,
                                  const yarp::sig::Vector& xdot_desired);

                yarp::sig::Vector getReference() const;

                void getReference(yarp::sig::Vector& x_desired,
                                  yarp::sig::Vector& xdot_desired) const;

                void setStiffness(const yarp::sig::Matrix& K);
                void setDamping(const yarp::sig::Matrix& D);
                void setStiffnessDamping(const yarp::sig::Matrix& K, const yarp::sig::Matrix& D);

                yarp::sig::Matrix getStiffness();
                yarp::sig::Matrix getDamping();
                void getStiffnessDamping(yarp::sig::Matrix& K, yarp::sig::Matrix& D);

                yarp::sig::Vector getActualPositions();

                yarp::sig::Vector getActualVelocities();

                yarp::sig::Vector getSpringForce();
                yarp::sig::Vector getDampingForce();

                void useInertiaMatrix(const bool use);

            };
        }
    }
 }

#endif
