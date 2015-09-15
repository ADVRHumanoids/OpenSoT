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

#ifndef __TASKS_VELOCITY_COM_H__
#define __TASKS_VELOCITY_COM_H__

#include <OpenSoT/Task.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>



 namespace OpenSoT {
    namespace tasks {
        namespace force {

            class CoM : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<CoM> Ptr;
            private:
                #define BASE_LINK_COM "world"
                #define DISTAL_LINK_COM "CoM"

                iDynUtils& _robot;

                /**
                 * @brief _g gravity vector in world frame
                 */
                yarp::sig::Vector _g;

                /**
                 * @brief _desiredAccelerationCoM in world frame computed as:
                 *
                 *  ddx_x = ddx_ref + K1(dx_ref-dx) + K2(x_ref-x)
                 */
                yarp::sig::Vector _desiredAcceleration;
                yarp::sig::Vector _desiredPosition;
                yarp::sig::Vector _desiredVelocity;

                yarp::sig::Vector _actualPosition;
                yarp::sig::Vector _actualVelocity;

                double _lambda2;

                void update_b();

            public:

                yarp::sig::Vector positionError;
                yarp::sig::Vector velocityError;

                /**
                 * @brief CoM
                 * @param x the initial configuration of the robot
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoM(const yarp::sig::Vector& x,
                    iDynUtils& robot);

                ~CoM();

                void _update(const yarp::sig::Vector& x);


                void setReference(const yarp::sig::Vector& desiredPosition);


                void setReference(const yarp::sig::Vector& desiredPosition,
                                  const yarp::sig::Vector& desiredVelocity);

                void setReference(const yarp::sig::Vector& desiredPosition,
                                  const yarp::sig::Vector& desiredVelocity,
                                  const yarp::sig::Vector& desiredAcceleration);



                yarp::sig::Vector getReference() const;

                void getReference(yarp::sig::Vector& desiredPosition,
                                  yarp::sig::Vector& desiredVelocity) const;

                void getReference(yarp::sig::Vector& desiredPosition,
                                  yarp::sig::Vector& desiredVelocity,
                                  yarp::sig::Vector& desiredAcceleration) const;


                yarp::sig::Vector getActualPosition() const;

                yarp::sig::Vector getActualVelocity() const;

                /**
                 * @brief getBaseLink an utility function that always returns "world"
                 * @return "world"
                 */
                std::string getBaseLink();

                /**
                 * @brief getDistalLink an utility function that always
                 * @return
                 */
                std::string getDistalLink();

                void setLambda(double lambda, double lambda2);


                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                yarp::sig::Vector getError();

                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                yarp::sig::Vector getVelocityError();

                yarp::sig::Matrix computeW(const std::vector<std::string>& ft_in_contact);

            };
        }
    }
 }

#endif
