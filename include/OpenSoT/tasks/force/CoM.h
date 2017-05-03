/*
 * Copyright (C) 2014 Walkman
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

#ifndef __TASKS_FORCE_COM_H__
#define __TASKS_FORCE_COM_H__

#include <OpenSoT/Task.h>
#include <XBotInterface/ModelInterface.h>
#include <kdl/frames.hpp>


 namespace OpenSoT {
    namespace tasks {
        namespace force {

            /**
             * @brief The CoM task computes the wrenches at the contact, in world frame, in
             * order to realize a certain acceleration and variation of angular momentum
             * at the CoM considering the Centroidal Dynamics:
             *
             *      m*ddr = sum(f) + mg
             *      dL = sum(pxf + tau)
             *
             * Note: variables are wrenches: w = [f tau]'
             *
             */
            class CoM : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<CoM> Ptr;
            private:
                #define BASE_LINK_COM "world"
                #define DISTAL_LINK_COM "CoM"

                XBot::ModelInterface& _robot;

                /**
                 * @brief _g gravity vector in world frame
                 */
                Eigen::Vector3d _g;

                /**
                 * @brief _desiredAcceleration (linear) of CoM in world frame computed as:
                 *
                 *  ddr = ddr_ref + K1(dr_ref-dr) + K2(r_ref-r)
                 *
                 */
                Eigen::Vector3d _desiredAcceleration;
                Eigen::Vector3d _desiredPosition;
                Eigen::Vector3d _desiredVelocity;

                Eigen::Vector3d _desiredVariationAngularMomentum;
                Eigen::Vector3d _desiredAngularMomentum;

                Eigen::Vector3d _actualPosition;
                Eigen::Vector3d _actualVelocity;
                Eigen::Vector3d _actualAngularMomentum;

                Eigen::Vector6d _centroidalMomentum;

                Eigen::MatrixXd A;
                Eigen::MatrixXd B;

                double _lambda2;

                double _lambdaAngularMomentum;

                void update_b();

                std::vector<std::string> _links_in_contact;

                Eigen::Matrix3d _I;
                Eigen::Matrix3d _P;
                Eigen::Affine3d _T;
                Eigen::Matrix3d _O;

            public:

                Eigen::Vector3d positionError;
                Eigen::Vector3d velocityError;
                Eigen::Vector3d angularMomentumError;

                /**
                 * @brief CoM
                 * @param x the initial configuration of the robot
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoM(const Eigen::VectorXd& x, std::vector<std::string>& links_in_contact,
                    XBot::ModelInterface& robot);

                ~CoM();

                void _update(const Eigen::VectorXd& x);


                void setLinearReference(const Eigen::Vector3d& desiredPosition);


                void setLinearReference(const Eigen::Vector3d& desiredPosition,
                                  const Eigen::Vector3d& desiredVelocity);

                void setLinearReference(const Eigen::Vector3d& desiredPosition,
                                  const Eigen::Vector3d& desiredVelocity,
                                  const Eigen::Vector3d& desiredAcceleration);

                void setAngularReference(const Eigen::Vector3d& desiredAngularMomentum);

                void setAngularReference(const Eigen::Vector3d& desiredAngularMomentum,
                                         const Eigen::Vector3d& desiredVariationAngularMomentum);


                void setLinksInContact(const std::vector<std::string>& links_in_contact);
                std::vector<std::string> getLinksInContact();



                Eigen::Vector3d getLinearReference() const;

                void getLinearReference(Eigen::Vector3d& desiredPosition,
                                  Eigen::Vector3d& desiredVelocity) const;

                void getLinearReference(Eigen::Vector3d& desiredPosition,
                                  Eigen::Vector3d& desiredVelocity,
                                  Eigen::Vector3d& desiredAcceleration) const;

                Eigen::Vector3d getAngularReference() const;

                void getAngularReference(Eigen::Vector3d& desiredAngularMomentum,
                                  Eigen::Vector3d& desiredVariationAngularMomentum) const;


                Eigen::Vector3d getActualPosition() const;

                Eigen::Vector3d getActualVelocity() const;

                Eigen::Vector3d getActualAngularMomentum() const;

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

                void setLambda(double lambda, double lambda2, double lambdaAngularMomentum);


                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                Eigen::Vector3d getError();

                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                Eigen::Vector3d getVelocityError();

                Eigen::Vector3d getAngularMomentumError();

                Eigen::MatrixXd computeW(const std::vector<std::string>& links_in_contact);

            };
        }
    }
 }

#endif
