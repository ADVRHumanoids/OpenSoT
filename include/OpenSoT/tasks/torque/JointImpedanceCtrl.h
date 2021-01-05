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

#ifndef __TASKS_VIRTUAL_MODEL_JOINT_IMPEDANCE_CTRL_H__
#define __TASKS_VIRTUAL_MODEL_JOINT_IMPEDANCE_CTRL_H__


 #include <OpenSoT/Task.h>
 #include <XBotInterface/ModelInterface.h>

 namespace OpenSoT {
    namespace tasks {
        namespace torque {

            /**
             * @brief The JointImpedanceCtrl class implements a joint impedance controller:
             *  \f$ \boldsymbol{\tau} = \mathbf{K}\left(\mathbf{q}_d-\mathbf{q}\right) + \mathbf{D}\left(\mathbf{\dot{q}}_d-\mathbf{\dot{q}}\right) \f$
             *
             * NOTE: if used in the null-space, it realizes the NULL-SPACE Stiffness as in:
             * "Cartesian Impedance Control of Redundant and Flexible-Joint Robots", by. Christian Ott
             */
            class JointImpedanceCtrl : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<JointImpedanceCtrl> Ptr;
            protected:
                Eigen::VectorXd _x_desired;
                Eigen::VectorXd _xdot_desired;
                Eigen::VectorXd _x;
                Eigen::VectorXd _x_dot;

                Eigen::VectorXd _spring_force;
                Eigen::VectorXd _damping_force;

                XBot::ModelInterface& _robot;

                bool _use_inertia_matrix;

                Eigen::MatrixXd _K;
                Eigen::MatrixXd _D;

                void update_b();

                virtual void _log(XBot::MatLogger2::Ptr logger);

                /**
                 * @brief _update
                 * @param x ///TODO: Remove this!!!
                 */
                void _update(const Eigen::VectorXd& x);


            public:

                /**
                 * @brief JointImpedanceCtrl contructor
                 * @param x actual joint values ///TODO: Remove this!
                 * @param robot a robot model
                 */
                JointImpedanceCtrl(const Eigen::VectorXd& x, XBot::ModelInterface &robot);

                ~JointImpedanceCtrl();

                /**
                 * @brief setReference set deisred joint space configuration
                 * @param x_desired a vector of joint values
                 */
                void setReference(const Eigen::VectorXd& x_desired);

                /**
                 * @brief setReference set deisred joint space configuration and velocities
                 * @param x_desired a vector of joint values
                 * @param xdot_desired a vector of joint velocity values
                 */
                void setReference(const Eigen::VectorXd& x_desired,
                                  const Eigen::VectorXd& xdot_desired);

                /**
                 * @brief getReference return the vector of desired joint space values
                 * @param reference a vector of joint values
                 */
                void getReference(Eigen::VectorXd& reference) const;

                /**
                 * @brief getReference return the vector of desired joint space values and velocities
                 * @param x_desired a vector of joint values
                 * @param xdot_desired a vector of joint velocity values
                 */
                void getReference(Eigen::VectorXd& x_desired,
                                  Eigen::VectorXd& xdot_desired) const;

                /**
                 * @brief setStiffness a matrix of desired joint stiffness
                 * @param K matrix of joint stiffness
                 * NOTE: K has to be PD!
                 */
                void setStiffness(const Eigen::MatrixXd& K);

                /**
                 * @brief setDamping a matrix of desired joint damping
                 * @param D matrix of joint damping
                 * NOTE: D has to be PD!
                 */
                void setDamping(const Eigen::MatrixXd& D);

                /**
                 * @brief setStiffnessDamping set stiffness and damping matrices together
                 * @param K matrix of joint stiffness
                 * @param D matrix of joint damping
                 */
                void setStiffnessDamping(const Eigen::MatrixXd& K, const Eigen::MatrixXd& D);

                /**
                 * @brief getStiffness return joint stiffness matrix
                 * @param K matrix of joint stiffness
                 */
                void getStiffness(Eigen::MatrixXd& K);

                /**
                 * @brief getDamping return joint damping matrix
                 * @param D matrix of joint damping
                 */
                void getDamping(Eigen::MatrixXd& D);

                /**
                 * @brief getStiffnessDamping return joint stiffness and damping matrices
                 * @param K matrix of joint stiffness
                 * @param D matrix of joint damping
                 */
                void getStiffnessDamping(Eigen::MatrixXd& K, Eigen::MatrixXd& D);

                /**
                 * @brief getActualPositions return the vector of actual joint values
                 * @param actual_positions vector of joint values
                 */
                void getActualPositions(Eigen::VectorXd& actual_positions);

                /**
                 * @brief getActualVelocities return the vector of actual joint velocities
                 * @param actual_velocities vector of joint velocities
                 */
                void getActualVelocities(Eigen::VectorXd& actual_velocities);

                /**
                 * @brief getSpringForce return the spring force given by:
                 * \f$ \boldsymbol{\tau}_s = \mathbf{K}\left(\mathbf{q}_d-\mathbf{q}\right)\f$
                 * @param spring_force vector of joint torques
                 */
                void getSpringForce(Eigen::VectorXd& spring_force);

                /**
                 * @brief getDampingForce return the spring damping given by:
                 * \f$\boldsymbol{\tau}_d = \mathbf{D}\left(\mathbf{\dot{q}}_d-\mathbf{\dot{q}}\right) \f$
                 * @param damping_force vector of joint torques
                 */
                void getDampingForce(Eigen::VectorXd& damping_force);

                /**
                 * @brief useInertiaMatrix if true, the weight for the task is set to \f$M^{-1}\f$
                 * @param use true or false
                 * NOTE: to have proper handling of priorities, this should be set to TRUE!
                 * TODO: Remove this, \f$M^{-1}\f$ should be passed from outside!
                 */
                void useInertiaMatrix(const bool use);

            };
        }
    }
 }

#endif
