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
 #include <kdl/frames.hpp>

 namespace OpenSoT {
    namespace tasks {
        namespace torque {

            class JointImpedanceCtrl : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<JointImpedanceCtrl> Ptr;
            protected:
                Eigen::VectorXd _x_desired;
                Eigen::VectorXd _xdot_desired;
                Eigen::VectorXd _x;
                Eigen::VectorXd _x_dot;

                XBot::ModelInterface& _robot;

                Eigen::MatrixXd _M;

                bool _use_inertia_matrix;

                Eigen::MatrixXd _K;
                Eigen::MatrixXd _D;

                void update_b();

            public:

                JointImpedanceCtrl(const Eigen::VectorXd& x, XBot::ModelInterface &robot);

                ~JointImpedanceCtrl();

                void _update(const Eigen::VectorXd& x);

                void setReference(const Eigen::VectorXd& x_desired);

                void setReference(const Eigen::VectorXd& x_desired,
                                  const Eigen::VectorXd& xdot_desired);

                Eigen::VectorXd getReference() const;

                void getReference(Eigen::VectorXd& x_desired,
                                  Eigen::VectorXd& xdot_desired) const;

                void setStiffness(const Eigen::MatrixXd& K);
                void setDamping(const Eigen::MatrixXd& D);
                void setStiffnessDamping(const Eigen::MatrixXd& K, const Eigen::MatrixXd& D);

                Eigen::MatrixXd getStiffness();
                Eigen::MatrixXd getDamping();
                void getStiffnessDamping(Eigen::MatrixXd& K, Eigen::MatrixXd& D);

                Eigen::VectorXd getActualPositions();

                Eigen::VectorXd getActualVelocities();

                Eigen::VectorXd getSpringForce();
                Eigen::VectorXd getDampingForce();

                void useInertiaMatrix(const bool use);

            };
        }
    }
 }

#endif
