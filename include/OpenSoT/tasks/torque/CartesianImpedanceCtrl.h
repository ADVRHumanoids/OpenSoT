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

#ifndef __TASKS_VIRTUAL_MODEL_CARTESIAN_IMPEDANCE_CTRL_H__
#define __TASKS_VIRTUAL_MODEL_CARTESIAN_IMPEDANCE_CTRL_H__

 #include <OpenSoT/Task.h>
 #include <XBotInterface/ModelInterface.h>
 #include <kdl/frames.hpp>
 #include <Eigen/Dense>
 #include <OpenSoT/utils/cartesian_utils.h>
 #include <OpenSoT/utils/Indices.h>

 #define WORLD_FRAME_NAME "world"


 namespace OpenSoT {
    namespace tasks {
        namespace torque {
            class CartesianImpedanceCtrl : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<CartesianImpedanceCtrl> Ptr;
            protected:
                XBot::ModelInterface& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                Eigen::MatrixXd _actualPose;
                Eigen::MatrixXd _desiredPose;
                Eigen::VectorXd _desiredTwist;

                Eigen::MatrixXd _K;
                Eigen::MatrixXd _D;

                bool _base_link_is_world;

                bool _use_inertia_matrix;

                void update_b();

                Eigen::MatrixXd _M;
                Eigen::MatrixXd _Minv;
                Eigen::MatrixXd _J;
                Eigen::MatrixXd _tmpJ;
                //pseudoInverse<Eigen::MatrixXd> pinv;
                LDLTInverse<Eigen::MatrixXd> inv;

                Eigen::Affine3d _tmp_affine;
                Eigen::VectorXd _spring_force;
                Eigen::VectorXd _damping_force;

                Eigen::VectorXd _qdot;
                Eigen::VectorXd _xdot;

                Eigen::VectorXd _F;
                Eigen::VectorXd _tmpF;

                Eigen::VectorXd _tmp_vec;
                Eigen::MatrixXd _tmpA;

                virtual void _log(XBot::MatLogger::Ptr logger);

                Indices _rows_indices;

                void generateA(const Eigen::MatrixXd& _tmpA);
                void generateF(const Eigen::VectorXd& _tmpF);
                inline void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
                {
                    A.conservativeResize(A.rows()+B.rows(), A.cols());
                    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
                }
                inline void pile(Eigen::VectorXd &a, const Eigen::VectorXd &b)
                {
                    a.conservativeResize(a.rows()+b.rows());
                    a.segment(a.rows()-b.rows(),b.rows())<<b;
                }
            public:

                Eigen::VectorXd positionError;
                Eigen::VectorXd orientationError;
                Eigen::VectorXd linearVelocityError;
                Eigen::VectorXd orientationVelocityError;


                /*********** TASK PARAMETERS ************/



                /****************************************/


                CartesianImpedanceCtrl(std::string task_id,
                          const Eigen::VectorXd& x,
                          XBot::ModelInterface &robot,
                          std::string distal_link,
                          std::string base_link,
                          const std::list<unsigned int> rowIndices =
                        std::list<unsigned int>());

                ~CartesianImpedanceCtrl();

                void _update(const Eigen::VectorXd& x);

                void setReference(const Eigen::MatrixXd& desiredPose);
                void setReference(const KDL::Frame& desiredPose);

                void setReference(const Eigen::MatrixXd& desiredPose,
                                  const Eigen::VectorXd& desiredTwist);

                const Eigen::MatrixXd getReference() const;

                void getReference(Eigen::MatrixXd& desiredPose,
                                  Eigen::VectorXd& desiredTwist) const;


                const Eigen::MatrixXd getActualPose() const;

                const KDL::Frame getActualPoseKDL() const;

                const std::string getDistalLink() const;
                const std::string getBaseLink() const;
                const bool baseLinkIsWorld() const;

                void setStiffness(const Eigen::MatrixXd& Stiffness);
                void setDamping(const Eigen::MatrixXd& Damping);
                void setStiffnessDamping(const Eigen::MatrixXd& Stiffness,
                                         const Eigen::MatrixXd& Damping);

                Eigen::MatrixXd getStiffness();
                Eigen::MatrixXd getDamping();
                void getStiffnessDamping(Eigen::MatrixXd& Stiffness, Eigen::MatrixXd& Damping);



                static bool isCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                static OpenSoT::tasks::torque::CartesianImpedanceCtrl::Ptr asCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                void useInertiaMatrix(const bool use);

                Eigen::VectorXd getSpringForce();
                Eigen::VectorXd getDamperForce();

            };
        }
    }
 }

#endif
