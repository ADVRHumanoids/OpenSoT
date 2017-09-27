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
            /**
             * @brief The CartesianImpedanceCtrl class implements a Cartesian Impedance Controller
             * in the form:
             * \f$ \min_{\boldsymbol{\tau}} \parallel \mathbf{J}\mathbf{B}^{-1}\boldsymbol{\tau} - \mathbf{J}\mathbf{B}^{-1}\mathbf{J}^T\f \parallel^2 \mathbf{f}$
             * where:
             * \f$ \mathbf{f} = \mathbf{K}(\mathbf{x}_d - \mathbf{x}) + \mathbf{D}(\mathbf{\dot{x}}_d - \mathbf{\dot{x}}) \f$
             *
             * NOTE: in this task is NOT possible to use the active_joint_mask and the SubTask!
             */
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

                Eigen::MatrixXd _Minv;
                Eigen::MatrixXd _J;
                Eigen::MatrixXd _tmpJ;

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

                void _update(const Eigen::VectorXd& x);

                /**
                 * @brief applyActiveJointsMask is not usable for this task, so we do nothing
                 * @param A does not change!
                 */
                void applyActiveJointsMask(Eigen::MatrixXd& A)
                {
                    std::cout<<"ACTIVE JOINT MASK IS NOT AVAILABLE FOR "<<getTaskID()<<std::endl;
                }

            public:

                Eigen::VectorXd positionError;
                Eigen::VectorXd orientationError;
                Eigen::VectorXd linearVelocityError;
                Eigen::VectorXd orientationVelocityError;

                /**
                 * @brief CartesianImpedanceCtrl constructor
                 * @param task_id a name for the task
                 * @param x actual joint values ///TODO: Remove this!
                 * @param robot a robot model
                 * @param distal_link link to control
                 * @param base_link link from which we specify controller quantities
                 * @param rowIndices a list of indices used to cut rows of the task
                 * (instead of using the SubTask)
                 *
                 * NOTE:
                 *  Jacobians are specified in the following way:
                 *  \f$ ^{b}\mathbf{J}_{b,e}
                 *  where b is the base_link and e is the distal link. This means that
                 *  we control the relative force between b and e in the frame of b.
                 */
                CartesianImpedanceCtrl(std::string task_id,
                          const Eigen::VectorXd& x,
                          XBot::ModelInterface &robot,
                          std::string distal_link,
                          std::string base_link,
                          const std::list<unsigned int> rowIndices =
                        std::list<unsigned int>());

                ~CartesianImpedanceCtrl();

                /**
                 * @brief setReference set desired pose for the end-effector in base_link
                 * @param desiredPose a desired 4x4 pose matrix
                 */
                void setReference(const Eigen::MatrixXd& desiredPose);
                void setReference(const KDL::Frame& desiredPose);

                /**
                 * @brief setReference set desired pose and velocity reference for the
                 * end-effector in base_link
                 * @param desiredPose a desired 4x4 pose matrix
                 * @param desiredTwist a desired 1x6 twist vector
                 */
                void setReference(const Eigen::MatrixXd& desiredPose,
                                  const Eigen::VectorXd& desiredTwist);
                void setReference(const KDL::Frame& desiredPose,
                                  const KDL::Twist& desiredTwist);

                /**
                 * @brief getReference get the pose reference for the end-effector in base_link
                 * @param desired_pose a desired 4x4 pose matrix
                 */
                const void getReference(Eigen::MatrixXd& desired_pose) const;
                const void getReference(KDL::Frame& desired_pose) const;

                /**
                 * @brief getReference get the pose reference and the twist reference
                 * for the end-effector in base_link
                 * @param desiredPose a desired 4x4 pose matrix
                 * @param desiredTwist a desired 1x6 twist vector
                 */
                void getReference(Eigen::MatrixXd& desiredPose,
                                  Eigen::VectorXd& desiredTwist) const;
                void getReference(KDL::Frame& desiredPose,
                                  KDL::Twist& desiredTwist) const;

                /**
                 * @brief getActualPose get the actual pose of the end-effector in base_link
                 * @param actual_pose 4x4 pose matrix
                 */
                const void getActualPose(Eigen::MatrixXd& actual_pose) const;
                const void getActualPose(KDL::Frame& actual_pose) const;

                /**
                 * @brief getDistalLink get distal_link
                 * @return a string
                 */
                const std::string getDistalLink() const;

                /**
                 * @brief getBaseLink get base_link
                 * @return a string
                 */
                const std::string getBaseLink() const;

                /**
                 * @brief baseLinkIsWorld check if base_link is "world"
                 * @return true if base_link == "world"
                 */
                const bool baseLinkIsWorld() const;

                /**
                 * @brief setStiffness set Cartesian Stiffness matrix for the end-effector
                 * in base_link
                 * @param Stiffness a 6x6 matrix
                 */
                void setStiffness(const Eigen::MatrixXd& Stiffness);

                /**
                 * @brief setDamping set Cartesian Damping matrix for the end-effector
                 * in base_link
                 * @param Damping a 6x6 matrix
                 */
                void setDamping(const Eigen::MatrixXd& Damping);

                /**
                 * @brief setStiffnessDamping set Cartesian Stiffness and Damping matrix for
                 * the end-effector in base_link
                 * @param Stiffness a 6x6 matrix
                 * @param Damping a 6x6 matrix
                 */
                void setStiffnessDamping(const Eigen::MatrixXd& Stiffness,
                                         const Eigen::MatrixXd& Damping);

                /**
                 * @brief getStiffness return the Cartesian Stiffness for
                 * the end-effector in base_link
                 * @param Stiffness a 6x6 matrix
                 */
                void getStiffness(Eigen::MatrixXd& Stiffness);

                /**
                 * @brief getDamping return the Cartesian Damping for
                 * the end-effector in base_link
                 * @param Damping a 6x6 matrix
                 */
                void getDamping(Eigen::MatrixXd& Damping);

                /**
                 * @brief getStiffnessDamping return the Cartesian Stiffness and Damping for
                 * the end-effector in base_link
                 * @param Stiffness a 6x6 matrix
                 * @param Damping a 6x6 matrix
                 */
                void getStiffnessDamping(Eigen::MatrixXd& Stiffness, Eigen::MatrixXd& Damping);

                /**
                 * @brief isCartesianImpedanceCtrl check if a general Task is a Cartesian Impedance one
                 * @param task a general OpenSoT::Task
                 * @return true if OpenSoT::Task is CartesianImpedanceCtrl
                 */
                static bool isCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                /**
                 * @brief asCartesianImpedanceCtrl cast a general Task to a Cartesian Impedance one
                 * @param task a general OpenSoT::Task
                 * @return a pointer to a CartesianImpedanceCtrl Task
                 */
                static OpenSoT::tasks::torque::CartesianImpedanceCtrl::Ptr asCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                /**
                 * @brief useInertiaMatrix if true, the weight for the task is set to \f$M^{-1}\f$
                 * @param use true or false
                 * NOTE: to have proper handling of priorities, this should be set to TRUE!
                 * TODO: Remove this, \f$M^{-1}\f$ should be passed from outside!
                 */
                void useInertiaMatrix(const bool use);

                /**
                 * @brief getSpringForce return the spring force given by:
                 * \f$ \mathbf{f} = \mathbf{K}(\mathbf{x}_d - \mathbf{x})\f$
                 * @param spring_force vector of joint torques
                 */
                void getSpringForce(Eigen::VectorXd& spring_force);

                /**
                 * @brief getDampingForce return the spring damping given by:
                 * \f$ \mathbf{f} = \mathbf{D}(\mathbf{\dot{x}}_d - \mathbf{\dot{x}})\f$
                 * @param damping_force vector of joint torques
                 */
                void getDamperForce(Eigen::VectorXd& damper_force);

            };
        }
    }
 }

#endif
