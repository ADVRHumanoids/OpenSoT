/*
 * Copyright (C) 2014 Walkman
 * Author: Cheng Fang
 * email:  cheng.fang@iit.it
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

#ifndef SELFCOLLISIONAVOIDANCE_H
#define SELFCOLLISIONAVOIDANCE_H


 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/Cartesian.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/convex_hull.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {


        struct Capsule{
            Eigen::Vector3d P0;
            Eigen::Vector3d P1;
            double radius;
        };

        struct CapsulePair{
            Capsule S1;
            int S1_index;
            Capsule S2;
            int S2_index;
        };
            /**
             * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
             *
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<SelfCollisionAvoidance> Ptr;
            private:
                double _Capsule_threshold;
                iDynUtils& robot_col;
                int base_index;

                Eigen::MatrixXd from_yarp_to_Eigen_matrix(const yarp::sig::Matrix &Y_M);
                yarp::sig::Matrix from_Eigen_to_Yarp_matrix(const Eigen::MatrixXd &E_M);
                Eigen::VectorXd from_yarp_to_Eigen_vector(const yarp::sig::Vector &Y_V);
                yarp::sig::Vector from_Eigen_to_Yarp_vector(const Eigen::VectorXd &E_V);

                Eigen::Vector3d Transform_name_to_point (std::string Link_name, int base_index, int & Link_index);
                CapsulePair Generate_CapsulePair (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, int S1P1_index, double radius_1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, int S2P1_index, double radius_2);

                double dist3D_Segment_to_Segment (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, Eigen::Vector3d & CP1, Eigen::Vector3d & CP2);
                Eigen::MatrixXd Skew_symmetric_operator (const Eigen::Vector3d & r_cp);
                void Calculate_Aineq_bUpperB (const yarp::sig::Vector & x, yarp::sig::Matrix & Aineq_fc, yarp::sig::Vector & bUpperB_fc );

            public:
                /**
                 * @brief SelfCollisionAvoidanceConstraint

                 */
                SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       const double Capsule_threshold = 0.0);

                double get_Capsule_threshold();

                void set_Capsule_threshold(const double Capsule_threshold);

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }


#endif // SELFCOLLISIONAVOIDANCE_H

