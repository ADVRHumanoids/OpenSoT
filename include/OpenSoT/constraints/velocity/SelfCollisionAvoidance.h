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
             *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
             *  where the dimension of Aineq is n * m, n is the number of capsule pairs to be constrained, and m is total DOFs of the robot to be controlled;
             *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1; 
             *  the bUpperBound is the minimum distance vector of all the capsule pairs, the dimension of which is n * 1. 
             *  the element in bUpperBound is the minimum distance between the corresponding capsule pair with taking the capsule pair threshold into account.
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<SelfCollisionAvoidance> Ptr;
            protected:
                // _CapsulePair_threshold is the allowable minimum distance between every capsule pair
                double _CapsulePair_threshold;
                iDynUtils& robot_col;
                // all the calculation and expression is described in a base link frame which is "waist" link frame
                int base_index;
            public:
                // the following four functions are responsible for the transformation between the yarp data type and Eigen data type
                Eigen::MatrixXd from_yarp_to_Eigen_matrix(const yarp::sig::Matrix &Y_M);
                yarp::sig::Matrix from_Eigen_to_Yarp_matrix(const Eigen::MatrixXd &E_M);
                Eigen::VectorXd from_yarp_to_Eigen_vector(const yarp::sig::Vector &Y_V);
                yarp::sig::Vector from_Eigen_to_Yarp_vector(const Eigen::VectorXd &E_V);

                // function "Transform_name_to_point" is used to get the position of the origin of the link frame from the given link frame name
                Eigen::Vector3d Transform_name_to_point (std::string Link_name, int base_index, int & Link_index);

                /* function "Generate_CapsulePair" is used to generate a capsule pair which needs to be detected and constrained
                 * please type the information of two capsules about the positions of the two endpoints, the radius, and the reference link index of the capsule respectively
                 * Note: the reference link index of the capsule is used to calculate the Jacobian of the origin of the link frame and then further to get the Jacobian of the closest point on the capsule
                 * Note: please make sure you set a reasonable radius for each capsule for the current robot posture at the very beginning, 
                 * which means the minimum distance between any capsule pair should be greater than the capsule pair threshold, otherwise, the constraint would make the program crash.
                 */ 
                CapsulePair Generate_CapsulePair (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, int S1P1_index, double radius_1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, int S2P1_index, double radius_2);

                // function "dist3D_Segment_to_Segment" is used to calculate the minimum distance between any spatial line segments
                // return the minimum distance and the position of the closest point on each capsule
                double dist3D_Segment_to_Segment (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, Eigen::Vector3d & CP1, Eigen::Vector3d & CP2);
                
                /* function "Skew_symmetric_operator" is used to get the transformation matrix which is used to transform the base Jacobian to goal Jacobian
                 * the input vector is the vector measured from the origin of the reference link frame to the closest point
                 * Note: the base Jacobian should be multiplied by the output matrix on its left side. 
                 */
                Eigen::MatrixXd Skew_symmetric_operator (const Eigen::Vector3d & r_cp);

                // function "Calculate_Aineq_bUpperB" is the core function which is used to update the variables Aineq and bUpperBound for this constraint
                void Calculate_Aineq_bUpperB (const yarp::sig::Vector & x, yarp::sig::Matrix & Aineq_fc, yarp::sig::Vector & bUpperB_fc );

            public:
                /**
                 * @brief SelfCollisionAvoidanceConstraint
                 * @param x the robot current configuration vector
                 * @param robot the robot model reference
                 * @param CapsulePair_threshold the minimum distance between each capsule pair 
                 */
                SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       const double CapsulePair_threshold = 0.0);

                double get_CapsulePair_threshold();

                void set_CapsulePair_threshold(const double CapsulePair_threshold);

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }


#endif // SELFCOLLISIONAVOIDANCE_H

