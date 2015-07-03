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
 #include <idynutils/collision_utils.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <OpenSoT/constraints/velocity/svm.h>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {

            /**
             * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
             *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
             *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
             *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1; 
             *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
             *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<SelfCollisionAvoidance> Ptr;
            protected:
                double _boundScaling;
                /**
                 * @brief _LinkPair_threshold is the allowable minimum distance between every Link pair
                 */
                double _linkPair_threshold;

                iDynUtils& robot_col;
                ComputeLinksDistance computeLinksDistance;

                /**
                 * @brief _x_cache is a copy of last q vector used to update the constraints.
                 * It is used in order to avoid multiple updated when the constraint is present
                 * in multiple tasks in the same stack. It is a simple speedup waiting for the stack system to support
                 * constraint caching - or for idynutils to support some form of cache system
                 */
                yarp::sig::Vector _x_cache;

                /**
                 * @brief base_index all the calculation and expression is described in
                 * a base link frame which is "waist" link frame
                 */
                int base_index;

                // used for online prediction of link pair selection
                // used for online prediction of link pair selection

                struct svm_node * x_larm_rarm, * x_larm_torso, * x_rarm_torso,
                                * x_larm_lleg, * x_rarm_rleg, * x_larm_rleg, * x_rarm_lleg;

                struct svm_model * model_larm_rarm, * model_larm_torso, * model_rarm_torso,
                                 * model_larm_lleg, * model_rarm_rleg, * model_larm_rleg, * model_rarm_lleg;

                std::ifstream scale_larm_rarm, scale_larm_torso, scale_rarm_torso,
                              scale_larm_lleg, scale_rarm_rleg, scale_larm_rleg, scale_rarm_lleg;

                int number_larm, number_rarm, number_torso, number_head, number_lleg, number_rleg;
                int number_larm_rarm, number_larm_torso, number_rarm_torso,
                    number_larm_lleg, number_rarm_rleg, number_larm_rleg, number_rarm_lleg;

                std::vector<double> min_larm_rarm, max_larm_rarm, temp_larm_rarm;
                std::vector<double> min_larm_torso, max_larm_torso, temp_larm_torso;
                std::vector<double> min_rarm_torso, max_rarm_torso, temp_rarm_torso;
                std::vector<double> min_larm_lleg, max_larm_lleg, temp_larm_lleg;
                std::vector<double> min_rarm_rleg, max_rarm_rleg, temp_rarm_rleg;
                std::vector<double> min_larm_rleg, max_larm_rleg, temp_larm_rleg;
                std::vector<double> min_rarm_lleg, max_rarm_lleg, temp_rarm_lleg;

                double d_recent_L_R_Arms[3], d_recent_L_Arm_Torso[3], d_recent_R_Arm_Torso[3],
                       d_recent_L_Arm_L_Leg[3], d_recent_R_Arm_R_Leg[3], d_recent_L_Arm_R_Leg[3], d_recent_R_Arm_L_Leg[3];

                void predict_SCAFoIs( const yarp::sig::Vector & q, std::list<std::pair<std::string,std::string>> & linkpair_updated_list,
                                      std::list<LinkPairDistance> & linkpair_constrained_list );

                void store_l_r_arms(const double & d_current);
                void store_larm_torso(const double & d_current);
                void store_rarm_torso(const double & d_current);
                void store_larm_lleg(const double & d_current);
                void store_rarm_rleg(const double & d_current);
                void store_larm_rleg(const double & d_current);
                void store_rarm_lleg(const double & d_current);

                // used for online prediction of link pair selection
                // used for online prediction of link pair selection

            public:
                ///TODO: Move in cartesian utils
                // the following four functions are responsible for the transformation between the yarp data type and Eigen data type
                Eigen::MatrixXd from_yarp_to_Eigen_matrix(const yarp::sig::Matrix &Y_M);
                yarp::sig::Matrix from_Eigen_to_Yarp_matrix(const Eigen::MatrixXd &E_M);
                Eigen::VectorXd from_yarp_to_Eigen_vector(const yarp::sig::Vector &Y_V);
                yarp::sig::Vector from_Eigen_to_Yarp_vector(const Eigen::VectorXd &E_V);
                
                /**
                 * @brief Skew_symmetric_operator is used to get the transformation matrix which is used to transform
                 * the base Jacobian to goal Jacobian
                 * @param r_cp the vector measured from the origin of the reference link frame to the closest point
                 * @return Skew symmetric matrix of the input vector
                 * NOTE: the base Jacobian should be multiplied by the output matrix on its left side.
                 */
                Eigen::MatrixXd skewSymmetricOperator (const Eigen::Vector3d & r_cp);

                /**
                 * @brief calculate_Aineq_bUpperB the core function which is used to update the variables Aineq and
                 * bUpperBound for this constraint
                 * @param Aineq_fc Aineq matrix of this constraint
                 * @param bUpperB_fc bUpperBound of this constraint
                 */
                void calculate_Aineq_bUpperB (yarp::sig::Matrix & Aineq_fc, yarp::sig::Vector & bUpperB_fc, std::list<LinkPairDistance> & interested_LinkPairs );

            public:
                /**
                 * @brief SelfCollisionAvoidanceConstraint
                 * @param x the robot current configuration vector
                 * @param robot the robot model reference
                 * @param linkPair_threshold the minimum distance between each Link pair
                 * @param boundScaling the bound scaling for the capsule distance (a lower number means we will approach
                 *        the linkPair_threshold more slowly)
                 */
                SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       double linkPair_threshold = 0.0,
                                       const double boundScaling = 1.0);


                ~SelfCollisionAvoidance();

                /**
                 * @brief getLinkPairThreshold
                 * @return _LinkPair_threshold
                 */
                double getLinkPairThreshold();

                /**
                 * @brief setLinkPairThreshold set _LinkPair_threshold
                 * @param linkPair_threshold (always positive)
                 */
                void setLinkPairThreshold(const double linkPair_threshold);


                /**
                 * @brief update recomputes Aineq and bUpperBound if x is different than the previously stored value
                 * @param x the state vector. It gets cached so that we won't recompute capsules distances if x didn't change
                 * TODO if the capsules distances are given by a server (i.e. a separate thread updaing capsules at a given rate)
                 * then the inelegant caching of x is not needed anymore (notice that, however, it would be nice to have a caching system
                 * at the stack level so that, if two tasks in the same stack are constrained by the same constraint, this constraint
                 * gets updated only once per stack update)
                 */
                void update(const yarp::sig::Vector &x);


                /**
                 * @brief setCollisionWhiteList resets the allowed collision matrix by setting all collision pairs as disabled.
                 *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
                 *        which are disabled in the SRDF
                 * @param whiteList a list of links pairs for which to not check collision detection
                 * @return true on success
                 */
                bool setCollisionWhiteList(std::list< LinkPairDistance::LinksPair > whiteList);

                /**
                 * @brief setCollisionBlackList resets the allowed collision matrix by setting all collision pairs
                 *        (for which collision geometries are avaiable) as enabled.
                 *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
                 *        which are disabled in the SRDF
                 * @param blackList a list of links pairs for which to not check collision detection
                 * @return true on success
                 */
                bool setCollisionBlackList(std::list< LinkPairDistance::LinksPair > blackList);

                /**
                 * @brief setBoundScaling sets bound scaling for the capsule constraint
                 * @param boundScaling is a number which should be lower than 1.0
                 *        (e.g. 1./2. means we are looking two steps ahead and will avoid
                 *         collision with the capsule by slowing down)
                 */
                void setBoundScaling(const double boundScaling);

                std::list<std::pair<std::string,std::string>> whitelist_L_R_Arms,
                                                              whitelist_L_Arm_Torso,
                                                              whitelist_R_Arm_Torso,
                                                              whitelist_L_Arm_L_Leg,
                                                              whitelist_R_Arm_R_Leg,
                                                              whitelist_L_Arm_R_Leg,
                                                              whitelist_R_Arm_L_Leg;

                bool    is_active_SCAFoI_L_R_Arms,
                        is_active_SCAFoI_L_Arm_Torso,
                        is_active_SCAFoI_R_Arm_Torso,
                        is_active_SCAFoI_L_Arm_L_Leg,
                        is_active_SCAFoI_R_Arm_R_Leg,
                        is_active_SCAFoI_L_Arm_R_Leg,
                        is_active_SCAFoI_R_Arm_L_Leg;


                /* upper and lower threshold for SCAFoIs activation */
                double d_threshold_upper, d_threshold_lower;

                /* these get updated each time you call udpate() */
                std::list<std::pair<std::string,std::string>> Linkpair_updated_list_all;
                std::list<LinkPairDistance> Linkpair_constrained_list_all;
            };
        }
    }
 }


#endif // SELFCOLLISIONAVOIDANCE_H

