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

#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <yarp/math/Math.h>

using namespace yarp::math;

using namespace OpenSoT::constraints::velocity;

using namespace Eigen;

const double SMALL_NUM = pow(10.0, -5);

SelfCollisionAvoidance::SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                               iDynUtils &robot,
                                               const double Capsule_threshold):
    Constraint(x.size()),
    _Capsule_threshold(Capsule_threshold),
    robot_col(robot){

    update(x);

}

double SelfCollisionAvoidance::get_Capsule_threshold()
{
    return _Capsule_threshold;
}

void SelfCollisionAvoidance::set_Capsule_threshold(const double Capsule_threshold)
{
    _Capsule_threshold = std::fabs(Capsule_threshold);
    //this->update();
}

void SelfCollisionAvoidance::update(const yarp::sig::Vector &x)
{
    Calculate_Aineq_bUpperB ( x, _Aineq, _bUpperBound );
//    std::cout << "_Aineq" << _Aineq.toString() << std::endl << std::endl;
//    std::cout << "_bUpperBound" << _bUpperBound.toString() << std::endl << std::endl;
}

double SelfCollisionAvoidance::dist3D_Segment_to_Segment (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, Eigen::Vector3d & CP1, Eigen::Vector3d & CP2)
{

    Vector3d   u = S1P1 - S1P0;
    Vector3d   v = S2P1 - S2P0;
    Vector3d   w = S1P0 - S2P0;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (std::fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (std::fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    CP1 = S1P0 + sc * u;
    CP2 = S2P0 + tc * v;

//    std::cout << "CP1: " << std::endl << CP1 << std::endl;
//    std::cout << "CP2: " << std::endl << CP2 << std::endl;

    // get the difference of the two closest points
    Vector3d   dP = CP1 - CP2;  // =  S1(sc) - S2(tc)

    double Dm = dP.norm();   // return the closest distance

    std::cout << "Dm: " << std::endl << Dm << std::endl;

    return Dm;
}

Eigen::MatrixXd SelfCollisionAvoidance::Skew_symmetric_operator (const Eigen::Vector3d & r_cp)
{
    MatrixXd J_transform(3,6);
    Matrix3d left_part, right_part;
    left_part = MatrixXd::Identity(3,3);
    right_part << 0, r_cp(2), -r_cp(1),
                   -r_cp(2), 0, r_cp(0),
                   r_cp(1), -r_cp(0), 0;
    J_transform.block(0,0,3,3) = left_part;
    J_transform.block(0,3,3,3) = right_part;

    return J_transform;
}

Eigen::Vector3d SelfCollisionAvoidance::Transform_name_to_point (std::string Link_name, int base_index, int & Link_index)
{
    Vector3d Link_reference_origin;

    Link_index = robot_col.iDyn3_model.getLinkIndex(Link_name);
    if(Link_index == -1)
        std::cout << "Failed to get link index for " << Link_name << std::endl;

    yarp::sig::Matrix Link_transformation = robot_col.iDyn3_model.getPosition(base_index,Link_index);
    MatrixXd Link_transformation_base = from_yarp_to_Eigen_matrix(Link_transformation);
    Link_reference_origin = Link_transformation_base.block(0,3,3,1);

    return Link_reference_origin;
}

CapsulePair SelfCollisionAvoidance::Generate_CapsulePair (const Eigen::Vector3d & S1P0, const Eigen::Vector3d & S1P1, int S1P1_index,  double radius_1, const Eigen::Vector3d & S2P0, const Eigen::Vector3d & S2P1, int S2P1_index, double radius_2)
{
    CapsulePair _CapsulePair;

    _CapsulePair.S1.P0 = S1P0;
    _CapsulePair.S1.P1 = S1P1;
    _CapsulePair.S1.radius = radius_1;
    _CapsulePair.S1_index = S1P1_index;

    _CapsulePair.S2.P0 = S2P0;
    _CapsulePair.S2.P1 = S2P1;
    _CapsulePair.S2.radius = radius_2;
    _CapsulePair.S2_index = S2P1_index;

    return _CapsulePair;
}


void SelfCollisionAvoidance::Calculate_Aineq_bUpperB (const yarp::sig::Vector & x, yarp::sig::Matrix & Aineq_fc, yarp::sig::Vector & bUpperB_fc )
{


//    std::string Left_wrist_center_name = "LWrMot2";
//    int Left_wrist_center_index = robot_col.iDyn3_model.getLinkIndex(Left_wrist_center_name);

//    if(Left_wrist_center_index == -1)
//        std::cout << "Failed to get link index for left_wrist" << std::endl;

//    std::string Left_hand_center_name = "LWrMot3";
//    int Left_hand_center_index = robot_col.iDyn3_model.getLinkIndex(Left_hand_center_name);

//    if(Left_hand_center_index == -1)
//        std::cout << "Failed to get link index for left_hand" << std::endl;

//    std::string Right_wrist_center_name = "RWrMot2";
//    int Right_wrist_center_index = robot_col.iDyn3_model.getLinkIndex(Right_wrist_center_name);

//    if(Right_wrist_center_index == -1)
//        std::cout << "Failed to get link index for right_wrist" << std::endl;

//    std::string Right_hand_center_name = "RWrMot3";
//    int Right_hand_center_index = robot_col.iDyn3_model.getLinkIndex(Right_hand_center_name);

//    if(Right_hand_center_index == -1)
//        std::cout << "Failed to get link index for right_hand" << std::endl;

//    yarp::sig::Matrix left_wrist = robot_col.iDyn3_model.getPosition(base_index,Left_wrist_center_index);
//    MatrixXd left_wrist_base = from_yarp_to_Eigen_matrix(left_wrist);

//    yarp::sig::Matrix left_hand = robot_col.iDyn3_model.getPosition(base_index,Left_hand_center_index);
//    MatrixXd left_hand_base = from_yarp_to_Eigen_matrix(left_hand);

//    yarp::sig::Matrix right_wrist = robot_col.iDyn3_model.getPosition(base_index,Right_wrist_center_index);
//    MatrixXd right_wrist_base = from_yarp_to_Eigen_matrix(right_wrist);

//    yarp::sig::Matrix right_hand = robot_col.iDyn3_model.getPosition(base_index,Right_hand_center_index);
//    MatrixXd right_hand_base = from_yarp_to_Eigen_matrix(right_hand);

//    left_hand_A = left_wrist_base.block(0,3,3,1);
//    left_hand_B = left_hand_base.block(0,3,3,1);
//    right_hand_A = right_wrist_base.block(0,3,3,1);
//    right_hand_B = right_hand_base.block(0,3,3,1);



    robot_col.updateiDyn3Model(x, false);

    std::vector<CapsulePair> CapsulePair_vec;

    std::string base_name = "Waist";
    int base_index = robot_col.iDyn3_model.getLinkIndex(base_name);

    if(base_index == -1)
        std::cout << "Failed to get base_index" << std::endl;

    Vector3d left_hand_P0, left_hand_P1, right_hand_P0, right_hand_P1;
    int left_hand_P0_index, left_hand_P1_index, right_hand_P0_index, right_hand_P1_index;
    left_hand_P0 = Transform_name_to_point ("LWrMot2", base_index, left_hand_P0_index);
    left_hand_P1 = Transform_name_to_point ("LWrMot3", base_index, left_hand_P1_index);
    right_hand_P0 = Transform_name_to_point ("RWrMot2", base_index, right_hand_P0_index);
    right_hand_P1 = Transform_name_to_point ("RWrMot3", base_index, right_hand_P1_index);

    CapsulePair lefthand_vs_righthand;
    lefthand_vs_righthand = Generate_CapsulePair (left_hand_P0, left_hand_P1, left_hand_P1_index, 0.2, right_hand_P0, right_hand_P1, right_hand_P1_index, 0.2);

    CapsulePair_vec.push_back(lefthand_vs_righthand);

    /*//////////////////////////////////////////////////////////*/

    MatrixXd Aineq_fc_Eigen(CapsulePair_vec.size(), robot_col.iDyn3_model.getNrOfDOFs());
    VectorXd bUpperB_fc_Eigen(CapsulePair_vec.size());

    Vector3d Capsule1_P0, Capsule1_P1, Capsule2_P0, Capsule2_P1;
    double Capsule1_R, Capsule2_R;

    double Dm_CapsulePair;
    Vector3d CP1_Capsule1, CP2_Capsule2, closepoint_dir;

    Vector3d CP1_Capsule1_border, CP2_Capsule2_border;

    MatrixXd CP1_Capsule1_border_Jaco, CP2_Capsule2_border_Jaco;
    yarp::sig::Matrix CP1_Capsule1_border_Jaco_temp, CP2_Capsule2_border_Jaco_temp;

    yarp::sig::Matrix Waist_frame_world = robot_col.iDyn3_model.getPosition(base_index, true);
    MatrixXd Waist_frame_world_Eigen = from_yarp_to_Eigen_matrix(Waist_frame_world);
    Matrix3d Waist_frame_world_Eigen_Ro = Waist_frame_world_Eigen.block(0,0,3,3);
    MatrixXd temp_trans_matrix(6,6);
    temp_trans_matrix.block(0,0,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(3,3,3,3) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block(0,3,3,3) = MatrixXd::Zero(3,3);
    temp_trans_matrix.block(3,0,3,3) = MatrixXd::Zero(3,3);

    int Capsule1_P1_index, Capsule2_P1_index;

    for (unsigned int j = 0; j < CapsulePair_vec.size(); ++j)
    {

        Capsule1_P0 = CapsulePair_vec[j].S1.P0;
        Capsule1_P1 = CapsulePair_vec[j].S1.P1;
        Capsule2_P0 = CapsulePair_vec[j].S2.P0;
        Capsule2_P1 = CapsulePair_vec[j].S2.P1;
        Capsule1_R = CapsulePair_vec[j].S1.radius;
        Capsule2_R = CapsulePair_vec[j].S2.radius;
        Capsule1_P1_index = CapsulePair_vec[j].S1_index;
        Capsule2_P1_index = CapsulePair_vec[j].S2_index;


        Dm_CapsulePair = dist3D_Segment_to_Segment(Capsule1_P0, Capsule1_P1, Capsule2_P0, Capsule2_P1, CP1_Capsule1, CP2_Capsule2);
        closepoint_dir = CP2_Capsule2 - CP1_Capsule1;
        closepoint_dir = closepoint_dir / Dm_CapsulePair;
        CP1_Capsule1_border = CP1_Capsule1 + Capsule1_R * closepoint_dir;
        CP2_Capsule2_border = CP2_Capsule2 - Capsule2_R * closepoint_dir;


        robot_col.iDyn3_model.getRelativeJacobian( Capsule1_P1_index, base_index, CP1_Capsule1_border_Jaco_temp, true);
        CP1_Capsule1_border_Jaco = from_yarp_to_Eigen_matrix (CP1_Capsule1_border_Jaco_temp);
        CP1_Capsule1_border_Jaco = temp_trans_matrix * CP1_Capsule1_border_Jaco;
        CP1_Capsule1_border_Jaco = Skew_symmetric_operator(CP1_Capsule1_border - Capsule1_P1) * CP1_Capsule1_border_Jaco;


        robot_col.iDyn3_model.getRelativeJacobian( Capsule2_P1_index, base_index, CP2_Capsule2_border_Jaco_temp, true);
        CP2_Capsule2_border_Jaco = from_yarp_to_Eigen_matrix (CP2_Capsule2_border_Jaco_temp);
        CP2_Capsule2_border_Jaco = temp_trans_matrix * CP2_Capsule2_border_Jaco;
        CP2_Capsule2_border_Jaco = Skew_symmetric_operator(CP2_Capsule2_border - Capsule2_P1) * CP2_Capsule2_border_Jaco;


        Aineq_fc_Eigen.row(j) = closepoint_dir.transpose() * ( CP1_Capsule1_border_Jaco - CP2_Capsule2_border_Jaco );
        bUpperB_fc_Eigen(j) = Dm_CapsulePair - Capsule1_R - Capsule2_R - 2 * _Capsule_threshold;

    }

    Aineq_fc = from_Eigen_to_Yarp_matrix(Aineq_fc_Eigen);
    bUpperB_fc = from_Eigen_to_Yarp_vector(bUpperB_fc_Eigen);

}


MatrixXd SelfCollisionAvoidance::from_yarp_to_Eigen_matrix(const yarp::sig::Matrix& Y_M)
{
      int rows = Y_M.rows();
      int columns = Y_M.cols();

      MatrixXd A_M(rows, columns);

      for(int i=0;i<rows;i++)
      {
       for(int j=0;j<columns;j++)
       {

        A_M(i,j) = Y_M(i,j);

       }

      }
      return A_M;
}

yarp::sig::Matrix SelfCollisionAvoidance::from_Eigen_to_Yarp_matrix(const MatrixXd& E_M)
{

      int rows = E_M.rows();
      int columns = E_M.cols();

      yarp::sig::Matrix A_M(rows, columns);

      for(int i=0;i<rows;i++)
      {
       for(int j=0;j<columns;j++)
       {

        A_M(i,j) = E_M(i,j);

       }

      }
      return A_M;

}

VectorXd SelfCollisionAvoidance::from_yarp_to_Eigen_vector(const yarp::sig::Vector& Y_V)
{

      int length = Y_V.length();

      VectorXd A_V(length);

      for(int i=0;i<length;i++)
      {

         A_V(i) = Y_V(i);

      }
      return A_V;

}

yarp::sig::Vector SelfCollisionAvoidance::from_Eigen_to_Yarp_vector(const VectorXd& E_V)
{

      int rows = E_V.rows();

      yarp::sig::Vector A_V(rows);

      for(int i=0;i<rows;i++)
      {

        A_V(i) = E_V(i);

      }
      return A_V;

}

