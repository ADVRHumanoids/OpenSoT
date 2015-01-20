#include <OpenSoT/tasks/velocity/Self_collision_avoidance.h>

using namespace Eigen;

const double infinitesimal_constant = pow(10.0, -5);

Self_collision_avoidance::Self_collision_avoidance(iDynUtils &robot):
	robot_col(robot)

{

        // unit: meter
        l0 = 0.1515;
        l_u = 0.1375;
        l_l = 0.1947;
        offset = 0.005;

        R_u = 0.05;
        R_l = 0.05;
        R_torso_left = 0.03;
        R_torso_right = 0.03;
        R_torso_center = 0.07;

        // unit: rad
        q_step = 0.1;

    }


//////////////////////////////////////////////////////////////////////////
// Please note that all the input vector variables must be column vectors
//////////////////////////////////////////////////////////////////////////

// calculate the shortest distance between point A and line segment BC
double Self_collision_avoidance::minp2l(const Vector3d &A, const Vector3d &B, const Vector3d &C){

    // declaration for the inner variables
    Vector3d BC, alpha;
    double bc, a, b, c, d, Dm;

    // preparing the data
    BC = C - B;
    bc = BC.norm();
    alpha = BC / bc;
    a = B(0) - A(0);
    b = B(1) - A(1);
    c = B(2) - A(2);
    d = a * alpha(0) + b * alpha(1) + c * alpha(2);

    // discuss different cases
    if ( (-d >= 0) && (-d <= bc) )   // the shortest distance occurs in the middle of points B and C
        Dm = sqrt( pow(a,2) + pow(b,2) + pow(c,2) - pow(d,2) );
    else if ( -d < 0 )				 // the shortest distance occurs at point B
        Dm = sqrt( pow(a,2) + pow(b,2) + pow(c,2) );
    else							 // the shortest distance occurs at point C
        Dm = sqrt( pow(bc,2) + 2*d*bc + pow(a,2) + pow(b,2) + pow(c,2) );


    return Dm;
}

// calculate the shortest distance between two line segments AB and CD when AB and CD are aligned
double Self_collision_avoidance::case1(const Vector3d &A, const Vector3d &B, const Vector3d &C, const Vector3d &D){

    // declaration for the inner variables
    Vector3d AC, BC, AD, BD, AB, CD;
    double ac, bc, ad, bd, ab, cd, Dm;

    // calculate the lengths of vectors AC, BC, AD, BD, AB, CD
    AC = C - A;
    BC = C - B;
    AD = D - A;
    BD = D - B;
    AB = B - A;
    CD = D - C;

    ac = AC.norm();
    bc = BC.norm();
    ad = AD.norm();
    bd = BD.norm();
    ab = AB.norm();
    cd = CD.norm();

    // judge whether segments AB and CD are overlapped
    if ( (fabs(bc+bd-cd) < infinitesimal_constant) || (fabs(ac+ad-cd) < infinitesimal_constant) || (fabs(ac+bc-ab) < infinitesimal_constant) || (fabs(ad+bd-ab) < infinitesimal_constant) )
        Dm = 0;
    else{
        Vector4d temp(ac, ad, bc, bd);
        Dm = temp.minCoeff();
    }


    return Dm;
}

// calculate the shortest distance between two line segments AB and CD when AB and CD are in the same plane
double Self_collision_avoidance::case2(const Vector3d &A, const Vector3d &B, const Vector3d &C, const Vector3d &D){

    // declaration for the inner variables
    Vector3d AB, CD, R_v1, R_v2, R_v3, AA, BB, CC, DD, alpha;
    double ab, cd, a, b, c, d, Dm, i, j;
    Matrix3d R;

    // calculate the unit vectors of AB and CD
    AB = B - A;
    CD = D - C;

    ab = AB.norm();
    cd = CD.norm();

    // discuss different cases
    // AB and CD are parallel
    if ( fabs( fabs( ( AB/ab ).dot( CD/cd ) ) - 1 ) < infinitesimal_constant ){
        a = minp2l( A, C, D );
        b = minp2l( B, C, D );
        c = minp2l( C, A, B );
        d = minp2l( D, A, B );
        Vector4d temp(a, b, c, d);
        Dm = temp.minCoeff();
    }
    // AB and CD are not parallel
    else{
        R_v1 = AB / ab ;
        R_v3 = AB.cross(CD) / ( AB.cross(CD) ).norm();
        R_v2 = R_v3.cross( R_v1 );
        R.col(0) = R_v1;
        R.col(1) = R_v2;
        R.col(2) = R_v3;
        AA = R.transpose() * (A - A);
        BB = R.transpose() * (B - A);
        CC = R.transpose() * (C - A);
        DD = R.transpose() * (D - A);
        alpha = (DD - CC) / cd;
        j = -CC(1) / alpha(1);
        i = CC(0) - ( alpha(0) / alpha(1) ) * CC(1);
        if ( (i >= 0) && (i <= ab) && (j >= 0) && (j <= cd) )
            Dm = 0;
        else{
            a = minp2l( A, C, D );
            b = minp2l( B, C, D );
            c = minp2l( C, A, B );
            d = minp2l( D, A, B );
            Vector4d temp(a, b, c, d);
            Dm = temp.minCoeff();
        }

    }

    return Dm;

}

// calculate the shortest distance between two line segments AB and CD when AB and CD are not in the same plane
double Self_collision_avoidance::case3(const Vector3d &A, const Vector3d &B, const Vector3d &C, const Vector3d &D){

    // declaration for the inner variables
    Vector3d AB, CD, AC, L, AA, BB;
    double D1, D2, Dm;

    // calculate the common perpendicular of segments AB and CD
    AB = B - A;
    CD = D - C;
    AC = C - A;

    L = AB.cross(CD) / ( AB.cross(CD) ).norm();

    // adjust the direction of vector L and calculate the length of L, D1
    if ( AC.dot(L) > 0 )
        D1 = AC.dot(L);
    else{
        D1 = -AC.dot(L);
        L = -1 * L;
    }

    // calculate the shortest distance
    AA = A + D1 * L;
    BB = B + D1 * L;
    D2 = case2(AA, BB, C, D);
    Dm = sqrt( pow(D1,2) + pow(D2,2) );

    return Dm;

}

// calculate the shortest distance between two arbitrary spatial segments AB and CD
double Self_collision_avoidance::Dmin(const Vector3d &A, const Vector3d &B, const Vector3d &C, const Vector3d &D){

    // declaration for the inner variables
    Vector3d AC, BC, AD, BD;
    double Dm;

    // calculate the unit vectors of AC, BC, AD, BD
    AC = C - A;
    BC = C - B;
    AD = D - A;
    BD = D - B;

    AC = AC / AC.norm();
    BC = BC / BC.norm();
    AD = AD / AD.norm();
    BD = BD / BD.norm();

    // discuss different cases
    if ( ( (AC-BC).norm() < infinitesimal_constant ) && ( (AD-BD).norm() < infinitesimal_constant ) )
        Dm = case1(A, B, C, D);
    else if ( ( (AC-BC).norm() < infinitesimal_constant ) || ( (AD-BD).norm() < infinitesimal_constant ) )
        Dm = case2(A, B, C, D);
    else{
        if ( fabs( fabs( ( AC.cross(BC) / (AC.cross(BC)).norm() ).dot( AD.cross(BD) / (AD.cross(BD)).norm() ) ) - 1 ) < infinitesimal_constant )
            Dm = case2(A, B, C, D);
        else
            Dm = case3(A, B, C, D);
    }

    return Dm;

}

// calculate the corresponding the positions of the desired line segments according to the current joint angles
// Q is the configuration variable vector and its zero position is the posture where the right arm points to the very right horizontally and the left arm points to the very left horizontally
MatrixXd Self_collision_avoidance::jointangle2position (const VectorXd &Q){

    Vector3d left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B, right_lowerarm_A, right_lowerarm_B;
    Vector3d torso_left_A, torso_left_B, torso_right_A, torso_right_B, torso_center_A, torso_center_B;
    MatrixXd position_mat(3,14);


    std::string Torso_name = "torso";
    int Torso_index = robot_col.iDyn3_model.getLinkIndex(Torso_name);

    if(Torso_index == -1)
        std::cout << "Failed to get link index for torso" << std::endl;

    std::string Left_shoulder_center_name = "LShr";
    int Left_shoulder_center_index = robot_col.iDyn3_model.getLinkIndex(Left_shoulder_center_name);

    if(Left_shoulder_center_index == -1)
        std::cout << "Failed to get link index for left_shoulder" << std::endl;

    std::string Right_shoulder_center_name = "RShr";
    int Right_shoulder_center_index = robot_col.iDyn3_model.getLinkIndex(Right_shoulder_center_name);

    if(Right_shoulder_center_index == -1)
        std::cout << "Failed to get link index for Right_shoulder" << std::endl;

    std::string Left_elbow_center_name = "LElb";
    int Left_elbow_center_index = robot_col.iDyn3_model.getLinkIndex(Left_elbow_center_name);

    if(Left_elbow_center_index == -1)
        std::cout << "Failed to get link index for Left_elbow" << std::endl;

    std::string Right_elbow_center_name = "RElb";
    int Right_elbow_center_index = robot_col.iDyn3_model.getLinkIndex(Right_elbow_center_name);

    if(Right_elbow_center_index == -1)
        std::cout << "Failed to get link index for Right_elbow" << std::endl;

    std::string Left_wrist_center_name = "l_wrist";
    int Left_wrist_center_index = robot_col.iDyn3_model.getLinkIndex(Left_wrist_center_name);

    if(Left_wrist_center_index == -1)
        std::cout << "Failed to get link index for left_wrist" << std::endl;

    std::string Right_wrist_center_name = "r_wrist";
    int Right_wrist_center_index = robot_col.iDyn3_model.getLinkIndex(Right_wrist_center_name);

    if(Right_wrist_center_index == -1)
        std::cout << "Failed to get link index for right_wrist" << std::endl;


    yarp::sig::Vector Q_updated;
    Q_updated = from_Eigen_to_Yarp_vector( Q );
    robot_col.updateiDyn3Model(Q_updated, false);


    yarp::sig::Matrix left_shoulder = robot_col.iDyn3_model.getPosition(Torso_index,Left_shoulder_center_index);
    MatrixXd left_shoulder_torso = from_yarp_to_Eigen_matrix(left_shoulder);

    yarp::sig::Matrix left_elbow = robot_col.iDyn3_model.getPosition(Torso_index,Left_elbow_center_index);
    MatrixXd left_elbow_torso = from_yarp_to_Eigen_matrix(left_elbow);

    yarp::sig::Matrix left_wrist = robot_col.iDyn3_model.getPosition(Torso_index,Left_wrist_center_index);
    MatrixXd left_wrist_torso = from_yarp_to_Eigen_matrix(left_wrist);

    yarp::sig::Matrix right_shoulder = robot_col.iDyn3_model.getPosition(Torso_index,Right_shoulder_center_index);
    MatrixXd right_shoulder_torso = from_yarp_to_Eigen_matrix(right_shoulder);

    yarp::sig::Matrix right_elbow = robot_col.iDyn3_model.getPosition(Torso_index,Right_elbow_center_index);
    MatrixXd right_elbow_torso = from_yarp_to_Eigen_matrix(right_elbow);

    yarp::sig::Matrix right_wrist = robot_col.iDyn3_model.getPosition(Torso_index,Right_wrist_center_index);
    MatrixXd right_wrist_torso = from_yarp_to_Eigen_matrix(right_wrist);


    left_upperarm_A = left_shoulder_torso.block(0,3,3,1);
    left_upperarm_B = left_elbow_torso.block(0,3,3,1);
    left_lowerarm_A = left_elbow_torso.block(0,3,3,1);
    left_lowerarm_B = left_wrist_torso.block(0,3,3,1);
    right_upperarm_A = right_shoulder_torso.block(0,3,3,1);
    right_upperarm_B = right_elbow_torso.block(0,3,3,1);
    right_lowerarm_A = right_elbow_torso.block(0,3,3,1);
    right_lowerarm_B = right_wrist_torso.block(0,3,3,1);


    torso_left_A = left_shoulder_torso.block(0,3,3,1);
    torso_left_B << 0.0,
                    0.0,
                   -0.12;
    torso_right_A = right_shoulder_torso.block(0,3,3,1);
    torso_right_B << 0.0,
                     0.0,
                    -0.12;
    torso_center_A  << 0.0,
                       0.0,
                       0.0;
    torso_center_B  << 0.0,
                       0.0,
                      -0.12;

    position_mat.col(0) = torso_right_A;
    position_mat.col(1) = torso_right_B;
    position_mat.col(2) = torso_center_A;
    position_mat.col(3) = torso_center_B;
    position_mat.col(4) = torso_left_A;
    position_mat.col(5) = torso_left_B;
    position_mat.col(6) = right_upperarm_A;
    position_mat.col(7) = right_upperarm_B;
    position_mat.col(8) = right_lowerarm_A;
    position_mat.col(9) = right_lowerarm_B;
    position_mat.col(10) = left_upperarm_A;
    position_mat.col(11) = left_upperarm_B;
    position_mat.col(12) = left_lowerarm_A;
    position_mat.col(13) = left_lowerarm_B;

    return position_mat;

}

// calculate the shortest distance between the dual arms and the torso and the gradient of the distance with respect to the joint angles
//////////////////////////////////////////////////////////////////////////////
// the structure of the returned vector is [Dm;Gradient], a column vector (18,1)
// the structure of the Gradient vector is [Q_w; Q_r; Q_l] dimension (17,1)
// the input Q is for the whole joint vector of the robot, for example, [29,1] for coman
//////////////////////////////////////////////////////////////////////////////
VectorXd Self_collision_avoidance::shortest_distance_gradient(const VectorXd &Q){

    // declaration of the inner variables
    Vector3d left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B, right_lowerarm_A, right_lowerarm_B;
    Vector3d torso_left_A, torso_left_B, torso_right_A, torso_right_B, torso_center_A, torso_center_B;
    double left_u2right_u, left_u2right_l,
        left_l2torso_l, left_l2torso_c, left_l2torso_r, left_l2right_u, left_l2right_l,
        right_l2torso_l, right_l2torso_c, right_l2torso_r;

    int gra_dim = Q.rows();
    VectorXd Dm_Gradient(gra_dim+1), Gradient(gra_dim);
    MatrixXd Position_mat(3,14);

    ///*

    // insert codes about transforming input joint angle vectors to the position vectors of the dual arms
    Position_mat = jointangle2position (Q);

    torso_right_A = Position_mat.col(0);
    torso_right_B = Position_mat.col(1);
    torso_center_A = Position_mat.col(2);
    torso_center_B = Position_mat.col(3);
    torso_left_A = Position_mat.col(4);
    torso_left_B = Position_mat.col(5);
    right_upperarm_A = Position_mat.col(6);
    right_upperarm_B = Position_mat.col(7);
    right_lowerarm_A = Position_mat.col(8);
    right_lowerarm_B = Position_mat.col(9);
    left_upperarm_A = Position_mat.col(10);
    left_upperarm_B = Position_mat.col(11);
    left_lowerarm_A = Position_mat.col(12);
    left_lowerarm_B = Position_mat.col(13);


    // calculate the shortest distances between each pair of segments which could collide
    left_u2right_u = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u; // between left upperarm and right upperarm
    left_u2right_l = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l; // between left upperarm and right lowerarm

    left_l2torso_l = Dmin( left_lowerarm_A, left_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left; // between left lowerarm and the left part of the torso
    left_l2torso_c = Dmin( left_lowerarm_A, left_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center; // between left lowerarm and the center part of the torso
    left_l2torso_r = Dmin( left_lowerarm_A, left_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right; // between left lowerarm and the right part of the torso
    left_l2right_u = Dmin( left_lowerarm_A, left_lowerarm_B, right_upperarm_A, right_upperarm_B ) - R_l - R_u; // between left lowerarm and right upperarm
    left_l2right_l = Dmin( left_lowerarm_A, left_lowerarm_B, right_lowerarm_A, right_lowerarm_B ) - R_l - R_l; // between left lowerarm and right lowerarm

    right_l2torso_l = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left; // between right lowerarm and the left part of the torso
    right_l2torso_c = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center; // between right lowerarm and the center part of the torso
    right_l2torso_r = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right; // between right lowerarm and the right part of the torso

    // find the shortest distances between the dual arms and the torso, and where the shortest distance occur
    VectorXd temp(10);
    double Dm;
    std::ptrdiff_t min_i, min_j;

    temp<< left_u2right_u,
           left_u2right_l,
           left_l2torso_l,
           left_l2torso_c,
           left_l2torso_r,
           left_l2right_u,
           left_l2right_l,
           right_l2torso_l,
           right_l2torso_c,
           right_l2torso_r;

    Dm = temp.minCoeff(&min_i,&min_j);

    //test
    //temp.print("Dm_temp =");
    //test

    VectorXd Q_temp;
    Gradient.setZero();
    double Dm1, Dm2;
    unsigned int index_temp;

    // calculate the gradient of the shortest distance
    switch (min_i)
    {
        // the shortest distance occurs between the left upperarm and right upperarm
        case 0:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_upperarm_A, right_upperarm_B ) - R_u - R_u;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;


            break;

        // the shortest distance occurs between the left upperarm and right lowerarm
        case 1:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm1 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_upperarm_A = Position_mat.col(10);
            left_upperarm_B = Position_mat.col(11);
            Dm2 = Dmin( left_upperarm_A, left_upperarm_B, right_lowerarm_A, right_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;


            break;

        case 2:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);

            Dm1 = Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_left_A, torso_left_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_left - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;


            break;

        case 3:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_center_A, torso_center_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_center - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 4:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( torso_right_A, torso_right_B, left_lowerarm_A, left_lowerarm_B ) - R_torso_right - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 5:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_upperarm_A = Position_mat.col(6);
            right_upperarm_B = Position_mat.col(7);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 =  Dmin( right_upperarm_A, right_upperarm_B, left_lowerarm_A, left_lowerarm_B ) - R_u - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 6:
            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.left_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            left_lowerarm_A = Position_mat.col(12);
            left_lowerarm_B = Position_mat.col(13);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, left_lowerarm_A, left_lowerarm_B ) - R_l - R_l;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 7:
            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_left_A = Position_mat.col(4);
            torso_left_B = Position_mat.col(5);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_left_A, torso_left_B ) - R_l - R_torso_left;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 8:
            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_center_A = Position_mat.col(2);
            torso_center_B = Position_mat.col(3);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_center_A, torso_center_B ) - R_l - R_torso_center;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;

        case 9:
            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[0];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[1];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[2];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            Q_temp = Q;
            index_temp = robot_col.right_arm.joint_numbers[3];
            Q_temp(index_temp) = Q_temp(index_temp) + q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm1 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Q_temp(index_temp) = Q_temp(index_temp) - 2 * q_step;
            Position_mat = jointangle2position (Q_temp);
            right_lowerarm_A = Position_mat.col(8);
            right_lowerarm_B = Position_mat.col(9);
            torso_right_A = Position_mat.col(0);
            torso_right_B = Position_mat.col(1);
            Dm2 = Dmin( right_lowerarm_A, right_lowerarm_B, torso_right_A, torso_right_B ) - R_l - R_torso_right;
            Gradient(index_temp) = (Dm1 - Dm2) / q_step / 2;

            break;


    }

    Dm_Gradient(0) = Dm;
    Dm_Gradient.block(1,0,gra_dim,1) = Gradient;

    std::cout << "Dm_Gradient" << std::endl;
    std::cout << min_i << std::endl;
    std::cout << Dm_Gradient << std::endl;

    return Dm_Gradient;

}




MatrixXd Self_collision_avoidance::from_yarp_to_Eigen_matrix(const yarp::sig::Matrix& Y_M)
{
      int rows = Y_M.rows();
      int columns = Y_M.cols();
      
      MatrixXd A_M(rows, columns);
  
      for(int i=0;i<rows;i++)
      {
	   for(int j=0;j<columns;j++)
	   {
		A_M(i,j) = Y_M.getRow(i)[j];
	   }

      }         
      return A_M;
}

yarp::sig::Matrix Self_collision_avoidance::from_Eigen_to_Yarp_matrix(const MatrixXd& E_M)
{
      
      int rows = E_M.rows();
      int columns = E_M.cols();
      
      yarp::sig::Matrix A_M(rows, columns);
  
      for(int i=0;i<rows;i++)
      {
	   for(int j=0;j<columns;j++)
	   {
		A_M.getRow(i)[j] = E_M(i,j);
	   }

      }         
      return A_M;

}

VectorXd Self_collision_avoidance::from_yarp_to_Eigen_vector(const yarp::sig::Vector& Y_V)
{
      
      int length = Y_V.length();
      
      VectorXd A_V(length);
  
      for(int i=0;i<length;i++)
      {
        
         A_V(i) = Y_V(i);

      }         
      return A_V;

}

yarp::sig::Vector Self_collision_avoidance::from_Eigen_to_Yarp_vector(const VectorXd& E_V)
{

      int rows = E_V.rows();
      
      yarp::sig::Vector A_V(rows);
  
      for(int i=0;i<rows;i++)
      {

	A_V(i) = E_V(i);

      }         
      return A_V;

}

