#ifndef __TASKS_VELOCITY_Self_collision_avoidance_H__
#define __TASKS_VELOCITY_Self_collision_avoidance_H__

#include <idynutils/idynutils.h>
#include <Eigen/Dense>
#include <cmath>
#include <yarp/sig/all.h>



class Self_collision_avoidance
{
public:
    Self_collision_avoidance(iDynUtils &robot);
    Eigen::VectorXd shortest_distance_gradient(const Eigen::VectorXd &Q);

    Eigen::MatrixXd from_yarp_to_Eigen_matrix(const yarp::sig::Matrix &Y_M);
    yarp::sig::Matrix from_Eigen_to_Yarp_matrix(const Eigen::MatrixXd &E_M);
    Eigen::VectorXd from_yarp_to_Eigen_vector(const yarp::sig::Vector &Y_V);
    yarp::sig::Vector from_Eigen_to_Yarp_vector(const Eigen::VectorXd &E_V);

protected:

private:
    double minp2l(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &C);
    double case1(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &C, const Eigen::Vector3d &D);
    double case2(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &C, const Eigen::Vector3d &D);
    double case3(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &C, const Eigen::Vector3d &D);
    double Dmin(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &C, const Eigen::Vector3d &D);
    Eigen::MatrixXd jointangle2position (const Eigen::VectorXd &Q);

    double R_u, R_l, R_torso_le_u, R_torso_ri_u, R_torso_le_l, R_torso_ri_l;
    double q_step;

    Eigen::Vector3d Opening_Upperright, Opening_Upperleft, Opening_lowerright, Opening_lowerleft;
    double R_Opening;
    Eigen::Vector3d valve_pos;
    double Opening_height, Opening_width, Opening_depth;

    iDynUtils robot_col;
};

#endif // SELF_COLLISION_AVOIDANCE_H
