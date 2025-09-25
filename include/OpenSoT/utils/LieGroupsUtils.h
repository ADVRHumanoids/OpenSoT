#ifndef __OPENSOT_LIEGROUPSUTILS_H__
#define __OPENSOT_LIEGROUPSUTILS_H__

#include <Eigen/Dense>


#define EPSILON 1e-8

namespace Eigen{
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;

// Typedef for 6x6 matrix  
typedef Matrix<double, 6, 6> Matrix6d;
}

namespace OpenSoT{
// Hat operator: R^3 -> so(3)
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return m;
}

// Vee (unhat) operator: so(3) -> R^3
inline Eigen::Vector3d unhat(const Eigen::Matrix3d& m) {
    return Eigen::Vector3d(m(2,1), m(0,2), m(1,0));
}

// Exponential map: so(3) -> SO(3)
inline Eigen::Matrix3d Exp3(const Eigen::Vector3d& p, double epsilon = 1e-8) {
    double theta = p.norm();
    if (theta < epsilon) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d a = p / theta;
    double c = std::cos(theta);
    double s = std::sin(theta);

    return c * Eigen::Matrix3d::Identity()
         + (1.0 - c) * (a * a.transpose())
         + s * hat(a);
}

// Logarithm map: SO(3) -> so(3)
inline Eigen::Vector3d Log3(const Eigen::Matrix3d& R, double eps = 1e-8) {
    // Clamp argument of arccos to [-1,1]
    double cos_theta = (R.trace() - 1.0) / 2.0;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double theta = std::acos(cos_theta);

    // Case 1: theta ~ 0
    if (std::abs(theta) < eps) {
        return Eigen::Vector3d::Zero();
    }

    // Case 2: theta ~ pi
    if (std::abs(theta - M_PI) < eps) {
        double r00 = R(0,0), r11 = R(1,1), r22 = R(2,2);
        double r02 = R(0,2), r12 = R(1,2);
        double r01 = R(0,1), r21 = R(2,1);
        double r10 = R(1,0), r20 = R(2,0);

        if (std::abs(r22 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r22));
            return multiplier * Eigen::Vector3d(r02, r12, 1.0 + r22);
        } else if (std::abs(r11 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r11));
            return multiplier * Eigen::Vector3d(r01, 1.0 + r11, r21);
        } else if (std::abs(r00 + 1.0) > eps) {
            double multiplier = theta / std::sqrt(2.0 * (1.0 + r00));
            return multiplier * Eigen::Vector3d(1.0 + r00, r10, r20);
        }
        // In theory, one of the above should always apply.
        throw std::runtime_error("log_rotation: numerical issue near pi");
    }

    // Case 3: general case
    Eigen::Matrix3d mat = R - R.transpose();
    Eigen::Vector3d r = unhat(mat);   // uses our earlier unhat()
    return (theta / (2.0 * std::sin(theta))) * r;
}

// Left Jacobian of SO(3)
inline Eigen::Matrix3d J_l(const Eigen::Vector3d& theta) {
    double n = theta.norm();  // Euclidean norm of theta

    if (n < EPSILON) {
        return Eigen::Matrix3d::Identity();  // Return identity matrix if norm is too small
    }

    double n_sq = n * n;
    double n_3 = n_sq * n;
    double c = cos(n);
    double s = sin(n);

    Eigen::Matrix3d hat_theta = hat(theta);
    Eigen::Matrix3d hat_theta_sq = hat_theta * hat_theta;  // hat_theta squared (matrix multiplication)

    // JL computation
    Eigen::Matrix3d JL = Eigen::Matrix3d::Identity() + hat_theta * ((1.0 - c) / n_sq) + hat_theta_sq * ((n - s) / n_3);

    return JL;
}

// Left Jacobian of SO(3)
inline Eigen::Matrix3d J_l_inv(const Eigen::Vector3d& theta) {
    double n = theta.norm();  // Euclidean norm of theta
    
    if (n < EPSILON) {
        return Eigen::Matrix3d::Identity();  // Return identity matrix if norm is too small
    }

    double n_sq = n * n;
    double n_3 = n_sq * n;
    double c = cos(n);
    double s = sin(n);

    Eigen::Matrix3d hat_theta = hat(theta);
    Eigen::Matrix3d hat_theta_sq = hat_theta * hat_theta;  // hat_theta squared (matrix multiplication)

    // JL_inv computation
    Eigen::Matrix3d JL_inv = Eigen::Matrix3d::Identity() - 0.5 * hat_theta + (1.0 / n_sq - (1 + c)/(2*n * s))* hat_theta_sq;

    return JL_inv;
}

inline Eigen::Matrix4d Exp6(const Eigen::Vector6d & tau){
    Eigen::Vector3d rho = tau.head(3);
    Eigen::Vector3d theta = tau.tail(3);

    Eigen::Matrix4d M;
    M.setIdentity();
    M.block<3,3>(0,0) = Exp3(theta);
    M.block<3,1>(0,3) = J_l(theta) * rho;

    return M;

}

inline Eigen::Vector6d Log6(const Eigen::Affine3d& T){
    Eigen::Vector6d v;

    v.head(3) = J_l_inv(Log3(T.linear())) * T.translation();
    v.tail(3) = Log3(T.linear());
    return v;
}



// Q(ρ,θ)
inline Eigen::Matrix3d computeQ(const Eigen::Vector6d & tau) {
    Eigen::Vector3d rho = tau.head(3);
    Eigen::Vector3d theta = tau.tail(3);

    double theta_norm = theta.norm();
    
    if (theta_norm < 1e-8) {
        // Small angle approximation
        return 0.5 * hat(rho);
    }
    
    double sin_theta = std::sin(theta_norm);
    double cos_theta = std::cos(theta_norm);
    double theta2 = theta_norm * theta_norm;
    double theta3 = theta2 * theta_norm;
    double theta4 = theta3 * theta_norm;
    double theta5 = theta4 * theta_norm;
    
    Eigen::Matrix3d rho_cross = hat(rho);
    Eigen::Matrix3d theta_cross = hat(theta);
    
    // First term: (1/2)ρ×
    Eigen::Matrix3d Q = 0.5 * rho_cross;
    
    // Second term: (θ-sin θ)/θ³ (θ×ρ× + ρ×θ× + θ×ρ×θ×)
    double coeff2 = (theta_norm - sin_theta) / theta3;
    Q += coeff2 * (theta_cross * rho_cross + rho_cross * theta_cross + theta_cross * rho_cross * theta_cross);
    
    // Third term: -(1-θ²/2-cos θ)/θ⁴ (θ²×ρ× + ρ×θ²× - 3θ×ρ×θ×)
    double coeff3 = -(1.0 - theta2/2.0 - cos_theta) / theta4;
    Eigen::Matrix3d theta_cross_squared = theta_cross * theta_cross;
    Q += coeff3 * (theta_cross_squared * rho_cross + rho_cross * theta_cross_squared - 3.0 * theta_cross * rho_cross * theta_cross);
    
    // Fourth term: -(1/2) * ((1-θ²/2-cos θ)/θ⁴ - 3(θ-sin θ-θ³/6)/θ⁵) * (θ×ρ×θ²× + θ²×ρ×θ×)
    double term1 = (1.0 - theta2/2.0 - cos_theta) / theta4;
    double term2 = 3.0 * (theta_norm - sin_theta - theta3/6.0) / theta5;
    double coeff4 = -0.5 * (term1 - term2);
    Q += coeff4 * (theta_cross * rho_cross * theta_cross_squared + theta_cross_squared * rho_cross * theta_cross);
    
    return Q;
}

// Left Jacobian for SE(3)
static Eigen::Matrix6d J_l6(const Eigen::Vector6d & tau) {
    Eigen::Vector3d rho = tau.head(3);
    Eigen::Vector3d theta = tau.tail(3);
    Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
    
    Eigen::Matrix3d J_left = J_l(theta);
    Eigen::Matrix3d Q = computeQ(tau);
    
    // Upper left block: J_l(θ)
    J.block<3,3>(0,0) = J_left;
    
    // Upper right block: Q(ρ,θ)  
    J.block<3,3>(0,3) = Q;
    
    // Lower left block: 0
    J.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
    
    // Lower right block: J_l(θ)
    J.block<3,3>(3,3) = J_left;
    
    return J;
}

// Left Jacobian Inverse for SE(3)
static Eigen::Matrix6d J_l6_inv(const Eigen::Vector6d & tau) {
    Eigen::Vector3d theta = tau.tail(3);
    Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
    
    Eigen::Matrix3d J_left = J_l(theta);
    Eigen::Matrix3d Q = computeQ(tau);
    
    // Upper left block: J_l(θ)
    J.block<3,3>(0,0) = J_left;
    
    // Upper right block: Q(ρ,θ)  
    J.block<3,3>(0,3) = Q;
    
    // Lower left block: 0
    J.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
    
    // Lower right block: J_l(θ)
    J.block<3,3>(3,3) = J_left;
    
    return J;
}

static Eigen::Matrix6d Adjoint(const Eigen::Matrix4d M){
    Eigen::Vector3d t = M.block(0,3,3,1);
    Eigen::Vector3d R = M.block(0,0,3,3);

    Eigen::Matrix6d Adj;
    Adj.setZero();

    Adj.block(0,0,3,3) = R;
    Adj.block(3,3,3,3) = R;
    Adj.block(0,3,3,3) = hat(t) * R;

    return Adj;
}

// Exponential map: Quaternion -> SO(3)
inline Eigen::Matrix3d QUATtoSO3(const Eigen::Vector4d& q) {
    Eigen::Quaterniond Q(q(3), q(0), q(1), q(2));
    Q.normalize();
    return Q.toRotationMatrix();
}

// Logarithm map: SO(3) -> Quaternion
inline Eigen::Vector4d SO3toQUAT(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    return q.coeffs();
}


inline Eigen::Matrix4d XYZQUATtoSE3(const Eigen::Vector7d& q) {
    Eigen::Matrix4d M;

    M.block<3,3>(0,0) = QUATtoSO3(q.tail(4));
    M.block<3,1>(0,3) = q.head(3);
    return M;
}

// Logarithm map: SO(3) -> Quaternion
inline Eigen::Vector7d SE3toXYZQUAT(const Eigen::Matrix4d& M) {
    Eigen::Vector7d q;

    q.head(3) = M.block<3,1>(0,3);
    q.tail(4) = SO3toQUAT(M.block<3,3>(0,0));

    return q;

}




} // namespace OpenSot

#endif // __OPENSOT_LIEGROUPSUTILS_H__