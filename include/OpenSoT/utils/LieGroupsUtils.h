#ifndef __OPENSOT_LIEGROUPSUTILS_H__
#define __OPENSOT_LIEGROUPSUTILS_H__

#include <Eigen/Dense>


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

// Exponential map: Quaternion -> SO(3)
inline Eigen::Matrix3d Exp_quat(const Eigen::Vector4d& q) {
    Eigen::Quaterniond Q(q(3), q(0), q(1), q(2));
    Q.normalize();
    return Q.toRotationMatrix();
}

// Logarithm map: SO(3) -> Quaternion
inline Eigen::Vector4d Log_quat(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    return q.coeffs();
}

} // namespace OpenSot

#endif // __OPENSOT_LIEGROUPSUTILS_H__