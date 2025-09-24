#include <OpenSoT/oc/SE3Derivatives.h>

using namespace OpenSoT::oc;

SE3Derivatives::SE3Derivatives(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               const double dt):
    Task< Eigen::MatrixXd, Eigen::VectorXd> ("SE3Derivatives", dX.getInputSize()),
    _robot(robot),
    _dU(dU),
    _dX(dX),
    _dt(dt)
{
    if(_dX.getOutputSize() != 6)
        throw std::runtime_error("_dX != 6");

    if(_dU.getOutputSize() != 6)
        throw std::runtime_error("_dU != 6");


    _W.setIdentity(dX.getOutputSize(), dX.getOutputSize());

    update();
}

void SE3Derivatives::_update()
{
    // Eigen::VectorXd q;
    // _robot.getJointPosition(q);

    Eigen::VectorXd qdot;
    _robot.getJointVelocity(qdot);

    Eigen::Vector6d xi = qdot.segment<6>(0) * _dt;

    Eigen::Matrix3d RbT = Exp3(xi.tail(3)).transpose();    
    Eigen::Matrix3d t_skew = hat(xi.head(3));


    Eigen::Matrix6d Fx;
    Fx.setZero();
    Fx.block<3,3>(0,0) = RbT;
    Fx.block<3,3>(3,3) = RbT;
    Fx.block<3,3>(0,3) = -RbT * t_skew;

    Eigen::Matrix6d Fu = J_l6(-xi) * _dt;

    _dXnext = Fx * _dX + Fu * _dU;

    _A = _dXnext.getM();
    _b = -_dXnext.getq();
}
