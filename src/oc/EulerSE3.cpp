#include <OpenSoT/oc/EulerSE3.h>

using namespace OpenSoT::oc;

EulerSE3::EulerSE3(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               const AffineHelper& Xk,
                               const AffineHelper& Uk,
                               const AffineHelper& Xk_1,
                               const double dt):
    Task< Eigen::MatrixXd, Eigen::VectorXd> ("EulerSE3", dX.getInputSize()),
    _robot(robot),
    _dU(dU),
    _dX(dX),
    _Xk(Xk),
    _Uk(Uk),
    _Xk_1(Xk_1),
    _dt(dt)
{
    if(_dX.getOutputSize() != 6)
        throw std::runtime_error("_dX != 6");

    if(_dU.getOutputSize() != 6)
        throw std::runtime_error("_dU != 6");


    _W.setIdentity(dX.getOutputSize(), dX.getOutputSize());

    update();
}

void EulerSE3::_update()
{  
    _robot.getJointVelocity(_qdot);

    _xi = _qdot.segment<6>(0) * _dt;

    _RbT = Exp3(_xi.tail(3)).transpose();    
    _t_skew = hat(_xi.head(3));

    _Fx.setZero();
    _Fx.block<3,3>(0,0) = _RbT;
    _Fx.block<3,3>(3,3) = _RbT;
    _Fx.block<3,3>(0,3) = -_RbT * _t_skew;

    _Fu = J_l6(-_xi) * _dt;

    _dXnext = _Fx * _dX + _Fu * _dU + Log6((XYZQUATtoSE3(_Xk.getValue()) * Exp6(_Uk.getValue()*_dt)).inverse() * XYZQUATtoSE3(_Xk_1.getValue()));

    _A = _dXnext.getM();
    _b = -_dXnext.getq();
}
