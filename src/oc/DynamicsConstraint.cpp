#include <OpenSoT/oc/DynamicsConstraint.h>

using namespace OpenSoT::oc;

DynamicsConstraint::DynamicsConstraint(XBot::ModelInterface &robot,
                                       const AffineHelper &dX,
                                       const AffineHelper &dU,
                                       const double dt) : Task<Eigen::MatrixXd, Eigen::VectorXd>("DynamicsConstraint", dX.getInputSize()),
                                                          _robot(robot),
                                                          _dU(dU),
                                                          _dX(dX),
                                                          _dt(dt)
{
    // if(_dX.getOutputSize() != 6)
    //     throw std::runtime_error("_dX != 6");

    // if(_dU.getOutputSize() != 6)
    //     throw std::runtime_error("_dU != 6");

    _W.setIdentity(dX.getOutputSize(), dX.getOutputSize());

    update();

    _dtau_dq.resize(robot.getNv(), robot.getNv());
    _dtau_dv.resize(robot.getNv(), robot.getNv());
    _dtau_da.resize(robot.getNv(), robot.getNv());
}

void DynamicsConstraint::_update()
{
    _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da);

    _Fx.resize(_robot.getNv(), _dX.getOutputSize());
    _Fu.resize(_robot.getNv(), _dU.getOutputSize());

    _Fx.setZero();
    _Fu.setZero();

    _Fx.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_dq;
    _Fx.block(0, _robot.getNv(), _robot.getNv(), _robot.getNv()) = _dtau_dv;
    _Fu.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_da;

    // _upperBound = ( _jointLimitsMax - _dq);
    // _lowerBound = ( _jointLimitsMin - _dq);

    _dTAU = _Fx * _dX + _Fu * _dU;

    _A = _dTAU.getM();
    _b = -_dTAU.getq();
}
