#include <OpenSoT/oc/TorquesTask.h>

using namespace OpenSoT::oc;

TorquesTask::TorquesTask(XBot::ModelInterface &robot,
                                       const AffineHelper &dX,
                                       const AffineHelper &dU) : Task<Eigen::MatrixXd, Eigen::VectorXd>("TorquesTask", dX.getInputSize()),
                                                                _robot(robot),
                                                                _dU(dU),
                                                                _dX(dX)
{

    _W.setIdentity(_robot.getNv(), _robot.getNv());

    update();

    _dtau_dq.resize(robot.getNv(), robot.getNv());
    _dtau_dv.resize(robot.getNv(), robot.getNv());
    _dtau_da.resize(robot.getNv(), robot.getNv());
}

void TorquesTask::_update()
{
    _robot.computeInverseDynamicsDerivative(_dtau_dq, _dtau_dv, _dtau_da);

    _Fx.resize(_robot.getNv(), _dX.getOutputSize());
    _Fu.resize(_robot.getNv(), _dU.getOutputSize());

    _Fx.setZero();
    _Fu.setZero();

    _Fx.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_dq;
    _Fx.block(0, _robot.getNv(), _robot.getNv(), _robot.getNv()) = _dtau_dv;
    _Fu.block(0, 0, _robot.getNv(), _robot.getNv()) = _dtau_da;

    _dTAU = _Fx * _dX + _Fu * _dU + _robot.computeInverseDynamics(); //why is this here?

    _A = _dTAU.getM();
    _b = -_dTAU.getq();
}
