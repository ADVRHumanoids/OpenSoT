#include <OpenSoT/constraints/force/NormalTorque.h>

using namespace OpenSoT::constraints::force;

NormalTorque::NormalTorque(const std::string &contact_link,
                           const AffineHelper &wrench,
                           XBot::ModelInterface &model,
                           const double &X, const double &Y,
                           const double &mu):
    Constraint(contact_link+"_NormalTorque", wrench.getInputSize()),
    _contact_link(contact_link),
    _model(model),
    _X(X), _Y(Y),
    _mu(mu)
{
    _A.setZero(8, 6);
    _AAd = _A;
    _updateA();

    _Ad.resize(6,6); _Ad.setZero(6,6);

    _bUpperBound.setZero(_A.rows());
    _bLowerBound = -1.0e20*Eigen::VectorXd::Ones(_A.rows());

    update(Eigen::VectorXd(1));
}

void NormalTorque::update(const Eigen::VectorXd &x)
{
    _model.getPose(_contact_link, _T);
    _Ti = _T.inverse();

    _Ad.block<3,3>(0,0) = _Ti.linear();
    _Ad.block<3,3>(3,3) = _Ti.linear();

    _AAd = _A*_Ad;

    _constraint = _AAd * _wrench;
    _Aineq = _constraint.getM();
}

void NormalTorque::_updateA()
{
    double K = -_mu*(_X+_Y);

    _A.row(0) << -_Y, -_X, K, -_mu, -_mu, 1;
    _A.row(1) << -_Y,  _X, K, -_mu,  _mu, 1;
    _A.row(2) <<  _Y, -_X, K,  _mu, -_mu, 1;
    _A.row(3) <<  _Y,  _X, K,  _mu,  _mu, 1;

    _A.row(4) <<  _Y,  _X, K, -_mu, -_mu, -1;
    _A.row(5) <<  _Y, -_X, K, -_mu,  _mu, -1;
    _A.row(6) << -_Y,  _X, K,  _mu, -_mu, -1;
    _A.row(7) << -_Y, -_X, K,  _mu,  _mu, -1;
}

void NormalTorque::setMu(const double mu)
{
    _mu = mu;
    _updateA();
}


