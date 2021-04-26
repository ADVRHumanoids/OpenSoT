#include <OpenSoT/constraints/force/NormalTorque.h>

using namespace OpenSoT::constraints::force;

NormalTorque::NormalTorque(const std::string &contact_link,
                           const AffineHelper &wrench,
                           XBot::ModelInterface &model,
                           const Eigen::Vector2d& X_Lims,
                           const Eigen::Vector2d& Y_Lims,
                           const double &mu):
    Constraint(contact_link+"_NormalTorque", wrench.getInputSize()),
    _contact_link(contact_link),
    _model(model),
    _wrench(wrench),
    _mu(mu)
{
    _A.setZero(8, 6);
    _AAd = _A;
    _updateA();

    _Ad.resize(6,6); _Ad.setZero(6,6);
    _Ad2.resize(6,6); _Ad2.setIdentity(6,6);
    double px = (X_Lims[1] + X_Lims[0])/2.;
    double py = (Y_Lims[1] + Y_Lims[0])/2.;
    _Ad2(3,2) = py;
    _Ad2(4,2) = -px;
    _Ad2(5,0) = -py; _Ad2(5,1) = px;


    _X = (std::fabs(X_Lims[0]) + std::fabs(X_Lims[1]))/2.;
    _Y = (std::fabs(Y_Lims[0]) + std::fabs(Y_Lims[1]))/2.;

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

    Eigen::MatrixXd tmpA = _A;
    _A = tmpA*_Ad2;
}

void NormalTorque::setMu(const double mu)
{
    _mu = mu;
    _updateA();
}

///////

NormalTorques::NormalTorques(const std::vector<std::string>& contact_name,
                             const std::vector<AffineHelper>& wrench,
                             XBot::ModelInterface &robot,
                             const std::vector<Eigen::Vector2d> & Xs,
                             const std::vector<Eigen::Vector2d> & Ys,
                             const std::vector<double> & mu):
    Constraint("NormalTorques", wrench[0].getInputSize())
{
    std::list<ConstraintPtr> constraint_list;
    for(unsigned int i = 0; i < contact_name.size(); ++i){
        auto nt = ::boost::make_shared<NormalTorque>(
                    contact_name[i],
                    wrench[i],
                    robot,
                    Xs[i], Ys[i],
                    mu[i]);
        constraint_list.push_back(nt);
    }

    _internal_constraint = boost::make_shared<OpenSoT::constraints::Aggregated>
            (constraint_list, wrench[0].getInputSize());

    update(Eigen::VectorXd(0));
}

NormalTorque::Ptr NormalTorques::getNormalTorque(const std::string& contact_name)
{
    if(_normal_torque_map.count(contact_name))
        return _normal_torque_map[contact_name];
    else
        return NULL;
}

void NormalTorques::update(const Eigen::VectorXd &x)
{
    _internal_constraint->update(x);
    generateBounds();
}

void NormalTorques::generateBounds()
{
    _Aineq = _internal_constraint->getAineq();
    _bUpperBound = _internal_constraint->getbUpperBound();
    _bLowerBound = _internal_constraint->getbLowerBound();
}


