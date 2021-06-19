#include <OpenSoT/tasks/acceleration/AngularMomentum.h>

using namespace OpenSoT::tasks::acceleration;

AngularMomentum::AngularMomentum(const Eigen::VectorXd &x, XBot::ModelInterface &robot, const AffineHelper &qddot):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("angular_momentum", qddot.getInputSize()),
    _robot(robot),
    _qddot(qddot),
    _base_link(BASE_LINK_COM),
    _distal_link(DISTAL_LINK_COM),
    _is_init(false)
{
    _lambda = 1.;
    _hessianType = OpenSoT::HST_SEMIDEF;

    _W.resize(3,3);
    _W.setIdentity(3,3);

    _K.resize(3,3);
    _K.setIdentity(3,3);

    _Ldot_d.setZero();
    _update(Eigen::VectorXd(1));
}

AngularMomentum::~AngularMomentum()
{

}

void AngularMomentum::_update(const Eigen::VectorXd &x)
{
    //1. get centroidal momentum matrix and momentum
    _robot.getCentroidalMomentumMatrix(_Mom, _Momdot);
    _robot.getCentroidalMomentum(_L);

    //2. if not init, initialize momentum reference with actual momentum
    if(!_is_init)
    {
        _L_d = _L.tail(3);
        _is_init = true;
    }

    //3. compute simple control law
    _Ldot_ref = _Ldot_d + _lambda*_K*(_L_d - _L.tail(3));

    //4. write task
     _momentum_task = _Mom*_qddot + _Momdot;
     _momentum_task = _momentum_task - _Ldot_ref;

     _A = _momentum_task.getM();
     _b = -_momentum_task.getq();

     //5. reset references for safety reasons
     _L_d.setZero(3);
     _Ldot_d.setZero(3);
}

void AngularMomentum::setMomentumGain(const Eigen::Matrix3d& K)
{
    _K = K;
}

void AngularMomentum::setReference(const Eigen::Vector3d& desiredAngularMomentum)
{
    _L_d = desiredAngularMomentum;
    _Ldot_d.setZero(3);
}

void AngularMomentum::setReference(const Eigen::Vector3d& desiredAngularMomentum, const Eigen::Vector3d& desiredAngularMomentumVariation)
{
    _L_d = desiredAngularMomentum;
    _Ldot_d = desiredAngularMomentumVariation;
}

void AngularMomentum::getReference(Eigen::Vector3d& desiredAngularMomentum) const
{
    desiredAngularMomentum = _L_d;
}

void AngularMomentum::getReference(Eigen::Vector3d& desiredAngularMomentum, Eigen::Vector3d& desiredAngularMomentumVariation) const
{
   getReference(desiredAngularMomentum);
   desiredAngularMomentumVariation = _Ldot_d;
}

const std::string& AngularMomentum::getBaseLink() const
{
    return _base_link;
}

const std::string& AngularMomentum::getDistalLink() const
{
    return _distal_link;
}

bool AngularMomentum::isAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<AngularMomentum>(task);
}

OpenSoT::tasks::acceleration::AngularMomentum::Ptr AngularMomentum::asAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<AngularMomentum>(task);
}


