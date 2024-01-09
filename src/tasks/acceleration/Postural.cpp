#include <OpenSoT/tasks/acceleration/Postural.h>

OpenSoT::tasks::acceleration::Postural::Postural(
         const XBot::ModelInterface& robot, const std::string task_id):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, robot.getNv()),
    _robot(robot),
    _gain_type(OpenSoT::tasks::acceleration::GainType::Acceleration)
{

    _hessianType = HST_SEMIDEF;

    robot.getJointPosition(_qref);

    _qddot = AffineHelper::Identity(_x_size);

    _A.setZero(_x_size, _x_size);

    _Kp.setIdentity(_qref.size(), _qref.size());
    _Kd.setIdentity(_qref.size(), _qref.size());

    setLambda(10.0);
    setWeight(Eigen::MatrixXd::Identity(_x_size, _x_size));

    _qdot_ref.setZero(robot.getNv());
    _qddot_ref.setZero(robot.getNv());
    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;

    _Mi.setIdentity(robot.getNv(), robot.getNv());

    _update(_q);
}

OpenSoT::tasks::acceleration::Postural::Postural(const XBot::ModelInterface& robot,
                                                 OpenSoT::AffineHelper qddot, const std::string task_id):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _qddot(qddot),
    _gain_type(OpenSoT::tasks::acceleration::GainType::Acceleration)
{

    _hessianType = HST_SEMIDEF;
    
    robot.getJointPosition(_qref);
    
    _A.setZero(qddot.getOutputSize(), qddot.getInputSize());

    _Kp.setIdentity(robot.getNv(), robot.getNv());
    _Kd.setIdentity(robot.getNv(), robot.getNv());
    
    setLambda(10.);
    setWeight(Eigen::MatrixXd::Identity(qddot.getOutputSize(), qddot.getOutputSize()));

    _qdot_ref.setZero(robot.getNv());
    _qddot_ref.setZero(robot.getNv());
    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;

    
    _update(_q);
}

void OpenSoT::tasks::acceleration::Postural::setGainType(GainType type)
{
    _gain_type = type;
}

OpenSoT::tasks::acceleration::GainType OpenSoT::tasks::acceleration::Postural::getGainType() const
{
    return _gain_type;
}

void OpenSoT::tasks::acceleration::Postural::setLambda(double lambda)
{
    if(lambda < 0){
        XBot::Logger::error("in %s: illegal lambda (%f < 0) \n", __func__, lambda);
        return;
    }
    
    _lambda = lambda;
    _lambda2 = 2*std::sqrt(lambda);
}

void OpenSoT::tasks::acceleration::Postural::setLambda(double lambda1, double lambda2)
{
    if( lambda1 < 0 || lambda2 < 0 )
    {
        XBot::Logger::error("in %s: illegal lambda (%f < 0 || %f < 0) \n", __func__, lambda1, lambda2);
        return;
    }
    
    _lambda = lambda1;
    _lambda2 = lambda2;
}



void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref)
{
    _qref = qref;
    _qdot_ref.setZero(_robot.getNv());
    _qddot_ref.setZero(_robot.getNv());

    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;

}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref,
                                                          const Eigen::VectorXd& dqref)
{
    _qref = qref;
    _qdot_ref = dqref;
    _qddot_ref.setZero(_robot.getNv());

    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;

}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref,
                                                          const Eigen::VectorXd& dqref,
                                                          const Eigen::VectorXd& ddqref)
{
    _qref = qref;
    _qdot_ref = dqref;
    _qddot_ref = ddqref;

    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;

}


void OpenSoT::tasks::acceleration::Postural::_update(const Eigen::VectorXd& x)
{
    _qdot_ref_cached = _qdot_ref;
    _qddot_ref_cached = _qddot_ref;


    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    _robot.difference(_qref, _q, _position_error);
    _velocity_error = _qdot_ref - _qdot;

    if(_gain_type == Acceleration)
    {
        _qddot_d = _qddot_ref + _lambda2*_Kd*_velocity_error + _lambda*_Kp*_position_error;
    }
    else if(_gain_type == Force)
    {
        _robot.computeInertiaInverse(_Mi);

        _qddot_d = _qddot_ref + _Mi*_lambda2*_Kd*_velocity_error + _Mi*_lambda*_Kp*_position_error;
    }


    _postural_task = (_qddot - _qddot_d);

    _A = _postural_task.getM();
    _b = - _postural_task.getq();

    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add(_task_id + "_position_error", _position_error);
    logger->add(_task_id + "_velocity_error", _velocity_error);
    logger->add(_task_id + "_qref", _qref);

    logger->add(_task_id + "_velocity_reference", _qdot_ref_cached);
    logger->add(_task_id + "_acceleration_reference", _qddot_ref_cached);

    logger->add(_task_id + "_lambda2", _lambda2);
}

const Eigen::VectorXd &OpenSoT::tasks::acceleration::Postural::getReference() const
{
    return _qref;
}

void OpenSoT::tasks::acceleration::Postural::getReference(Eigen::VectorXd& q_desired) const
{
    q_desired = _qref;
}
void OpenSoT::tasks::acceleration::Postural::getReference(Eigen::VectorXd& q_desired,
                  Eigen::VectorXd& qdot_desired) const
{
    q_desired = _qref;
    qdot_desired = _qdot_ref;
}

void OpenSoT::tasks::acceleration::Postural::getReference(Eigen::VectorXd& q_desired,
                  Eigen::VectorXd& qdot_desired,
                  Eigen::VectorXd& qddot_desired) const
{
    q_desired = _qref;
    qdot_desired = _qdot_ref;
    qddot_desired = _qddot_ref;
}

const Eigen::VectorXd& OpenSoT::tasks::acceleration::Postural::getActualPositions() const
{
    return _q;
}

const Eigen::VectorXd& OpenSoT::tasks::acceleration::Postural::getError() const
{
    return _position_error;
}

const Eigen::VectorXd& OpenSoT::tasks::acceleration::Postural::getVelocityError() const
{
    return _velocity_error;
}

bool OpenSoT::tasks::acceleration::Postural::reset()
{
    _robot.getJointPosition(_qref);

    _qdot_ref.setZero();
    _qddot_ref.setZero();
    return true;
}

void OpenSoT::tasks::acceleration::Postural::getLambda(double & lambda, double & lambda2)
{
    lambda = _lambda;
    lambda2 = _lambda2;
}

const double OpenSoT::tasks::acceleration::Postural::getLambda2() const
{
    return _lambda2;
}

const Eigen::VectorXd& OpenSoT::tasks::acceleration::Postural::getCachedVelocityReference() const
{
    return _qdot_ref_cached;
}

const Eigen::VectorXd& OpenSoT::tasks::acceleration::Postural::getCachedAccelerationReference() const
{
    return _qddot_ref_cached;
}

void OpenSoT::tasks::acceleration::Postural::setKp(const Eigen::MatrixXd& Kp)
{
    _Kp = Kp;
}

void OpenSoT::tasks::acceleration::Postural::setKd(const Eigen::MatrixXd& Kd)
{
    _Kd = Kd;
}

void OpenSoT::tasks::acceleration::Postural::setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd)
{
    setKp(Kp);
    setKd(Kd);
}

const Eigen::MatrixXd& OpenSoT::tasks::acceleration::Postural::getKp() const
{
    return _Kp;
}

const Eigen::MatrixXd& OpenSoT::tasks::acceleration::Postural::getKd() const
{
    return _Kd;
}

void OpenSoT::tasks::acceleration::Postural::getGains(Eigen::MatrixXd& Kp, Eigen::MatrixXd& Kd)
{
    Kp = _Kp;
    Kd = _Kd;
}


