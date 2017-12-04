#include <OpenSoT/tasks/acceleration/Postural.h>

OpenSoT::tasks::acceleration::Postural::Postural(const std::string task_id,
         const XBot::ModelInterface& robot,const int x_size):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, x_size),
    _robot(robot)
{
    _na = _robot.getActuatedJointNum();

    robot.getJointPosition(_qref);
    robot.getPosturalJacobian(_Jpostural);

    _qddot = AffineHelper::Identity(x_size);

    _A.setZero(_na, _qddot.getInputSize());

    setLambda(10.0);
    setWeight(Eigen::MatrixXd::Identity(_na, _na));

    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());

    _update(_q);
}

OpenSoT::tasks::acceleration::Postural::Postural(const std::string task_id, 
                                                 const XBot::ModelInterface& robot, 
                                                 OpenSoT::AffineHelper qddot): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _qddot(qddot)
{
    _na = _robot.getActuatedJointNum();
    
    
    robot.getJointPosition(_qref);
    robot.getPosturalJacobian(_Jpostural);
    
    if(_qddot.getInputSize() == 0){
        _qddot = AffineHelper::Identity(_na);
    }
    
    _A.setZero(_na, _qddot.getInputSize());
    
    setLambda(10.);
    setWeight(Eigen::MatrixXd::Identity(_na, _na));

    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
    
    _update(_q);
}

void OpenSoT::tasks::acceleration::Postural::setLambda(double lambda)
{
    if(lambda < 0){
        XBot::Logger::error("in %s: illegal lambda (%f < 0) \n", __func__, lambda);
        return;
    }
    
    
    _lambda = lambda;
    _lambda2 = 4*std::sqrt(lambda);
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
    _qref.tail(_na) = qref.tail(_na);
    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref)
{
    _qref.tail(_na) = qref.tail(_na);
    _qdot_ref.tail(_na) = dqref.tail(_na);
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref,
                  const Eigen::VectorXd& ddqref)
{
    _qref.tail(_na) = qref.tail(_na);
    _qdot_ref.tail(_na) = dqref.tail(_na);
    _qddot_ref.tail(_na) = ddqref.tail(_na);
}


void OpenSoT::tasks::acceleration::Postural::_update(const Eigen::VectorXd& x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    _qddot_d = _qddot_ref + _lambda2*(_qdot_ref - _qdot) + _lambda*(_qref - _q);
    _postural_task = _Jpostural * (_qddot - _qddot_d);

    _A = _postural_task.getM();
    _b = - _postural_task.getq();

    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(_task_id + "_error", _qref - _q);
    logger->add(_task_id + "_qref", _qref);
}

