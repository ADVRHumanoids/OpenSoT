#include <OpenSoT/tasks/acceleration/Postural.h>

OpenSoT::tasks::acceleration::Postural::Postural(const std::string task_id,
         const XBot::ModelInterface& robot,const int x_size):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, x_size),
    _robot(robot)
{
    int na = _robot.getActuatedJointNum();

    robot.getJointPosition(_qref);
    robot.getPosturalJacobian(_Jpostural);

    _qddot = AffineHelper::Identity(na);

    _A.setZero(na, _qddot.getInputSize());

    setLambda(10.);
    setLambda2(2.*sqrt(_lambda));
    setWeight(Eigen::MatrixXd::Identity(na, na));

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
    int na = _robot.getActuatedJointNum();
    
    
    robot.getJointPosition(_qref);
    robot.getPosturalJacobian(_Jpostural);
    
    if(_qddot.getInputSize() == 0){
        _qddot = AffineHelper::Identity(na);
    }
    
    _A.setZero(na, _qddot.getInputSize());
    
    setLambda(10.);
    setLambda2(2.*sqrt(_lambda));
    setWeight(Eigen::MatrixXd::Identity(na, na));

    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
    
    _update(_q);
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref)
{
    _qref = qref;
    _qdot_ref.setZero(_qref.size());
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref)
{
    _qref = qref;
    _qdot_ref = dqref;
    _qddot_ref.setZero(_qref.size());
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& dqref,
                  const Eigen::VectorXd& ddqref)
{
    _qref = qref;
    _qdot_ref = dqref;
    _qddot_ref = ddqref;
}

void OpenSoT::tasks::acceleration::Postural::setLambda2(const double lambda2)
{
    _lambda2 = lambda2;
}

void OpenSoT::tasks::acceleration::Postural::_update(const Eigen::VectorXd& x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    _qddot_d = _qddot_ref + _lambda2*(_qdot_ref - _qdot) + _lambda*(_qref - _q);
    _postural_task = _Jpostural * (_qddot - _qddot_d);

    _A = _postural_task.getM();
    _b = - _postural_task.getq();
}
