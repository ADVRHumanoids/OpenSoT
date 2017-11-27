#include <OpenSoT/tasks/acceleration/Postural.h>


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
    
    setLambda(0.1);
    setWeight(Eigen::MatrixXd::Identity(na, na));
    
    _update(_q);
}

void OpenSoT::tasks::acceleration::Postural::setReference(const Eigen::VectorXd& qref)
{
    _qref = qref;
}

void OpenSoT::tasks::acceleration::Postural::_update(const Eigen::VectorXd& x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    _qddot_ref = 2*_lambda*_qdot + _lambda*_lambda*(_qref - _q);
    
    _postural_task = _Jpostural * _qddot;
    _A = _postural_task.getM();
    _b = - _postural_task.getq();
}
