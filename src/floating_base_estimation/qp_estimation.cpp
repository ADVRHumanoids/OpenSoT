#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <OpenSoT/utils/Affine.h>


OpenSoT::floating_base_estimation::qp_estimation::qp_estimation(XBot::ModelInterface::Ptr model,
                                                            XBot::ImuSensor::ConstPtr imu,
                                                            std::vector<string> contact_links,
                                                            const Eigen::MatrixXd &contact_matrix):
FloatingBaseEstimation(model, imu, contact_links, contact_matrix)
{
    for(unsigned int i = 0; i < contact_links.size(); ++i)
    {
        OpenSoT::tasks::floating_base::Contact::Ptr tmp(
            new OpenSoT::tasks::floating_base::Contact(*_model, contact_links[i], _contact_matrix));
        _contact_tasks.push_back(tmp);
        _map_tasks[contact_links[i]] = i;
    }

    _aggregated_tasks.reset(new tasks::Aggregated(_contact_tasks, 6));

    if(imu){
        _imu_task.reset(new OpenSoT::tasks::floating_base::IMU(*_model, imu));
        Eigen::MatrixXd W = 100.*Eigen::MatrixXd::Identity(3,3);
        _imu_task->setWeight(W);}

    AffineHelper var = AffineHelper::Identity(6);
    Eigen::VectorXd ub = 10.*Eigen::VectorXd::Ones(6);
    Eigen::VectorXd lb = -ub;
    _fb_limits.reset(new constraints::GenericConstraint("fb_limits", var, ub, lb,
        constraints::GenericConstraint::Type::BOUND));

    if(imu)
        _autostack.reset(new AutoStack(_aggregated_tasks + _imu_task));
    else
        _autostack.reset(new AutoStack(_aggregated_tasks));
    _autostack<<_fb_limits;

    _solver.reset(new solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds()));

    _Qdot.setZero(6);
    _Q.setZero();
    if(!update(0))
        throw std::runtime_error("Update failed!");
}

OpenSoT::floating_base_estimation::qp_estimation::~qp_estimation()
{

}

bool OpenSoT::floating_base_estimation::qp_estimation::setContactState(
        const std::string& contact_link, const bool state)
{
    if(!OpenSoT::FloatingBaseEstimation::setContactState(contact_link, state))
        return false;

    unsigned int i = _map_tasks[contact_link];
    std::list<OpenSoT::tasks::Aggregated::TaskPtr>::iterator it =
            std::next(_contact_tasks.begin(), i);
    (*it)->setActive(state);
}

bool OpenSoT::floating_base_estimation::qp_estimation::update(double dT)
{
    if(_imu){
        _model->setFloatingBaseState(_imu);
        _model->update();}

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);

    _Q = _q.segment(0,6);
    _autostack->update(Eigen::VectorXd::Zero(0));

    if(!_solver->solve(_Qdot)){
        XBot::Logger::error("Solver in floating base estimation return false!\n");
        return false;
    }
    
    if(!_imu)
    {
        _Q += _Qdot*dT;
        _q.segment(0,6) = _Q;
        _qdot.segment(0,6) = _Qdot;
    }
    else
    {
        _Q.segment(0,3) += _Qdot.segment(0,3)*dT;
        _q.segment(0,3) = _Q.segment(0,3);
        _qdot.segment(0,3) = _Qdot.segment(0,3);
    }

    

    _model->setJointPosition(_q);
    _model->setJointVelocity(_qdot);
    _model->update();
    return true;
}

void OpenSoT::floating_base_estimation::qp_estimation::log(XBot::MatLogger::Ptr logger)
{
    _solver->log(logger);
    _autostack->log(logger);
}
