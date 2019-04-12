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
        Eigen::MatrixXd W = 500.*Eigen::MatrixXd::Identity(6,6);
        _imu_task->setWeight(W);}

    AffineHelper var = AffineHelper::Identity(6);
    Eigen::VectorXd ub(6);
    ub<<1e6*Eigen::VectorXd::Ones(3),1e6*Eigen::VectorXd::Ones(3);
    Eigen::VectorXd lb = -ub;
    _fb_limits.reset(new constraints::GenericConstraint("fb_limits", var, ub, lb,
        constraints::GenericConstraint::Type::BOUND));

    if(imu)
        _autostack.reset(new AutoStack(_aggregated_tasks + _imu_task));
    else
        _autostack.reset(new AutoStack(_aggregated_tasks));
    _autostack<<_fb_limits;

    _solver.reset(new solvers::iHQP(_autostack->getStack(), _autostack->getBounds()));
    _solver->setSolverID("FloatingBaseEstimation");

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
    return true;
}

bool OpenSoT::floating_base_estimation::qp_estimation::update(double dT)
{
//    if(_imu){
//        _model->setFloatingBaseState(_imu);
//        _model->update();}

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);

    _Q = _q.segment(0,6);
    _autostack->update(Eigen::VectorXd::Zero(0));

    if(!_solver->solve(_Qdot)){
        XBot::Logger::error("Solver in floating base estimation return false!\n");
        return false;
    }

//    if(!_imu)
//    {
//        _Q += _Qdot*dT;
//        _q.segment(0,6) = _Q;
//        _qdot.segment(0,6) = _Qdot;
//    }
//    else
//    {
//        _Q.segment(0,3) += _Qdot.segment(0,3)*dT;
//        _q.segment(0,3) = _Q.segment(0,3);
//        _qdot.segment(0,3) = _Qdot.segment(0,3);
//    }

    _Q += _Qdot*dT;
    _q.segment(0,6) = _Q;
    _qdot.segment(0,6) = _Qdot;

    

    _model->setJointPosition(_q);
    _model->setJointVelocity(_qdot);
    _model->update();
    return true;
}

void OpenSoT::floating_base_estimation::qp_estimation::log(XBot::MatLogger::Ptr logger)
{
    _solver->log(logger);
    _autostack->log(logger);
    logger->add("qp_estimation_Q", _Q);
    logger->add("qp_estimation_Qdot", _Qdot);
}


////////////////
OpenSoT::floating_base_estimation::kinematic_estimation::kinematic_estimation(XBot::ModelInterface::Ptr model,
                                                                              const std::string& anchor_link,
                                                                              const Eigen::Affine3d& anchor_pose):
    _model(model)
{
    if(_model->getLinkID(anchor_link) == -1)
        throw std::runtime_error(anchor_link + " for anchor link does not exists!");
    _anchor_link = anchor_link;

    _world_T_anchor = anchor_pose;

    _model->getFloatingBaseLink(_base_link);

}


const std::string& OpenSoT::floating_base_estimation::kinematic_estimation::getAnchor()
{
    return _anchor_link;
}

bool OpenSoT::floating_base_estimation::kinematic_estimation::setAnchor(const std::string& anchor_link)
{
    if(_model->getLinkID(anchor_link) == -1)
        return false;

    if(_anchor_link == anchor_link)
        return true;

    _model->getPose(anchor_link, _anchor_link, _old_anchor_T_new_anchor);
    _anchor_link = anchor_link;
    _world_T_new_anchor = _world_T_anchor*_old_anchor_T_new_anchor;
    setAnchorPose(_world_T_new_anchor);

    return true;
}

const Eigen::Affine3d& OpenSoT::floating_base_estimation::kinematic_estimation::getAnchorPose()
{
    return _world_T_anchor;
}

void OpenSoT::floating_base_estimation::kinematic_estimation::getAnchorPose(Eigen::Affine3d& anchor_pose)
{
    anchor_pose = _world_T_anchor;
}

void OpenSoT::floating_base_estimation::kinematic_estimation::setAnchorPose(const Eigen::Affine3d& world_T_anchor)
{
    _world_T_anchor = world_T_anchor;
}

void OpenSoT::floating_base_estimation::kinematic_estimation::update(const bool update_model)
{
    _model->getPose(_base_link, _anchor_link, _anchor_T_base_link);

    _world_T_base_link = _world_T_anchor * _anchor_T_base_link;


    if(update_model)
    {
        _model->setFloatingBasePose(_world_T_base_link);
        _model->update();
    }
}

const Eigen::Affine3d& OpenSoT::floating_base_estimation::kinematic_estimation::getFloatingBasePose()
{
    return _world_T_base_link;
}

