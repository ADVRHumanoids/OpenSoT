#include <OpenSoT/utils/ForceOptimization.h>
#include <OpenSoT/constraints/force/StaticConstraint.h>



OpenSoT::utils::ForceOptimization::ForceOptimization(XBot::ModelInterface::Ptr model, 
                                           std::vector< std::string > contact_links,
                                           bool optimize_torque):
    _model(model),
    _contact_links(contact_links)
{
    /* Do we want to consider contact torques? */
    const bool optimize_contact_torque = optimize_torque;
    
    /* Define optimization vector by stacking all contact wrenches */
    OpenSoT::OptvarHelper::VariableVector vars;

    for(auto cl : _contact_links){
        vars.emplace_back(cl, optimize_contact_torque ? 6 : 3); // put 6 for full wrench
    }
    
    vars.emplace_back("tau", _model->getActuatedJointNum());

    OpenSoT::OptvarHelper opt(vars);
    
    /* Wrench bounds */
    Eigen::VectorXd wrench_ub(6), wrench_lb(6);
    wrench_ub << 1000, 1000, 1000, 50, 50, 50;
    wrench_lb << -1000, -1000, 0, -50, -50, -50;

    std::vector<OpenSoT::constraints::GenericConstraint::Ptr> wrench_bounds;
    std::list<OpenSoT::solvers::iHQP::TaskPtr> min_wrench_tasks;

    /* Define affine mappings for all wrenches */
    for(auto cl : _contact_links){

        _wrenches.emplace_back(opt.getVariable(cl) /
                               OpenSoT::AffineHelper::Zero(opt.getSize(), optimize_contact_torque ? 0 : 3)
                              );
        
        
        wrench_bounds.push_back( boost::make_shared<OpenSoT::constraints::GenericConstraint>(cl+"_bound",
                                                                                             _wrenches.back(),
                                                                                             wrench_ub,
                                                                                             wrench_lb,
                                                                                             OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT)
                               );
        
        auto min_wrench = boost::make_shared<OpenSoT::tasks::MinimizeVariable>("MIN_" + cl + "_WRENCH", 
            _wrenches.back()
        );
//         
        min_wrench_tasks.push_back(min_wrench);
        
    }
    
    /* Minimum effort task */
    auto min_tau = boost::make_shared<OpenSoT::tasks::MinimizeVariable>("MIN_TORQUE", 
            opt.getVariable("tau")
        );
    
    Eigen::MatrixXd min_tau_weight;
    min_tau_weight.setIdentity(min_tau->getTaskSize(), min_tau->getTaskSize());
    
    Eigen::VectorXd tau_max;
    _model->getEffortLimits(tau_max);
    min_tau_weight.diagonal() = tau_max.tail(_model->getActuatedJointNum());
    
    min_tau_weight = min_tau_weight * min_tau_weight;
    
    /* Static constraint */
    auto static_constr = boost::make_shared<OpenSoT::constraints::force::StaticConstraint>
                        (*_model,
                         _contact_links,
                         _wrenches,
                         opt.getVariable("tau"));
    
    /* Define friction cones */
    OpenSoT::constraints::force::FrictionCones::friction_cones friction_cones;

    for(auto cl : _contact_links)
    {
        friction_cones.emplace_back(Eigen::Matrix3d::Identity(), 0.5);
    }
    
    _friction_cone = boost::make_shared<OpenSoT::constraints::force::FrictionCones>(_contact_links,
                                                                                    _wrenches, 
                                                                                    *_model, 
                                                                                    friction_cones);
    
    
    /* Construct forza giusta task */
    _forza_giusta = boost::make_shared<OpenSoT::tasks::force::FloatingBase>(*_model, 
                                                                            _wrenches, 
                                                                            _contact_links);
    
    if(!_forza_giusta->checkConsistency())
    {
        throw std::runtime_error("porcodio");
    }
    
    /* Min wrench aggregated */
    auto min_force_aggr = boost::make_shared<OpenSoT::tasks::Aggregated>(min_wrench_tasks, opt.getSize());

    /* Define optimization problem */
    _autostack = _forza_giusta /
                    (min_tau_weight*min_tau);
                    
    _autostack << _friction_cone << static_constr;
    
    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_autostack->getStack(), 
                                                         _autostack->getBounds(),
                                                         1.0, 
                                                         OpenSoT::solvers::solver_back_ends::OSQP
                                                        );
    
    /* Initialize solution */
    _x_value.setZero(opt.getSize());
    
}

void OpenSoT::utils::ForceOptimization::setContactRotationMatrix(const std::string& contact_link, 
                                                                 const Eigen::Matrix3d& w_R_c)
{
    auto fc = _friction_cone->getFrictionCone(contact_link);
    
    if(!fc)
    {
        throw std::out_of_range("Contact link '" + contact_link + "' is undefined");
    }
        
    
    fc->setContactRotationMatrix(w_R_c);
}


bool OpenSoT::utils::ForceOptimization::compute(const Eigen::VectorXd& fixed_base_torque, 
                                      std::vector<Eigen::Vector6d>& Fc,
                                      Eigen::VectorXd& tau)
{
    Fc.resize(_contact_links.size());
    
    _forza_giusta->setFloatingBaseTorque(fixed_base_torque.head<6>());
    _autostack->update(_x_value);
    
    if(!_solver->solve(_x_value))
    {
        return false;
    }
    
    tau = fixed_base_torque;
    
    for(int i = 0; i < _contact_links.size(); i++)
    {
        
        _wrenches[i].getValue(_x_value, _Fci);
        Fc[i] = _Fci;
        
        _model->getJacobian(_contact_links[i], _JC);
        _fc_i.noalias() = _JC.transpose()*_Fci;
        tau -= _fc_i;
    }
    
    
    return true;
    
}

void OpenSoT::utils::ForceOptimization::log(XBot::MatLogger::Ptr logger)
{
    _autostack->log(logger);
}
