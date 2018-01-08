#include <OpenSoT/constraints/GenericConstraint.h>

OpenSoT::constraints::GenericConstraint::GenericConstraint(std::string task_id, 
                                                           const OpenSoT::AffineHelper& variable, 
                                                           const Eigen::VectorXd& upper_bound, 
                                                           const Eigen::VectorXd& lower_bound,
                                                           const Type constraint_type):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >(task_id, variable.getInputSize()),
    _ub(upper_bound),
    _lb(lower_bound),
    _var(variable),
    _type(constraint_type)
{
    if(!setBounds(upper_bound, lower_bound)){
        throw std::invalid_argument("Bounds not valid");
    }
}

bool OpenSoT::constraints::GenericConstraint::setBounds(const Eigen::VectorXd& upper_bound, 
                                                        const Eigen::VectorXd& lower_bound)
{
    if( ((upper_bound - lower_bound).array() < 0).any() ){
        return false;
    }
    
    if( (upper_bound.size() != _var.getOutputSize()) || (lower_bound.size() != _var.getOutputSize()) ){
        return false;
    }
    
    _ub = upper_bound;
    _lb = lower_bound;
    
    if(_type == Type::CONSTRAINT)
    {
        _Aineq = _var.getM();
        _bUpperBound = _ub - _var.getq();
        _bLowerBound = _lb - _var.getq();
    }
    else if(_type == Type::BOUND)
    {
        _upperBound = _ub - _var.getq();
        _lowerBound = _lb - _var.getq();
    }
    else
        return false;
    
    return true;
}


void OpenSoT::constraints::GenericConstraint::update(const Eigen::VectorXd& x)
{

}
