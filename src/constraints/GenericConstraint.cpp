#include <OpenSoT/constraints/GenericConstraint.h>

OpenSoT::constraints::GenericConstraint::GenericConstraint(std::string constraint_id,
                                                           const OpenSoT::AffineHelper& variable, 
                                                           const Eigen::VectorXd& upper_bound, 
                                                           const Eigen::VectorXd& lower_bound,
                                                           const Type constraint_type):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >(constraint_id, variable.getInputSize()),
    _ub(upper_bound),
    _lb(lower_bound),
    _var(variable),
    _type(constraint_type)
{
    if(!setBounds(upper_bound, lower_bound)){
        throw std::invalid_argument("Bounds not valid");
    }
}

OpenSoT::constraints::GenericConstraint::GenericConstraint(std::string constraint_id,
                                                          const Eigen::VectorXd& upper_bound,
                                                          const Eigen::VectorXd& lower_bound,
                                                          const int x_size):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >(constraint_id, x_size),
    _ub(upper_bound),
    _lb(lower_bound)
{
    _var = _var.Identity(x_size);
    _type = Type::BOUND;

    if(!setBounds(upper_bound, lower_bound)){
        throw std::invalid_argument("Bounds not valid");
    }
}

bool OpenSoT::constraints::GenericConstraint::setBounds(const Eigen::VectorXd& upper_bound, 
                                                        const Eigen::VectorXd& lower_bound)
{
    if( ((upper_bound - lower_bound).array() < 0).any() ){
        XBot::Logger::error("%s: ((upper_bound - lower_bound).array() < 0).any() in setBounds! \n", getConstraintID().c_str());
        return false;
    }
    
    if( (upper_bound.size() != _var.getOutputSize()) || (lower_bound.size() != _var.getOutputSize()) ){
        XBot::Logger::error("%s: (upper_bound.size() != _var.getOutputSize()) || (lower_bound.size() != _var.getOutputSize()) in setBounds! \n",
                            getConstraintID().c_str());
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
    {
        XBot::Logger::error("Type not defined in setBounds!");
        return false;}
    
    return true;
}

bool OpenSoT::constraints::GenericConstraint::setConstraint(const AffineHelper& var,
                   const Eigen::VectorXd& upper_bound,
                   const Eigen::VectorXd& lower_bound)
{
    _var = var;
    return setBounds(upper_bound, lower_bound);
}


void OpenSoT::constraints::GenericConstraint::update()
{

}
