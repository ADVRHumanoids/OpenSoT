#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>

using namespace OpenSoT::constraints::force;

WrenchLimits::WrenchLimits(const std::string& contact_name,
                           const Eigen::VectorXd& lowerLims,
                           const Eigen::VectorXd& upperLims,
                           OpenSoT::AffineHelper wrench) :
    Constraint(contact_name+"_wrench_limits", wrench.getInputSize()),
    _lowerLims(lowerLims),
    _upperLims(upperLims),
    _is_released(false)
{
    _constr_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>
            (contact_name + "_wrench_limits_internal",
             wrench,
             upperLims,
             lowerLims,
             OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);


   _zeros.setZero(lowerLims.size());

   this->generateBounds();
}

void WrenchLimits::getWrenchLimits(Eigen::VectorXd& lowerLims, Eigen::VectorXd& upperLims)
{
    lowerLims = _lowerLims;
    upperLims = _upperLims;
}

void WrenchLimits::setWrenchLimits(const Eigen::VectorXd& lowerLims, const Eigen::VectorXd& upperLims)
{
    _lowerLims = lowerLims;
    _upperLims = upperLims;

    _constr_internal->setBounds(_upperLims, _lowerLims);

    this->generateBounds();
}

void WrenchLimits::generateBounds()
{
    if(_constr_internal->getType() == OpenSoT::constraints::GenericConstraint::Type::BOUND)
    {
        _upperBound = _constr_internal->getUpperBound();
        _lowerBound = _constr_internal->getLowerBound();
    }
    else if(_constr_internal->getType() == OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT)
    {
        _Aineq = _constr_internal->getAineq();
        _bUpperBound = _constr_internal->getbUpperBound();
        _bLowerBound = _constr_internal->getbLowerBound();
    }

}

void WrenchLimits::releaseContact(bool released)
{
    if(released)
        _constr_internal->setBounds(_zeros, _zeros);
    else
        _constr_internal->setBounds(_upperLims, _lowerLims);
    _is_released = released;
    this->generateBounds();
}

bool WrenchLimits::isReleased()
{
    return _is_released;
}

WrenchesLimits::WrenchesLimits(const std::vector<std::string>& contact_name,
               const Eigen::VectorXd& lowerLims,
               const Eigen::VectorXd& upperLims,
               const std::vector<AffineHelper>& wrench):
    Constraint("wrenches_limits", wrench[0].getInputSize())
{
    std::list<ConstraintPtr> constraint_list;
    for(unsigned int i = 0; i < contact_name.size(); ++i){
        _wrench_lims_constraints[contact_name[i]] = std::make_shared<WrenchLimits>
                (contact_name[i], lowerLims, upperLims, wrench[i]);
        constraint_list.push_back(_wrench_lims_constraints[contact_name[i]]);
    }

    _aggregated_constraint = std::make_shared<OpenSoT::constraints::Aggregated>
            (constraint_list, wrench[0].getInputSize());

    update(Eigen::VectorXd(0));
}

WrenchesLimits::WrenchesLimits(const std::vector<std::string>& contact_name,
               const std::vector<Eigen::VectorXd>& lowerLims,
               const std::vector<Eigen::VectorXd>& upperLims,
               const std::vector<AffineHelper>& wrench):
    Constraint("wrenches_limits", wrench[0].getInputSize())
{
    std::list<ConstraintPtr> constraint_list;
    for(unsigned int i = 0; i < contact_name.size(); ++i){
        _wrench_lims_constraints[contact_name[i]] = std::make_shared<WrenchLimits>
                (contact_name[i], lowerLims[i], upperLims[i], wrench[i]);
        constraint_list.push_back(_wrench_lims_constraints[contact_name[i]]);
    }

    _aggregated_constraint = std::make_shared<OpenSoT::constraints::Aggregated>
            (constraint_list, wrench[0].getInputSize());

    update(Eigen::VectorXd(0));
}

WrenchesLimits::WrenchesLimits(const std::map<std::string, WrenchLimits::Ptr>& wrench_lims_constraints,
                               const std::vector<AffineHelper>& wrench):
    Constraint("wrenches_limits", wrench[0].getInputSize()),
    _wrench_lims_constraints(wrench_lims_constraints)
{
    std::list<ConstraintPtr> constraint_list;

    for (const auto& wrench : _wrench_lims_constraints) {
        constraint_list.push_back(wrench.second);
    }

    _aggregated_constraint = std::make_shared<OpenSoT::constraints::Aggregated>
            (constraint_list, wrench[0].getInputSize());

    update(Eigen::VectorXd(0));
}

WrenchLimits::Ptr WrenchesLimits::getWrenchLimits(const std::string& contact_name)
{
    if(_wrench_lims_constraints.count(contact_name))
        return _wrench_lims_constraints[contact_name];
    else
        return NULL;
}

void WrenchesLimits::update(const Eigen::VectorXd &x)
{
    _aggregated_constraint->update(x);
    generateBounds();
}

void WrenchesLimits::generateBounds()
{
    if(_aggregated_constraint->isInequalityConstraint())
    {
        _Aineq = _aggregated_constraint->getAineq();
        _bUpperBound = _aggregated_constraint->getbUpperBound();
        _bLowerBound = _aggregated_constraint->getbLowerBound();
    }
    else if(_aggregated_constraint->isBound())
    {
        _upperBound = _aggregated_constraint->getUpperBound();
        _lowerBound = _aggregated_constraint->getLowerBound();
    }
}

