#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <OpenSoT/utils/Affine.h>
#include <boost/make_shared.hpp>
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
    _constr_internal = boost::make_shared<OpenSoT::constraints::GenericConstraint>
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
