#include <OpenSoT/constraints/acceleration/TorqueLimits.h>

using namespace OpenSoT::constraints::acceleration;

TorqueLimits::TorqueLimits(const XBot::ModelInterface &robot,
                           const AffineHelper &qddot,
                           const std::vector<AffineHelper> &wrenches,
                           const std::vector<std::__cxx11::string> &contact_links,
                           const Eigen::VectorXd &torque_limits):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >("torque_limits", qddot.getInputSize()),
    _robot(robot),
    _qddot(qddot),
    _wrenches(wrenches),
    _contact_links(contact_links),
    _torque_limits(torque_limits)
{
    if(_qddot.getOutputSize() != _torque_limits.size())
        throw std::runtime_error("_qddot.getOutputSize() != _torque_limits.size()");

    _enabled_contacts.assign(_contact_links.size(), true);

    update(Eigen::VectorXd(0));
}

void TorqueLimits::update(const Eigen::VectorXd &x)
{
    _robot.getInertiaMatrix(_B);
    _robot.computeNonlinearTerm(_h);

    _dyn_constraint = _B*_qddot;

    for(int i = 0; i < _enabled_contacts.size(); i++)
    {
        if(!_enabled_contacts[i]){
            continue;
        }
        else {
            _robot.getJacobian(_contact_links[i], _Jtmp);
            _dyn_constraint = _dyn_constraint + (-_Jtmp.transpose()) * _wrenches[i];
        }
    }

    _Aeq = _dyn_constraint.getM();
    _bLowerBound = -_torque_limits - _h;
    _bUpperBound = _torque_limits - _h;
}

bool TorqueLimits::enableContact(const std::string& contact_link)
{
    std::vector<std::string>::iterator itr = std::find(_contact_links.begin(), _contact_links.end(), contact_link);
    if(itr != _contact_links.cend())
    {
        unsigned int i = std::distance(_contact_links.begin(), itr);
        _enabled_contacts[i] = true;
        return true;
    }
    return false;
}

bool TorqueLimits::disableContact(const std::string& contact_link)
{
    std::vector<std::string>::iterator itr = std::find(_contact_links.begin(), _contact_links.end(), contact_link);
    if(itr != _contact_links.cend())
    {
        unsigned int i = std::distance(_contact_links.begin(), itr);
        _enabled_contacts[i] = false;
        return true;
    }
    return false;
}

const std::vector<bool>& TorqueLimits::getEnabledContacts() const
{
    return _enabled_contacts;
}

