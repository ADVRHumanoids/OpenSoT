#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>
#include <xbot2_interface/logger.h>
using XBot::Logger;

OpenSoT::tasks::acceleration::DynamicFeasibility::DynamicFeasibility(const std::string task_id,
                                                                     const XBot::ModelInterface& robot,
                                                                     const OpenSoT::AffineHelper& qddot,
                                                                     const std::vector< OpenSoT::AffineHelper >& wrenches,
                                                                     const std::vector< std::string >& contact_links):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _qddot(qddot),
    _wrenches(wrenches),
    _contact_links(contact_links)
{
    _enabled_contacts.assign(_contact_links.size(), true);
    _W.setIdentity(6,6);
    _hessianType = OpenSoT::HessianType::HST_SEMIDEF;
    update();
}

void OpenSoT::tasks::acceleration::DynamicFeasibility::_update()
{
    _robot.computeInertiaMatrix(_B);
    _robot.computeNonlinearTerm(_h);
    _Bu = _B.topRows(6);
    _hu = _h.topRows(6);

    _dyn_constraint = _Bu*_qddot + _hu;

    for(int i = 0; i < _enabled_contacts.size(); i++)
    {
        if(!_enabled_contacts[i]){
            continue;
        }
        else {
            _robot.getJacobian(_contact_links[i], _Jtmp);
            _Jf = _Jtmp.block<6,6>(0,0).transpose();
            _dyn_constraint = _dyn_constraint + (-_Jf) * _wrenches[i];
        }
    }

    _A = _dyn_constraint.getM();
    _b = -_dyn_constraint.getq();

}

bool OpenSoT::tasks::acceleration::DynamicFeasibility::enableContact(const std::string& contact_link)
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

bool OpenSoT::tasks::acceleration::DynamicFeasibility::disableContact(const std::string& contact_link)
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

const std::vector<bool>& OpenSoT::tasks::acceleration::DynamicFeasibility::getEnabledContacts() const
{
    return _enabled_contacts;
}

Eigen::VectorXd OpenSoT::tasks::acceleration::DynamicFeasibility::checkTask(const Eigen::VectorXd& x)
{
    Eigen::VectorXd value;
    _dyn_constraint.getValue(x, value);
    return value;
}
