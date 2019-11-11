#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

OpenSoT::constraints::acceleration::DynamicFeasibility::DynamicFeasibility(const std::string constraint_id, 
                                                                           const XBot::ModelInterface& robot, 
                                                                           const OpenSoT::AffineHelper& qddot,
                                                                           const std::vector< OpenSoT::AffineHelper >& wrenches, 
                                                                           const std::vector< std::string >& contact_links): 
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >(constraint_id, qddot.getInputSize()), 
    _robot(robot),
    _qddot(qddot),
    _wrenches(wrenches),
    _contact_links(contact_links)
{
    _enabled_contacts.assign(_contact_links.size(), true);
    update(_h);
}



void OpenSoT::constraints::acceleration::DynamicFeasibility::update(const Eigen::VectorXd& x)
{
    _robot.getInertiaMatrix(_B);
//    _robot.computeNonlinearTerm(_h);

    _B.setZero();
    _robot.computeGravityCompensation(_h);


    _Bu = _B.topRows(3);
    _hu = _h.topRows(3);

    _dyn_constraint = _Bu*_qddot + _hu;
    
    for(int i = 0; i < _enabled_contacts.size(); i++)
    {
        if(!_enabled_contacts[i]){
            continue;
        }
        else {
            _robot.getJacobian(_contact_links[i], _Jtmp);
            _Jf = _Jtmp.block<6,3>(0,0).transpose();
            _dyn_constraint = _dyn_constraint + (-_Jf) * _wrenches[i];
        }
    }
    
    _Aeq = _dyn_constraint.getM();
    _beq = -_dyn_constraint.getq();
    
}

Eigen::VectorXd OpenSoT::constraints::acceleration::DynamicFeasibility::checkConstraint(const Eigen::VectorXd& x)
{
    Eigen::VectorXd value;
    _dyn_constraint.getValue(x, value);
    return value;
}
