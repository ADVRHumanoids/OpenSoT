#include <OpenSoT/constraints/force/StaticConstraint.h>


OpenSoT::constraints::force::StaticConstraint::StaticConstraint(const XBot::ModelInterface& robot,
                                                              std::vector<std::string> contact_links,
                                                              std::vector<AffineHelper> forces,
                                                              OpenSoT::AffineHelper robot_torque):
    Constraint("StaticConstraint", forces[0].getInputSize()),
    _robot(robot),
    _contact_links(contact_links),
    _forces(forces),
    _robot_torque(robot_torque)
{
    update(Eigen::VectorXd());
}

void OpenSoT::constraints::force::StaticConstraint::update(const Eigen::VectorXd& x)
{
    _constr.setZero(getXSize(), _robot.getActuatedNv());
    
    for(int i = 0; i < _contact_links.size(); i++)
    {
        _robot.getJacobian(_contact_links.at(i), _J);
        _constr = _constr + _J.transpose().bottomRows(_robot.getActuatedNv()) * _forces[i];
    }
    
    _constr = _constr + _robot_torque;
    
    _robot.computeGravityCompensation(_gcomp);
    
    _Aeq = _constr.getM();
    _beq = _gcomp.tail(_robot.getActuatedNv()) - _constr.getq();
    
}
