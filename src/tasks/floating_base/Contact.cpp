#include <OpenSoT/tasks/floating_base/Contact.h>

OpenSoT::tasks::floating_base::Contact::Contact(XBot::ModelInterface &robot,
                                        const std::string link_in_contact,
                                        const Eigen::MatrixXd contact_matrix):
    Task("Postural", 6),_robot(robot),_link_in_contact(link_in_contact),
    _contact_matrix(contact_matrix)
{
    _hessianType = HST_SEMIDEF;

    _W.setIdentity(6,6);

    _J.setZero(6,_robot.getJointNum());
    _A.setZero(6,6);

    _dqm.setZero(_robot.getJointNum());

    _update(Eigen::VectorXd::Zero(robot.getJointNum()));
}

OpenSoT::tasks::floating_base::Contact::~Contact()
{

}

void OpenSoT::tasks::floating_base::Contact::_update(const Eigen::VectorXd &x)
{
    _robot.getJacobian(_link_in_contact, _link_in_contact, _J);
    _Jcontact = _contact_matrix*_J;
    _robot.getJointVelocity(_dqm);

    _A = _Jcontact.block(0,0,6,6);
    _b = -_Jcontact.block(0,6,6,_dqm.size()-6)*_dqm.segment(6,_dqm.size()-6);
}

const std::string& OpenSoT::tasks::floating_base::Contact::getLinkInContact() const
{
    return _link_in_contact;
}

void OpenSoT::tasks::floating_base::Contact::setLinkInContact(const std::string link_in_contact)
{
    _link_in_contact = link_in_contact;
}
