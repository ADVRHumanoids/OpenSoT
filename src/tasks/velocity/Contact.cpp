#include <OpenSoT/tasks/velocity/Contact.h>

OpenSoT::tasks::velocity::Contact::Contact(std::string task_id,
                                        const XBot::ModelInterface& model,
                                        std::string link_name,
                                        const Eigen::MatrixXd& contact_matrix
                                    ):
    Task<Eigen::MatrixXd, Eigen::VectorXd>(task_id, model.getJointNum()),
    _distal_link(link_name),
    _model(model),
    _K(contact_matrix)
{
    _update(Eigen::VectorXd(0));
    _W.setIdentity(getTaskSize(), getTaskSize());
}


void OpenSoT::tasks::velocity::Contact::_update(const Eigen::VectorXd& x)
{
    /* Body jacobian */
    _model.getJacobian(_distal_link, _distal_link, _Jtmp);

    /* Update task A matrix */
    _A = _K * _Jtmp;

//     std::cout << "_A = _K * _Jtmp\n" << _A.format(Eigen::IOFormat(1)) << std::endl;

    /* Update task b vector */
    _b.setZero(getTaskSize());
}

bool OpenSoT::tasks::velocity::Contact::baseLinkIsWorld() const
{
    return true;
}

const Eigen::VectorXd& OpenSoT::tasks::velocity::Contact::getError() const
{
    return _error;
}

const std::string& OpenSoT::tasks::velocity::Contact::getLinkName() const
{
    return _distal_link;
}

OpenSoT::tasks::velocity::Contact::~Contact()
{

}

void OpenSoT::tasks::velocity::Contact::update_b()
{

}
