#include <OpenSoT/tasks/force/Force.h>
#include <boost/make_shared.hpp>

using namespace OpenSoT::tasks::force;

Wrench::Wrench(const std::string &contact_name, OpenSoT::AffineHelper wrench):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(contact_name+"_wrench",wrench.getInputSize()),
    _contact_name(contact_name)
{
    _min_var = boost::make_shared<OpenSoT::tasks::MinimizeVariable>(contact_name+"_wrench_internal", wrench);
    Eigen::VectorXd zeros; zeros.setZero(wrench.getOutputSize());
    _min_var->setReference(zeros);
}

void Wrench::_update(const Eigen::VectorXd &x)
{
    _min_var->update(x);

    _A = _min_var->getA();
    _b = _min_var->getb();
}

bool Wrench::setReference(const Eigen::VectorXd &ref)
{
    return _min_var->setReference(ref);
}

void Wrench::getReference(Eigen::VectorXd &ref)
{
    _min_var->getReference(_tmp);
    ref = _tmp;
}

const std::string& Wrench::getContactName()
{
    return _contact_name;
}
