#include <OpenSoT/tasks/force/Force.h>

using namespace OpenSoT::tasks::force;

Wrench::Wrench(const std::string &contact_name, OpenSoT::AffineHelper &wrench):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(contact_name+"_wrench",wrench.getInputSize()),
    _contact_name(contact_name)
{
    _min_var = boost::make_shared<OpenSoT::tasks::MinimizeVariable>(contact_name+"_wrench_internal", wrench);
    _W = _min_var->getWeight();
    Eigen::VectorXd zeros; zeros.setZero(wrench.getOutputSize());
    _min_var->setReference(zeros);
    _update(Eigen::VectorXd(0));
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

Wrenches::Wrenches(const std::vector<std::string>& contact_name,
         std::vector<AffineHelper> wrenches):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("wrenches",wrenches[0].getInputSize())
{
    std::list<TaskPtr> task_list;
    for(unsigned int i = 0; i < contact_name.size(); ++i){
        wrench_tasks[contact_name[i]] = boost::make_shared<Wrench>(contact_name[i], wrenches[i]);
        task_list.push_back(wrench_tasks[contact_name[i]]);}
    _aggregated_task = boost::make_shared<OpenSoT::tasks::Aggregated>
            (task_list, wrenches[0].getInputSize());

    _update(Eigen::VectorXd(0));
}

Wrench::Ptr Wrenches::getWrenchTask(const std::string& contact_name)
{
    if(wrench_tasks.count(contact_name))
        return wrench_tasks[contact_name];
    else
        return NULL;
}

void Wrenches::_update(const Eigen::VectorXd& x)
{
    _aggregated_task->update(x);
    _A = _aggregated_task->getA();
    _b = _aggregated_task->getb();
}
