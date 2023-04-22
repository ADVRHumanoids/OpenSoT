#include <OpenSoT/tasks/force/Force.h>

using namespace OpenSoT::tasks::force;

Wrench::Wrench(const std::string& id,
               const std::string& distal_link, const std::string& base_link,
               const AffineHelper& wrench):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(id,wrench.getInputSize()),
    _distal_link(distal_link),
    _base_link(base_link)
{
    _min_var = std::make_shared<OpenSoT::tasks::MinimizeVariable>(distal_link+"_wrench_internal", wrench);
    _W = _min_var->getWeight();
    Eigen::VectorXd zeros; zeros.setZero(wrench.getOutputSize());
    _min_var->setReference(zeros);
    _hessianType = OpenSoT::HessianType::HST_SEMIDEF;
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

const std::string& Wrench::getDistalLink() const
{
    return _distal_link;
}

const std::string& Wrench::getBaseLink() const
{
    return _base_link;
}

// ---- //

Wrenches::Wrenches(const std::string& id,
                   const std::vector<std::string>& distal_links,
                   const std::vector<std::string>& base_links,
                   const std::vector<AffineHelper>& wrenches):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(id, wrenches[0].getInputSize())
{
    if(distal_links.size() != base_links.size())
        throw std::runtime_error("distal_links.size() != base_links.size()");
    if(distal_links.size() != wrenches.size())
        throw std::runtime_error("distal_links.size() != wrenches.size()");

    std::list<TaskPtr> task_list;
    for(unsigned int i = 0; i < distal_links.size(); ++i){
        wrench_tasks[distal_links[i]] =
                std::make_shared<Wrench>(id+"_"+distal_links[i],
                    distal_links[i],base_links[i], wrenches[i]);
        task_list.push_back(wrench_tasks[distal_links[i]]);}
    _aggregated_task = std::make_shared<OpenSoT::tasks::Aggregated>
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
