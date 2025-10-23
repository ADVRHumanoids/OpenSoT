#include <OpenSoT/oc/oc.h>

using namespace OpenSoT;

ocp::ocp()
{

}

void ocp::addStage(Stage::Ptr stage)
{
    _stages.push_back(stage);
}

unsigned int ocp::getNumberOfNodes()
{
    //for(unsigned int Ns = 0; Ns < _stages.size(); ++Ns)
    //{
    //    if(_stages[Ns]->isFinalStage())
    //        return Ns;
    //}
    return _stages.size();
}

void ocp::update(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    if(x0.size() != (u0.size() + 1))
        throw std::runtime_error("x0.size() != (u0.size() + 1)");

    for(unsigned int i = 0; i < u0.size(); ++i)
    {
        _stages[i]->update(x0[i], u0[i]);
    }
    _stages[_stages.size()-1]->update(x0[_stages.size()-1], Eigen::VectorXd(0));
}


double ocp::cost()
{
    double cost = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        cost += _stages[i]->stage_cost();
    return cost;
}

double ocp::dynamics_defect()
{
    double defect = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        defect += _stages[i]->stage_dynamics_defect();
    return defect;
}

double ocp::constraint_violation()
{
    double violation = 0.;
    for(unsigned int i = 0; i < _stages.size(); ++i)
        violation += _stages[i]->stage_constraint_violation();
    return violation;

}