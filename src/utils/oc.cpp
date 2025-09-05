#include <OpenSoT/utils/oc.h>

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
    for(unsigned int Ns = 0; Ns < _stages.size(); ++Ns)
    {
        if(_stages[Ns]->isFinalStage())
            return Ns;
    }
    return 0;
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
        cost += _stages[i]->cost();
    return cost;
}

double ocp::cost(const unsigned int i)
{
    return _stages[i]->cost();
}

double ocp::der(const unsigned int i, const Eigen::MatrixXd& dx, const Eigen::MatrixXd& du)
{
    return _stages[i]->der(dx, du);
}

