#include "../include/soth/HCOD_wrapper.h"
#include "../include/soth/HCOD.hpp"
#include "../external/Eigen/Dense"

using namespace soth;

HCOD_wrapper::HCOD_wrapper(const unsigned int sizeProblem, const unsigned int nbStage):
    _problem_size(sizeProblem)
{
    _hcod = std::make_shared<HCOD>(sizeProblem, nbStage);
}

void HCOD_wrapper::pushBackStage(const unsigned int nr, const double *Jdata, const Bound *bdata)
{
    _hcod->pushBackStage(nr, Jdata, bdata);
}

void HCOD_wrapper::setNameByOrder(const std::string root)
{
    _hcod->setNameByOrder(root);
}

void HCOD_wrapper::setDamping(const double &d)
{
    _hcod->setDamping(d);
}

void HCOD_wrapper::activeSearch(double *u)
{
    Eigen::VectorXd solution(_problem_size);

    _hcod->activeSearch(solution);

    Eigen::VectorXd::Map(u, solution.size()) = solution;
}

void HCOD_wrapper::setInitialActiveSet()
{
    _hcod->setInitialActiveSet();
}

bool HCOD_wrapper::updateStage(const unsigned int i, const double* Jdata, const Bound* bdata)
{
    if(i >= _hcod->stages.size())
        return false;

    _hcod->stages[i]->set(Jdata, bdata);

    return true;
}

HCOD_wrapper::~HCOD_wrapper()
{

}
