#include <OpenSoT/solvers/HCOD.h>
#include <soth/HCOD_wrapper.h>
#include <soth/Bound.hpp>

using namespace OpenSoT::solvers;

#define DEFAULT_DISABLE_WEIGHTS_COMPUTATION false

HCOD::HCOD(OpenSoT::AutoStack &stack_of_tasks, const double damping):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds()),
    _W(stack_of_tasks.getStack().size()),
    _sqrt(stack_of_tasks.getStack().size()),
    _Wb(stack_of_tasks.getStack().size()),
    _disable_weights_computation(DEFAULT_DISABLE_WEIGHTS_COMPUTATION)
{
    std::list<ConstraintPtr> constraints_in_tasks_list;
    for(auto& task : _tasks)
    {
        for(auto& constraint : task->getConstraints())
            constraints_in_tasks_list.push_back(constraint);
    }

    if(constraints_in_tasks_list.size() > 0)
    {
        constraints_in_tasks_list.push_back(stack_of_tasks.getBounds());
        _bounds = std::make_shared<constraints::Aggregated>(constraints_in_tasks_list, _tasks[0]->getA().cols());
    }

    init(damping);
}

HCOD::HCOD(Stack& stack_of_tasks, ConstraintPtr bounds, const double damping):
    Solver(stack_of_tasks, bounds),
    _W(stack_of_tasks.size()),
    _sqrt(stack_of_tasks.size()),
    _Wb(stack_of_tasks.size()),
    _disable_weights_computation(DEFAULT_DISABLE_WEIGHTS_COMPUTATION)
{
    std::list<ConstraintPtr> constraints_in_tasks_list;
    for(auto& task : _tasks)
    {
        for(auto& constraint : task->getConstraints())
            constraints_in_tasks_list.push_back(constraint);
    }

    if(constraints_in_tasks_list.size() > 0)
    {
        constraints_in_tasks_list.push_back(bounds);
        _bounds = std::make_shared<constraints::Aggregated>(constraints_in_tasks_list, _tasks[0]->getA().cols());
    }


    init(damping);
}

void HCOD::init(const double damping)
{
    _VARS = _tasks[0]->getA().cols();

    _A.set(Eigen::MatrixXd(0, _VARS));
    _lA.set(Eigen::MatrixXd(0,1));
    _uA.set(Eigen::MatrixXd(0,1));


    // Level of priorities is given by Constraints + Tasks Levels
    _CL = 0;
    if(_bounds)
        _CL = 1;

    int TL = _tasks.size();
    _I.setIdentity(_VARS, _VARS);

    // here we create the hcod solver
    _hcod = std::make_shared<soth::HCOD_wrapper>(_VARS, _CL+TL);
    _vector_bounds.resize(_CL+TL);
    _vector_J.resize(_CL+TL);

    //creates bounds and tasks (here order is important!)
    if(_CL > 0)
        copy_bounds();
    copy_tasks();

    // Pushback stages
    for (unsigned int i = 0; i < _vector_J.size(); ++i)
    {
        _hcod->pushBackStage(_vector_J[i].rows(), _vector_J[i].data(), _vector_bounds[i].data());
        _hcod->setNameByOrder("level_" + i);
    }

    // Set damping to all levels and initial active set
    _hcod->setDamping(damping);
    _hcod->setInitialActiveSet();

    printSOT();
}

void HCOD::printSOT()
{
    XBot::Logger::info("HCOD:\n");
    XBot::Logger::info("    TOTAL STAGES: %i\n", _CL+_tasks.size());
    XBot::Logger::info("    STACK STAGES: %i\n", _tasks.size());
    XBot::Logger::info("    CONSTRAINTS: %i\n", _A.rows());
    for(unsigned int i = _CL; i < _vector_J.size(); ++i)
        XBot::Logger::info("    TOTAL TASKS STAGE %i: %i\n", i, _vector_J[i].rows());
}

bool HCOD::solve(Eigen::VectorXd &solution)
{
    if(_CL > 0)
        copy_bounds();

    copy_tasks();

    solution.setZero(_VARS);
    try
    {
        _hcod->activeSearch(solution.data());
    }
    catch(int)
    {
        return false;
    }
    return true;
}

void HCOD::copy_tasks()
{
    int s = _tasks.size();
    int c = 0;
    if(_CL > 0)
        c+=1;

    for(unsigned int i = 0; i < s; ++i)
    {
        if(_disable_weights_computation)
        {
            _vector_J[c] = _tasks[i]->getA();

            int ss = _tasks[i]->getb().size();
            if(_vector_bounds[c].size() != ss)
                _vector_bounds[c].resize(ss);

            for(unsigned int j = 0; j < ss; ++j)
                _vector_bounds[c][j] = _tasks[i]->getb()[j];
        }
        else
        {
            _W[i] = _tasks[i]->getWeight();
            if(_tasks[i]->getWeightIsDiagonalFlag()) //weight matrix is diagonal
            {
                for(unsigned int j = 0; i < _W[j].rows(); ++j)
                    _W[i](j,j) = std::sqrt(_tasks[i]->getWeight()(j,j));
            }
            else //if not diagonal we assume weight matrix positive-definite symmetric
            {
                _sqrt[i].compute(_tasks[i]->getWeight());
                _W[i] = _sqrt[i].operatorSqrt();
            }

            _vector_J[c] = _W[i]*_tasks[i]->getA();

            int ss = _tasks[i]->getb().size();
            if(_vector_bounds[c].size() != ss)
                _vector_bounds[c].resize(ss);

            _Wb[i] = _W[i]*_tasks[i]->getb();
            for(unsigned int j = 0; j < ss; ++j)
                _vector_bounds[c][j] = _Wb[i][j];
        }

        c+=1;
    }
}

void HCOD::copy_bounds()
{
    _A.reset();
    _lA.reset();
    _uA.reset();

    if(_bounds->getLowerBound().size() > 0)
    {
        _A.pile(_I);
        _lA.pile(_bounds->getLowerBound());
        _uA.pile(_bounds->getUpperBound());
    }

    if(_bounds->getAineq().rows() > 0)
    {
        _A.pile(_bounds->getAineq());
        _lA.pile(_bounds->getbLowerBound());
        _uA.pile(_bounds->getbUpperBound());
    }

    if(_bounds->getAeq().rows() > 0)
    {
        _A.pile(_bounds->getAeq());
        _lA.pile(_bounds->getbeq());
        _uA.pile(_bounds->getbeq());
    }


    _vector_J[0] = _A.generate_and_get();

    int s = _lA.generate_and_get().size();

    if(_vector_bounds[0].size() != s)
        _vector_bounds[0].resize(s);

    for(unsigned int i = 0; i < s; ++i)
    {
        double lAi = _lA.generate_and_get()(i,0);
        double uAi = _uA.generate_and_get()(i,0);
        if(std::fabs(lAi-uAi) <= std::numeric_limits<double>::epsilon())
            _vector_bounds[0][i] = soth::Bound(lAi);
        else
            _vector_bounds[0][i] = soth::Bound(lAi, uAi);
    }
}

void HCOD::setDisableWeightsComputation(const bool disable)
{
    _disable_weights_computation = disable;
}

bool HCOD::getDisableWeightsComputation()
{
    return _disable_weights_computation;
}

void HCOD::setDamping(double damping)
{
    _hcod->setDamping(damping);
}



HCOD::~HCOD()
{

}
