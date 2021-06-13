#include <OpenSoT/solvers/HCOD.h>

using namespace OpenSoT::solvers;

HCOD::HCOD(OpenSoT::AutoStack &stack_of_tasks, const double damping):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds())
{
    init(damping);
}

HCOD::HCOD(Stack& stack_of_tasks, ConstraintPtr bounds, const double damping):
    Solver(stack_of_tasks, bounds)
{
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
    _hcod = std::make_shared<soth::HCOD>(_VARS, _CL+TL);
    _vector_bounds.resize(_CL+TL);
    _vector_J.resize(_CL+TL);

    //creates bounds and tasks (here order is important!)
    if(_CL > 0)
        copy_bounds();
    copy_tasks();

    // Pushback stages
    for (unsigned int i = 0; i < _vector_J.size(); ++i)
    {
        _hcod->pushBackStage(_vector_J[i], _vector_bounds[i]);
        _hcod->setNameByOrder("level_");
    }

    // Set damping to all levels and initial active set
    _hcod->setDamping(damping);
    _hcod->setInitialActiveSet();
}

bool HCOD::solve(Eigen::VectorXd &solution)
{
    if(_CL > 0)
        copy_bounds();

    copy_tasks();

    _hcod->activeSearch(solution);
    return true; ///TODO: HOW TO CHECK IF EVERYTHING WENT FINE???
}

void HCOD::copy_tasks()
{
    int s = _tasks.size();
    int c = 0;
    if(_CL > 0)
        c+=1;

    for(unsigned int i = 0; i < s; ++i)
    {
        _vector_J[c] = _tasks[i]->getA();

        int ss = _tasks[i]->getb().size();
        if(_vector_bounds[c].size() != ss)
            _vector_bounds[c].resize(ss);

        for(unsigned int j = 0; j < ss; ++j)
            _vector_bounds[c][j] = _tasks[i]->getb()[j];
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
        _vector_bounds[0][i] = soth::Bound(_lA.generate_and_get()(i,0), _uA.generate_and_get()(i,0));
}
