#include <wb_sot/solvers/QPOases.h>
#include <yarp/math/Math.h>

using namespace yarp::math;

#define INITIAL_NWSR 127

using namespace wb_sot::solvers;

QPOasesProblem::QPOasesProblem():
    _problem(),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(),
    _constraints(),
    _nWSR(INITIAL_NWSR),
    _solution(0), _dual_solution(0),
    _is_initialized(false),
    _initial_guess(false)
{

}

QPOasesProblem::QPOasesProblem(const int number_of_variables,
                               const int number_of_constraints,
                               qpOASES::HessianType hessian_type):
    _problem(number_of_variables, number_of_constraints, hessian_type),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(),
    _constraints(),
    _nWSR(127),
    _solution(number_of_variables), _dual_solution(number_of_variables),
    _is_initialized(false),
    _initial_guess(false)
{
    qpOASES::Options opt;
    opt.printLevel = qpOASES::PL_HIGH;
    opt.setToReliable();
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= 2E2;
    _problem.setOptions(opt);
}

void QPOasesProblem::addProblem(const qpOASES::QProblem &problem)
{
    _problem = problem;
}

void QPOasesProblem::setOptions(const qpOASES::Options &options)
{
    _problem.setOptions(options);
}

void QPOasesProblem::initProblem(const Matrix &H, const Vector &g,
                                 const Matrix &A,
                                 const Vector &lA, const Vector &uA,
                                 const Vector &l, const Vector &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
    _is_initialized = true;
}

bool QPOasesProblem::updateTask(const Matrix &H, const Vector &g)
{
    if( (H.rows() == _H.rows() && H.cols() == _H.cols()) &&
         g.size() == _g.size() && _is_initialized)
    {
        _H = H;
        _g = g;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if( (A.rows() == _A.rows() && A.cols() == _A.cols()) &&
         lA.size() == _lA.size() &&
         uA.size() == _uA.size() && _is_initialized)
    {
        _A = A;
        _lA = lA;
        _uA = uA;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateBounds(const Vector &l, const Vector &u)
{
    if( l.size() == _l.size() && u.size() == _u.size() && _is_initialized)
    {
        _l = l;
        _u = u;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateProblem(const Matrix &H, const Vector &g,
                                   const Matrix &A, const Vector &lA, const Vector &uA,
                                   const Vector &l, const Vector &u)
{
    return updateTask(H, g) && updateConstraints(A, lA, uA) && updateBounds(l, u);
}

bool QPOasesProblem::addTask(const Matrix &H, const Vector &g)
{
    if(_is_initialized)
    {
        _H = pile(_H, H);
        _g = cat(_g, g);
        _initial_guess = false;
        return true;
    }
    return false;
}

bool QPOasesProblem::addConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if(_is_initialized)
    {
        _A = pile(_A, A);
        _lA = cat(_lA, lA);
        _uA = cat(_uA, uA);
        _initial_guess = false;
        return true;
    }
    return false;
}

bool QPOasesProblem::addBounds(const Vector &l, const Vector &u)
{
    if(_is_initialized)
    {
        _l = cat(_l, l);
        _u = cat(_u, u);
        _initial_guess = false;
        return true;
    }
    return false;
}

bool QPOasesProblem::addProblem(const Matrix &H, const Vector &g,
                                const Matrix &A, const Vector &lA, const Vector &uA,
                                const Vector &l, const Vector &u)
{
    return addTask(H, g) && addConstraints(A, lA, uA) && addBounds(l, u);
}

bool QPOasesProblem::solve()
{
    // To solve the problem it has to be initialized
    if(_is_initialized)
    {
        // If initial guess true we use part of the previous solution..
        if(_initial_guess)
        {
            _problem.init( _H.data(), _g.data(),
                           _A.data(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           _nWSR,0,
                           _solution.data(), _dual_solution.data(),
                           &_bounds, &_constraints);
        }
        else
        {
            _problem.init( _H.data(),_g.data(),
                           _A.data(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           _nWSR,0);
        }

        // If solution has changed of size we update the size
        if(_solution.size() != _problem.getNV())
        {
            _solution.resize(_problem.getNV());
            _initial_guess = false;
        }
        if(_dual_solution.size() != _problem.getNV() + _problem.getNC())
        {
            _dual_solution.resize(_problem.getNV()+ _problem.getNC());
            _initial_guess = false;
        }

        //We get the solution
        int success = _problem.getPrimalSolution(_solution.data());
        _problem.getDualSolution(_dual_solution.data());
        _problem.getBounds(_bounds);
        _problem.getConstraints(_constraints);

        if(success == qpOASES::RET_QP_NOT_SOLVED ||
          (success != qpOASES::RET_QP_SOLVED &&
           success != qpOASES::SUCCESSFUL_RETURN))
        {
            std::cout<<"ERROR OPTIMIZING TASK! ERROR "<<success<<std::endl;
            _initial_guess = false;
            return _initial_guess;
        }
        else
        {
            return true;
        }
    }
    return false;
}
