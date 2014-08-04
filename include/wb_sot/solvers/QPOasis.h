#ifndef _WB_SOT_SOLVERS_QP_OASES_H_
#define _WB_SOT_SOLVERS_QP_OASES_H_

#include <qpOASES.hpp>
#include <wb_sot/Solver.h>
#include <vector>
#include <iostream>
#include <yarp/sig/all.h>

using namespace yarp::sig;

namespace wb_sot{
    namespace solvers{

    class QPOasesProblem {
    public:
        QPOasesProblem():
            _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
            _bounds(),
            _constraints(),
            _nWSR(127),
            _solution(0), _dual_solution(0),
            _is_initialized(false),
            _initial_guess(false)
        {
            _options.printLevel = qpOASES::PL_HIGH;
            _options.setToReliable();
            _options.enableRegularisation = qpOASES::BT_TRUE;
            _options.epsRegularisation *= 2E2;
        }

        QPOasesProblem(const int number_of_variables,
                       const int number_of_constraints,
                       qpOASES::HessianType hessian_type = qpOASES::HST_UNKNOWN):
            _problem(number_of_variables, number_of_constraints, hessian_type),
            _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
            _bounds(),
            _constraints(),
            _nWSR(127),
            _solution(0), _dual_solution(0),
            _is_initialized(false),
            _initial_guess(false)
        {
            _options.printLevel = qpOASES::PL_HIGH;
            _options.setToReliable();
            _options.enableRegularisation = qpOASES::BT_TRUE;
            _options.epsRegularisation *= 2E2;

            _problem.setOptions(_options);
        }
        ~QPOasesProblem(){}

        void setProblem(const qpOASES::QProblem& problem)
        {
            _problem = problem;
        }

        void initProblem(const Matrix& H, const Vector& g,
                        const Matrix& A,
                        const Vector& lA, const Vector& uA,
                        const Vector& l, const Vector& u)
        {
            _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

            _is_initialized = true;
            initProblem();
        }

        void initProblem()
        {
            if(_is_initialized)
            {
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

                if(_solution.size() != _problem.getNV())
                {
                    _solution.resize(_problem.getNV());
                    _initial_guess = false;
                }
                if(_dual_solution.size() != _problem.getNV() + _problem.getNC()) {
                    _dual_solution.resize(_problem.getNV()+ _problem.getNC());
                    _initial_guess = false;
                }
            }
            else
            {
                std::cerr<<"QProblem not properly initialized!"<<std::endl;
            }
        }

        bool solve()
        {
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

        void setInitialGuess(const bool initial_guess){_initial_guess = initial_guess;}
        bool getInitialGuess(){return _initial_guess;}
        const Vector& getSolution(){return _solution;}
        qpOASES::HessianType getHessianType(){return _problem.getHessianType();}
        void setHessianType(const qpOASES::HessianType ht){_problem.setHessianType(ht);}
        qpOASES::Options getOptions(){return _options;}
        void setOptions(const qpOASES::Options& options){_options = options;}
        const qpOASES::QProblem& getProblem(){return _problem;}
        int getNWSR(){return _nWSR;}
        void setNWSR(const int nWSR){_nWSR = nWSR;}
        bool isQProblemInitialized(){return _is_initialized;}

    protected:
        qpOASES::Options _options;
        qpOASES::QProblem _problem;
        qpOASES::Bounds _bounds;
        qpOASES::Constraints _constraints;
        int _nWSR;
        bool _is_initialized;
        bool _initial_guess;

        Matrix _H;
        Vector _g;
        Matrix _A;
        Vector _lA;
        Vector _uA;
        Vector _l;
        Vector _u;

        Vector _solution;
        Vector _dual_solution;
    };

    class QPOases: public Solver<Matrix, Vector>
    {
    public:
        QPOases():
            _qpProblems(0),
            _initial_guess(false)
        {}

        ~QPOases(){}

        void solve(Vector& solution)
        {

        }

        Vector solve()
        {
            Vector solution;
            solve(solution);
            return solution;
        }

        void addTask()//(const Task& task)
        {
            //QPOasesProblem problem(task.getXSize(), );
        }

        unsigned int getNumberOfStacks(){return _qpProblems.size();}

    protected:
        vector< QPOasesProblem > _qpProblems;
        bool _initial_guess;
    };

    }
}

#endif
