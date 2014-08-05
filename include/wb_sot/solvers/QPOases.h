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
        QPOasesProblem();

        QPOasesProblem(const int number_of_variables,
                       const int number_of_constraints,
                       qpOASES::HessianType hessian_type = qpOASES::HST_UNKNOWN);

        ~QPOasesProblem(){}

        void addProblem(const qpOASES::SQProblem& problem);
        const qpOASES::SQProblem& getProblem(){return _problem;}

        const qpOASES::Options& getOptions(){return _problem.getOptions();}
        void setOptions(const qpOASES::Options& options);

        bool initProblem(const Matrix& H, const Vector& g,
                        const Matrix& A,
                        const Vector& lA, const Vector& uA,
                        const Vector& l, const Vector& u);

        /*
         * This set of function update current problem copying input data
         */
        bool updateTask(const Matrix& H, const Vector& g);
        bool updateConstraints(const Matrix& A, const Vector& lA, const Vector& uA);
        bool updateBounds(const Vector& l, const Vector& u);
        bool updateProblem(const Matrix& H, const Vector& g,
                           const Matrix& A,
                           const Vector& lA, const Vector& uA,
                           const Vector& l, const Vector& u);


        bool solve();

        const Vector& getSolution(){return _solution;}

        qpOASES::HessianType getHessianType(){return _problem.getHessianType();}
        void setHessianType(const qpOASES::HessianType ht){_problem.setHessianType(ht);}


        int getnWSR(){return _nWSR;}
        void setnWSR(const int nWSR){_nWSR = nWSR;}

        bool isQProblemInitialized(){return _is_initialized;}

        bool resetProblem(){return _problem.reset();}

    protected:
        qpOASES::SQProblem _problem;
        qpOASES::Bounds _bounds;
        qpOASES::Constraints _constraints;
        int _nWSR;
        bool _is_initialized;

        /*
         * Define a cost function
         */
        Matrix _H;
        Vector _g;

        /*
         * Define a set of constraints weighted with A
         */
        Matrix _A;
        Vector _lA;
        Vector _uA;

        /*
         * Define a set of bounds on solution
         */
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
