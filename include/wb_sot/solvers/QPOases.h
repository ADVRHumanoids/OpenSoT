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

    /**
     * @brief The QPOasesProblem class handle variables, options and execution of a
     * single qpOases problem. Is implemented using yarp::sig Matrix and Vector.
     */
    class QPOasesProblem {
    public:
        /**
         * @brief QPOasesProblem Default constructor. If used remember to use
         * setProblem(const qpOASES::SQProblem& problem) to add a QP problem to
         * the class!
         * Ex.
         *
         *  QPOasesProblem p;
         *  qpOASES::SQProblem testProblem(2, 2, HST_IDENTITY);
         *  p.setProblem(testProblem);
         */
        QPOasesProblem();

        /**
         * @brief QPOasesProblem constructor with creation of a QP problem.
         * @param number_of_variables of the QP problem
         * @param number_of_constraints of the QP problem
         * @param hessian_type of the QP problem
         */
        QPOasesProblem(const int number_of_variables,
                       const int number_of_constraints,
                       qpOASES::HessianType hessian_type = qpOASES::HST_UNKNOWN);

        /**
          * @brief ~QPOasesProblem destructor
          */
        ~QPOasesProblem(){}

        /**
         * @brief setProblem copy a QP Problem in the internal object of the class.
         * @param problem to copy
         */
        void setProblem(const qpOASES::SQProblem& problem);

        /**
         * @brief getProblem return the internal QP problem
         * @return reference to internal QP problem
         */
        const qpOASES::SQProblem& getProblem(){return _problem;}

        /**
         * @brief getOptions return the options of the QP problem
         * @return reference to options
         */
        const qpOASES::Options& getOptions(){return _problem.getOptions();}

        /**
         * @brief setOptions of the QP problem. Default are set to:
         *      qpOASES::Options opt;
         *      opt.printLevel = qpOASES::PL_HIGH;
         *      opt.setToReliable();
         *      opt.enableRegularisation = qpOASES::BT_TRUE;
         *      opt.epsRegularisation *= 2E2;
         * @param options
         */
        void setOptions(const qpOASES::Options& options);

        /**
         * @brief initProblem initialize the QP problem and get the solution, the dual solution,
         * bounds and constraints.
         * The QP problem has the following structure:
         *
         *      min = ||Hx - g||
         *  st.     lA <= Ax <= uA
         *           l <=  x <= u
         * @param H
         * @param g
         * @param A
         * @param lA
         * @param uA
         * @param l
         * @param u
         * @return true if the problem can be solved
         */
        bool initProblem(const Matrix& H, const Vector& g,
                        const Matrix& A,
                        const Vector& lA, const Vector& uA,
                        const Vector& l, const Vector& u);

        /**
         * This set of function update current problem copying input data. Use these
         * methods to update existing matrices of the QP problem.
         */

        /**
         * @brief updateTask update H and g
         * @param H
         * @param g
         * @return true if the Problem has been initialized using initProblem( ... ) and
         * if the size of H and g is the same as the one in the QP problem.
         */
        bool updateTask(const Matrix& H, const Vector& g);

        /**
         * @brief updateConstraints update A, lA and uA
         * @param A
         * @param lA
         * @param uA
         * @return true if the Problem has been initialized using initProblem( ... ) and
         * if the size of A, lA and uA is the same as the one in the QP problem.
         */
        bool updateConstraints(const Matrix& A, const Vector& lA, const Vector& uA);

        /**
         * @brief updateBounds update l and u
         * @param l
         * @param u
         * @return rue if the Problem has been initialized using initProblem( ... ) and
         * if the size of l and u is the same as the one in the QP problem.
         */
        bool updateBounds(const Vector& l, const Vector& u);

        /**
         * @brief updateProblem update the whole problem
         * @param H
         * @param g
         * @param A
         * @param lA
         * @param uA
         * @param l
         * @param u
         * @return true if the previous update methods return true
         */
        bool updateProblem(const Matrix& H, const Vector& g,
                           const Matrix& A,
                           const Vector& lA, const Vector& uA,
                           const Vector& l, const Vector& u);

        /*
         * This set of function add input data to the problem
         */
        bool addTask(const Matrix& H, const Vector& g, const bool init_problem = true);
        bool addConstraints(const Matrix& A, const Vector& lA, const Vector& uA,
                            const bool init_problem = true);
        bool addBounds(const Vector& l, const Vector& u, const bool init_problem = true);
        bool addProblem(const Matrix& H, const Vector& g,
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
