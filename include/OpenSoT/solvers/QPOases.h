/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _WB_SOT_SOLVERS_QP_OASES_H_
#define _WB_SOT_SOLVERS_QP_OASES_H_

#include <qpOASES.hpp>
#include <vector>
#include <iostream>
#include <yarp/sig/all.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/Aggregated.h>

using namespace yarp::sig;

namespace OpenSoT{
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
         * @brief setDefaultOptions to internal qpOases problem
         */
        void setDefaultOptions();

        /**
         * @brief setProblem copy a QP Problem in the internal object of the class.
         * @param problem to copy
         */
        void setProblem(const boost::shared_ptr<qpOASES::SQProblem> &problem);

        /**
         * @brief getProblem return the internal QP problem
         * @return reference to internal QP problem
         */
        const boost::shared_ptr<qpOASES::SQProblem>& getProblem(){return _problem;}

        /**
         * @brief getOptions return the options of the QP problem
         * @return reference to options
         */
        qpOASES::Options getOptions(){return _problem->getOptions();}

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
        bool addTask(const Matrix& H, const Vector& g);
        bool addConstraints(const Matrix& A, const Vector& lA, const Vector& uA);
        bool addBounds(const Vector& l, const Vector& u);
        bool addProblem(const Matrix& H, const Vector& g,
                           const Matrix& A,
                           const Vector& lA, const Vector& uA,
                           const Vector& l, const Vector& u);

        /**
         * @brief solve the QP problem
         * @return true if the QP problem is initialized and solved
         */
        bool solve();

        /**
         * @brief getSolution return the actual solution of the QP problem
         * @return solution
         */
        const Vector& getSolution(){return _solution;}

        /**
         * @brief getHessianType return the hessian type f the problem
         * @return hessian type
         */
        qpOASES::HessianType getHessianType(){return _problem->getHessianType();}

        /**
         * @brief setHessianType of the problem
         * @param ht hessian type
         */
        void setHessianType(const qpOASES::HessianType ht){_problem->setHessianType(ht);}

        /**
         * @brief getnWSR return maximum number of working set recalculations
         * @return maximum number of working set recalculations
         */
        int getnWSR(){return _nWSR;}

        /**
         * @brief setnWSR set maximum number of working set recalculations
         * @param nWSR Maximum number of working set recalculations
         */
        void setnWSR(const int nWSR){_nWSR = nWSR;}

        /**
         * @brief isQProblemInitialized
         * @return true if the internal problem is initialized
         */
        bool isQProblemInitialized(){return _is_initialized;}

        /**
         * @brief resetProblem call the reset method of the SQProblem
         * @return true if reset
         */
        bool resetProblem(){return _problem->reset();}

        /**
         * @brief getActiveBounds return the active bounds of the solved QP problem
         * @return active bounds
         */
        const qpOASES::Bounds& getActiveBounds(){return _bounds;}

        /**
         * @brief getActiveConstraints return the active constraints of the solved QP problem
         * @return active constraints
         */
        const qpOASES::Constraints& getActiveConstraints(){return _constraints;}

    protected:
        /**
         * @brief _problem is the internal SQProblem
         */
        boost::shared_ptr<qpOASES::SQProblem> _problem;

        /**
         * @brief _bounds are the active bounds of the SQProblem
         */
        qpOASES::Bounds _bounds;

        /**
         * @brief _constraints are the active constraints of the SQProblem
         */
        qpOASES::Constraints _constraints;

        /**
         * @brief _nWSR is the maximum number of working set recalculations
         */
        int _nWSR;

        /**
         * @brief _is_initialized is set to true when the problem is initialized
         */
        bool _is_initialized;

        /**
         * Define a cost function: ||Hx - g||
         */
        Matrix _H;
        Vector _g;

        /**
         * Define a set of constraints weighted with A: lA <= Ax <= uA
         */
        Matrix _A;
        Vector _lA;
        Vector _uA;
        double * _A_ptr;
        double *_lA_ptr;
        double *_uA_ptr;


        /**
         * Define a set of bounds on solution: l <= x <= u
         */
        Vector _l;
        Vector _u;
        double *_l_ptr;
        double *_u_ptr;

        /**
         * Solution and dual solution of the QP problem
         */
        Vector _solution;
        Vector _dual_solution;

        /**
         * @brief hack take care of the problem of yarp::sig::Matrix and yarp::sig::Vector
         * that does not return NULL when empty!
         */
        void hack();
    };


    /**
     * @brief The QPOasesTask class wrapper around QPOasesProblem to easily pass a Task
     */
    class QPOasesTask: public QPOasesProblem
    {
    public:
        /**
         * @brief QPOasesTask constructor takes a shared_ptr to have automatically
         * updated task matrices
         * @param task task to solve
         */
        QPOasesTask(const boost::shared_ptr< Task<Matrix, Vector> >& task);

        ~QPOasesTask();

        /**
         * @brief solve the internal qp problem
         * @param update_constraints if the problem have to update alone its constraints based
         * on the internal task
         * @return true if solved/solvable
         */
        bool solve(bool update_constraints = true);

        /**
         * @brief printProblemInformation couts some information about the problem.
         * @param i, if i = -1 the ID is printed without number:
         * eg:
         *  printProblemInformation();
         *      "PROBLEM 0 ID: com"
         *  printProblemInformation(-1);
         *      "PROBLEM ID: com"
         *  printProblemInformation(2);
         *      "PROBLEM 2 ID: com"
         */
        void printProblemInformation(int i = 0);

        void getCostFunction(Matrix& H, Vector& g);
        void getConstraints(Matrix& A, Vector& lA, Vector& uA);
        void getBounds(Vector& l, Vector& u);

    protected:
        /**
         * @brief _task pointer to task to optimize
         */
        boost::shared_ptr< Task<Matrix, Vector> > _task;

        /**
         * @brief prepareData compute matrices for QPOases
         * @param update_constraints if set to false does not update the constraint matrices
         */
        void prepareData(bool update_constraints = true);
    };





    class QPOases_sot: public Solver<Matrix, Vector>
    {
    public:
        QPOases_sot(vector <boost::shared_ptr< Task<Matrix, Vector> >>& stack_of_tasks,
                    boost::shared_ptr<OpenSoT::constraints::Aggregated>& constraints);

        ~QPOases_sot(){}

        bool solve(Vector& solution);
        unsigned int getNumberOfTasks();

        /**
         * @brief getNumberOfConstraintsInQP return for each task the number of
         *  constraints contained
         * @return a vector of std::pair<constrain_id, number_of_constraints>
         */
        std::vector<std::pair<std::string, int>> getNumberOfConstraintsInQP();

        std::vector<std::pair<std::string, int>> getNumberOfConstraintsInTaskList();

    protected:
        vector <QPOasesTask> _qp_stack_of_tasks;

        bool prepareSoT();
        bool expandProblem(unsigned int i);
        bool updateExpandedProblem(unsigned int i);

    };

    }
}

#endif
