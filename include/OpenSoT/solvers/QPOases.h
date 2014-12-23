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

#include <vector>
#include <iostream>
#include <yarp/sig/all.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/Aggregated.h>
#include "QPOasesProblem.h"



using namespace yarp::sig;

namespace qpOASES {
    class SQProblem;
    class Options;
    class Bounds;
    class Constraints;
}


namespace OpenSoT{
    namespace solvers{



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
        QPOasesTask(const boost::shared_ptr< Task<Matrix, Vector> >& task, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        /**
         * @brief QPOasesTask constructor takes a shared_ptr to have automatically
         * updated task matrices
         * @param task task to solve
         */
        QPOasesTask(const boost::shared_ptr< Task<Matrix, Vector> >& task,
                    const boost::shared_ptr< Constraint<Matrix, Vector> >& bounds, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

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
        std::string getTaskID(){return _task->getTaskID();}

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
	typedef boost::shared_ptr<QPOases_sot> Ptr;

        QPOases_sot(Stack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);
        QPOases_sot(Stack& stack_of_tasks,
                    boost::shared_ptr<OpenSoT::constraints::Aggregated>& constraints,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        ~QPOases_sot(){}

        bool solve(Vector& solution);
        unsigned int getNumberOfTasks();


        bool setOptions(const unsigned int i, const qpOASES::Options &opt);
        bool getOptions(const unsigned int i, qpOASES::Options& opt);

    protected:
        vector <QPOasesTask> _qp_stack_of_tasks;
        double _epsRegularisation;

        bool prepareSoT();
        bool expandProblem(unsigned int i);
        bool updateExpandedProblem(unsigned int i);

    };

    }
}

#endif
