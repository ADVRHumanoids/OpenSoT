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
#include "QPOasesTask.h"



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
     * @brief The QPOases_sot class implement a solver that accept a Stack of Tasks with Bounds and Constraints
     */
    class QPOases_sot: public Solver<Matrix, Vector>
    {
    public:
	typedef boost::shared_ptr<QPOases_sot> Ptr;
        /**
         * @brief QPOases_sot constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param eps_regularisation regularisation factor
         */
        QPOases_sot(Stack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        /**
         * @brief QPOases_sot constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param bounds a vector of bounds passed to all the stacks
         * @param eps_regularisation regularisation factor
         */
        QPOases_sot(Stack& stack_of_tasks,
                    boost::shared_ptr<OpenSoT::constraints::Aggregated>& bounds,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION);


        ~QPOases_sot(){}

        bool solve(Vector& solution);
        unsigned int getNumberOfTasks(){return _qp_stack_of_tasks.size();}

        bool setOptions(const unsigned int i, const qpOASES::Options &opt);
        bool getOptions(const unsigned int i, qpOASES::Options& opt);

    protected:
        vector <QPOasesProblem> _qp_stack_of_tasks;
        double _epsRegularisation;

        void prepareSoT();

        void computeVelCtrlCostFunction(const TaskPtr& task, yarp::sig::Matrix& H, yarp::sig::Vector& g);
        void computeVelCtrlOptimalityConstraint(const TaskPtr& task, QPOasesProblem& problem,
                                                yarp::sig::Matrix& A, yarp::sig::Vector& lA, yarp::sig::Vector& uA);

    };

    }
}

#endif
