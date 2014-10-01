/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <wb_sot/Task.h>
#include <wb_sot/Constraint.h>
#include <list>

using namespace std;

 namespace OpenSoT {
    template < class Matrix_type, class Vector_type >
    class Solver {
    public:
        typedef Solver< Matrix_type, Vector_type > SolverType;
        typedef boost::shared_ptr<SolverType> SolverPointer;

        Solver(){}
        virtual ~Solver(){}

        /**
         * @brief solve solve an Optimization problem
         * @param solution the solution
         * @return  true if solved/solvable
         */
        virtual bool solve(Vector_type& solution) = 0;

//        virtual unsigned int addTask() = 0;
//        const int* getTasks();
//        const Task<Matrix_type, Vector_type, x_size>* getTask(const unsigned int taskId);

//        void addGlobalLowerBounds(const Vector_type lowerBounds);
//        void addGlobalUpperBounds(const Vector_type upperBounds);
    };
 }

#endif
