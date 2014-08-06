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
#include <wb_sot/Bounds.h>
#include <list>

using namespace std;

 namespace wb_sot {
    template < class Matrix_type, class Vector_type >
    class Solver {
    public:
        Solver(){}
        virtual ~Solver(){}

        /**
         * @brief solve Method to run QP optimization
         * @param stack_of_tasks a list of task. Each element represent a stack.
         * @param stack_of_bounds a list of bounds. Each element represent a stack.
         * @param solution
         */
        virtual Vector_type solve() = 0;

//        virtual unsigned int addTask() = 0;
//        const int* getTasks();
//        const Task<Matrix_type, Vector_type, x_size>* getTask(const unsigned int taskId);

//        void addGlobalLowerBounds(const Vector_type lowerBounds);
//        void addGlobalUpperBounds(const Vector_type upperBounds);
    };
 }

#endif
