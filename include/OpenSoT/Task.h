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

#ifndef __TASK_H__
#define __TASK_H__

 #include <list>
 #include <string>
 #include <OpenSoT/Constraint.h>
 #include <assert.h>
 #include <boost/shared_ptr.hpp>

 namespace OpenSoT {

    /** Summarises all possible types of the QP's Hessian matrix. From qpOASES/Types.hpp */
    enum HessianType
    {
        HST_ZERO,                   /**< Hessian is zero matrix (i.e. LP formulation). */
        HST_IDENTITY,               /**< Hessian is identity matrix. */
        HST_POSDEF,                 /**< Hessian is (strictly) positive definite. */
        HST_POSDEF_NULLSPACE,       /**< Hessian is positive definite on null space of active bounds/constraints. */
        HST_SEMIDEF,                /**< Hessian is positive semi-definite. */
        HST_UNKNOWN                 /**< Hessian type is unknown. */
    };

    /** Task represents
    */
    template <class Matrix_type, class Vector_type>
    /**
     * @brief The Task class describe a task and its constraints.
     */
    class Task {

    public:
        typedef Task< Matrix_type, Vector_type > TaskType;
        typedef boost::shared_ptr<TaskType> TaskPtr;
        typedef Constraint< Matrix_type, Vector_type > ConstraintType;
        typedef boost::shared_ptr<ConstraintType> ConstraintPtr;
    protected:

        /**
         * @brief _task_id unique name of the task
         */
        std::string _task_id;

        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int _x_size;

        /**
         * @brief _hessianType Type of Hessian associated to the Task
         */
        HessianType _hessianType;

        /**
         * @brief _A Jacobian of the Task
         */
        Matrix_type _A;

        /**
         * @brief _b error associated to the Task
         */
        Vector_type _b;

        /**
         * @brief _W Weight multiplied to the task Jacobian
         */
        Matrix_type _W;

        /**
         * @brief _alpha error scaling,
         * NOTE:
         *          0.0 <= _alpha <= 1.0
         */
        double _lambda;

        /**
         * @brief _bounds related to the Task
         */
        std::list< boost::shared_ptr<ConstraintType> > _constraints;

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
            @param x variable state at the current step (input) */
        virtual void _update(const Vector_type &x) = 0;

    public:

        Task(const std::string task_id,
             const unsigned int x_size) :
            _task_id(task_id), _x_size(x_size)
        {
            _lambda = 1.0;
            _hessianType = HST_UNKNOWN;
        }

        virtual ~Task(){}

        const Matrix_type& getA() const { return _A; }
        const HessianType getHessianAtype() { return _hessianType; }
        const Vector_type& getb() const { return _b; }

        const Matrix_type& getWeight() const { return _W; }
        void setWeight(const Matrix_type& W) { _W = W; }

        const double getLambda() const { return _lambda; }
        void setLambda(double lambda)
        {
            assert(lambda <= 1.0 && lambda > 0.0);
            _lambda = lambda;
        }
        
        /** TODO should we transform the list to a list of pointers?  */
        /**
         * @brief getConstraints return a reference to the constraint list. Use the standard list methods
         * to add, remove, clear, ... the constraints list.
         * i.e.:
         *              task.getConstraints().push_back(new_constraint)
         * @return
         */
        std::list< boost::shared_ptr<ConstraintType> >& getConstraints() { return _constraints; }

        /** Gets the number of variables for the task.
            @return the number of columns of A */
        const unsigned int getXSize() const { return _x_size; }

        /** Gets the task size.
            @return the number of rows of A */
        const unsigned int getTaskSize() const { return _A.rows(); }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Vector_type &x) {
            for(typename std::list< boost::shared_ptr<ConstraintType> >::iterator i = _constraints.begin();
                i != _constraints.end(); ++i) (*i)->update(x);
            this->_update(x); }

        /**
         * @brief getTaskID return the task id
         * @return a string with the task id
         */
        std::string getTaskID(){ return _task_id; };
    };
 }

#endif
