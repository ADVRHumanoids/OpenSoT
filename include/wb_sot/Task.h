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
 #include <wb_sot/Bounds.h>

 namespace wb_sot {

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
    class Task {
    protected:
        typedef Bounds< Matrix_type, Vector_type > BoundType;

        std::string _task_id;

        unsigned int _x_size;
        Vector_type _x0;

        Vector_type _zeroVector;
        Matrix_type _zeroMatrix;
        Matrix_type _eyeMatrix;

        HessianType _hessianType;

        Matrix_type _A;
        Vector_type _b;

        Matrix_type _W;
        double _alpha;

        Vector_type _residual;

        std::list< BoundType > _bounds;

        Vector_type _x;

    public:
        Task(const std::string task_id,
             const Vector_type& x,
             const unsigned int x_size) :
            _task_id(task_id), _x0(x), _x_size(x_size)
        {
            _A = _zeroMatrix;
            _b = _zeroVector;

            _W = _eyeMatrix;
            _alpha = 1.0;

            _residual = _zeroVector;
        }

        virtual ~Task();

        virtual const Matrix_type& getA() { return _A; }
        virtual const HessianType getAtype() { return HST_UNKNOWN; }
        virtual const Vector_type& getb() { return _b; }

        virtual const Matrix_type& getWeight() const { return _W; }
        virtual void setWeight(const Matrix_type& W) { _W = W; }

        virtual const double getAlpha() const { return _alpha; }
        virtual void setAlpha(double alpha) { _alpha = alpha; }
        
        virtual const std::list< BoundType >& getConstraints() const { return _bounds; }

        virtual const Vector_type getResidual() const { return _residual; }
        virtual void setResidual(const Vector_type residual) { _residual = residual; }

        /** Gets the number of variables for the task.
            @return the number of columns of A */
        const unsigned int getXSize() const { return _x_size; }

        /** Gets the task size.
            @return the number of rows of A */
        virtual const unsigned int getTaskSize() const { return _A.rows(); }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Vector_type x);
    };
 }

#endif
