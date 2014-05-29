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
    template <class Matrix_type, class Vector_type,
              unsigned int x_size>
    class Task {
        const Vector_type x;
    public:
        Task(const Vector_type& x0);
        ~Task();

        /** Return */
        const Matrix_type getA();
        const HessianType getAtype();
        const Vector_type getb();

        const Matrix_type getWeight() const;
        void setWeight(const Matrix_type& W);

        const double getAlpha() const;
        void setAlpha(double alpha);
        
        const Vector_type getLowerBound();
        const Vector_type getUpperBound();

        const Matrix_type getAeq();
        const Vector_type getbeq();

        const Matrix_type getAineq();

        const Vector_type getbLowerBound();
        const Vector_type getbUpperBound();

        const Vector_type getResidual();
        void setResidual(const Vector_type residual);

        /** Gets the number of variables for the task.
            @return the number of columns of A */
        const unsigned int getXSize();

        /** Gets the task size.
            @return the number of rows of A */
        const unsigned int getTaskSize();

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Vector_type x);
    };
 }

#endif
