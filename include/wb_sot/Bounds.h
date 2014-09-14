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

#ifndef __BOUNDS_H__
#define __BOUNDS_H__

 namespace wb_sot {

    /** Task represents
    */
 template <class Matrix_type, class Vector_type>
    /**
     * @brief The Bounds class describes all the different types of constraints:
     * 1. bounds & bilateral
     * 2. equalities
     * 3. unilateral
     */
    class Bounds {
    public:
        typedef Bounds< Matrix_type, Vector_type > BoundType;
    protected:
        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int _x_size;

        /**
         * @brief _lowerBound lower bounds on controlled variables
         * e.g.:
         *              _lowerBound <= x
         */
        Vector_type _lowerBound;

        /**
         * @brief _upperBound upper bounds on controlled variables
         * e.g.:
         *              x <= _upperBound
         */
        Vector_type _upperBound;

        /**
         * @brief _Aeq Matrix for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Matrix_type _Aeq;

        /**
         * @brief _beq constraint vector for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Vector_type _beq;

        /**
         * @brief _Aineq Matrix for inequality constraint
         * e.g.:
         *              _bLowerBound <= _Aineq*x <= _bUpperBound
         */
        Matrix_type _Aineq;

        /**
         * @brief _bLowerBound lower bounds in generic inequality constraints
         * e.g.:
         *              _bLowerBound <= _Aineq*x
         */
        Vector_type _bLowerBound;

        /**
         * @brief _bUpperBound upper bounds in generic inequality constraints
         * e.g.:
         *              _Aineq*x <= _bUpperBound
         */
        Vector_type _bUpperBound;

    public:
        Bounds(const unsigned int x_size) : _x_size(x_size) {}
        virtual ~Bounds() {}

        virtual const Vector_type& getLowerBound() { return _lowerBound; }
        virtual const Vector_type& getUpperBound() { return _upperBound; }

        virtual const Matrix_type& getAeq() { return _Aeq; }
        virtual const Vector_type& getbeq() { return _beq; }

        virtual const Matrix_type& getAineq() { return _Aineq; }
        virtual const Vector_type& getbLowerBound() { return _bLowerBound; }
        virtual const Vector_type& getbUpperBound() { return _bUpperBound; }


        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        virtual void update(const Vector_type& x){}
    };
 }

#endif
