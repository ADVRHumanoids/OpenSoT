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
    class Bounds {
    protected:
        unsigned int _x_size;

        Vector_type _zeroVector;
        Matrix_type _zeroMatrix;

        Vector_type _lowerBound;
        Vector_type _upperBound;

        Matrix_type _Aeq;
        Vector_type _beq;

        Matrix_type _Aineq;
        Vector_type _bLowerBound;
        Vector_type _bUpperBound;

        Vector_type _x;
    public:
        Bounds(const unsigned int x_size) : _x_size(x_size) {
            _lowerBound = _zeroVector;
            _upperBound = _zeroVector;

            _Aeq = _zeroMatrix;
            _beq = _zeroVector;

            _Aineq = _zeroMatrix;
            _bLowerBound = _zeroVector;
            _bUpperBound = _zeroVector;
        }
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
        virtual void update(const Vector_type& x) { _x = x; }
    };
 }

#endif
