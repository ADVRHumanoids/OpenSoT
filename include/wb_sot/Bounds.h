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

 namespace wb_sot {

    /** Task represents
    */
 template <class Matrix_type, class Vector_type,
           unsigned int x_size>
    class Bounds {
    public:
        Bounds();
        ~Bounds();

        const Vector_type getLowerBound();
        const Vector_type getUpperBound();

        const Matrix_type getAeq();
        const Vector_type getbeq();

        const Matrix_type getAineq();
        const Vector_type getbLowerBound();
        const Vector_type getbUpperBound();


        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Vector_type& x);
    };
 }
