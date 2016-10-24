/*
 * Copyright (C) 2016 Cogimon, Walkman
 * Author: Enrico Mingo Hoffman, Alessio Rocchi
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

#ifndef _MATH_UTILS_
#define _MATH_UTILS_

#include <Eigen/Dense>

namespace OpenSoT{
namespace utils{
namespace math{

/**
 * @brief pile operation between two matrices:
 *
 *              A = [A;B]
 * @param A
 * @param B
 */
static void pile(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& A,
                 const Eigen::MatrixXd& B)
{
    A.conservativeResize(A.rows()+B.rows(), A.cols());
    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
}

/**
 * @brief pile operation between two matrices:
 *
 *              A = [A;B]
 * @param A
 * @param B
 */
static void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    A.conservativeResize(A.rows()+B.rows(), A.cols());
    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
}

/**
 * @brief pile operation between two vectors:
 *
 *              a = [a;b]
 * @param a
 * @param b
 */
static void pile(Eigen::VectorXd &a, const Eigen::VectorXd &b)
{
    a.conservativeResize(a.rows()+b.rows());
    a.segment(a.rows()-b.rows(),b.rows())<<b;
}

}
}
}

#endif
