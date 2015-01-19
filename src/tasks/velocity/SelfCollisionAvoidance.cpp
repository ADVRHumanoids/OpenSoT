/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/tasks/velocity/SelfCollisionAvoidance.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;
using namespace Eigen;

SelfCollisionAvoidance::SelfCollisionAvoidance(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot) :
    Task(task_id, x.size()), Gradient(robot)
{
    
//    int dim;
//    dim = x.length();
//    MatrixXd _A_E(dim,dim);
//    _A_E = MatrixXd::Identity(dim,dim);
//    _A = Gradient.from_Eigen_to_Yarp_matrix(_A_E);

//    _W.resize(_A.rows(), _A.rows());
//    _W.eye();

    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    _hessianType = HST_IDENTITY;

    _Alpha = 1.0;

    //this->_update(x);
    _b.resize(_x_size, 0.0);
}

SelfCollisionAvoidance::~SelfCollisionAvoidance()
{
}

void SelfCollisionAvoidance::_update(const yarp::sig::Vector &x) {

    this->update_b(x);

}


void SelfCollisionAvoidance::setAlpha(double Alpha)
{
    this->_Alpha = Alpha;
}

double SelfCollisionAvoidance::getAlpha()
{
    return _Alpha;
}

void SelfCollisionAvoidance::update_b(const yarp::sig::Vector& x) {
	
     VectorXd _b_E_temp, _b_E, Q;
     int x_dim = x.length();
     Q = Gradient.from_yarp_to_Eigen_vector(x);
     _b_E_temp = Gradient.shortest_distance_gradient(Q);
     _b_E = _b_E_temp.block(1,0,x_dim,1);
     _b = _Alpha * Gradient.from_Eigen_to_Yarp_vector(_b_E);


}
