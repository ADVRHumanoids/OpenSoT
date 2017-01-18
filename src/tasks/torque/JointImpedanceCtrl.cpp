/*
 * Copyright (C) 2016 Cogimon
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

#include <OpenSoT/tasks/torque/JointImpedanceCtrl.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::torque;

JointImpedanceCtrl::JointImpedanceCtrl(const Eigen::VectorXd& x, XBot::ModelInterface &robot) :
    Task("JointImpedanceCtrl", x.size()), _x(x), _x_dot(x), _robot(robot),
    _x_desired(x.size()), _xdot_desired(x.size()), _use_inertia_matrix(true)
{
    _x_desired.setZero(x.size());
    _xdot_desired.setZero(x.size());

    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);

    _K = _W;
    _K = 100.*_K;

    _D = _W;
    _D = 1.*_D;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _lambda = 1.0;

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->_update(x);
}

JointImpedanceCtrl::~JointImpedanceCtrl()
{
}

void JointImpedanceCtrl::_update(const Eigen::VectorXd &x) {
    _x = x;

    _robot.getJointVelocity(_x_dot);

    _W.setIdentity(_W.rows(), _W.cols());
    _hessianType = HST_IDENTITY;
    if(_use_inertia_matrix)
    {
        _hessianType = HST_POSDEF;

        _robot.getInertiaMatrix(_M);

        _W = _W*_M.inverse();
    }

    /************************* COMPUTING TASK *****************************/

    this->update_b();

    _xdot_desired.setZero(_xdot_desired.size());

    /**********************************************************************/
}

void JointImpedanceCtrl::setReference(const Eigen::VectorXd& x_desired) {
    assert(x_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired.setZero(_xdot_desired.size());
    this->update_b();
}

void JointImpedanceCtrl::setReference(const Eigen::VectorXd &x_desired,
                                      const Eigen::VectorXd &xdot_desired)
{
    assert(x_desired.size() == _x_size);
    assert(xdot_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired = xdot_desired;
    this->update_b();
}

Eigen::VectorXd JointImpedanceCtrl::getReference() const
{
    return _x_desired;
}

void JointImpedanceCtrl::getReference(Eigen::VectorXd &x_desired,
                                      Eigen::VectorXd &xdot_desired) const
{
    x_desired = _x_desired;
    xdot_desired = _xdot_desired;
}

void JointImpedanceCtrl::update_b() {
    _b = getSpringForce() + getDampingForce();
}

Eigen::VectorXd JointImpedanceCtrl::getSpringForce()
{
    return _K*(_x_desired - _x);
}

Eigen::VectorXd JointImpedanceCtrl::getDampingForce()
{
    return _D*(_xdot_desired - _x_dot);
}

void JointImpedanceCtrl::setStiffness(const Eigen::MatrixXd &K)
{
    if(K.cols() == _K.cols() && K.rows() == _K.rows())
        _K = K;
}

void JointImpedanceCtrl::setDamping(const Eigen::MatrixXd &D)
{
    if(D.cols() == _D.cols() && D.rows() == _D.rows())
        _D = D;
}

void JointImpedanceCtrl::setStiffnessDamping(const Eigen::MatrixXd& K, const Eigen::MatrixXd &D)
{
    setStiffness(K);
    setDamping(D);
}

Eigen::MatrixXd JointImpedanceCtrl::getStiffness()
{
    return _K;
}

Eigen::MatrixXd JointImpedanceCtrl::getDamping()
{
    return _D;
}

void JointImpedanceCtrl::getStiffnessDamping(Eigen::MatrixXd &K, Eigen::MatrixXd &D)
{
    K = getStiffness();
    D = getDamping();
}

Eigen::VectorXd JointImpedanceCtrl::getActualPositions()
{
    return _x;
}

Eigen::VectorXd JointImpedanceCtrl::getActualVelocities()
{
    return _x_dot;
}

void JointImpedanceCtrl::useInertiaMatrix(const bool use)
{
    _use_inertia_matrix = use;
}

