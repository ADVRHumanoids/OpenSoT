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

#include <OpenSoT/tasks/virtual_model/JointSpringDamper.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::virtual_model;
using namespace yarp::math;

JointSpringDamper::JointSpringDamper(const yarp::sig::Vector& x, iDynUtils &robot) :
    Task("JointSpringDamper", x.size()), _x(x), _x_dot(x), _robot(robot),
    _x_desired(x.size(),0.0), _xdot_desired(x.size(),0.0), _use_inertia_matrix(true)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _K = _W;
    _K = 100.*_K;

    _D = _W;
    _D = 1.*_D;

    _A.resize(_x_size, _x_size);
    _A.eye();

    _lambda = 1.0;

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->_update(x);
}

JointSpringDamper::~JointSpringDamper()
{
}

void JointSpringDamper::_update(const yarp::sig::Vector &x) {
    _x = x;

    _x_dot = _robot.iDyn3_model.getDAng();

    _W.eye();
    _hessianType = HST_IDENTITY;
    if(_use_inertia_matrix)
    {
        _hessianType = HST_POSDEF;

        _M.resize(6+_x_size, 6+_x_size);
        _robot.iDyn3_model.getFloatingBaseMassMatrix(_M);
        _M = _M.removeCols(0,6); _M = _M.removeRows(0,6);

        _W = _W*yarp::math::luinv(_M);
    }

    /************************* COMPUTING TASK *****************************/

    this->update_b();

    _xdot_desired.zero();

    /**********************************************************************/
}

void JointSpringDamper::setReference(const yarp::sig::Vector& x_desired) {
    assert(x_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired.zero();
    this->update_b();
}

void JointSpringDamper::setReference(const yarp::sig::Vector &x_desired,
                                                      const yarp::sig::Vector &xdot_desired)
{
    assert(x_desired.size() == _x_size);
    assert(xdot_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired = xdot_desired;
    this->update_b();
}

yarp::sig::Vector JointSpringDamper::getReference() const
{
    return _x_desired;
}

void JointSpringDamper::getReference(yarp::sig::Vector &x_desired,
                                                      yarp::sig::Vector &xdot_desired) const
{
    x_desired = _x_desired;
    xdot_desired = _xdot_desired;
}

void JointSpringDamper::update_b() {
    _b = getSpringForce() + getDampingForce();
}

yarp::sig::Vector JointSpringDamper::getSpringForce()
{
    return _K*(_x_desired - _x);
}

yarp::sig::Vector JointSpringDamper::getDampingForce()
{
    return _D*(_xdot_desired - _x_dot);
}

void JointSpringDamper::setStiffness(const yarp::sig::Matrix &K)
{
    if(K.cols() == _K.cols() && K.rows() == _K.rows())
        _K = K;
}

void JointSpringDamper::setDamping(const yarp::sig::Matrix &D)
{
    if(D.cols() == _D.cols() && D.rows() == _D.rows())
        _D = D;
}

void JointSpringDamper::setStiffnessDamping(const yarp::sig::Matrix& K, const yarp::sig::Matrix &D)
{
    setStiffness(K);
    setDamping(D);
}

yarp::sig::Matrix JointSpringDamper::getStiffness()
{
    return _K;
}

yarp::sig::Matrix JointSpringDamper::getDamping()
{
    return _D;
}

void JointSpringDamper::getStiffnessDamping(yarp::sig::Matrix &K, yarp::sig::Matrix &D)
{
    K = getStiffness();
    D = getDamping();
}

yarp::sig::Vector JointSpringDamper::getActualPositions()
{
    return _x;
}

yarp::sig::Vector JointSpringDamper::getActualVelocities()
{
    return _x_dot;
}

