/*
 * Copyright (C) 2016 Cogimon
 * Authors: Enrico Mingo Hoffman, Alessio Rocchi
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

#include <OpenSoT/tasks/virtual_model/CartesianSpringDamper.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::virtual_model;
using namespace yarp::math;

#define LAMBDA_THS 1E-12

CartesianSpringDamper::CartesianSpringDamper(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _desiredTwist(6, 0.0), _use_inertia_matrix(true)
{
    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);

    if(!this->_base_link_is_world) {
        this->_base_link_index = robot.iDyn3_model.getLinkIndex(_base_link);
        assert(this->_base_link_index >= 0);
    }

    this->_distal_link_index = robot.iDyn3_model.getLinkIndex(_distal_link);
    assert(this->_distal_link_index >= 0);

    if(!this->_base_link_is_world)
        assert(this->_distal_link_index != _base_link_index);

    _K.resize(6, 6);
    _K.eye(); _K = 1000.0*_K;
    _D.resize(6, 6);
    _D.eye(); _D = 100.0*_D;

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

    _W.resize(_A.rows(), _A.rows());
    _W.eye();

    _lambda = 1.0;

    _hessianType = HST_SEMIDEF;
}

CartesianSpringDamper::~CartesianSpringDamper()
{
}

void CartesianSpringDamper::_update(const yarp::sig::Vector &x) {

    /************************* COMPUTING TASK *****************************/

    if(_base_link_is_world) {
        bool res = _robot.iDyn3_model.getJacobian(_distal_link_index,_A);
        assert(res);
        _A = _A.removeCols(0,6);    // removing unactuated joints (floating base)
    } else{
        bool res = _robot.iDyn3_model.getRelativeJacobian(_distal_link_index,
                                                          _base_link_index,
                                                          _A, true);
        assert(res);
        yarp::sig::Matrix base_R_world = _robot.iDyn3_model.getPosition(_base_link_index).submatrix(0,2,0,2).transposed();
        yarp::sig::Matrix Adj(6,6); Adj = Adj.eye();
        Adj.setSubmatrix(base_R_world, 0,0);
        Adj.setSubmatrix(base_R_world, 3,3);
        _A = Adj*_A;
    }
    _J = _A;

    if (_use_inertia_matrix)
    {
        _M.resize(6+_x_size, 6+_x_size);
        _robot.iDyn3_model.getFloatingBaseMassMatrix(_M);
        _M = _M.removeCols(0,6); _M = _M.removeRows(0,6);

        _A = _A*yarp::math::luinv(_M);
    }


    if(_base_link_is_world)
        _actualPose = _robot.iDyn3_model.getPosition(_distal_link_index);
    else
        _actualPose = _robot.iDyn3_model.getPosition(_base_link_index, _distal_link_index);

    if(_desiredPose.rows() == 0) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.resize(_A.rows(), 0.0);
    }

    this->update_b();

    this->_desiredTwist.zero();

    /**********************************************************************/
}

void CartesianSpringDamper::setStiffness(const yarp::sig::Matrix &Stiffness)
{
    if(Stiffness.cols() == 6 && Stiffness.rows() == 6)
        _K = Stiffness;
}

void CartesianSpringDamper::setDamping(const yarp::sig::Matrix &Damping)
{
    if(Damping.cols() == 6 && Damping.rows() == 6)
        _D = Damping;
}

void CartesianSpringDamper::setStiffnessDamping(const yarp::sig::Matrix &Stiffness, const yarp::sig::Matrix &Damping)
{
    setStiffness(Stiffness);
    setDamping(Damping);
}

yarp::sig::Matrix CartesianSpringDamper::getStiffness()
{
    return _K;
}

yarp::sig::Matrix CartesianSpringDamper::getDamping(){
    return _D;
}

void CartesianSpringDamper::getStiffnessDamping(yarp::sig::Matrix &Stiffness, yarp::sig::Matrix &Damping)
{
    Stiffness = getStiffness();
    Damping = getDamping();
}

void CartesianSpringDamper::setReference(const yarp::sig::Matrix& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist.zero();
    this->update_b();
}

void CartesianSpringDamper::setReference(const yarp::sig::Matrix &desiredPose,
                                                       const yarp::sig::Vector &desiredTwist)
{
    assert(desiredTwist.size() == 6);
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist = desiredTwist;
    this->update_b();
}

const yarp::sig::Matrix CartesianSpringDamper::getReference() const {
    return _desiredPose;
}

void CartesianSpringDamper::getReference(yarp::sig::Matrix &desiredPose,
                                                       yarp::sig::Vector &desiredTwist) const
{
    desiredPose = _desiredPose;
    desiredTwist = _desiredTwist;
}

const yarp::sig::Matrix CartesianSpringDamper::getActualPose() const
{
    return _actualPose;
}

const KDL::Frame CartesianSpringDamper::getActualPoseKDL() const
{
    KDL::Frame actualPoseKDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(_actualPose, actualPoseKDL);
    return actualPoseKDL;
}

const std::string CartesianSpringDamper::getDistalLink() const
{
    return _distal_link;
}

const std::string CartesianSpringDamper::getBaseLink() const
{
    return _base_link;
}

const bool CartesianSpringDamper::baseLinkIsWorld() const
{
    return _base_link_is_world;
}

yarp::sig::Vector CartesianSpringDamper::getSpringForce()
{
    return _K*yarp::math::cat(positionError, -1.0*orientationError);
}

yarp::sig::Vector CartesianSpringDamper::getDamperForce()
{
    return _D*yarp::math::cat(linearVelocityError, orientationVelocityError);
}

void CartesianSpringDamper::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);



    yarp::sig::Vector qdot = _robot.iDyn3_model.getDAng();
    yarp::sig::Vector xdot = _J*qdot;
    linearVelocityError = _desiredTwist.subVector(0,2) - xdot.subVector(0,2);
    orientationVelocityError = _desiredTwist.subVector(3,5) - xdot.subVector(3,5);

    yarp::sig::Vector F = getDamperForce() + getSpringForce();

    _b = _A*_J.transposed()*F;
}

bool CartesianSpringDamper::isCartesianSpringDamper(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<CartesianSpringDamper>(task);
}

CartesianSpringDamper::Ptr CartesianSpringDamper::asCartesianSpringDamper(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<CartesianSpringDamper>(task);
}

void CartesianSpringDamper::useInertiaMatrix(const bool use)
{
    _use_inertia_matrix = use;
}
