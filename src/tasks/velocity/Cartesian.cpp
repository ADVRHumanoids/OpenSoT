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

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

#define LAMBDA_THS 1E-12

Cartesian::Cartesian(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _orientationErrorGain(1.0), _desiredTwist(6, 0.0)
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

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

    _W.resize(_A.rows(), _A.rows());
    _W.eye();


    _hessianType = HST_SEMIDEF;
}

Cartesian::~Cartesian()
{
}

void Cartesian::_update(const yarp::sig::Vector &x) {

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

void Cartesian::setReference(const yarp::sig::Matrix& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist.zero();
    this->update_b();
}

void OpenSoT::tasks::velocity::Cartesian::setReference(const yarp::sig::Matrix &desiredPose,
                                                       const yarp::sig::Vector &desiredTwist)
{
    assert(desiredTwist.size() == 6);
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist = desiredTwist;
    this->update_b();
}

const yarp::sig::Matrix Cartesian::getReference() const {
    return _desiredPose;
}

void OpenSoT::tasks::velocity::Cartesian::getReference(yarp::sig::Matrix &desiredPose,
                                                       yarp::sig::Vector &desiredTwist) const
{
    desiredPose = _desiredPose;
    desiredTwist = _desiredTwist;
}

const yarp::sig::Matrix Cartesian::getActualPose() const
{
    return _actualPose;
}

const KDL::Frame Cartesian::getActualPoseKDL() const
{
    KDL::Frame actualPoseKDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(_actualPose, actualPoseKDL);
    return actualPoseKDL;
}


void Cartesian::setOrientationErrorGain(const double &orientationErrorGain)
{
    this->_orientationErrorGain = orientationErrorGain;
}

const double OpenSoT::tasks::velocity::Cartesian::getOrientationErrorGain() const
{
    return _orientationErrorGain;
}

const std::string OpenSoT::tasks::velocity::Cartesian::getDistalLink() const
{
    return _distal_link;
}

const std::string OpenSoT::tasks::velocity::Cartesian::getBaseLink() const
{
    return _base_link;
}

const bool OpenSoT::tasks::velocity::Cartesian::baseLinkIsWorld() const
{
    return _base_link_is_world;
}

void OpenSoT::tasks::velocity::Cartesian::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

yarp::sig::Vector OpenSoT::tasks::velocity::Cartesian::getError()
{
    return yarp::math::cat(positionError, -_orientationErrorGain*orientationError);
}

void Cartesian::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);

    _b = _desiredTwist + _lambda*this->getError();
}

bool OpenSoT::tasks::velocity::Cartesian::isCartesian(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}

OpenSoT::tasks::velocity::Cartesian::Ptr OpenSoT::tasks::velocity::Cartesian::asCartesian(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}
