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
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

Cartesian::Cartesian(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    orientationErrorGain(1.0), _desiredVelocity(6, 0.0)
{
    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);

    if(!this->_base_link_is_world) {
        this->_base_link_index = robot.coman_iDyn3.getLinkIndex(_base_link);
        assert(this->_base_link_index >= 0);
    }

    this->_distal_link_index = robot.coman_iDyn3.getLinkIndex(_distal_link);
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
        assert(_robot.coman_iDyn3.getJacobian(_distal_link_index,_A));
        _A = _A.removeCols(0,6);    // removing unactuated joints (floating base)
    } else
        assert(_robot.coman_iDyn3.getRelativeJacobian(_distal_link_index,
                                                      _base_link_index,
                                                      _A, true));

    if(_base_link_is_world)
        _actualPose = _robot.coman_iDyn3.getPosition(_distal_link_index);
    else
        _actualPose = _robot.coman_iDyn3.getPosition(_base_link_index, _distal_link_index);

    if(_desiredPose.rows() == 0) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.resize(_A.rows(), 0.0);
    }

    this->update_b();

    this->_desiredVelocity.zero();

    /**********************************************************************/
}

void Cartesian::setReference(const yarp::sig::Matrix& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredVelocity.zero();
    this->update_b();
}

void OpenSoT::tasks::velocity::Cartesian::setReference(const yarp::sig::Matrix &desiredPose,
                                                       const yarp::sig::Vector &desiredVelocity)
{
    assert(desiredVelocity.size() == 6);
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredVelocity = desiredVelocity;
    this->update_b();
}

const yarp::sig::Matrix Cartesian::getReference() const {
    return _desiredPose;
}

void OpenSoT::tasks::velocity::Cartesian::getReference(const yarp::sig::Matrix &desiredPose,
                                                       const yarp::sig::Vector &desiredVelocity) const
{
    desiredPose = _desiredPose;
    desiredVelocity = _desiredVelocity;
}

const yarp::sig::Matrix Cartesian::getActualPose() const
{
    return _actualPose;
}

void Cartesian::setOrientationErrorGain(const double &orientationErrorGain)
{
    this->orientationErrorGain = orientationErrorGain;
}

const std::string OpenSoT::tasks::velocity::Cartesian::getDistalLink() const
{
    return _distal_link;
}

const std::string OpenSoT::tasks::velocity::Cartesian::getBaseLink() const
{
    return _base_link;
}

void OpenSoT::tasks::velocity::Cartesian::setLambda(double lambda)
{
    this->_lambda = lambda;
    this->update_b();
}

void Cartesian::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);

    _b = yarp::math::cat(positionError, -orientationErrorGain*orientationError) + desiredVelocity/_lambda;
}
