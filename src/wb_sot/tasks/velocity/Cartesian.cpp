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

#include <wb_sot/tasks/velocity/Cartesian.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace wb_sot::tasks::velocity;
using namespace yarp::math;

Cartesian::Cartesian(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    orientationErrorGain(1.0)
{
    this->_base_link_index = robot.coman_iDyn3.getLinkIndex(_base_link);
    assert(this->_base_link_index >= 0);
    this->_distal_link_index = robot.coman_iDyn3.getLinkIndex(_distal_link);
    assert(this->_distal_link_index > 0);

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

    if(_base_link_index == 0)
        assert(_robot.coman_iDyn3.getJacobian(_distal_link_index,_A));
    else
        assert(_robot.coman_iDyn3.getRelativeJacobian(_distal_link_index,
                                                      _base_link_index,
                                                      _A, true));

    _A = _A.removeCols(0,6);    // removing unactuated joints (floating base)

    if(_base_link_index == 0)
        _actualPose = _robot.coman_iDyn3.getPosition(_distal_link_index);
    else
        _actualPose = _robot.coman_iDyn3.getPosition(_base_link_index, _distal_link_index);

    if(_desiredPose.rows() == 0) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.resize(_A.rows(), 0.0);
    }

    this->update_b();

    /**********************************************************************/
}

void Cartesian::setReference(const yarp::sig::Matrix& desiredPose) {
    _desiredPose = desiredPose;
    this->update_b();
}

void Cartesian::setOrientationErrorGain(const double &orientationErrorGain)
{
    this->orientationErrorGain = orientationErrorGain;
}

void Cartesian::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);

    _b = yarp::math::cat(positionError, -orientationErrorGain*orientationError);
}
