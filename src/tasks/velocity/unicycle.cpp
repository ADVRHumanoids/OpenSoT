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

#include <OpenSoT/tasks/velocity/unicycle.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

#define LAMBDA_THS 1E-12

unicycle::unicycle(std::string task_id,
                     const Eigen::VectorXd& x,
                     XBot::ModelInterface &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _orientationErrorGain(1.0), _is_initialized(false),
    _error(6)
{
    _error.setZero(6);

    _desiredTwist.setZero(6);

    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);

    if(!this->_base_link_is_world) {
        this->_base_link_index = _robot.getLinkID(_base_link);
        //assert(this->_base_link_index >= 0);
    }

    this->_distal_link_index = _robot.getLinkID(_distal_link);
    //assert(this->_distal_link_index >= 0);

    if(!this->_base_link_is_world)
        assert(this->_distal_link_index != _base_link_index);

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

    _W.setIdentity(_A.rows(), _A.rows());

    _hessianType = HST_SEMIDEF;
}

unicycle::~unicycle()
{
}

void unicycle::_update(const Eigen::VectorXd &x) {

    /************************* COMPUTING TASK *****************************/

    if(_base_link_is_world)
        _robot.getJacobian(_distal_link,_A);
    else
        _robot.getRelativeJacobian(_distal_link, _base_link, _A);

    if(_base_link_is_world)
        _robot.getPose(_distal_link, _actualPose);
    else
        _robot.getPose(_distal_link, _base_link, _actualPose);

    if(!_is_initialized) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.setZero(_A.rows());
        _is_initialized = true;
    }

    this->update_b();

    this->_desiredTwist.setZero(6);

    /**********************************************************************/
}

const Eigen::MatrixXd unicycle::getActualPose() const
{
    return _actualPose.matrix();
}

const void unicycle::getActualPose(KDL::Frame& actual_pose) const
{
    actual_pose.p.x(_actualPose(0,3));
    actual_pose.p.y(_actualPose(1,3));
    actual_pose.p.z(_actualPose(2,3));
    actual_pose.M(0,0) = _actualPose(0,0); actual_pose.M(0,1) = _actualPose(0,1); actual_pose.M(0,2) = _actualPose(0,2);
    actual_pose.M(1,0) = _actualPose(1,0); actual_pose.M(1,1) = _actualPose(1,1); actual_pose.M(1,2) = _actualPose(1,2);
    actual_pose.M(2,0) = _actualPose(2,0); actual_pose.M(2,1) = _actualPose(2,1); actual_pose.M(2,2) = _actualPose(2,2);
}

void unicycle::setOrientationErrorGain(const double &orientationErrorGain)
{
    this->_orientationErrorGain = orientationErrorGain;
}

const double OpenSoT::tasks::velocity::unicycle::getOrientationErrorGain() const
{
    return _orientationErrorGain;
}

const std::string OpenSoT::tasks::velocity::unicycle::getDistalLink() const
{
    return _distal_link;
}

const std::string OpenSoT::tasks::velocity::unicycle::getBaseLink() const
{
    return _base_link;
}

const bool OpenSoT::tasks::velocity::unicycle::baseLinkIsWorld() const
{
    return _base_link_is_world;
}

void OpenSoT::tasks::velocity::unicycle::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

const Eigen::VectorXd OpenSoT::tasks::velocity::unicycle::getError() const
{
    return _error;
}

void unicycle::update_b() {
    cartesian_utils::computeCartesianError(_actualPose.matrix(), _desiredPose.matrix(),
                                           positionError, orientationError);

    _error<<positionError,-_orientationErrorGain*orientationError;
    _b = _desiredTwist + _lambda*_error;
}

bool unicycle::setBaseLink(const std::string& base_link)
{
    if(base_link.compare(_base_link) == 0)
        return true;

    if(base_link.compare("world") == 0)
        _robot.getPose(_base_link, _tmpMatrix);
    else if(_base_link.compare("world") == 0){
        _robot.getPose(base_link, _tmpMatrix2);
        _tmpMatrix = _tmpMatrix2.inverse();
    }
    else if(_robot.getLinkID(base_link) == -1)
        return false;
    else
        _robot.getPose(_base_link, base_link, _tmpMatrix);

    _base_link = base_link;
    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);
    _tmpMatrix2 = _tmpMatrix*_desiredPose;
    _desiredPose = _tmpMatrix2;

    return true;
}

bool OpenSoT::tasks::velocity::unicycle::isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::unicycle>(task);
}

OpenSoT::tasks::velocity::unicycle::Ptr OpenSoT::tasks::velocity::unicycle::asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::unicycle>(task);
}
