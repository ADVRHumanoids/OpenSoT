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
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

#define LAMBDA_THS 1E-12

Cartesian::Cartesian(std::string task_id,
                     const Eigen::VectorXd& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _orientationErrorGain(1.0)
{
    _desiredTwist.setZero(6);

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

    _W.setIdentity(_A.rows(), _A.rows());

    _hessianType = HST_SEMIDEF;
}

Cartesian::~Cartesian()
{
}

void Cartesian::_update(const Eigen::VectorXd &x) {

    /************************* COMPUTING TASK *****************************/

    if(_base_link_is_world) {
        bool res = _robot.getJacobian(_distal_link_index,_A);
        assert(res);
        _A = _A.block(0,6,_A.rows(),_x_size);
    } else{
        bool res = _robot.getRelativeJacobian(_distal_link_index,
                                                          _base_link_index,
                                                          _A, true);
        assert(res);
        Eigen::MatrixXd base_R_world = _robot.getPosition(_base_link_index).block(0,0,3,3).transpose();
        Eigen::MatrixXd Adj(6,6);
        Adj.setIdentity(6,6);
        Adj.block(0,0,3,3)<<base_R_world;
        Adj.block(3,3,3,3)<<base_R_world;
        _A = Adj*_A;
    }

    if(_base_link_is_world)
        _actualPose = _robot.getPosition(_distal_link_index);
    else
        _actualPose = _robot.getPosition(_base_link_index, _distal_link_index);

    if(_desiredPose.rows() == 0) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.setZero(_A.rows());
    }

    this->update_b();

    this->_desiredTwist.setZero(6);

    /**********************************************************************/
}

void Cartesian::setReference(const Eigen::MatrixXd& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist.setZero(6);
    this->update_b();
}

void Cartesian::setReference(const KDL::Frame& desiredPose)
{
    _desiredPose(0,3) = desiredPose.p.x();
    _desiredPose(1,3) = desiredPose.p.y();
    _desiredPose(2,3) = desiredPose.p.z();
    _desiredPose(0,0) = desiredPose.M(0,0); _desiredPose(0,1) = desiredPose.M(0,1); _desiredPose(0,2) = desiredPose.M(0,2);
    _desiredPose(1,0) = desiredPose.M(1,0); _desiredPose(1,1) = desiredPose.M(1,1); _desiredPose(1,2) = desiredPose.M(1,2);
    _desiredPose(2,0) = desiredPose.M(2,0); _desiredPose(2,1) = desiredPose.M(2,1); _desiredPose(2,2) = desiredPose.M(2,2);

    _desiredTwist.setZero(6);
    this->update_b();
}

void Cartesian::setReference(const Eigen::MatrixXd &desiredPose,
                             const Eigen::VectorXd &desiredTwist)
{
    assert(desiredTwist.rows() == 6);
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist = desiredTwist;
    this->update_b();
}

void Cartesian::setReference(const KDL::Frame& desiredPose,
                  const KDL::Twist& desiredTwist)
{
    _desiredPose(0,3) = desiredPose.p.x();
    _desiredPose(1,3) = desiredPose.p.y();
    _desiredPose(2,3) = desiredPose.p.z();
    _desiredPose(0,0) = desiredPose.M(0,0); _desiredPose(0,1) = desiredPose.M(0,1); _desiredPose(0,2) = desiredPose.M(0,2);
    _desiredPose(1,0) = desiredPose.M(1,0); _desiredPose(1,1) = desiredPose.M(1,1); _desiredPose(1,2) = desiredPose.M(1,2);
    _desiredPose(2,0) = desiredPose.M(2,0); _desiredPose(2,1) = desiredPose.M(2,1); _desiredPose(2,2) = desiredPose.M(2,2);

    _desiredTwist(0) = desiredTwist[0]; _desiredTwist(1) = desiredTwist[1]; _desiredTwist(2) = desiredTwist[2];
    _desiredTwist(3) = desiredTwist[3]; _desiredTwist(4) = desiredTwist[4]; _desiredTwist(5) = desiredTwist[5];

    this->update_b();
}

const Eigen::MatrixXd Cartesian::getReference() const {
    return _desiredPose;
}


void OpenSoT::tasks::velocity::Cartesian::getReference(Eigen::MatrixXd &desiredPose,
                                                       Eigen::VectorXd &desiredTwist) const
{
    desiredPose = _desiredPose;
    desiredTwist = _desiredTwist;
}

void Cartesian::getReference(KDL::Frame& desiredPose,
                  KDL::Vector& desiredTwist) const
{
    desiredPose.p.x(_desiredPose(0,3));
    desiredPose.p.y(_desiredPose(1,3));
    desiredPose.p.z(_desiredPose(2,3));
    desiredPose.M(0,0) = _desiredPose(0,0); desiredPose.M(0,1) = _desiredPose(0,1); desiredPose.M(0,2) = _desiredPose(0,2);
    desiredPose.M(0,0) = _desiredPose(1,0); desiredPose.M(1,1) = _desiredPose(1,1); desiredPose.M(1,2) = _desiredPose(1,2);
    desiredPose.M(0,0) = _desiredPose(2,0); desiredPose.M(2,1) = _desiredPose(2,1); desiredPose.M(2,2) = _desiredPose(2,2);

    desiredTwist[0] = _desiredTwist(0); desiredTwist[1] = _desiredTwist(1); desiredTwist[2] = _desiredTwist(2);
    desiredTwist[3] = _desiredTwist(3); desiredTwist[4] = _desiredTwist(4); desiredTwist[5] = _desiredTwist(5);
}

const Eigen::MatrixXd Cartesian::getActualPose() const
{
    return _actualPose;
}

const void Cartesian::getActualPose(KDL::Frame& actual_pose) const
{
    actual_pose.p.x(_actualPose(0,3));
    actual_pose.p.y(_actualPose(1,3));
    actual_pose.p.z(_actualPose(2,3));
    actual_pose.M(0,0) = _actualPose(0,0); actual_pose.M(0,1) = _actualPose(0,1); actual_pose.M(0,2) = _actualPose(0,2);
    actual_pose.M(0,0) = _actualPose(1,0); actual_pose.M(1,1) = _actualPose(1,1); actual_pose.M(1,2) = _actualPose(1,2);
    actual_pose.M(0,0) = _actualPose(2,0); actual_pose.M(2,1) = _actualPose(2,1); actual_pose.M(2,2) = _actualPose(2,2);
}

const KDL::Frame Cartesian::getActualPoseKDL() const
{
    KDL::Frame actualPoseKDL;
    getActualPose(actualPoseKDL);
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

Eigen::VectorXd OpenSoT::tasks::velocity::Cartesian::getError()
{
    Eigen::VectorXd tmp(6);
    tmp<<positionError,-_orientationErrorGain*orientationError;
    return tmp;
}

void Cartesian::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);

    _b = _desiredTwist + _lambda*this->getError();
}

bool OpenSoT::tasks::velocity::Cartesian::isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}

OpenSoT::tasks::velocity::Cartesian::Ptr OpenSoT::tasks::velocity::Cartesian::asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}
