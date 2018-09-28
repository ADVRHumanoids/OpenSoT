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
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

#define LAMBDA_THS 1E-12

Cartesian::Cartesian(std::string task_id,
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

Cartesian::~Cartesian()
{
}

void Cartesian::_update(const Eigen::VectorXd &x) {

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

void Cartesian::setReference(const Eigen::MatrixXd& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);
    _desiredPose.matrix() = desiredPose;
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

    _desiredPose.matrix() = desiredPose;
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
    return _desiredPose.matrix();
}

const void Cartesian::getReference(KDL::Frame& desiredPose) const {
    desiredPose.p.x(_desiredPose(0,3));
    desiredPose.p.y(_desiredPose(1,3));
    desiredPose.p.z(_desiredPose(2,3));
    desiredPose.M(0,0) = _desiredPose(0,0); desiredPose.M(0,1) = _desiredPose(0,1); desiredPose.M(0,2) = _desiredPose(0,2);
    desiredPose.M(1,0) = _desiredPose(1,0); desiredPose.M(1,1) = _desiredPose(1,1); desiredPose.M(1,2) = _desiredPose(1,2);
    desiredPose.M(2,0) = _desiredPose(2,0); desiredPose.M(2,1) = _desiredPose(2,1); desiredPose.M(2,2) = _desiredPose(2,2);
}

void OpenSoT::tasks::velocity::Cartesian::getReference(Eigen::MatrixXd &desiredPose,
                                                       Eigen::VectorXd &desiredTwist) const
{
    desiredPose = _desiredPose.matrix();
    desiredTwist = _desiredTwist;
}

void Cartesian::getReference(KDL::Frame& desiredPose,
                  KDL::Vector& desiredTwist) const
{
    desiredPose.p.x(_desiredPose(0,3));
    desiredPose.p.y(_desiredPose(1,3));
    desiredPose.p.z(_desiredPose(2,3));
    desiredPose.M(0,0) = _desiredPose(0,0); desiredPose.M(0,1) = _desiredPose(0,1); desiredPose.M(0,2) = _desiredPose(0,2);
    desiredPose.M(1,0) = _desiredPose(1,0); desiredPose.M(1,1) = _desiredPose(1,1); desiredPose.M(1,2) = _desiredPose(1,2);
    desiredPose.M(2,0) = _desiredPose(2,0); desiredPose.M(2,1) = _desiredPose(2,1); desiredPose.M(2,2) = _desiredPose(2,2);

    desiredTwist[0] = _desiredTwist(0); desiredTwist[1] = _desiredTwist(1); desiredTwist[2] = _desiredTwist(2);
    desiredTwist[3] = _desiredTwist(3); desiredTwist[4] = _desiredTwist(4); desiredTwist[5] = _desiredTwist(5);
}

const Eigen::MatrixXd Cartesian::getActualPose() const
{
    return _actualPose.matrix();
}

const void Cartesian::getActualPose(KDL::Frame& actual_pose) const
{
    actual_pose.p.x(_actualPose(0,3));
    actual_pose.p.y(_actualPose(1,3));
    actual_pose.p.z(_actualPose(2,3));
    actual_pose.M(0,0) = _actualPose(0,0); actual_pose.M(0,1) = _actualPose(0,1); actual_pose.M(0,2) = _actualPose(0,2);
    actual_pose.M(1,0) = _actualPose(1,0); actual_pose.M(1,1) = _actualPose(1,1); actual_pose.M(1,2) = _actualPose(1,2);
    actual_pose.M(2,0) = _actualPose(2,0); actual_pose.M(2,1) = _actualPose(2,1); actual_pose.M(2,2) = _actualPose(2,2);
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

const Eigen::Vector6d OpenSoT::tasks::velocity::Cartesian::getError() const
{
    return _error;
}

void Cartesian::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);

    _error<<positionError,-_orientationErrorGain*orientationError;
    _b = _desiredTwist + _lambda*_error;
}

bool Cartesian::setBaseLink(const std::string& base_link)
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

bool OpenSoT::tasks::velocity::Cartesian::setDistalLink(const std::string& distal_link)
{
    if(distal_link.compare(_distal_link) == 0){
        return true;
    }

    if(distal_link.compare("world") == 0){
        std::cerr << "Error in " << __func__ << ": cannot pass world as distal link." << std::endl;
        return false;
    }
    
    Eigen::Affine3d base_T_distal;
    
    if(!_robot.getPose(distal_link, _base_link, base_T_distal)){
        std::cerr << "Error in " << __func__ << ": base link -> distal link transform cannot be obtained." << std::endl;
        return false;
    }
    
    _distal_link = distal_link;
    
    setReference(base_T_distal.matrix());

    return true;
}


bool OpenSoT::tasks::velocity::Cartesian::isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}

OpenSoT::tasks::velocity::Cartesian::Ptr OpenSoT::tasks::velocity::Cartesian::asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task);
}

void Cartesian::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(_task_id + "_error", _error);
    logger->add(_task_id + "_pos_ref", _desiredPose.translation());
    logger->add(_task_id + "_pos_actual", _actualPose.translation());
    logger->add(_task_id + "_pose_ref", _desiredPose.matrix());
}

bool Cartesian::reset()
{
    _is_initialized = false;
    _update(Eigen::VectorXd(1));

    return true;
}
