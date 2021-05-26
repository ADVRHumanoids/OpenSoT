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

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>


using namespace OpenSoT::tasks::velocity;


#define LAMBDA_THS 1E-12

CoM::CoM(   const Eigen::VectorXd& x,
            XBot::ModelInterface &robot,
            const std::string& id
        ) :
    Task(id, x.size()), _robot(robot), _base_link(BASE_LINK_COM), _distal_link(DISTAL_LINK_COM)
{
    _desiredPosition.setZero();
    _actualPosition.setZero();
    _positionError.setZero();
    _desiredVelocity.setZero();
    _desiredVelocityRef = _desiredVelocity;

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);


    /* initializing to zero error */
    _desiredPosition = _actualPosition;
    _b.setZero(3);
    _positionError = _b;

    _W.resize(3,3);
    _W.setIdentity(3,3);

    _hessianType = HST_SEMIDEF;
}

CoM::~CoM()
{
}

void CoM::_update(const Eigen::VectorXd &x)
{

    /************************* COMPUTING TASK *****************************/
    _desiredVelocityRef = _desiredVelocity;

    _robot.getCOM(_actualPosition);

    _robot.getCOMJacobian(_A);

    this->update_b();

    this->_desiredVelocity.setZero(3);

    /**********************************************************************/
}

void CoM::setReference(const KDL::Vector& desiredPosition,
                  const KDL::Vector& desiredVelocity)
{
    _desiredPosition(0) = desiredPosition.x();
    _desiredPosition(1) = desiredPosition.y();
    _desiredPosition(2) = desiredPosition.z();

    _desiredVelocity(0) = desiredVelocity.x();
    _desiredVelocity(1) = desiredVelocity.y();
    _desiredVelocity(2) = desiredVelocity.z();

    _desiredVelocityRef = _desiredVelocity;
    this->update_b();
}

void CoM::setReference(const KDL::Vector& desiredPosition)
{
    _desiredPosition(0) = desiredPosition.x();
    _desiredPosition(1) = desiredPosition.y();
    _desiredPosition(2) = desiredPosition.z();

    _desiredVelocity.setZero(3);
    _desiredVelocityRef = _desiredVelocity;
    this->update_b();
}

void CoM::setReference(const Eigen::Vector3d& desiredPosition)
{
	_desiredPosition = desiredPosition;
    _desiredVelocity.setZero(3);
    _desiredVelocityRef = _desiredVelocity;
    this->update_b();
}

void OpenSoT::tasks::velocity::CoM::setReference(const Eigen::Vector3d &desiredPosition,
                                                 const Eigen::Vector3d &desiredVelocity)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    _desiredVelocityRef = _desiredVelocity;
    this->update_b();
}

const Eigen::Vector3d& CoM::getReference() const
{
    return _desiredPosition;
}

void OpenSoT::tasks::velocity::CoM::getReference(Eigen::Vector3d &desiredPosition, Eigen::Vector3d &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}

const Eigen::Vector3d& CoM::getActualPosition() const
{
    return _actualPosition;
}

const std::string& OpenSoT::tasks::velocity::CoM::getBaseLink() const
{
    return _base_link;
}

const std::string& OpenSoT::tasks::velocity::CoM::getDistalLink() const
{
    return _distal_link;
}

void CoM::update_b()
{
    _positionError = _desiredPosition - _actualPosition;
    _b = _desiredVelocity + _lambda*_positionError;
}

void OpenSoT::tasks::velocity::CoM::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

const Eigen::Vector3d& OpenSoT::tasks::velocity::CoM::getError() const
{
    return _positionError;
}

OpenSoT::tasks::velocity::CoM::Ptr OpenSoT::tasks::velocity::CoM::asCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
}


bool OpenSoT::tasks::velocity::CoM::isCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
}

void OpenSoT::tasks::velocity::CoM::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add(getTaskID() + "_position_err", _positionError);
    logger->add(_task_id + "_pos_ref", _desiredPosition);
    logger->add(_task_id + "_pos_actual", _actualPosition);
    logger->add(_task_id + "_desiredVelocityRef", _desiredVelocityRef);
}

bool OpenSoT::tasks::velocity::CoM::reset()
{
    _robot.getCOM(_actualPosition);

    _robot.getCOMJacobian(_A);

    this->_desiredVelocity.setZero(3);
    _desiredVelocityRef = _desiredVelocity;

    _desiredPosition = _actualPosition;

    this->update_b();

    return true;
}

const Eigen::Vector3d& OpenSoT::tasks::velocity::CoM::getCachedVelocityReference() const
{
    return _desiredVelocityRef;
}
