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

#include <OpenSoT/tasks/velocity/affine/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>


using namespace OpenSoT::tasks::velocity::affine;


#define LAMBDA_THS 1E-12

CoM::CoM(   const Eigen::VectorXd& x,
            XBot::ModelInterface &robot) :
    Task("CoM", x.size()), _robot(robot)
{
    _desiredPosition.setZero(3);
    _actualPosition.setZero(3);
    _positionError.setZero(3);
    _desiredVelocity.setZero(3);

    _qdot = AffineHelper::Identity(x.size());

    /* first update. Setting desired pose equal to the actual pose */
    _robot.getCOM(_actualPosition);


    /* initializing to zero error */
    _desiredPosition = _actualPosition;
    __b.setZero(3);
    _positionError = __b;

    this->_update(x);

    _W.resize(__A.rows(), __A.rows());
    _W.setIdentity(__A.rows(), __A.rows());

    _hessianType = HST_SEMIDEF;
}

CoM::CoM( XBot::ModelInterface &robot,
          const AffineHelper& qdot ):
    Task("CoM", qdot.getInputSize()), _robot(robot), _qdot(qdot)
{
    _desiredPosition.setZero(3);
    _actualPosition.setZero(3);
    _positionError.setZero(3);
    _desiredVelocity.setZero(3);

    /* first update. Setting desired pose equal to the actual pose */
    _robot.getCOM(_actualPosition);


    /* initializing to zero error */
    _desiredPosition = _actualPosition;
    __b.setZero(3);
    _positionError = __b;

    this->_update(Eigen::VectorXd(1));

    _W.resize(__A.rows(), __A.rows());
    _W.setIdentity(__A.rows(), __A.rows());


    _hessianType = HST_SEMIDEF;
}

CoM::~CoM()
{
}

void CoM::_update(const Eigen::VectorXd &x)
{

    /************************* COMPUTING TASK *****************************/

    _robot.getCOM(_actualPosition);

    _robot.getCOMJacobian(__A);

    this->update_b();

    //HERE __A and __b are updated
    _com_task = __A*_qdot;
    _com_task = _com_task - __b;

    _A = _com_task.getM();
    _b = -_com_task.getq();
    //

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

    this->update_b();
}

void CoM::setReference(const KDL::Vector& desiredPosition)
{
    _desiredPosition(0) = desiredPosition.x();
    _desiredPosition(1) = desiredPosition.y();
    _desiredPosition(2) = desiredPosition.z();

    _desiredVelocity.setZero(3);
    this->update_b();
}

void CoM::setReference(const Eigen::Vector3d& desiredPosition)
{
	_desiredPosition = desiredPosition;
    _desiredVelocity.setZero(3);
    this->update_b();
}

void CoM::setReference(const Eigen::Vector3d &desiredPosition,
                                                 const Eigen::Vector3d &desiredVelocity)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    this->update_b();
}

Eigen::VectorXd CoM::getReference() const
{
    return _desiredPosition;
}

void CoM::getReference(Eigen::Vector3d &desiredPosition, Eigen::Vector3d &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}

Eigen::Vector3d CoM::getActualPosition() const
{
    return _actualPosition;
}

std::string CoM::getBaseLink()
{
    return BASE_LINK_COM;
}

std::string CoM::getDistalLink()
{
    return DISTAL_LINK_COM;
}

void CoM::update_b()
{
    _positionError = _desiredPosition - _actualPosition;
    __b = _desiredVelocity + _lambda*_positionError;
}

void CoM::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

Eigen::Vector3d CoM::getError()
{
    return _positionError;
}

CoM::Ptr CoM::asCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<CoM>(task);
}


bool CoM::isCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<CoM>(task);
}

void CoM::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(getTaskID() + "_position_err", _positionError);
}
