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

#include <OpenSoT/tasks/force/CoM.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>
#include <iCub/iDynTree/yarp_kdl.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>

using namespace OpenSoT::tasks::force;
using namespace yarp::math;

#define LAMBDA_THS 1E-12

CoM::CoM(   const yarp::sig::Vector& x,
            iDynUtils &robot) :
    Task("CoM", x.size()), _robot(robot),
    _desiredPosition(3,0.0), _desiredVelocity(3,0.0), _desiredAcceleration(3,0.0),
    _actualPosition(3,0.0), _actualVelocity(3, 0.0),
    positionError(3, 0.0), velocityError(3, 0.0),
    _g(3, 0.0),
    _lambda2(1.0)
{
    _g(2) = -9.81;

    _desiredPosition = _robot.iDyn3_model.getCOM();
    _desiredVelocity = _robot.iDyn3_model.getVelCOM();

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);



    _W.resize(3,3);
    _W.eye();
    _lambda = 1.0;

    _hessianType = HST_SEMIDEF;
}

CoM::~CoM()
{
}

void CoM::_update(const yarp::sig::Vector &x)
{

    /************************* COMPUTING TASK *****************************/
    _actualPosition = _robot.iDyn3_model.getCOM();
    _actualVelocity = _robot.iDyn3_model.getVelCOM();

    this->update_b();

    this->_desiredVelocity.zero();
    this->_desiredAcceleration.zero();

    /**
      * Now I have to compute the Jacobian of the task that in this case is the matrix W
      * that maps the Fcom to the Fd
    **/
    // First I have to know which are the contacts
    std::vector<std::string> ft_in_contact;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(_robot.getForceTorqueFrameNames(),
               std::vector<std::string>{std::begin(_robot.getLinksInContact()),
                                        std::end(_robot.getLinksInContact())},
                                        _robot,
                                        ft_in_contact);
    yarp::sig::Matrix O(3, 3*ft_in_contact.size()); O.zero();
    _A = yarp::math::cat(computeW(ft_in_contact), O);
    /**********************************************************************/
}

void CoM::setReference(const yarp::sig::Vector& desiredPosition)
{
    assert(desiredPosition.size() == 3);

	_desiredPosition = desiredPosition;
    _desiredVelocity.zero();
    _desiredAcceleration.zero();
    this->update_b();
}

void OpenSoT::tasks::force::CoM::setReference(const yarp::sig::Vector &desiredPosition,
                                                 const yarp::sig::Vector &desiredVelocity)
{
    assert(desiredPosition.size() == 3);
    assert(desiredVelocity.size() == 3);

    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    _desiredAcceleration.zero();
    this->update_b();
}

void OpenSoT::tasks::force::CoM::setReference(const yarp::sig::Vector &desiredPosition,
                                              const yarp::sig::Vector &desiredVelocity,
                                              const yarp::sig::Vector &desiredAcceleration)
{
    assert(desiredPosition.size() == 3);
    assert(desiredVelocity.size() == 3);
    assert(desiredAcceleration.size() == 3);

    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    _desiredAcceleration = desiredAcceleration;
    this->update_b();
}

yarp::sig::Matrix OpenSoT::tasks::force::CoM::computeW(const std::vector<std::string> &ft_in_contact)
{
    yarp::sig::Matrix W(3, 3*ft_in_contact.size());

    yarp::sig::Matrix Pi(3,3);
    yarp::sig::Vector pi(3,0.0);
    yarp::sig::Matrix I(3,3); I.eye();
    yarp::sig::Matrix world_T_ft(4,4);
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i){
//        world_T_ft = _robot.iDyn3_model.getPosition(
//                    _robot.iDyn3_model.getLinkIndex(ft_in_contact[i]));
//        W.setSubmatrix(I*world_T_ft.submatrix(0,2,0,2), 0, i*3);
        W.setSubmatrix(I, 0, i*I.rows());


//        pi = -1.0*_actualPosition + world_T_ft.getCol(3).subVector(0,2);

//        Pi.zero();
//        Pi(0,1) = -pi(2); Pi(0,2) = -pi(1);
//        Pi(1,0) =  pi(2); Pi(1,2) = -pi(0);
//        Pi(2,0) = -pi(1); Pi(2,1) =  pi(0);
//        W.setSubmatrix(Pi, 3, i*3);
    }

    return W;
}

yarp::sig::Vector CoM::getReference() const
{
    return _desiredPosition;
}

void OpenSoT::tasks::force::CoM::getReference(yarp::sig::Vector &desiredPosition, yarp::sig::Vector &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}

yarp::sig::Vector CoM::getActualPosition() const
{
    return _actualPosition;
}

yarp::sig::Vector CoM::getActualVelocity() const
{
    return _actualVelocity;
}

std::string OpenSoT::tasks::force::CoM::getBaseLink()
{
    return BASE_LINK_COM;
}

std::string OpenSoT::tasks::force::CoM::getDistalLink()
{
    return DISTAL_LINK_COM;
}

void CoM::update_b()
{
    yarp::sig::Vector acceleration_ref = _desiredAcceleration +
            _lambda2*getVelocityError() +_lambda*getError();

    yarp::sig::Matrix M(6+_robot.iDyn3_model.getNrOfDOFs(), 6+_robot.iDyn3_model.getNrOfDOFs());
    _robot.iDyn3_model.getFloatingBaseMassMatrix(M);
    double m = M(0,0);

    _b = m*(acceleration_ref -_g);
}

void OpenSoT::tasks::force::CoM::setLambda(double lambda, double lambda2)
{
    if(lambda >= 0.0)
        this->_lambda = lambda;

    if(lambda2 >= 0.0)
        this->_lambda2 = lambda2;

    if(lambda >= 0.0 || lambda2 >= 0.0)
        this->update_b();
}

yarp::sig::Vector OpenSoT::tasks::force::CoM::getVelocityError()
{
    velocityError = _desiredVelocity - _actualVelocity;
    return velocityError;
}

yarp::sig::Vector OpenSoT::tasks::force::CoM::getError()
{
    positionError =  _desiredPosition - _actualPosition;
    return positionError;
}
