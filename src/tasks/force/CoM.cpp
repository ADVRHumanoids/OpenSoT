/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo Hoffman, Alessio Rocchi
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

#include <OpenSoT/tasks/force/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>


using namespace OpenSoT::tasks::force;

#define LAMBDA_THS 1E-12

CoM::CoM( const Eigen::VectorXd& x, 
          std::vector<std::string>& links_in_contact,
          XBot::ModelInterface &robot) :
    Task("CoM", x.rows()), _robot(robot),
    _desiredPosition(), _desiredVelocity(), _desiredAcceleration(),
    _actualPosition(), _actualVelocity(), _actualAngularMomentum(),
    _desiredVariationAngularMomentum(), _desiredAngularMomentum(),
    positionError(), velocityError(), angularMomentumError(),
    _links_in_contact(links_in_contact),_I(), _O(), _P(), _T(),
    _g(),
    _lambda2(1.0), _lambdaAngularMomentum(1.0),
    A(3,6),B(3,6)
{
    A.setZero(3,6);
    B.setZero(3,6);

    _desiredPosition.setZero();
    _desiredVelocity.setZero();
    _desiredAcceleration.setZero();

    _desiredVariationAngularMomentum.setZero();
    _desiredAngularMomentum.setZero();

    _actualPosition.setZero();
    _actualVelocity.setZero();
    _actualAngularMomentum.setZero();

    positionError.setZero();
    velocityError.setZero();
    angularMomentumError.setZero();

    _centroidalMomentum.setZero();

    _P.setZero();
    _T.matrix().setZero();

    _g.setZero();
    _g(2) = -9.81;

    _robot.getCOM(_desiredPosition);
    _robot.getCOMVelocity(_desiredVelocity);
    _robot.getCentroidalMomentum(_centroidalMomentum);
    _desiredAngularMomentum = _centroidalMomentum.segment(3,3);
    /* first update. Setting desired pose equal to the actual pose */

    _W.resize(6,6);
    _W.setIdentity(6,6);
    _lambda = 1.0;


    _I.setIdentity();
    _O.setZero();


    
    OptvarHelper::VariableVector var;
    
    for(auto l : links_in_contact){
        var.emplace_back(l + "_wrench", 6);
    }
    
    OptvarHelper opt(var);
    
    _wrenches.setZero(x.size(), 0);
    
    for( auto v : opt.getAllVariables() ){
        _wrenches = _wrenches / v;
    }
    
    this->_update(x);
    _hessianType = HST_SEMIDEF;
}


CoM::CoM( std::vector<AffineHelper> wrenches, 
          std::vector<std::string>& links_in_contact,
          XBot::ModelInterface &robot) :
    Task("CoM", wrenches[0].getInputSize()), _robot(robot),
    _desiredPosition(), _desiredVelocity(), _desiredAcceleration(),
    _actualPosition(), _actualVelocity(), _actualAngularMomentum(),
    _desiredVariationAngularMomentum(), _desiredAngularMomentum(),
    positionError(), velocityError(), angularMomentumError(),
    _links_in_contact(links_in_contact),_I(), _O(), _P(), _T(),
    _g(),
    _lambda2(1.0), _lambdaAngularMomentum(1.0),
    A(3,6),B(3,6)
{
    A.setZero(3,6);
    B.setZero(3,6);

    _desiredPosition.setZero();
    _desiredVelocity.setZero();
    _desiredAcceleration.setZero();

    _desiredVariationAngularMomentum.setZero();
    _desiredAngularMomentum.setZero();

    _actualPosition.setZero();
    _actualVelocity.setZero();
    _actualAngularMomentum.setZero();

    positionError.setZero();
    velocityError.setZero();
    angularMomentumError.setZero();

    _centroidalMomentum.setZero();

    _P.setZero();
    _T.matrix().setZero();

    _g.setZero();
    _g(2) = -9.81;

    _robot.getCOM(_desiredPosition);
    _robot.getCOMVelocity(_desiredVelocity);
    _robot.getCentroidalMomentum(_centroidalMomentum);
    _desiredAngularMomentum = _centroidalMomentum.segment(3,3);
    /* first update. Setting desired pose equal to the actual pose */

    _W.resize(6,6);
    _W.setIdentity(6,6);
    _lambda = 1.0;


    _I.setIdentity();
    _O.setZero();

    
    _wrenches.setZero(wrenches[0].getInputSize(), 0);
    for( auto v : wrenches ){
        _wrenches = _wrenches / v;
    }
    
    _hessianType = HST_SEMIDEF;
    
    _update(Eigen::VectorXd());
}



CoM::~CoM()
{
}

std::vector<std::string> CoM::getLinksInContact()
{
    return _links_in_contact;
}

void CoM::setLinksInContact(const std::vector<std::string>& links_in_contact)
{
    _links_in_contact = links_in_contact;
}

void CoM::_update(const Eigen::VectorXd &x)
{

    /************************* COMPUTING TASK *****************************/
    _robot.getCOM(_actualPosition);
    _robot.getCOMVelocity(_actualVelocity);
    _robot.getCentroidalMomentum(_centroidalMomentum);
    _actualAngularMomentum = _centroidalMomentum.segment(3,3);

    this->update_b();

    this->_desiredVelocity.setZero(this->_desiredVelocity.rows());
    this->_desiredAcceleration.setZero(this->_desiredAcceleration.rows());
    //this->_desiredAngularMomentum.setZero(this->_desiredAngularMomentum.rows());
    this->_desiredVariationAngularMomentum.setZero(this->_desiredVariationAngularMomentum.rows());

    /**
      * Now I have to compute the Jacobian of the task that in this case is the matrix W
      * that maps the Fcom to the Fd
    **/
    _A = computeA(_links_in_contact);
    
    
    _com_task = _A*_wrenches - _b;
    
    _A = _com_task.getM();
    _b = -_com_task.getq();
    /**********************************************************************/
}

void CoM::setLinearReference(const Eigen::Vector3d& desiredPosition)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity.setZero(_desiredVelocity.rows());
    _desiredAcceleration.setZero(_desiredAcceleration.rows());
    this->update_b();
}

void OpenSoT::tasks::force::CoM::setLinearReference(const Eigen::Vector3d &desiredPosition,
                                                 const Eigen::Vector3d &desiredVelocity)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    _desiredAcceleration.setZero(_desiredAcceleration.rows());
    this->update_b();
}

void OpenSoT::tasks::force::CoM::setLinearReference(const Eigen::Vector3d &desiredPosition,
                                              const Eigen::Vector3d &desiredVelocity,
                                              const Eigen::Vector3d &desiredAcceleration)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    _desiredAcceleration = desiredAcceleration;
    this->update_b();
}

void CoM::setAngularReference(const Eigen::Vector3d& desiredAngularMomentum)
{
    _desiredAngularMomentum = desiredAngularMomentum;
    _desiredVariationAngularMomentum.setZero(_desiredVariationAngularMomentum.rows());
}

void CoM::setAngularReference(const Eigen::Vector3d& desiredAngularMomentum,
                         const Eigen::Vector3d& desiredVariationAngularMomentum)
{
    _desiredAngularMomentum = desiredAngularMomentum;
    _desiredVariationAngularMomentum = desiredVariationAngularMomentum;
}

Eigen::MatrixXd OpenSoT::tasks::force::CoM::computeA(const std::vector<std::string> &links_in_contact)
{
    int m = links_in_contact.size();

    Eigen::MatrixXd G(6, 6*m); G.setZero(6, 6*m);

    _P.setZero();

    _T.setIdentity();

    A.setZero(3,6);
    B.setZero(3,6);

    for(unsigned int i = 0; i < m; ++i){
        _robot.getPose(links_in_contact[i], _T);
        _T.matrix()(0,3) -= _actualPosition(0);
        _T.matrix()(1,3) -= _actualPosition(1);
        _T.matrix()(2,3) -= _actualPosition(2);

        _P(0,0) = 0.0;      _P(0,1) = -_T(2,3); _P(0,2) = _T(1,3);
        _P(1,0) = -_P(0,1); _P(1,1) = 0.0;      _P(1,2) = -_T(0,3);
        _P(2,0) = -_P(0,2); _P(2,1) = -_P(1,2); _P(2,2) = 0.0;


        A<<_I,_O;
        B<<_P,_I;

        G.block(0,i*6,6,6)<<A,
                            B;
    }

    return G;
}

Eigen::Vector3d CoM::getLinearReference() const
{
    return _desiredPosition;
}

void OpenSoT::tasks::force::CoM::getLinearReference(Eigen::Vector3d &desiredPosition, Eigen::Vector3d &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}

void CoM::getLinearReference(Eigen::Vector3d& desiredPosition,
                  Eigen::Vector3d& desiredVelocity,
                  Eigen::Vector3d& desiredAcceleration) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
    desiredAcceleration = _desiredAcceleration;
}

Eigen::Vector3d CoM::getAngularReference() const
{
    return _desiredAngularMomentum;
}

void CoM::getAngularReference(Eigen::Vector3d& desiredAngularMomentum,
                  Eigen::Vector3d& desiredVariationAngularMomentum) const
{
    desiredAngularMomentum = _desiredAngularMomentum;
    desiredVariationAngularMomentum = _desiredVariationAngularMomentum;
}

Eigen::Vector3d CoM::getActualPosition() const
{
    return _actualPosition;
}

Eigen::Vector3d CoM::getActualVelocity() const
{
    return _actualVelocity;
}

Eigen::Vector3d CoM::getActualAngularMomentum() const
{
    return _actualAngularMomentum;
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
    Eigen::Vector3d acceleration_ref = _desiredAcceleration +
            _lambda2*getVelocityError() +_lambda*getError();

    Eigen::Vector3d variationAngularMomentum_ref = _desiredVariationAngularMomentum +
            _lambdaAngularMomentum*getAngularMomentumError();


    acceleration_ref = _robot.getMass()*(acceleration_ref-_g);

    _b.setZero(6);
    _b<<acceleration_ref,
        variationAngularMomentum_ref;
}

void OpenSoT::tasks::force::CoM::setLambda(double lambda, double lambda2, double lambdaAngularMomentum)
{
    if(lambda >= 0.0)
        this->_lambda = lambda;

    if(lambda2 >= 0.0)
        this->_lambda2 = lambda2;

    if(lambdaAngularMomentum >= 0.0)
        this->_lambdaAngularMomentum = lambdaAngularMomentum;

    if(lambda >= 0.0 || lambda2 >= 0.0 || lambdaAngularMomentum >= 0.0)
        this->update_b();



}

Eigen::Vector3d OpenSoT::tasks::force::CoM::getVelocityError()
{
    velocityError = _desiredVelocity - _actualVelocity;
    return velocityError;
}

Eigen::Vector3d OpenSoT::tasks::force::CoM::getError()
{
    positionError =  _desiredPosition - _actualPosition;
    return positionError;
}

Eigen::Vector3d CoM::getAngularMomentumError()
{
    angularMomentumError = _desiredAngularMomentum - _actualAngularMomentum;
    return angularMomentumError;
}

void CoM::_log(XBot::MatLogger::Ptr logger)
{

    logger->add(getTaskID() + "_error", getError());
    
}

