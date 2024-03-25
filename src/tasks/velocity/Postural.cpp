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

#include <OpenSoT/tasks/velocity/Postural.h>
#include <exception>
#include <cmath>


using namespace OpenSoT::tasks::velocity;

Postural::Postural(const XBot::ModelInterface& robot,
                   const std::string& task_id) :
    Task(task_id, robot.getNv()),
    _robot(robot)
{
    _q = _robot.getJointPosition();

    _q_desired = _robot.getNeutralQ();
    _v_desired.setZero(_x_size);
    _dq = _v_desired_ref = _v_desired;

    _W.setIdentity(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(_q);
    this->_update();
}

Postural::~Postural()
{
}

void Postural::_update() {
    _v_desired_ref = _v_desired;
    _q = _robot.getJointPosition();

    /************************* COMPUTING TASK *****************************/

    this->update_b();

    _v_desired.setZero(_x_size);

    /**********************************************************************/
}

void Postural::setReference(const Eigen::VectorXd& q_desired) {
    if(q_desired.size() == _robot.getNq())
    {
        _q_desired = q_desired;
        _v_desired.setZero(_x_size);
        _v_desired_ref = _v_desired;
        this->update_b();
    }
}

void OpenSoT::tasks::velocity::Postural::setReference(const Eigen::VectorXd &q_desired,
                                                      const Eigen::VectorXd &v_desired)
{
    if(q_desired.size() == _robot.getNq() && v_desired.size() == _x_size)
    {
        _q_desired = q_desired;
        _v_desired = v_desired;
        _v_desired_ref = _v_desired;
        this->update_b();
    }
}

const Eigen::VectorXd& OpenSoT::tasks::velocity::Postural::getReference() const
{
    return _q_desired;
}

void OpenSoT::tasks::velocity::Postural::getReference(Eigen::VectorXd &q_desired,
                                                      Eigen::VectorXd &v_desired) const
{
    q_desired = _q_desired;
    v_desired = _v_desired;
}

void Postural::update_b() {
    _robot.difference(_q_desired, _q, _dq);
    _b = _v_desired + _lambda*_dq;
}

void OpenSoT::tasks::velocity::Postural::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

Eigen::VectorXd Postural::getError()
{
    return _robot.difference(_q_desired, _q);
}

Eigen::VectorXd OpenSoT::tasks::velocity::Postural::getActualPositions()
{
    return _q;
}

bool OpenSoT::tasks::velocity::Postural::reset()
{
    _q_desired = _q;
    _update();

    return true;
}

bool OpenSoT::tasks::velocity::Postural::isPostural(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Postural>(task);
}

static OpenSoT::tasks::velocity::Postural::Ptr asPostural(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Postural>(task);
}

const Eigen::VectorXd& OpenSoT::tasks::velocity::Postural::getCachedVelocityReference() const
{
    return _v_desired_ref;
}

void OpenSoT::tasks::velocity::Postural::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add(_task_id + "_position_err", _dq);
    logger->add(_task_id + "_pos_ref", _q_desired);
    logger->add(_task_id + "_pos_actual", _q);
    logger->add(_task_id + "_desiredVelocityRef", _v_desired_ref);
}

