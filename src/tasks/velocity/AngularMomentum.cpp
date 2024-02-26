/*
 * Copyright (C) 2017 Walkman
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
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

#include <OpenSoT/tasks/velocity/AngularMomentum.h>

using namespace OpenSoT::tasks::velocity;

AngularMomentum::AngularMomentum(XBot::ModelInterface& robot):
    Task("AngularMomentum", robot.getNv()), _robot(robot),
    _base_link(BASE_LINK_COM), _distal_link(DISTAL_LINK_COM)
{
    _desiredAngularMomentum.setZero();
    this->_update(Eigen::VectorXd(0));

    _W.resize(3,3);
    _W.setIdentity(3,3);

    _hessianType = HST_SEMIDEF;

    _Momentum.resize(6, _x_size);
    _A.resize(3, _x_size);
    _b = _desiredAngularMomentum;
}

AngularMomentum::~AngularMomentum()
{

}

void AngularMomentum::_update(const Eigen::VectorXd& x)
{
    _robot.computeCentroidalMomentumMatrix(_Momentum);
    _A = _Momentum.block(3,0,3,_x_size);
    _b = _desiredAngularMomentum;
    //Reset for safety reasons!
    _desiredAngularMomentum.setZero(3);
}

void AngularMomentum::setReference(const Eigen::Vector3d& desiredAngularMomentum)
{
    _desiredAngularMomentum = desiredAngularMomentum;
}

void AngularMomentum::setReference(const KDL::Vector& desiredAngularMomentum)
{
    _desiredAngularMomentum[0] = desiredAngularMomentum[0];
    _desiredAngularMomentum[1] = desiredAngularMomentum[1];
    _desiredAngularMomentum[2] = desiredAngularMomentum[2];
}

void AngularMomentum::getReference(Eigen::Vector3d& desiredAngularMomentum) const
{
    desiredAngularMomentum = _desiredAngularMomentum;
}

void AngularMomentum::getReference(KDL::Vector& desiredAngularMomentum) const
{
    desiredAngularMomentum[0] = _desiredAngularMomentum[0];
    desiredAngularMomentum[1] = _desiredAngularMomentum[1];
    desiredAngularMomentum[2] = _desiredAngularMomentum[2];
}

const std::string& AngularMomentum::getBaseLink() const
{
    return _base_link;
}

const std::string& AngularMomentum::getDistalLink() const
{
    return _distal_link;
}

AngularMomentum::Ptr AngularMomentum::asAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<AngularMomentum>(task);
}


bool AngularMomentum::isAngularMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<AngularMomentum>(task);
}
