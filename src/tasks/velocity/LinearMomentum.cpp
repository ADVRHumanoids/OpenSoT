/*
 * Copyright (C) 2017 Walkman
 * Author: Enrico Mingo Hoffman, Pouya Mohammadi
 * email:  enrico.mingo@iit.it, p.mohammadi@tu-bs.de
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

#include <OpenSoT/tasks/velocity/LinearMomentum.h>

using namespace OpenSoT::tasks::velocity;

LinearMomentum::LinearMomentum(const Eigen::VectorXd& x, XBot::ModelInterface& robot):
    Task("LinearMomentum", x.size()), _robot(robot)
{
    _desiredLinearMomentum.setZero();
    this->_update(x);

    _W.resize(3,3);
    _W.setIdentity(3,3);

    _hessianType = HST_SEMIDEF;

    _Momentum.resize(6, _x_size);
    _A.resize(3, _x_size);
    _b = _desiredLinearMomentum;
}

LinearMomentum::~LinearMomentum()
{

}

void LinearMomentum::_update(const Eigen::VectorXd& x)
{
    _robot.computeCentroidalMomentumMatrix(_Momentum);
    _A = _Momentum.block(0,0,3,_x_size);
    _b = _desiredLinearMomentum;
}

void LinearMomentum::setReference(const Eigen::Vector3d& desiredLinearMomentum)
{
    _desiredLinearMomentum = desiredLinearMomentum;
}

void LinearMomentum::setReference(const KDL::Vector& desiredLinearMomentum)
{
    _desiredLinearMomentum[0] = desiredLinearMomentum[0];
    _desiredLinearMomentum[1] = desiredLinearMomentum[1];
    _desiredLinearMomentum[2] = desiredLinearMomentum[2];
}

void LinearMomentum::getReference(Eigen::Vector3d& desiredLinearMomentum) const
{
    desiredLinearMomentum = _desiredLinearMomentum;
}

void LinearMomentum::getReference(KDL::Vector& desiredLinearMomentum) const
{
    desiredLinearMomentum[0] = _desiredLinearMomentum[0];
    desiredLinearMomentum[1] = _desiredLinearMomentum[1];
    desiredLinearMomentum[2] = _desiredLinearMomentum[2];
}

std::string LinearMomentum::getBaseLink()
{
    return BASE_LINK_COM;
}

std::string LinearMomentum::getDistalLink()
{
    return DISTAL_LINK_COM;
}

LinearMomentum::Ptr LinearMomentum::asLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<LinearMomentum>(task);
}


bool LinearMomentum::isLinearMomentum(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<LinearMomentum>(task);
}
