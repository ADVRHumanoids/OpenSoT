/*
 * Copyright (C) 2014 Walkman
 * Authors:Francesco Ruscelli, Enrico Mingo
 * email:  francesco.ruscelli@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/tasks/velocity/CoMStabilizer.h>

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>



using namespace OpenSoT::tasks::velocity;

CoMStabilizer::CoMStabilizer(  const Eigen::VectorXd& x,
                                XBot::ModelInterface& robot,
                                
                                const double sample_time, const double mass,
                                const double ankle_height,
                                const Eigen::Vector2d& foot_size,
                                const double Fzmin,
                                const Eigen::Vector3d& K, const Eigen::Vector3d& C,
                                const Eigen::Vector3d& MaxLims,
                                const Eigen::Vector3d& MinLims,
                                const double samples2ODE,
                                const double freq) : CoM(x, robot, "CoMStabilizer"),
                                                     _stabilizer(sample_time, mass,
                                                                ankle_height,
                                                                foot_size,
                                                                Fzmin,
                                                                K, C,
                                                                MaxLims,
                                                                MinLims,
                                                                samples2ODE,
                                                                freq)
{
}

CoMStabilizer::~CoMStabilizer()
{
}

void CoMStabilizer::_update(const Eigen::VectorXd& x)

{
    
    // TODO flags for ensuring that all the inputs are setted
    Eigen::Vector2d CopPos_L, CopPos_R;
    
    CopPos_L(0) = _zmp_ref[0] - _l_sole_ref[0];
    CopPos_L(1) = _zmp_ref[1] - _l_sole_ref[1];

    CopPos_R(0) = _zmp_ref[0] - _r_sole_ref[0];
    CopPos_R(1) = _zmp_ref[1] - _r_sole_ref[1];
// 

    Eigen::Vector3d delta_com = _stabilizer.update(_left_wrench, _right_wrench,
                                                  CopPos_L, CopPos_R,
                                                  _l_sole_ref, _r_sole_ref);
    
    Eigen::Vector3d com_updated = _desiredPosition + delta_com;
    CoM::setReference(com_updated, _desiredVelocity);
    
    CoM::_update(x);
    
    _desiredVelocity.setZero();
}

void CoMStabilizer::setReference(const KDL::Vector& desiredPosition,
                  const KDL::Vector& desiredVelocity)
{
    _desiredPosition(0) = desiredPosition.x();
    _desiredPosition(1) = desiredPosition.y();
    _desiredPosition(2) = desiredPosition.z();

    _desiredVelocity(0) = desiredVelocity.x();
    _desiredVelocity(1) = desiredVelocity.y();
    _desiredVelocity(2) = desiredVelocity.z();

}

void CoMStabilizer::setReference(const KDL::Vector& desiredPosition)
{
    _desiredPosition(0) = desiredPosition.x();
    _desiredPosition(1) = desiredPosition.y();
    _desiredPosition(2) = desiredPosition.z();

    _desiredVelocity.setZero(3);
}

void CoMStabilizer::setReference(const Eigen::Vector3d& desiredPosition)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity.setZero(3);
}

void CoMStabilizer::setReference(const Eigen::Vector3d &desiredPosition,
                                                 const Eigen::Vector3d &desiredVelocity)
{
    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
}

Eigen::VectorXd CoMStabilizer::getReference() const
{
    return _desiredPosition;
}

void CoMStabilizer::getReference(Eigen::Vector3d &desiredPosition, Eigen::Vector3d &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}


void CoMStabilizer::setZMP(Eigen::Vector3d zmp_ref)
{
    _zmp_ref = zmp_ref;
}

void CoMStabilizer::setLeftSoleRef(Eigen::Vector3d l_sole_ref)
{
    _l_sole_ref = l_sole_ref;
}

void CoMStabilizer::setRightSoleRef(Eigen::Vector3d r_sole_ref)
{
    _r_sole_ref = r_sole_ref;
}

void CoMStabilizer::setLeftWrench(Eigen::Vector6d left_wrench)
{
    _left_wrench = left_wrench;
}

void CoMStabilizer::setRightWrench(Eigen::Vector6d right_wrench)
{
    _right_wrench = right_wrench;
}

void CoMStabilizer::setSoleRef(Vector3d l_sole_ref, Vector3d r_sole_ref)
{
    /**
     * @brief set left foot and right foot reference
     */
    _l_sole_ref = l_sole_ref;
    _r_sole_ref = r_sole_ref;
}

void CoMStabilizer::setWrench(Eigen::Vector6d left_wrench, Eigen::Vector6d right_wrench)
{
    /**
     * @brief set left foot and right foot wrench
     */
    _left_wrench = left_wrench;
    _right_wrench = right_wrench;
}






