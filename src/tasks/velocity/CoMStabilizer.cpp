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

/**
    * @brief CoMStabilizer. It requires, among the others:
    * l_sole and r_sole references
    * a ConstPtr for the l_sole and the r_sole force/torque sensors (this will automatically update the wrenches needed by the stabilizer)
    * */

CoMStabilizer::CoMStabilizer(  const Eigen::VectorXd& x,
                                XBot::ModelInterface& robot,
                                
                                Eigen::Affine3d l_sole,
                                Eigen::Affine3d r_sole,
                               
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor_l_sole,
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor_r_sole,
                               
                                const double sample_time, const double mass,
                                const double ankle_height,
                                const Eigen::Vector2d& foot_size,
                                const double Fzmin,
                                const Eigen::Vector3d& K, const Eigen::Vector3d& C,
                                const Eigen::Vector3d& MaxLims,
                                const Eigen::Vector3d& MinLims,
                                const bool invertFTSensors, //TODO take out
                                const double samples2ODE,
                                const double freq) : CoM(x, robot, "CoMStabilizer"),
                                                     _stabilizer(sample_time, mass, ankle_height, foot_size, Fzmin,
                                                                K, C, MaxLims, MinLims, samples2ODE, freq),
                                                     _ft_sensor_l_sole(ft_sensor_l_sole),
                                                     _ft_sensor_r_sole(ft_sensor_r_sole)
{   
    _desiredPosition = CoM::getReference();
    
    _left_wrench.setZero();
    _right_wrench.setZero();
    
    _ft_sensor_l_sole->getWrench(_left_wrench);
    _ft_sensor_r_sole->getWrench(_right_wrench);
    
    // =====================================
    _invertFTSensors = invertFTSensors;
    if (_invertFTSensors == true)
    {
        _left_wrench *= -1;
        _right_wrench *= -1;
    }
    
    _zmp_ref = CoM::getReference().head(2);
        
    _l_sole_ref = l_sole.translation();
    _r_sole_ref = r_sole.translation();
    
    _update(Eigen::VectorXd(0));
    
    
    
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
    
    _ft_sensor_l_sole->getWrench(_left_wrench);
    _ft_sensor_r_sole->getWrench(_right_wrench);
    
    if (_invertFTSensors == true)
    {
        _left_wrench *= -1;
        _right_wrench *= -1;
    }
    
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


void CoMStabilizer::setZMP(Eigen::Vector2d zmp_ref)
{
    _zmp_ref = zmp_ref;
}

void CoMStabilizer::setLeftSoleRef(Eigen::Affine3d l_sole_ref)
{
    _l_sole_ref = l_sole_ref.translation();
}

void CoMStabilizer::setRightSoleRef(Eigen::Affine3d r_sole_ref)
{
    _r_sole_ref = r_sole_ref.translation();
}

void CoMStabilizer::setSoleRef(Affine3d l_sole_ref, Affine3d r_sole_ref)
{
    /**
     * @brief set left foot and right foot reference
     */
    _l_sole_ref = l_sole_ref.translation();
    _r_sole_ref = r_sole_ref.translation();
}

void CoMStabilizer::_log(XBot::MatLogger::Ptr logger)
{
    OpenSoT::tasks::velocity::CoM::_log(logger);
    
    logger->add("com_pos_desired", _desiredPosition);
    logger->add("com_vel_desired", _desiredVelocity);
    logger->add("zmp_ref", _zmp_ref);
    logger->add("left_wrench", _left_wrench);
    logger->add("right_wrench", _right_wrench);
    logger->add("l_sole_ref", _l_sole_ref);
    logger->add("r_sole_ref", _r_sole_ref);
    
}

const CompliantStabilizer &OpenSoT::tasks::velocity::CoMStabilizer::getStabilizer()
{
    return _stabilizer;
}





