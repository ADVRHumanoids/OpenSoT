
/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi.it
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
 * 
 * Acknowledgment: this task implementation is inspired from the insight provided by 
 * Malgorzata Kamedula (malgorzata.kamedula@iit.it)
*/

#ifndef __OPENSOT_TASKS_VELOCITY_PURE_ROLLING_H__
#define __OPENSOT_TASKS_VELOCITY_PURE_ROLLING_H__

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/SubTask.h>

namespace OpenSoT { namespace tasks { namespace velocity {
    
    class PureRolling : public Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef std::shared_ptr<PureRolling> Ptr;
        
        PureRolling(std::string wheel_link_name, 
                    double radius,
                    const XBot::ModelInterface& model);
        
        virtual void _update();
        
        void setOutwardNormal(const Eigen::Vector3d& n);
        
        virtual void _log(XBot::MatLogger2::Ptr logger);
        
    private:
        
        const XBot::ModelInterface& _model;
        std::string _wheel_link_name;
        Eigen::Vector3d _world_contact_plane_normal;
        Eigen::Vector3d _wheel_axis;
        Eigen::Vector3d _wheel_contact_point;
        double _radius;
        
        Eigen::MatrixXd _Jc;
        Eigen::MatrixXd _S;
        
        Eigen::VectorXd _qdot;
        
        Eigen::Affine3d _world_T_wheel;
        Eigen::Matrix3d _world_R_wheel;
        Eigen::Matrix3d _local_R_world;
        
    };

    class PureRollingPosition : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef std::shared_ptr<PureRollingPosition> Ptr;

        PureRollingPosition(std::string wheel_link_name,
                    double radius,
                    const XBot::ModelInterface& model,
                    const bool control_z = false);

        void setOutwardNormal(const Eigen::Vector3d& n);
        
        virtual void _update();

        virtual void _log(XBot::MatLogger2::Ptr logger);


    private:
        
        PureRolling::Ptr _pure_rolling;
        OpenSoT::SubTask::Ptr _subtask;
        std::list<unsigned int> _position_indices;

    };

    class PureRollingOrientation : public Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef std::shared_ptr<PureRollingOrientation> Ptr;

        PureRollingOrientation(std::string wheel_link_name,
                    double radius,
                    const XBot::ModelInterface& model);

        virtual void _update();


    private:
        PureRolling::Ptr _pure_rolling;
        OpenSoT::SubTask::Ptr _subtask;
        std::list<unsigned int> _orientation_indices;

    };
    
} } }





#endif
