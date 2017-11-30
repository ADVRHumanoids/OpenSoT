/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
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

#ifndef __OPENSOT_ACCELERATION_TASK_CARTESIAN_H__
#define __OPENSOT_ACCELERATION_TASK_CARTESIAN_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>


namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Cartesian : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef boost::shared_ptr<Cartesian> Ptr;

        Cartesian(const std::string task_id,
                  const XBot::ModelInterface& robot,
                  const std::string& distal_link,
                  const std::string& base_link,
                  const Eigen::VectorXd& x
                 );
        
        Cartesian(const std::string task_id,
                  const XBot::ModelInterface& robot, 
                  const std::string& distal_link,
                  const std::string& base_link,
                  const AffineHelper& qddot
                 );
        
        void setPositionReference(const Eigen::Vector3d& pos_ref);
        void setReference(const Eigen::Affine3d& ref);
        void setReference(const Eigen::Affine3d& pose_ref,
                          const Eigen::Vector6d& vel_ref);
        void setReference(const Eigen::Affine3d& pose_ref,
                          const Eigen::Vector6d& vel_ref,
                          const Eigen::Vector6d& acc_ref);

        void getReference(Eigen::Affine3d& ref);
        void getActualPose(Eigen::Affine3d& actual);
        
        void resetReference();

        virtual void _update(const Eigen::VectorXd& x);
        
        virtual void _log(XBot::MatLogger::Ptr logger);

        void setLambda2(const double lambda2);
        
    private:
        
        static const std::string world_name;
        
        std::string _base_link, _distal_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _cartesian_task;
        
        Eigen::MatrixXd _J;
        Eigen::Vector6d _jdotqdot;
        
        Eigen::Affine3d _pose_ref, _pose_current;
        Eigen::Vector6d _pose_error, _vel_ref, _vel_current, _acc_ref;
        
        Eigen::Vector3d _orientation_error;

        double _lambda2;
        
    };
    
} } }






#endif
