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

#ifndef __OPENSOT_ACCELERATION_TASK_CONTACT_H__
#define __OPENSOT_ACCELERATION_TASK_CONTACT_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>


namespace OpenSoT { namespace tasks { namespace acceleration {
    
    class Contact : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
        
    public:
        
        typedef boost::shared_ptr<Contact> Ptr;

        Contact(const std::string& task_id,
                const XBot::ModelInterface& robot,
                const std::string& contact_link,
                const Eigen::VectorXd& x,
                const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd()
                );
        
        Contact(const std::string& task_id,
                const XBot::ModelInterface& robot, 
                const std::string& contact_link,
                const AffineHelper& qddot,
                const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd()
                 );
        

        virtual void _update(const Eigen::VectorXd& x);
        
        virtual void _log(XBot::MatLogger2::Ptr logger);
        
    private:
        
        std::string _contact_link;
        const XBot::ModelInterface& _robot;
        AffineHelper _qddot;
        AffineHelper _contact_task;
        
        Eigen::MatrixXd _J, _K;
        Eigen::Vector6d _jdotqdot;
        
    };
    
} } }









#endif
