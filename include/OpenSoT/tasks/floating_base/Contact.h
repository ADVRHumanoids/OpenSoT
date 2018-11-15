/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Enrico Mingo Hoffman
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

#ifndef _OPENSOT_FLOATING_BASE_CONTACT_ESTIMATION_
#define _OPENSOT_FLOATING_BASE_CONTACT_ESTIMATION_

#include <OpenSoT/Task.h>
#include <XBotInterface/ModelInterface.h>

namespace OpenSoT{
    namespace tasks{
        namespace floating_base{
            /**
             * @brief The Contact class estimates floating_base velocities (linear and angular)
             * from the joint velocities. We consider a link in contact with the environment which
             * does not move wrt the world
             */
            class Contact: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{
            public:
                typedef boost::shared_ptr<Contact> Ptr;

                /**
                 * @brief Contact constructor which accept a robot model, a link name which represent
                 * the link in contact and a contact matrix (default is Identity)
                 * @param robot
                 * @param link_in_contact
                 * @param contact_matrix
                 */
                Contact(XBot::ModelInterface& robot, 
                        const std::string& link_in_contact,
                        const Eigen::MatrixXd& contact_matrix = Eigen::MatrixXd::Identity(6,6), 
                        const Eigen::Affine3d& desired_contact_pose = Eigen::Affine3d::Identity() );
                ~Contact();
                virtual void _update(const Eigen::VectorXd& x);

                void setLinkInContact(const std::string link_in_contact);
                const std::string& getLinkInContact() const;

            private:
                std::string _link_in_contact;
                XBot::ModelInterface& _robot;
                Eigen::MatrixXd _J;
                Eigen::MatrixXd _Jcontact;
                Eigen::VectorXd _dqm;
                Eigen::MatrixXd _contact_matrix;
                Eigen::Affine3d _w_T_c_des;
            };
        }
    }
}

#endif
