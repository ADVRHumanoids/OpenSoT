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
#include <OpenSoT/utils/Piler.h>

namespace OpenSoT{
    namespace tasks{
        namespace floating_base{
            class Contact: public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{
            public:
                typedef boost::shared_ptr<Contact> Ptr;

                Contact(XBot::ModelInterface& robot, const std::string link_in_contact,
                        const Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Identity(6,6));
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
            };
        }
    }
}

#endif
