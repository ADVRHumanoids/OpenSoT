/*
 * Copyright (C) 2018 Cogimon/Centauro
 * Authors: Arturo Laurenzi, Enrico Mingo Hoffman
 * email:  arturo.laurenzi@iit.it, enrico.mingo@iit.it
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
#ifndef __TASKS_FORCE_FB_H__
#define __TASKS_FORCE_FB_H__

#include <OpenSoT/Task.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/Affine.h>

namespace OpenSoT {
   namespace tasks {
       namespace force {
       /**
         * @brief The FloatingBase class implement a task which maps Forces on the floating base virtual chain to
         * contacts.
         */
        class FloatingBase : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
            typedef std::shared_ptr<FloatingBase> Ptr;

            /**
             * @brief FloatingBase constructor
             * @param model of the robot
             * @param wrenches affine helper
             * @param contact_links vector of robot links which considered in contact with the environment
             */
            FloatingBase(XBot::ModelInterface& model,
                         const std::vector<OpenSoT::AffineHelper>& wrenches,
                         const std::vector<std::string>& contact_links);

            /**
             * @brief setFloatingBaseTorque set torques of "equivalent" fully-actuated floating base.
             * NOTE: floating_base_torque should contains gravity and non-linear term compensation!
             * @param floating_base_torque vector of 6 torques
             */
            void setFloatingBaseTorque(const Eigen::Vector6d& floating_base_torque);

            /**
             * @brief setEnabledContacts set vector of enabled contacts
             * @param enabled_contacts vector of true/false values for the contacts
             */
            void setEnabledContacts(const std::vector<bool>& enabled_contacts);

            private:
            virtual void _update(const Eigen::VectorXd& x);
            virtual void _log(XBot::MatLogger2::Ptr logger);

            std::vector<std::string> _contact_links;
            XBot::ModelInterface& _model;
            Eigen::Vector6d _floating_base_torque;

            std::vector<bool> _enabled_contacts;
            std::vector<OpenSoT::AffineHelper> _wrenches;
            OpenSoT::AffineHelper _task;
            Eigen::MatrixXd _J_i;
            Eigen::Matrix6d _Jfb_i;
        };

       }
   }
}

#endif
