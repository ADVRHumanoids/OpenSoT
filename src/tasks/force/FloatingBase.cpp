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

#include <OpenSoT/tasks/force/FloatingBase.h>

using namespace OpenSoT::tasks::force;

FloatingBase::FloatingBase(XBot::ModelInterface &model,
                           const std::vector<OpenSoT::AffineHelper> &wrenches,
                           const std::vector<std::string>& contact_links):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("FloatingBase", wrenches[0].getInputSize()),
    _model(model),
    _contact_links(contact_links),
    _wrenches(wrenches),
    _enabled_contacts(wrenches.size(), true)
{
    if(wrenches.size() != contact_links.size())
        throw std::invalid_argument("wrench and contact_links has different sizes!");

    update(Eigen::VectorXd());
}

void FloatingBase::setFloatingBaseTorque(const Eigen::Vector6d &floating_base_torque)
{
    _floating_base_torque = floating_base_torque;
}

void FloatingBase::setEnabledContacts(const std::vector<bool> &enabled_contacts)
{
    if(enabled_contacts.size() != _enabled_contacts.size())
        XBot::Logger::error("Wrong enabled_contacts size of %i instead of %i \n", enabled_contacts.size(), enabled_contacts.size());
    else
        _enabled_contacts = enabled_contacts;
}

void FloatingBase::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(_task_id + "_floating_base_torque", _floating_base_torque);
}

void FloatingBase::_update(const Eigen::VectorXd &x)
{
    _task.setZero(_wrenches[0].getInputSize(), 6);

    for(int i = 0; i < _enabled_contacts.size(); i++)
    {
        if(!_enabled_contacts[i]){
            continue;
        }
        else {
            _model.getJacobian(_contact_links[i], _J_i);
            _Jfb_i = _J_i.block<6,6>(0,0).transpose();
            _task = _task + _Jfb_i * _wrenches[i];
        }
    }

    _A = _task.getM();
    _b = -_task.getq() + _floating_base_torque;
}
