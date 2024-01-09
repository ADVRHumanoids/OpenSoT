/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Enrico Mingo Hoffman, Arturo Laurenzi
 * email:  enrico.mingo@iit.it, arturo.laurenzi@iit.it
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

#include <OpenSoT/tasks/floating_base/Contact.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <xbot2_interface/common/utils.h>

OpenSoT::tasks::floating_base::Contact::Contact(XBot::ModelInterface &robot,
                                        const std::string& link_in_contact,
                                        const Eigen::MatrixXd& contact_matrix, 
                                        const Eigen::Affine3d& desired_contact_pose
                                               ):
    Task("Contact_"+link_in_contact, 6),_robot(robot),_link_in_contact(link_in_contact),
    _contact_matrix(contact_matrix),
    _w_T_c_des(desired_contact_pose)
{
    _hessianType = HST_SEMIDEF;

    _W.setIdentity(contact_matrix.rows(), contact_matrix.rows());

    _dqm.setZero(_robot.getJointNum());
    
    _lambda = 0.0;

    _update(Eigen::VectorXd::Zero(robot.getJointNum()));
}

OpenSoT::tasks::floating_base::Contact::~Contact()
{

}

void OpenSoT::tasks::floating_base::Contact::_update(const Eigen::VectorXd &x)
{
    _robot.getJacobian(_link_in_contact, _J);
    XBot::Utils::rotate(_J, _robot.getPose(_link_in_contact).linear().transpose(), Jrot);
    _Jcontact = _contact_matrix*_J;
    _robot.getJointVelocity(_dqm);
    
    Eigen::Affine3d w_T_c;
    _robot.getPose(_link_in_contact, w_T_c);
    Eigen::Vector3d pos_err, rot_err;

    
    cartesian_utils::computeCartesianError(w_T_c, _w_T_c_des, pos_err, rot_err);
    Eigen::Vector6d err;
    err <<  pos_err,
           -rot_err;

    _A = _Jcontact.leftCols(6);
    _b = -_Jcontact.rightCols(_dqm.size()-6)*_dqm.tail(_dqm.size()-6) + _lambda * err;
}

const std::string& OpenSoT::tasks::floating_base::Contact::getLinkInContact() const
{
    return _link_in_contact;
}

void OpenSoT::tasks::floating_base::Contact::setLinkInContact(const std::string link_in_contact)
{
    _link_in_contact = link_in_contact;
}
