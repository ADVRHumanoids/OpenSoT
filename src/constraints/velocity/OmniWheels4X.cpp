/*
 * Copyright (C) 2023
 * Author: Enrico Mingo Hoffman
 * email:  enricomingo@gmail.com
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

#include <OpenSoT/constraints/velocity/OmniWheels4X.h>

using namespace OpenSoT::constraints::velocity;

OmniWheels4X::OmniWheels4X(const double l1, const double l2, const double r,
                         const std::vector<std::string> joint_wheels_name,
                         const std::string base_link,
                         XBot::ModelInterface &robot):
    Constraint("OmniWheels4X", robot.getNv()), _robot(robot), _base_link(base_link), _is_global_velocity(false)
{
    _J.resize(3, _x_size);
    _J.setZero();

    //select xdot, ydot, and wz
    _J(0,0) = 1.;
    _J(1,1) = 1.;
    _J(2,5) = 1.;

    if(joint_wheels_name.size() != 4)
        throw std::runtime_error("joint_wheels_name != 4");

    int fl_id = _robot.getDofIndex(joint_wheels_name[0]);
    _J(0, fl_id) -= 1.;
    _J(1, fl_id) -= -1.;
    _J(2, fl_id) -= -1./(l1 + l2);

    int fr_id = _robot.getDofIndex(joint_wheels_name[1]);
    _J(0, fr_id) -= 1.;
    _J(1, fr_id) -= 1.;
    _J(2, fr_id) -= 1./(l1 + l2);

    int hl_id = _robot.getDofIndex(joint_wheels_name[2]);
    _J(0, hl_id) -= 1.;
    _J(1, hl_id) -= 1.;
    _J(2, hl_id) -= -1./(l1 + l2);

    int hr_id = _robot.getDofIndex(joint_wheels_name[3]);
    _J(0, hr_id) -= 1.;
    _J(1, hr_id) -= -1.;
    _J(2, hr_id) -= 1./(l1 + l2);

    _J.rightCols(_x_size-6) *= r/4.;

    _bLowerBound.setZero(3);
    _bUpperBound.setZero(3);

    _Aineq = _J; //initialize

    update();

}

void OmniWheels4X::update()
{
    _w_T_b.setIdentity();

    if(_is_global_velocity)
        _robot.getPose(_base_link, _w_T_b);
    _Aineq.rightCols(_x_size-6).noalias() = _w_T_b.linear() * _J.rightCols(_x_size-6);
}


