/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/tasks/velocity/unicycle.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

Unicycle::Unicycle(std::string task_id,
                     const Eigen::VectorXd& x,
                     XBot::ModelInterface &robot,
                     std::string distal_link,
                     std::string base_link,
                     double wheel_radius) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _wheel_radius(wheel_radius)
{
    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);

    if(!this->_base_link_is_world) {
        this->_base_link_index = _robot.getLinkID(_base_link);
        //assert(this->_base_link_index >= 0);
    }

    this->_distal_link_index = _robot.getLinkID(_distal_link);
    //assert(this->_distal_link_index >= 0);

    if(!this->_base_link_is_world)
        assert(this->_distal_link_index != _base_link_index);

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

    _A.setZero(3, robot.getJointNumber());
    _constA = A;
    /// HERE fill _constA


    ///



    _b.setZero(_A.rows());


    _W.setIdentity(_A.rows(), _A.rows());

    _hessianType = HST_SEMIDEF;
}

Unicycle::~Unicycle()
{
}

void Unicycle::_update(const Eigen::VectorXd &x) {

    /************************* COMPUTING TASK *****************************/

    ///CHECK: HERE THE JACOBIANS ARE COMPUTED! We need e^J_{w,e} & e^J_{b,e}!
    if(_base_link_is_world)
        _robot.getJacobian(_distal_link,_J); //w^J_{w,e}
    else
        _robot.getRelativeJacobian(_distal_link, _base_link, _J); //b^J_{b,e}

    ///


    _A = _constA*_J;



    this->update_b();



    /**********************************************************************/
}


const std::string OpenSoT::tasks::velocity::Unicycle::getDistalLink() const
{
    return _distal_link;
}

const std::string OpenSoT::tasks::velocity::Unicycle::getBaseLink() const
{
    return _base_link;
}

const bool OpenSoT::tasks::velocity::Unicycle::baseLinkIsWorld() const
{
    return _base_link_is_world;
}

void OpenSoT::tasks::velocity::Unicycle::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}


bool Unicycle::setBaseLink(const std::string& base_link)
{
    if(base_link.compare(_base_link) == 0)
        return true;



    _base_link = base_link;
    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);


    return true;
}

bool OpenSoT::tasks::velocity::Unicycle::isUnicycle(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Unicycle>(task);
}

OpenSoT::tasks::velocity::Unicycle::Ptr OpenSoT::tasks::velocity::Unicycle::asUnicycle(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Unicycle>(task);
}
