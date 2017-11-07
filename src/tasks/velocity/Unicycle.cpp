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

#include <OpenSoT/tasks/velocity/Unicycle.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

Unicycle::Unicycle(std::string task_id,
                     const Eigen::VectorXd& x,
                     XBot::ModelInterface &robot,
                     std::string distal_link,
                     std::string base_link,
                     double wheel_radius, int signcorrection) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _wheel_radius(wheel_radius),
    _Swheel(robot.getJointNum()),
    _constA(3,6),
    _R(6,6),
    _J(6,robot.getJointNum()	
	)
{
	_signcorrection=signcorrection;
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
	_Swheel.setZero(robot.getJointNum());
	std::string wheel_joint_name = _robot.getUrdf().getLink(distal_link)->parent_joint->name;
	
	int id = _robot.getDofIndex(wheel_joint_name);
	_Swheel(id)=wheel_radius;
	_AuxVector=_Swheel;
	_A.setZero(3, robot.getJointNum());
    _constA.setZero(3,6);
    /// HERE fill _constA
// 	_constA(0,1)=1*0;
// 	_constA(1,3)=1.0*0;
// 	_constA(2,0)=1;
// 	
	_constA(0,1)=1*0;    // no Y motion		
	_constA(1,3)=1.0*0; //no rotation around X
	_constA(2,0)=1;    //just motion in X
	
	
	
	
	_b.setZero(_A.rows());
	_R.setZero(6,6);
	
	_W.setIdentity(_A.rows(), _A.rows());
	
	_hessianType = HST_SEMIDEF;
	    

    this->_update(x);

    ///

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
	
	if(_base_link_is_world)
        _robot.getPose(_distal_link, _T);
    else
        _robot.getPose(_distal_link, _base_link, _T); //base_link_T_distal_link

	_R.block(0,0,3,3) << _T.matrix().block(0,0,3,3).transpose();
	_R.block(3,3,3,3) << _T.matrix().block(0,0,3,3).transpose();
    ///


    _A = _constA*_J;//_R*_J;
	_AuxVector=_A.row(2);
	_A.row(2)=_AuxVector+_Swheel*_signcorrection;

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
