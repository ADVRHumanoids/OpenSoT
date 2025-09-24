#include <OpenSoT/oc/SE3Task.h>
#include <OpenSoT/utils/LieGroupsUtils.h>


using namespace OpenSoT::oc;

SE3Task::SE3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame):
Task(id, dx.getInputSize()),
_robot(robot),
_dx(dx),
_distal_frame(distal_frame)
{
    _W.setIdentity(dx.getOutputSize(), dx.getOutputSize());

    _ref = _robot.getPose(_distal_frame);

    update();
}

void SE3Task::_update()
{   
    _w_T_d = _robot.getPose(_distal_frame);

    Eigen::MatrixXd J(6, _robot.getNv());
    J.setZero();

    _robot.getJacobian(_distal_frame, _w_T_d.linear().transpose() * _w_T_d.translation(), J);
    
    Eigen::Matrix6d Adj;
    Adj.setZero();
    Adj.block<3,3>(0,0) = _w_T_d.linear().transpose();
    Adj.block<3,3>(3,3) = Adj.block<3,3>(0,0);

    J = Adj * J;
    _w = Log6(_w_T_d.inverse()* _ref);

    _task = J*_dx - _w;
    
    _A = _task.getM();
    _b = -_task.getq();
}

 
const Eigen::Vector6d& SE3Task::getError()
{
    
    return _w;
}