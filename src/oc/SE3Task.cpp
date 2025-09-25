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
    _d_T_w = _robot.getPose(_distal_frame).inverse();

    Eigen::MatrixXd J(6, _robot.getNv());
    J.setZero();

    /**  
     * Here we compute the spatial Jacobian in WORLD from the classic Jacobian in world computed by the model interface.
     * 
     * NOTE: we pass the translation from local to world because inside the getJacobian(frame, p, J)
     * first rotate p in world and then apply the skew.
     **/
    _robot.getJacobian(_distal_frame, _d_T_w.translation(), J); 
    
    /** 
     * We now compute the Adjoint to rotate to LOCAL
    **/
    Eigen::Matrix6d Adj;
    Adj.setZero();
    Adj.block<3,3>(0,0) = _d_T_w.linear();
    Adj.block<3,3>(0,3) = Adj.block<3,3>(0,0) * hat(_d_T_w.inverse().translation()).transpose();
    Adj.block<3,3>(3,3) = Adj.block<3,3>(0,0);

    J = Adj * J;
    _w = Log6(_d_T_w* _ref);

    _task = J*_dx - _w;
    
    _A = _task.getM();
    _b = -_task.getq();
}

 
const Eigen::Vector6d& SE3Task::getError()
{   
    return _w;
}