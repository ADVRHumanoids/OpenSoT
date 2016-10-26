#include <OpenSoT/tasks/velocity/Gaze.h>
#include <idynutils/cartesian_utils.h>

using namespace OpenSoT::tasks::velocity;


Gaze::Gaze(std::string task_id,
           const Eigen::VectorXd& x,
           iDynUtils &robot,
           std::string base_link) :
    Task(task_id, x.size()),
    _distal_link("gaze"),
    _cartesian_task(new Cartesian(task_id, x, robot, _distal_link, base_link)),
    _subtask(new SubTask(_cartesian_task, Indices::range(4,5))),
    _robot(robot), _gaze_T_obj(4,4), _tmp_vector(3)
{
    this->_update(x);
}

Gaze::~Gaze()
{

}

void Gaze::setGaze(const Eigen::MatrixXd &desiredGaze)
{    
    KDL::Frame bl_T_gaze_kdl;

    if(_cartesian_task->baseLinkIsWorld())
        bl_T_gaze_kdl = _robot.iDyn3_model.getPositionKDL(
                        _robot.iDyn3_model.getLinkIndex(_distal_link));
    else
        bl_T_gaze_kdl = _robot.iDyn3_model.getPositionKDL(
                        _robot.iDyn3_model.getLinkIndex(_cartesian_task->getBaseLink()),
                        _robot.iDyn3_model.getLinkIndex(_distal_link));

    _gaze_T_obj = toEigen(bl_T_gaze_kdl.Inverse())*desiredGaze;
    _tmp_vector(0) = _gaze_T_obj(0,3);
    _tmp_vector(1) = _gaze_T_obj(1,3);
    _tmp_vector(2) = _gaze_T_obj(2,3);

    KDL::Frame gaze_goal; gaze_goal = gaze_goal.Identity();
    cartesian_utils::computePanTiltMatrix(_tmp_vector, gaze_goal);
    //cartesian_utils::computePanTiltMatrix(gaze_T_obj.subcol(0, 3, 3), gaze_goal);

    gaze_goal = bl_T_gaze_kdl*gaze_goal;

    _cartesian_task->setReference(gaze_goal);
}

void Gaze::setOrientationErrorGain(const double& orientationErrorGain)
{
    _cartesian_task->setOrientationErrorGain(orientationErrorGain);
}

const double Gaze::getOrientationErrorGain() const
{
    return _cartesian_task->getOrientationErrorGain();
}

void Gaze::setWeight(const Eigen::MatrixXd &W)
{
    this->_W = W;
    _subtask->setWeight(W);
}

std::list<OpenSoT::SubTask::ConstraintPtr> &Gaze::getConstraints()
{
    return _subtask->getConstraints();
}

const unsigned int Gaze::getTaskSize() const
{
    return _subtask->getTaskSize();
}

void Gaze::_update(const Eigen::VectorXd &x)
{
    _subtask->update(x);
    this->_A = _subtask->getA();
    this->_b = _subtask->getb();
    this->_hessianType = _subtask->getHessianAtype();
    this->_W = _subtask->getWeight();
}

std::vector<bool> Gaze::getActiveJointsMask()
{
    return _subtask->getActiveJointsMask();
}

bool Gaze::setActiveJointsMask(const std::vector<bool> &active_joints_mask)
{
    return _subtask->setActiveJointsMask(active_joints_mask);
}
