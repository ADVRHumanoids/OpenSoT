#include <OpenSoT/tasks/velocity/Gaze.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;


Gaze::Gaze(std::string task_id,
           const yarp::sig::Vector& x,
           iDynUtils &robot,
           std::string base_link) :
    Task(task_id, x.size()),
    _distal_link("gaze"),
    _cartesian_task(new Cartesian(task_id, x, robot, _distal_link, base_link)),
    _subtask(new SubTask(_cartesian_task, Indices::range(4,5))),
    _robot(robot)
{
    this->_update(x);
}

Gaze::~Gaze()
{

}

void Gaze::setGaze(const yarp::sig::Matrix &desiredGaze)
{    
    KDL::Frame bl_T_gaze_kdl;

    if(_cartesian_task->baseLinkIsWorld())
        bl_T_gaze_kdl = _robot.iDyn3_model.getPositionKDL(
                        _robot.iDyn3_model.getLinkIndex(_distal_link));
    else
        bl_T_gaze_kdl = _robot.iDyn3_model.getPositionKDL(
                        _robot.iDyn3_model.getLinkIndex(_cartesian_task->getBaseLink()),
                        _robot.iDyn3_model.getLinkIndex(_distal_link));

    yarp::sig::Matrix bl_T_gaze;
    cartesian_utils::fromKDLFrameToYARPMatrix(bl_T_gaze_kdl, bl_T_gaze);
    yarp::sig::Matrix gaze_T_bl;
    cartesian_utils::fromKDLFrameToYARPMatrix(bl_T_gaze_kdl.Inverse(), gaze_T_bl);
    yarp::sig::Matrix gaze_T_obj = gaze_T_bl*desiredGaze;

    yarp::sig::Matrix gaze_goal(4,4);
    cartesian_utils::computePanTiltMatrix(gaze_T_obj.subcol(0, 3, 3), gaze_goal);

    gaze_goal = bl_T_gaze*gaze_goal;

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

void Gaze::setWeight(const yarp::sig::Matrix &W)
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

void Gaze::_update(const yarp::sig::Vector &x)
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
