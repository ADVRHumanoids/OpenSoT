#include <OpenSoT/tasks/velocity/Gaze.h>
#include <yarp/math/Math.h>

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
    _reference_gaze = _cartesian_task->getReference();
    this->_update(x);
}

Gaze::~Gaze()
{

}

void Gaze::setGaze(const yarp::sig::Matrix &desiredGaze)
{
    _reference_gaze = desiredGaze;

    yarp::sig::Matrix gaze_goal(4,4);
    cartesian_utils::computePanTiltMatrix(desiredGaze.subcol(0, 3, 3), gaze_goal);

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
