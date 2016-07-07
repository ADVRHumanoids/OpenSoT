#include <OpenSoT/tasks/velocity/Gaze.h>
#include <yarp/math/Math.h>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;


Gaze::Gaze(std::string task_id,
           const yarp::sig::Vector& x,
           iDynUtils &robot,
           std::string base_link) :
    _distal_link("gaze"),
    _base_link(base_link),
    _cartesian_task(new Cartesian(task_id, x, robot, _distal_link, _base_link)),
    SubTask(_cartesian_task, Indices::range(4,5)),
    _robot(robot)
{
    _reference_gaze = _cartesian_task->getReference();
}

Gaze::~Gaze()
{

}

yarp::sig::Matrix Gaze::getGaze()
{
    return _reference_gaze;
}

void Gaze::setGaze(const yarp::sig::Matrix &desiredGaze)
{
    _reference_gaze = desiredGaze;

    yarp::sig::Matrix gaze_goal(4,4);
    cartesian_utils::computePanTiltMatrix(desiredGaze.subcol(0, 3, 3), gaze_goal);

    _cartesian_task->setReference(gaze_goal);
}
