#include <OpenSoT/tasks/velocity/Interaction.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

Interaction::Interaction(std::string task_id,
                     const yarp::sig::Vector& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link,
                     std::string ft_frame):
    Cartesian(task_id, x, robot, distal_link, base_link),
    _ft_frame(ft_frame),
    _ft_index(-1),
    _C(6,6)
{
    moveit::core::LinkModel* ft_link = robot.moveit_robot_model->getLinkModel(ft_frame);

    _ft_index = robot.iDyn3_model.getFTSensorIndex(ft_link->getParentJointModel()->getName());
    if(_ft_index == -1)
        throw "Passed ft_frame is not in model!";

    _C = _C.eye();

    yarp::sig::Vector wrench_in_sensor_frame(6, 0.0);
    _robot.iDyn3_model.getSensorMeasurement(_ft_index, wrench_in_sensor_frame);

    KDL::Wrench wrench_in_sensor_frame_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_in_sensor_frame, wrench_in_sensor_frame_KDL);

    /** Wrench has to be transformed from ft_frame to base_link **/


    _desiredWrench = _actualWrench;

    _update(x);
}

Interaction::~Interaction()
{
}

void Interaction::_update(const yarp::sig::Vector &x) {
    yarp::sig::Vector delta_x = _C * (_desiredWrench - _actualWrench);

    yarp::sig::Matrix I(4,4); I = I.eye();
    setReference(I, delta_x);
}

void Interaction::setReferenceWrench(const yarp::sig::Vector &desiredWrench) {
    _desiredWrench = desiredWrench;
}

const yarp::sig::Vector Interaction::getReferenceWrench() const {
    return _desiredWrench;
}

const yarp::sig::Vector Interaction::getActualWrench() const
{
    return _actualWrench;
}

const yarp::sig::Matrix Interaction::getCompliance() const
{
    return _C;
}

void Interaction::setCompliance(const yarp::sig::Matrix& C)
{
    // Check size  [6x6] and if Positive Definite
    if(C.rows() == 6 && C.cols() == 6 && det(C) > 1E-6)
        _C = C;
}
