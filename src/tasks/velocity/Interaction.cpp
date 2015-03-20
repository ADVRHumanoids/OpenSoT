#include <OpenSoT/tasks/velocity/Interaction.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

#define COMPLIANCE_INITIAL_VALUE 1E-6 //[m/N] & [rad/Nm]

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
    _C = COMPLIANCE_INITIAL_VALUE*_C;

    updateActualWrench();

    _desiredWrench = _actualWrench;

    _update(x);
}

Interaction::~Interaction()
{
}

void Interaction::updateActualWrench()
{
    // Get ft measurement in sensor frame
    yarp::sig::Vector wrench_in_sensor_frame(6, 0.0);
    _robot.iDyn3_model.getSensorMeasurement(_ft_index, wrench_in_sensor_frame);

    KDL::Wrench wrench_in_sensor_frame_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_in_sensor_frame, wrench_in_sensor_frame_KDL);

    // Wrench has to be transformed from ft_frame to base_link **/
    yarp::sig::Matrix ft_frame_in_base_link;
    if(_base_link_is_world)
        ft_frame_in_base_link = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(_ft_frame));
    else
        ft_frame_in_base_link = _robot.iDyn3_model.getPosition(
                _robot.iDyn3_model.getLinkIndex(_base_link),
                _robot.iDyn3_model.getLinkIndex(_ft_frame));

    KDL::Frame ft_frame_in_base_link_KDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(ft_frame_in_base_link, ft_frame_in_base_link_KDL);

    KDL::Wrench wrench_in_base_link = ft_frame_in_base_link_KDL * wrench_in_sensor_frame_KDL; //Adjoint matrix is automatically computed by KDL!
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_in_base_link, _actualWrench);
}

void Interaction::_update(const yarp::sig::Vector &x)
{
    updateActualWrench();

    yarp::sig::Vector wrench_error = _desiredWrench - _actualWrench;
    forceError = wrench_error.subVector(0, 2);
    torqueError = wrench_error.subVector(3, 5);
    yarp::sig::Vector delta_x = _C * wrench_error;

    KDL::Twist delta_x_KDL;
    cartesian_utils::fromYARPVectortoKDLTwist(delta_x, delta_x_KDL);

    //update desired position!
    KDL::Frame actual_ref_KDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(getActualPose(), actual_ref_KDL);
    actual_ref_KDL.Integrate(delta_x_KDL, 1.0);
    yarp::sig::Matrix actual_ref(4,4);
    cartesian_utils::fromKDLFrameToYARPMatrix(actual_ref_KDL, actual_ref);

//    std::cout<<"xd: "<<std::endl;
//    cartesian_utils::printHomogeneousTransform(actual_ref);

    // We consider the delta_x as a feed_forward in velocity!
    setReference(actual_ref, delta_x);

    Cartesian::_update(x);
}

void Interaction::setReferenceWrench(const yarp::sig::Vector &desiredWrench) {
    assert(desiredWrench.size() == 6 && "Desired wrench musth be a 6-element vector");
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
    assert(C.rows() == 6 && C.cols() == 6 && C.transposed() == C &&
           "Matrix C must be 6x6 positive definite matrix");
    // Check size  [6x6] and if Positive Definite
    //if(C.rows() == 6 && C.cols() == 6 && det(C) >= 0.0)
        _C = C;
}
