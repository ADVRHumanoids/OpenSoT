#include <OpenSoT/tasks/velocity/Interaction.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;


#define COMPLIANCE_INITIAL_VALUE 1E-6 //[m/N] & [rad/Nm]

Interaction::Interaction(std::string task_id,
                     const Eigen::VectorXd& x,
                     iDynUtils &robot,
                     std::string distal_link,
                     std::string base_link,
                     std::string ft_frame):
    Cartesian(task_id, x, robot, distal_link, base_link),
    _ft_frame(ft_frame),
    _ft_index(-1),
    _C(6,6)
{
    const moveit::core::LinkModel* ft_link = robot.moveit_robot_model->getLinkModel(ft_frame);

    _ft_index = robot.iDyn3_model.getFTSensorIndex(ft_link->getParentJointModel()->getName());
    if(_ft_index == -1)
        throw "Passed ft_frame is not in model!";

    _C = _C.setIdentity(6,6);
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
    Eigen::VectorXd wrench_in_sensor_frame(6);
    wrench_in_sensor_frame.setZero(6);
    _robot.getSensorMeasurement(_ft_index, wrench_in_sensor_frame);

    KDL::Wrench wrench_in_sensor_frame_KDL = cartesian_utils::toKDLWrench(wrench_in_sensor_frame);

    // Wrench has to be transformed from ft_frame to base_link **/
    Eigen::MatrixXd ft_frame_in_base_link(4,4);
    if(_base_link_is_world)
        ft_frame_in_base_link = _robot.getPosition(_robot.iDyn3_model.getLinkIndex(_ft_frame));
    else
        ft_frame_in_base_link = _robot.getPosition(
                _robot.iDyn3_model.getLinkIndex(_base_link),
                _robot.iDyn3_model.getLinkIndex(_ft_frame));

//    std::cout<<"base_link is "<<_base_link<<std::endl;
//    std::cout<<"ft_frame_in_base_link:"<<std::endl;
//    cartesian_utils::printHomogeneousTransform(ft_frame_in_base_link);std::cout<<std::endl;

    KDL::Frame ft_frame_in_base_link_KDL = cartesian_utils::toKDLFrame(ft_frame_in_base_link);

    if(_distal_link != _ft_frame){
        Eigen::MatrixXd ft_frame_to_distal_link(4,4);
        ft_frame_to_distal_link= _robot.getPosition(
                    _robot.iDyn3_model.getLinkIndex(_ft_frame),
                    _robot.iDyn3_model.getLinkIndex(_distal_link));
        KDL::Vector distance_in_distal_link_KDL(ft_frame_to_distal_link(0,3),
                                                ft_frame_to_distal_link(1,3),
                                                ft_frame_to_distal_link(2,3));
        wrench_in_sensor_frame_KDL.RefPoint(distance_in_distal_link_KDL);
    }

    KDL::Wrench wrench_in_base_link = ft_frame_in_base_link_KDL.M * wrench_in_sensor_frame_KDL;

    _actualWrench = cartesian_utils::toEigen(wrench_in_base_link);
}

Eigen::VectorXd Interaction::getWrenchError()
{
    return _desiredWrench - _actualWrench;
}

void Interaction::_update(const Eigen::VectorXd &x)
{
    updateActualWrench();

    Eigen::VectorXd wrench_error(6);
    wrench_error= getWrenchError();
    forceError = wrench_error.segment(0,3);
    torqueError = wrench_error.segment(3,3);
    Eigen::VectorXd delta_x(6);
    delta_x= _C * wrench_error;

    KDL::Twist delta_x_KDL = cartesian_utils::toKDLTwist(delta_x);

    //update desired position!
    KDL::Frame actual_ref_KDL = cartesian_utils::toKDLFrame(getActualPose());
    actual_ref_KDL.Integrate(delta_x_KDL, 1.0);
    Eigen::MatrixXd actual_ref(4,4);
    actual_ref = cartesian_utils::toEigen(actual_ref_KDL);

//    std::cout<<"xd: "<<std::endl;
//    cartesian_utils::printHomogeneousTransform(actual_ref);

    // We consider the delta_x as a feed_forward in velocity!
    setReference(actual_ref, delta_x);

    Cartesian::_update(x);
}

void Interaction::setReferenceWrench(const Eigen::VectorXd &desiredWrench) {
    assert(desiredWrench.size() == 6 && "Desired wrench musth be a 6-element vector");
    _desiredWrench = desiredWrench;
}

const Eigen::VectorXd Interaction::getReferenceWrench() const {
    return _desiredWrench;
}

const Eigen::VectorXd Interaction::getActualWrench() const
{
    return _actualWrench;
}

const Eigen::MatrixXd Interaction::getCompliance() const
{
    return _C;
}

void Interaction::setCompliance(const Eigen::MatrixXd& C)
{
    assert(C.rows() == 6 && C.cols() == 6 && C.transposed() == C &&
           "Matrix C must be 6x6 positive definite matrix");
    // Check size  [6x6] and if Positive Definite
    //if(C.rows() == 6 && C.cols() == 6 && det(C) >= 0.0)
        _C = C;
}
