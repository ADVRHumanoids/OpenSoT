#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>

using namespace OpenSoT::tasks::velocity;

CartesianAdmittance::CartesianAdmittance(std::string task_id,
                                         const Eigen::VectorXd &x,
                                         XBot::ModelInterface &robot,
                                         std::string distal_link,
                                         std::string base_link,
                                         XBot::ForceTorqueSensor::ConstPtr ft_sensor):
    Cartesian(task_id, x, robot, distal_link, base_link),
    _ft_sensor(ft_sensor)
{
    _wrench_reference.setZero();
    _wrench_measured.setZero();
    _wrench_filt.setZero();

    _ft_sensor->getWrench(_wrench_measured);
    _wrench_reference = _wrench_measured;

    _C.setIdentity();
    _C *= 0.00015; //This was found by experiments

    _filter.setTimeStep(0.002); //This was found by experiments
    _filter.setDamping(1.); //This was found by experiments
    _filter.setOmega(2.*M_PI*8.); //This was found by experiments

    _lambda = 0.005; //This was found by experiments
}

void CartesianAdmittance::_update(const Eigen::VectorXd &x)
{
    _ft_sensor->getWrench(_wrench_measured);
    _wrench_error = _wrench_reference - _wrench_measured;

    _wrench_filt = _filter.process(_wrench_error);

    _desiredTwist = _C*_wrench_filt;

    Cartesian::_update(x);
}

bool CartesianAdmittance::reset()
{
    _ft_sensor->getWrench(_wrench_measured);
    _wrench_reference = _wrench_measured;

    return Cartesian::reset();
}

void CartesianAdmittance::setWrenchReference(const Eigen::Vector6d& wrench)
{
    _wrench_reference = wrench;
}

const Eigen::Vector6d& CartesianAdmittance::getWrenchReference()
{
    return _wrench_reference;
}

void CartesianAdmittance::getWrenchReference(Eigen::Vector6d& wrench_reference)
{
    wrench_reference = _wrench_reference;
}

bool CartesianAdmittance::isCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CartesianAdmittance>(task);
}

OpenSoT::tasks::velocity::CartesianAdmittance::Ptr CartesianAdmittance::asCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CartesianAdmittance>(task);
}

void CartesianAdmittance::setCartesianCompliance(const Eigen::Matrix6d& C)
{
    _C = C;
}

void CartesianAdmittance::setCartesianCompliance(const double C_linear, const double C_angular)
{
    _C.setIdentity();

    _C.block(0,0,3,3) *= C_linear;
    _C.block(3,3,3,3) *= C_angular;
}

const Eigen::Matrix6d& CartesianAdmittance::getCartesianCompliance()
{
    return _C;
}

void CartesianAdmittance::getCartesianCompliance(Eigen::MatrixXd& C)
{
    C = _C;
}

void CartesianAdmittance::setFilterParams(const double time_step, const double damping, const double omega)
{
    setFilterTimeStep(time_step);
    setFilterOmega(omega);
    setFilterDamping(damping);
}

void CartesianAdmittance::setFilterTimeStep(const double time_step)
{
    if(time_step > 0)
        _filter.setTimeStep(time_step);
    else
        XBot::Logger::warning("time_step filter is negative!");
}

void CartesianAdmittance::setFilterDamping(const double damping)
{
    if(damping)
        _filter.setDamping(damping);
    else
        XBot::Logger::warning("damping filter is negative!");
}

void CartesianAdmittance::setFilterOmega(const double omega)
{
    if(omega)
        _filter.setOmega(omega);
    else
        XBot::Logger::warning("omega filter is negative!");
}
