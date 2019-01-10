#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>

using namespace OpenSoT::tasks::velocity;

#define CHANNELS 6

CartesianAdmittance::CartesianAdmittance(std::string task_id,
                                         const Eigen::VectorXd &x,
                                         XBot::ModelInterface &robot,
                                         std::string base_link,
                                         XBot::ForceTorqueSensor::ConstPtr ft_sensor):
    Cartesian(task_id, x, robot, ft_sensor->getSensorName(), base_link),
    _ft_sensor(ft_sensor),
    _filter(CHANNELS)
{
    _wrench_reference.setZero();
    _wrench_measured.setZero();
    _wrench_filt.setZero();

    _ft_sensor->getWrench(_wrench_measured);
    _wrench_reference = _wrench_measured;

//    _C.setIdentity();
//    _C.segment(0,3) *= 1e-6; //This was found by experiments
//    _C.segment(3,3) *= 1e-7; //This was found by experiments

//    for(unsigned int i = 0; i < _filter.getNumberOfChannels(); ++i)
//        _filter.setOmega(2.*M_PI*2., i); //This was found by experiments

    _K.setIdentity();
    _K.segment(0,3) *= 1e4; _K.segment(3,3) *= 1e5;
    _D.setIdentity();
    _D.segment(0,3) *= 1e3*0.7958; _D.segment(3,3) *= 1e3*7.9577;

    setFilterTimeStep(0.002);
    setFilterDamping(1.);

    _lambda = 0.01; //This was found by experiments

    _tmp.assign(CHANNELS, 0.);
}

void CartesianAdmittance::_update(const Eigen::VectorXd &x)
{
    _ft_sensor->getWrench(_wrench_measured);
    _robot.getPose(_distal_link, _base_link, _bl_T_ft);

    _wrench_error = XBot::Utils::GetAdjointFromRotation(_bl_T_ft.linear())*_wrench_measured - _wrench_reference;

    Eigen::Vector6d::Map(&_tmp[0], CHANNELS) = _wrench_error;
    _wrench_filt = Eigen::Vector6d::Map(_filter.process(_tmp).data(), CHANNELS);

    _desiredTwist = _C.asDiagonal()*_wrench_filt;

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

const Eigen::Matrix6d& CartesianAdmittance::getCartesianCompliance()
{
    _tmp_mat6 = _C.asDiagonal();
    return _tmp_mat6;
}

void CartesianAdmittance::getCartesianCompliance(Eigen::Matrix6d& C)
{
    C = _C.asDiagonal();
}

void CartesianAdmittance::setFilterTimeStep(const double time_step)
{
    if(time_step > 0.)
    {
        _dt = time_step;
        for(unsigned int i = 0; i < _filter.getNumberOfChannels(); ++i)
            _filter.setTimeStep(time_step, i);
        computeParameters(_K, _D, _lambda, _dt, _C, _M, _w);
        setFilterOmega(_w);
    }
    else
        XBot::Logger::warning("time_step filter is negative!");
}

double CartesianAdmittance::getFilterTimeStep()
{
    return _dt;
}

void CartesianAdmittance::setFilterDamping(const double damping)
{
    if(damping >= 0.)
    {
        for(unsigned int i = 0; i < _filter.getNumberOfChannels(); ++i)
            _filter.setDamping(damping,i);
    }
    else
        XBot::Logger::warning("damping filter is negative!");
}

void CartesianAdmittance::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(_task_id + "_wrench_error", _wrench_error);
    logger->add(_task_id + "_wrench_filt", _wrench_filt);
    logger->add(_task_id + "_wrench_measured", _wrench_measured);
    logger->add(_task_id + "_wrench_reference", _wrench_reference);

    logger->add(_task_id + "_C", _C);
    logger->add(_task_id + "_M", _M);
    logger->add(_task_id + "_D", _D);
    logger->add(_task_id + "_K", _K);
    logger->add(_task_id + "_w", _w);
    logger->add(_task_id + "_dt", _dt);

    Cartesian::_log(logger);
}

const Eigen::Matrix6d& CartesianAdmittance::getStiffness()
{
    _tmp_mat6 = _K.asDiagonal();
    return _tmp_mat6;
}

void CartesianAdmittance::setStiffness(const Eigen::Vector6d& K)
{
    _K = K;
    computeParameters(_K, _D, _lambda, _dt, _C, _M, _w);
    setFilterOmega(_w);
}

const Eigen::Matrix6d& CartesianAdmittance::getDamping()
{
    _tmp_mat6 = _D.asDiagonal();
    return _tmp_mat6;
}

void CartesianAdmittance::setDamping(const Eigen::Vector6d& D)
{
    _D = D;
    computeParameters(_K, _D, _lambda, _dt, _C, _M, _w);
    setFilterOmega(_w);
}

bool CartesianAdmittance::computeParameters(const Eigen::Vector6d& K, const Eigen::Vector6d& D, const double lambda, const double dt,
                                            Eigen::Vector6d& C, Eigen::Vector6d& M, Eigen::Vector6d& w)
{
    C = lambda * (K.asDiagonal().inverse()).diagonal();

    M = (dt/lambda) * D - ((dt*dt)/lambda) * C;

    _tmp_vec6 = C.array()*M.array();
    w = dt*(_tmp_vec6.asDiagonal().inverse()).diagonal();
}

void CartesianAdmittance::setLambda(double lambda)
{
    if(lambda >= 0.0)
        this->_lambda = lambda;
    computeParameters(_K, _D, _lambda, _dt, _C, _M, _w);
    setFilterOmega(_w);
}

void CartesianAdmittance::setFilterOmega(const Eigen::Vector6d& w)
{
    for(unsigned int i = 0; i < CHANNELS; ++i)
        _filter.setOmega(w[i], i);
}
