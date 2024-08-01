#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>

using namespace OpenSoT::tasks::velocity;

#define CHANNELS 6

CartesianAdmittance::CartesianAdmittance(std::string task_id,
                                         XBot::ModelInterface &robot,
                                         std::string base_link,
                                         XBot::ForceTorqueSensor::ConstPtr ft_sensor):
    Cartesian(task_id, robot, ft_sensor->getName(), base_link),
    _ft_sensor(ft_sensor),
    _filter(CHANNELS)
{
    _deadzone.setZero();
    
    _wrench_reference.setZero();
    _wrench_measured.setZero();
    _wrench_filt.setZero();

    _ft_sensor->getWrench(_wrench_measured);

//    _C.setIdentity();
//    _C.segment(0,3) *= 1e-6; //This was found by experiments
//    _C.segment(3,3) *= 1e-7; //This was found by experiments

//    for(unsigned int i = 0; i < _filter.getNumberOfChannels(); ++i)
//        _filter.setOmega(2.*M_PI*2., i); //This was found by experiments

    _dt = 0.002;
    _lambda = 0.01;

    _K.setOnes();
    _K.segment(0,3) *= 1e4; _K.segment(3,3) *= 1e5;
    _D.setOnes();
    _D = 1.1*_K*_dt/_lambda;



    setFilterDamping(1.);

     //This was found by experiments

    if(!computeParameters(_K, _D, _lambda, _dt, _C, _M, _w))
        throw std::runtime_error("Error in computeParameters!");

    _tmp.assign(CHANNELS, 0.);
}

void CartesianAdmittance::_update()
{
    _ft_sensor->getWrench(_wrench_measured);
    if(_base_link == "world")
    {
        _robot.getPose(_distal_link, _bl_T_ft);
    }
    else 
    {
        _robot.getPose(_distal_link, _base_link, _bl_T_ft);
    }
    
    _wrench_error = XBot::Utils::adjointFromRotation(_bl_T_ft.linear())*_wrench_measured;
    
    apply_deadzone(_wrench_error);
    
    _wrench_error -= _wrench_reference;

    Eigen::Vector6d::Map(&_tmp[0], CHANNELS) = _wrench_error;
    _wrench_filt = Eigen::Vector6d::Map(_filter.process(_tmp).data(), CHANNELS);
    

    // Adding to the desired twist allows to still use a desired twist additionally
    // given by Cartesian::setReference(..., const Eigen::Vector6d& desiredTwist)
    _desiredTwist += _C.asDiagonal()*_wrench_filt;

    Cartesian::_update();
}

bool CartesianAdmittance::reset()
{
    _ft_sensor->getWrench(_wrench_measured);
    _wrench_reference.setZero();

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
    return (bool)std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CartesianAdmittance>(task);
}

OpenSoT::tasks::velocity::CartesianAdmittance::Ptr CartesianAdmittance::asCartesianAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CartesianAdmittance>(task);
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

void CartesianAdmittance::_log(XBot::MatLogger2::Ptr logger)
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
    
    logger->add(_task_id + "_deadzone", _deadzone);

    Cartesian::_log(logger);
}

const Eigen::Matrix6d CartesianAdmittance::getStiffness()
{
    return _K.asDiagonal();
}

const Eigen::Matrix6d CartesianAdmittance::getDamping()
{
    return _D.asDiagonal();
}



bool CartesianAdmittance::computeParameters(const Eigen::Vector6d& K, 
                                            const Eigen::Vector6d& D, 
                                            const double lambda, 
                                            const double dt,
                                            Eigen::Vector6d& C, 
                                            Eigen::Vector6d& M, 
                                            Eigen::Vector6d& w)
{
    for(unsigned int i = 0; i < 6; ++i)
    {
        if(D[i] <= (K[i]*dt)/lambda)
        {
            XBot::Logger::error("D[%d] <= (K[%d]*dt)/lambda \n", i, i);

            XBot::Logger::error("D[%d] = %f \n",i, D[i]);
            XBot::Logger::error("(K[%d]*dt)/lambda = %f \n",i, (K[i]*dt)/lambda);
            return false;
        }
    }

    C = lambda * (K.asDiagonal().inverse()).diagonal();

    M = (dt/lambda) * D - ((dt*dt)/lambda) * (C.asDiagonal().inverse()).diagonal();

    _tmp_vec6 = C.array()*M.array();
    w = dt*(_tmp_vec6.asDiagonal().inverse()).diagonal();

    return true;
}



void CartesianAdmittance::setImpedanceParams(const Eigen::Vector6d& K, 
                                             const Eigen::Vector6d& D, 
                                             const double lambda,
                                             const double dt)
{
   bool ok = true;
   for(unsigned int i = 0; i < 6; ++i)
   {
       if(D[i] <= 0.0)
       {
           ok = false;
           XBot::Logger::error("D[%d] <= 0.0 \n", i);
       }
       if(K[i] <= 0.0)
       {
           ok = false;
           XBot::Logger::error("K[%d] <= 0.0 \n", i);
       }
   }
   if(lambda <= 0.0 && lambda > 1.)
   {
       ok = false;
       XBot::Logger::error("lambda <= 0.0 or lambda > 1.0 \n");
   }
   if(dt <= 0.)
   {
       ok = false;
       XBot::Logger::error("time_step filter is negative!");
   }

   if(ok)
   {
        if(computeParameters(K, D, lambda, dt, _C, _M, _w))
        {
            this->_lambda = lambda;
            _D = D;
            _K = K;
            _dt = dt;
            setFilterOmega(_w);
        }
   }
}

bool OpenSoT::tasks::velocity::CartesianAdmittance::setRawParams(const Eigen::Vector6d& C, 
                                                                 const Eigen::Vector6d& omega, 
                                                                 const double lambda, 
                                                                 const double dt)
{

    if(lambda != 0)
    {
        throw std::invalid_argument("TBD lambda > 0");
    }

    if((C.array() < 0).any())
    {
        return false;
    }
    
    if((omega.array() < 0).any())
    {
        return false;
    }
    
    if(lambda < 0 || lambda > 1)
    {
        return false;
    }
    
    if(dt < 0)
    {
        return false;
    }
    
    _C = C;
    
    for(unsigned int i = 0; i < _filter.getNumberOfChannels(); ++i)
    {
        _filter.setTimeStep(dt, i);
        _filter.setOmega(omega[i], i);
    }
    
    _lambda = lambda;
    
    _K.setZero();
    _D = _C * dt;
    _M = _C.cwiseProduct(omega).cwiseInverse() * dt;
    _w = omega;
    _dt = dt;
    
    
    return true;
}

void CartesianAdmittance::setFilterOmega(const Eigen::Vector6d& w)
{
    for(unsigned int i = 0; i < CHANNELS; ++i)
        _filter.setOmega(w[i], i);
}

void CartesianAdmittance::apply_deadzone(Eigen::Vector6d& data)
{
    for(int i = 0; i < 6; i++)
    {
        if(data[i] > _deadzone[i])
        {
            data[i] -= _deadzone[i];
        }
        else if(data[i] < -_deadzone[i])
        {
            data[i] += _deadzone[i];
        }
        else
        {
            data[i] = 0.0;
        }
    }
}

bool OpenSoT::tasks::velocity::CartesianAdmittance::setDeadZone(const Eigen::Vector6d& dead_zone_amplitude)
{
    if((dead_zone_amplitude.array() < 0).any())
    {
        return false;
    }
    
    _deadzone = dead_zone_amplitude;
    return true;
}

const Eigen::Matrix6d  OpenSoT::tasks::velocity::CartesianAdmittance::getInertia()
{
    return _M.asDiagonal();
}


