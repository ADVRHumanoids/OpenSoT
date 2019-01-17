 #include <OpenSoT/tasks/velocity/JointAdmittance.h>

using namespace OpenSoT::tasks::velocity;

JointAdmittance::JointAdmittance(XBot::ModelInterface &model, const Eigen::VectorXd& x):
    Postural(x, "joint_admittance"),
    _model(model)
{
    _deadzone.setZero(this->getXSize());

    _model.getJointEffort(_tau);
    _tau_ref.setZero(_tau.size());

    _C.setIdentity(_W.rows(), _W.cols());
    _C *= 0.00015; //This was found by experiments

    _filter.setTimeStep(0.002); //This was found by experiments
    _filter.setDamping(1.); //This was found by experiments
    _filter.setOmega(2.*M_PI*8.); //This was found by experiments

    _lambda = 0.005; //This was found by experiments

    _tau_filt.setZero(this->getXSize());
    _tau_error.setZero(this->getXSize());
    _tau_ref.setZero(this->getXSize());


    _model.getJointPosition(_q);

    if( (x-_q).sum() < 1e-9 )
        _update(x);
    else
        throw std::runtime_error("There is a mismatch among model joint position and input x!");
}

bool JointAdmittance::reset()
{
    _model.getJointEffort(_tau);
    _tau_ref.setZero(_tau_ref.size());

    return Postural::reset();
}

void JointAdmittance::setTorqueReference(const Eigen::VectorXd& tau_ref)
{
    _tau_ref = tau_ref;
}

const Eigen::VectorXd& JointAdmittance::getTorqueReference()
{
    return _tau_ref;
}

void JointAdmittance::getTorqueReference(Eigen::VectorXd& tau_ref)
{
    tau_ref = _tau_ref;
}

void JointAdmittance::_update(const Eigen::VectorXd &x)
{
    _model.getJointEffort(_tau);

    _tau_error = _tau_ref-_tau;

    _tau_filt = _filter.process(_tau_error);

    apply_deadzone(_tau_filt);

    _xdot_desired = _C*_tau_filt;

    if(_model.isFloatingBase())
        _xdot_desired.head(6).setZero();

    _model.getJointPosition(_q);
    Postural::_update(_q);
}

void JointAdmittance::setJointCompliance(const Eigen::MatrixXd &C)
{
    if(C.rows() == C.cols() && C.rows() == this->getXSize())
        _C = C;
    else
        XBot::Logger::warning("C has wrong size matrix!");
}

void JointAdmittance::setJointCompliance(const double C)
{
    _C.setIdentity(_C.rows(), _C.cols());
    _C*=C;
}

const Eigen::MatrixXd& JointAdmittance::getJointCompliance()
{
    return _C;
}

void JointAdmittance::getJointCompliance(Eigen::MatrixXd& C)
{
    C = _C;
}

void JointAdmittance::setFilterTimeStep(const double time_step)
{
    if(time_step > 0)
        _filter.setTimeStep(time_step);
    else
        XBot::Logger::warning("time_step filter is negative!");
}

void JointAdmittance::setFilterDamping(const double damping)
{
    if(damping)
        _filter.setDamping(damping);
    else
        XBot::Logger::warning("damping filter is negative!");
}

void JointAdmittance::setFilterOmega(const double omega)
{
    if(omega)
        _filter.setOmega(omega);
    else
        XBot::Logger::warning("omega filter is negative!");
}

void JointAdmittance::setFilterParams(const double time_step, const double damping, const double omega)
{
    setFilterTimeStep(time_step);
    setFilterOmega(omega);
    setFilterDamping(damping);
}

bool JointAdmittance::isJointAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::JointAdmittance>(task);
}

JointAdmittance::Ptr JointAdmittance::asJointAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::JointAdmittance>(task);
}

void JointAdmittance::apply_deadzone(Eigen::VectorXd& data)
{
    for(int i = 0; i < data.size(); i++)
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


