 #include <OpenSoT/tasks/velocity/JointAdmittance.h>

using namespace OpenSoT::tasks::velocity;

JointAdmittance::JointAdmittance(XBot::ModelInterface &robot, XBot::ModelInterface &model):
    Postural(model, "joint_admittance"),
    _robot(robot),
    _model(model)
{
    _C.setIdentity(_W.rows(), _W.cols());
    _C *= 0.00015; //This was found by experiments

    _filter.setTimeStep(0.002); //This was found by experiments
    _filter.setDamping(1.); //This was found by experiments
    _filter.setOmega(2.*M_PI*8.); //This was found by experiments

    _lambda = 0.005; //This was found by experiments

    _tau.setZero(this->getXSize());
    _tau_filt.setZero(this->getXSize());
    _tau_error.setZero(this->getXSize());
    _h.setZero(this->getXSize());
    _qdot_desired.setZero(this->getXSize());


    _model.getJointPosition(_q);

    _update(Eigen::VectorXd(0));
}

void JointAdmittance::_update(const Eigen::VectorXd &x)
{
    _robot.getJointEffort(_tau);
    _robot.computeNonlinearTerm(_h);

    ///TODO: better estimation based on residuals?
    _tau_error = _h-_tau;

    _tau_filt = _filter.process(_tau_error);

    _qdot_desired = _C*_tau_filt;

    if(_model.isFloatingBase())
        _qdot_desired.head(6).setZero();

    Postural::_update(Eigen::VectorXd(0));
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
    return (bool)std::dynamic_pointer_cast<OpenSoT::tasks::velocity::JointAdmittance>(task);
}

JointAdmittance::Ptr JointAdmittance::asJointAdmittance(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<OpenSoT::tasks::velocity::JointAdmittance>(task);
}


