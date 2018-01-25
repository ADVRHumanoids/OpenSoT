#include <OpenSoT/tasks/MinimizeVariable.h>


void OpenSoT::tasks::MinimizeVariable::_update(const Eigen::VectorXd& x)
{

}

OpenSoT::tasks::MinimizeVariable::MinimizeVariable(std::string task_id, 
                                                   const OpenSoT::AffineHelper& variable): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, variable.getInputSize()),
    _var(variable)
{
    _A = _var.getM();
    _b = -_var.getq();
    _W.setIdentity(_A.rows(), _A.rows());
}