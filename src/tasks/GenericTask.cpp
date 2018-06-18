#include <OpenSoT/tasks/GenericTask.h>

using namespace OpenSoT::tasks;

GenericTask::GenericTask(const std::string &task_id, const Eigen::MatrixXd &A, const Eigen::VectorXd& b):
    Task(task_id, A.cols())
{
    if(A.rows() != b.size())
        throw std::runtime_error(task_id + " has A.rows() != b.size()");

    __A = A;
    __b = b;

    _var = AffineHelper::Identity(A.cols());

    _hessianType = HST_SEMIDEF;

    _update(Eigen::VectorXd(1));
    
    _W.setIdentity(_A.rows(), _A.rows());
}

GenericTask::GenericTask(const std::string &task_id, const Eigen::MatrixXd &A, const Eigen::VectorXd& b, const AffineHelper &var):
    Task(task_id, var.getInputSize()),
    _var(var)
{
    if(A.rows() != b.size())
        throw std::runtime_error(task_id + " has A.rows() != b.size()");

    __A = A;
    __b = b;

    _hessianType = HST_SEMIDEF;

    _W.setIdentity(_A.rows(), _A.rows());

    _update(Eigen::VectorXd(1));
}

GenericTask::~GenericTask()
{

}

void GenericTask::_update(const Eigen::VectorXd &x)
{
    _task = __A*_var - __b;

    _A = _task.getM();
    _b = -_task.getq();
}

bool GenericTask::setc(const Eigen::VectorXd& c)
{
    if(c.size() != _b.size())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct" << XBot::Logger::endl();
        return false;
    }

    _c = c;

    return true;
}

bool GenericTask::setA(const Eigen::MatrixXd& A)
{
    if(A.rows() != _b.size())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct" << XBot::Logger::endl();
        return false;
    }

    if(A.cols() != _var.getInputSize())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct A.cols() != x.size()" << XBot::Logger::endl();
        return false;
    }

    __A = A;
    return true;
}

bool GenericTask::setb(const Eigen::VectorXd& b)
{
    if(_A.rows() != b.size())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct" << XBot::Logger::endl();
        return false;
    }

    __b = b;
    return true;
}

bool GenericTask::setAb(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    if(A.rows() != b.size())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct A.rows() != b.size()" << XBot::Logger::endl();
        return false;
    }

    if(A.cols() != _var.getInputSize())
    {
        XBot::Logger::error() << "in " << __func__ << ": size not correct A.cols() != x.size()" << XBot::Logger::endl();
        return false;
    }

    if(A.rows() != _A.rows())
        _W.setIdentity(A.rows(), A.rows());

    __A = A;
    __b = b;

    return true;
}

void GenericTask::setHessianType(const HessianType hessian_type)
{
    _hessianType = hessian_type;
}


