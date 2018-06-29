#include <OpenSoT/tasks/GenericLPTask.h>

using namespace OpenSoT::tasks;

GenericLPTask::GenericLPTask(const std::string &task_id, const Eigen::VectorXd &c):
    Task(task_id, c.size())
{
    Eigen::MatrixXd O(c.size(), c.size()); O.setZero(O.rows(), O.cols());
    Eigen::VectorXd o(c.size()); o.setZero(o.size());
    _task.reset(new GenericTask(task_id+"_internal", O, o));
    _hessianType = HST_ZERO;
    _task->setc(c);
    _task->update(Eigen::VectorXd(1));

    _update(Eigen::VectorXd(1));
    _W = _task->getWeight();
    _W.setZero(_W.rows(), _W.cols());

    if(!checkConsistency()){
        std::string s = "Can not initialize "+task_id+", unconsistencies have been found!\n";
        throw std::runtime_error(s.c_str());}
}

GenericLPTask::GenericLPTask(const std::string &task_id, const Eigen::VectorXd& c, const AffineHelper &var):
    Task(task_id, c.size())
{
    Eigen::MatrixXd O(c.size(), c.size()); O.setZero(O.rows(), O.cols());
    Eigen::VectorXd o(c.size()); o.setZero(o.size());
    _task.reset(new GenericTask(task_id+"_internal", O, o, var));
    _hessianType = HST_ZERO;
    _task->setc(c);
    _task->update(Eigen::VectorXd(1));

    _update(Eigen::VectorXd(1));
    _W = _task->getWeight();
    _W.setZero(_W.rows(), _W.cols());

    if(!checkConsistency()){
        std::string s = "Can not initialize "+task_id+", unconsistencies have been found!\n";
        throw std::runtime_error(s.c_str());}
}

void GenericLPTask::_update(const Eigen::VectorXd &x)
{
    _task->update(x);

    _A = _task->getA();
    _b = _task->getb();
    _c = _task->getc();
}

bool GenericLPTask::setc(const Eigen::VectorXd& c)
{
    return _task->setc(c);
}

GenericLPTask::~GenericLPTask()
{

}
