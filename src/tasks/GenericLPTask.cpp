#include <OpenSoT/tasks/GenericLPTask.h>

using namespace OpenSoT::tasks;

GenericLPTask::GenericLPTask(const std::string &task_id, const Eigen::VectorXd &c):
    Task(task_id, c.size())
{
    Eigen::MatrixXd O(0, c.size()); O.setZero(0, O.cols());
    Eigen::VectorXd o;// o.setZero(0);
    _task.reset(new GenericTask(task_id+"_internal", O, o));
    _hessianType = HST_ZERO;
    _task->setc(c);
    _task->update();

    _update();
    _W = _task->getWeight();
    _W.setZero(_W.rows(), _W.cols());

    if(!checkConsistency()){
        std::string s = "Can not initialize "+task_id+", unconsistencies have been found!\n";
        throw std::runtime_error(s.c_str());}
}

GenericLPTask::GenericLPTask(const std::string &task_id, const Eigen::VectorXd& c, const AffineHelper &var):
    Task(task_id, var.getInputSize())
{
    Eigen::MatrixXd O(0, var.getOutputSize()); O.setZero(0, O.cols());
    Eigen::VectorXd o;// o.setZero(o.size());

    _task.reset(new GenericTask(task_id+"_internal", O, o, var));
    _hessianType = HST_ZERO;
    _task->setc(c);
    _task->update();

    _update();
    _W = _task->getWeight();
    _W.setZero(_W.rows(), _W.cols());

    if(!checkConsistency()){
        std::string s = "Can not initialize "+task_id+", unconsistencies have been found!\n";
        throw std::runtime_error(s.c_str());}
}

void GenericLPTask::_update()
{
    _task->update();

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
