#include <OpenSoT/utils/AffineUtils.h>
#include <boost/make_shared.hpp>


using namespace OpenSoT::AffineUtils;

AffineTask::Ptr AffineTask::toAffine(const OpenSoT::tasks::Aggregated::TaskPtr& task,
                         const OpenSoT::AffineHelper& var)
{
    return boost::make_shared<AffineTask>(task , var);
}

AffineTask::AffineTask(const OpenSoT::tasks::Aggregated::TaskPtr &task, const AffineHelper &var):
    Task("Affine"+task->getTaskID(), var.getInputSize()),
    _internal_task(task)
{
    _internal_generic_task = boost::make_shared<OpenSoT::tasks::GenericTask>(
                "foo", task->getA(), task->getb(), var);
    _lambda = 1.;
    _internal_generic_task->setWeight(task->getWeight());

    _update(Eigen::VectorXd(1));
}

AffineTask::~AffineTask()
{

}

void AffineTask::_update(const Eigen::VectorXd &x)
{
    //1. Update original task which will be used to set desired quantities
    _internal_task->update(x);

    //2. Update internal generic task and get matrices with affines
    _internal_generic_task->setA(_internal_task->getA());
    _internal_generic_task->setb(_internal_task->getb());
    _internal_generic_task->setc(_internal_task->getc());
    _internal_generic_task->setWeight(_internal_task->getWeight());
    _internal_generic_task->update(x);

    //3. Update Affine Task
    _A = _internal_generic_task->getA();
    _b = _internal_generic_task->getb();
    _c = _internal_generic_task->getc();
    _W = _internal_generic_task->getWeight();
}
