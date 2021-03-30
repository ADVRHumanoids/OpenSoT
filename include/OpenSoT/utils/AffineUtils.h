#ifndef _AFFINE_UTILS_H_
#define _AFFINE_UTILS_H_

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/GenericTask.h>


namespace OpenSoT
{
namespace AffineUtils
{

class AffineTask: public Task<Eigen::MatrixXd, Eigen::VectorXd>
{
public:
    typedef boost::shared_ptr<AffineTask> Ptr;

    AffineTask(const OpenSoT::tasks::Aggregated::TaskPtr& task,
               const AffineHelper& var);

    virtual void _update(const Eigen::VectorXd &x);
    ~AffineTask();

    static AffineTask::Ptr toAffine(const OpenSoT::tasks::Aggregated::TaskPtr& task,
                                    const AffineHelper& var);

    OpenSoT::tasks::Aggregated::TaskPtr& getTask(){ return _internal_task; }

private:
    OpenSoT::tasks::GenericTask::Ptr _internal_generic_task;
    OpenSoT::tasks::Aggregated::TaskPtr _internal_task;

};



}
}

#endif
