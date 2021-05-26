#ifndef _AFFINE_UTILS_H_
#define _AFFINE_UTILS_H_

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>


namespace OpenSoT
{
namespace AffineUtils
{

class AffineTask: public Task<Eigen::MatrixXd, Eigen::VectorXd>
{
public:
    typedef std::shared_ptr<AffineTask> Ptr;

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

class AffineConstraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{
public:
    typedef std::shared_ptr<AffineConstraint> Ptr;

    AffineConstraint(const OpenSoT::constraints::Aggregated::ConstraintPtr& constraint,
               const AffineHelper& var);

    virtual void update(const Eigen::VectorXd& x);
    ~AffineConstraint();

    static AffineConstraint::Ptr toAffine(const OpenSoT::constraints::Aggregated::ConstraintPtr& constraint,
                                          const AffineHelper& var);

    OpenSoT::constraints::Aggregated::ConstraintPtr& getConstraint(){ return _internal_constraint; }

private:
    OpenSoT::constraints::GenericConstraint::Ptr _internal_generic_constraint;
    OpenSoT::constraints::Aggregated::ConstraintPtr _internal_constraint;

    AffineHelper _constraint_affine; //this is used to map the constraint into an affine
    AffineHelper _var;
};



}
}

#endif
