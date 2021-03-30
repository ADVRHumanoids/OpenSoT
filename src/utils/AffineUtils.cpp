#include <OpenSoT/utils/AffineUtils.h>
#include <boost/make_shared.hpp>


using namespace OpenSoT::AffineUtils;

///TASK:

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

///CONSTRAINT:
AffineConstraint::Ptr AffineConstraint::toAffine(const OpenSoT::constraints::Aggregated::ConstraintPtr& constraint,
                                          const AffineHelper& var)
{
    return boost::make_shared<AffineConstraint>(constraint , var);
}

AffineConstraint::AffineConstraint(const OpenSoT::constraints::Aggregated::ConstraintPtr& constraint,
           const AffineHelper& var):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >("Affine"+constraint->getConstraintID(), var.getInputSize()),
    _var(var),
    _internal_constraint(constraint)
{
    if(constraint->isBound())
    {
        _internal_generic_constraint = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                    "foo",
                    var,
                    constraint->getUpperBound(),
                    constraint->getLowerBound(),
                    OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);
    }
    else
    {
        if(constraint->isEqualityConstraint())
        {
           _constraint_affine = constraint->getAeq()*var;
           _internal_generic_constraint = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                       "foo",
                       _constraint_affine,
                       constraint->getbeq(),
                       constraint->getbeq(),
                       OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);
        }
        else
        {
           _constraint_affine = constraint->getAineq()*var;
           _internal_generic_constraint = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                       "foo",
                       _constraint_affine,
                       constraint->getbUpperBound(),
                       constraint->getbLowerBound(),
                       OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);
        }
    }
    update(Eigen::VectorXd(1));
}

AffineConstraint::~AffineConstraint()
{

}

void AffineConstraint::update(const Eigen::VectorXd& x)
{
    //1. Update internal constraint
    _internal_constraint->update(x);

    //2. Update internal generic constraint
    if(_internal_constraint->isBound())
    {
        _internal_generic_constraint->setBounds(
                    _internal_constraint->getUpperBound(),
                    _internal_constraint->getLowerBound());
        _internal_generic_constraint->update(x);
    }
    else
    {
        if(_internal_constraint->isEqualityConstraint())
        {
            _constraint_affine = _internal_constraint->getAeq()*_var;
            _internal_generic_constraint->setConstraint(
                    _constraint_affine,
                    _internal_constraint->getbeq(),
                    _internal_constraint->getbeq());
            _internal_constraint->update(x);
        }
        else
        {
            _constraint_affine = _internal_constraint->getAineq()*_var;
            _internal_generic_constraint->setConstraint(
                    _constraint_affine,
                    _internal_constraint->getbUpperBound(),
                    _internal_constraint->getbLowerBound());
            _internal_constraint->update(x);
        }
    }

    //3. Update Affine Constraint (NOTE: we know from constructor that generic constraint type is CONSTRAINT)
    _Aineq = _internal_generic_constraint->getAineq();
    _bLowerBound = _internal_generic_constraint->getbLowerBound();
    _bUpperBound = _internal_generic_constraint->getbUpperBound();
}

