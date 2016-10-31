#include <OpenSoT/utils/VelocityAllocation.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

using namespace OpenSoT;

VelocityAllocation::VelocityAllocation(OpenSoT::AutoStack::Ptr autoStack,
                                       const double dT,
                                       const double min_velocity,
                                       const double max_velocity) :
    _dT(dT),
    _min_velocity(min_velocity),
    _max_velocity(max_velocity),
    _last_velocity(-1.0)
{
    this->processStack(autoStack->getStack());

    typedef std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>::const_iterator it_c;
    for(it_c constraint = autoStack->getBoundsList().begin();
        constraint != autoStack->getBoundsList().end();
        ++constraint)
    {
        if(boost::dynamic_pointer_cast<
            OpenSoT::constraints::velocity::VelocityLimits>(
                *constraint))
        {
            OpenSoT::constraints::velocity::VelocityLimits::Ptr velocityLimits =
                boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        *constraint);
            assert(_dT == velocityLimits->getDT());
            velocityLimits->setVelocityLimits(_max_velocity);
        }
    }
}

VelocityAllocation::VelocityAllocation(OpenSoT::AutoStack::Ptr autoStack,
                                       const double dT,
                                       const double min_velocity,
                                       const double max_velocity,
                                       const double last_velocity) :
    _dT(dT),
    _min_velocity(min_velocity),
    _max_velocity(max_velocity),
    _last_velocity(last_velocity)
{
    this->processStack(autoStack->getStack());

    typedef std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>::const_iterator it_c;
    for(it_c constraint = autoStack->getBoundsList().begin();
        constraint != autoStack->getBoundsList().end();
        ++constraint)
    {
        if(boost::dynamic_pointer_cast<
            OpenSoT::constraints::velocity::VelocityLimits>(
                *constraint))
        {
            OpenSoT::constraints::velocity::VelocityLimits::Ptr velocityLimits =
                boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        *constraint);
            assert(_dT == velocityLimits->getDT());
            velocityLimits->setVelocityLimits(_last_velocity);
        }
    }
}

VelocityAllocation::VelocityAllocation(OpenSoT::AutoStack::Ptr autoStack,
                                       const double dT,
                                       const std::vector<double> velocity_vector) :
    _dT(dT),
    _min_velocity(-1.0),
    _max_velocity(-1.0),
    _last_velocity(-1.0),
    _velocity_vector(velocity_vector)
{
    if(_velocity_vector.size() != 0 && _velocity_vector.size() != autoStack->getStack().size())
        throw "Error: elements in velocity_vector do not match number of tasks in the stack";

    this->processStack(autoStack->getStack());

    typedef std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>::const_iterator it_c;
    for(it_c constraint = autoStack->getBoundsList().begin();
        constraint != autoStack->getBoundsList().end();
        ++constraint)
    {
        if(boost::dynamic_pointer_cast<
            OpenSoT::constraints::velocity::VelocityLimits>(
                *constraint))
        {
            OpenSoT::constraints::velocity::VelocityLimits::Ptr velocityLimits =
                boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        *constraint);
            assert(_dT == velocityLimits->getDT());
            velocityLimits->setVelocityLimits(_last_velocity);
        }
    }
}

VelocityAllocation::VelocityAllocation(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack& stack,
                                       const double dT,
                                       const double min_velocity,
                                       const double max_velocity) :
    _dT(dT),
    _min_velocity(min_velocity),
    _max_velocity(max_velocity),
    _last_velocity(-1.0)
{
    this->processStack(stack);
}

VelocityAllocation::VelocityAllocation(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack& stack,
                                       const double dT,
                                       const double min_velocity,
                                       const double max_velocity,
                                       const double last_velocity) :
    _dT(dT),
    _min_velocity(min_velocity),
    _max_velocity(max_velocity),
    _last_velocity(last_velocity)
{
    this->processStack(stack);
}

VelocityAllocation::VelocityAllocation(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack& stack,
                                       const double dT,
                                       const std::vector<double> velocity_vector) :
    _dT(dT),
    _min_velocity(-1.0),
    _max_velocity(-1.0),
    _last_velocity(-1.0),
    _velocity_vector(velocity_vector)

{
    if(_velocity_vector.size() != 0 && _velocity_vector.size() != stack.size())
        throw "Error: elements in velocity_vector do not match number of tasks in the stack";

    this->processStack(stack);
}

void VelocityAllocation::processStack(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack& stack)
{
    for(unsigned int i = 0; i < stack.size(); ++i)
    {
        const double velocityLimit = this->computeVelocityLimit(i, stack.size());
        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task = stack[i];
        OpenSoT::constraints::velocity::VelocityLimits::Ptr velocityLimits;

        // TODO notice this does not work if every stack has already a VelocityLimits
        // but they are instances of the same object - we should check duplication
        typedef std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>::const_iterator it_c;
        for(it_c constraint = task->getConstraints().begin();
            constraint != task->getConstraints().end();
            ++constraint)
            if(boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        *constraint))
            {
                velocityLimits = boost::dynamic_pointer_cast<
                        OpenSoT::constraints::velocity::VelocityLimits>(
                            *constraint);
                assert(_dT == velocityLimits->getDT());
                velocityLimits->setVelocityLimits(velocityLimit);
            }

        if(!velocityLimits)
        {
            velocityLimits = OpenSoT::constraints::velocity::VelocityLimits::Ptr(
                        new OpenSoT::constraints::velocity::VelocityLimits(velocityLimit,
                                                                           _dT,
                                                                           task->getXSize()));
            task->getConstraints().push_back(velocityLimits);
        }
    }
}

double VelocityAllocation::computeVelocityLimit(const unsigned int taskIndex,
                                                const unsigned int stackSize)
{
    if(_velocity_vector.size() > 0 && _velocity_vector.size() == stackSize)
        return _velocity_vector[taskIndex];
    if(_last_velocity <= 0.0) // 3 parameters passed
    {
        if(stackSize > 1)
            return _min_velocity+taskIndex*(_max_velocity-_min_velocity)/(stackSize-1);
        else
            return _max_velocity;
    }
    else // 2 parameters passed
    {
        if(stackSize > 2)
        {
            if(taskIndex < stackSize - 1)
                return _min_velocity+taskIndex*(_max_velocity-_min_velocity)/(stackSize-2);
            else
                return _last_velocity;
        }
        else if(stackSize > 1)
            return _min_velocity+taskIndex*(_last_velocity-_min_velocity)/(stackSize-1);
        else
            return _last_velocity;
    }
}
