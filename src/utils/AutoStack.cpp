#include <OpenSoT/utils/AutoStack.h>
#include <algorithm>

namespace OpenSoT{

OpenSoT::tasks::Aggregated::TaskPtr operator*(const Eigen::MatrixXd& W,
                                              OpenSoT::tasks::Aggregated::TaskPtr task)
{
    Eigen::MatrixXd Wtask = task->getWeight();
    task->setWeight(W*Wtask);
    return OpenSoT::tasks::Aggregated::TaskPtr(task);
}

OpenSoT::tasks::Aggregated::TaskPtr operator*(const double w,
                                              OpenSoT::tasks::Aggregated::TaskPtr task)
{
    Eigen::MatrixXd I = task->getWeight();
    I.setIdentity(I.rows(), I.cols());
    I = w*I;
    return I*task;
}

OpenSoT::tasks::Aggregated::Ptr operator*(const double w,
                                          OpenSoT::tasks::Aggregated::Ptr task)
{
    Eigen::MatrixXd Wtask = task->getWeight();
    task->setWeight(w*Wtask);
    return OpenSoT::tasks::Aggregated::Ptr(task);
}


OpenSoT::SubTask::Ptr operator%(const OpenSoT::tasks::Aggregated::TaskPtr task,
                                const std::list<unsigned int>& rowIndices)
{
    OpenSoT::SubTask::Ptr sub_task;
    sub_task.reset(new OpenSoT::SubTask(task, rowIndices));
    return sub_task;
}

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task2)
{
    return OpenSoT::tasks::Aggregated::Ptr(
        new OpenSoT::tasks::Aggregated( task1,
                                        task2,
                                        task1->getXSize()));
}

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList = aggregated->getTaskList();
    taskList.push_back(task);
    OpenSoT::tasks::Aggregated::Ptr outAggregated(
        new OpenSoT::tasks::Aggregated(taskList,
                                       task->getXSize()));

    Eigen::MatrixXd W = outAggregated->getWeight();
    W.block(0,0,aggregated->getWeight().rows(), aggregated->getWeight().cols()) = aggregated->getWeight();
    outAggregated->setWeight(W);
    //outAggregated->setLambda(aggregated->getLambda());
    outAggregated->getConstraints() = aggregated->getConstraints();

    return outAggregated;
}

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList = aggregated->getTaskList();
    taskList.push_front(task);
    OpenSoT::tasks::Aggregated::Ptr outAggregated(
        new OpenSoT::tasks::Aggregated(taskList,
                                       task->getXSize()));

    Eigen::MatrixXd W = outAggregated->getWeight();
    Eigen::MatrixXd W_aggregated = aggregated->getWeight();
    double r = W.rows()-W_aggregated.rows();
    double c = W.cols()-W_aggregated.cols();
    W.block(r,c,W_aggregated.rows(),W_aggregated.cols()) = W_aggregated;
    outAggregated->setWeight(W);
    //outAggregated->setLambda(aggregated->getLambda());
    outAggregated->getConstraints() = aggregated->getConstraints();

    return outAggregated;
}

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated1,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated2)
{
    OpenSoT::tasks::Aggregated::Ptr outAggregated;
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    if(aggregated1->getLambda() == aggregated2->getLambda()) {
        taskList.insert(taskList.end(),
                        aggregated1->getTaskList().begin(),
                        aggregated1->getTaskList().end());
        taskList.insert(taskList.end(),
                        aggregated2->getTaskList().begin(),
                        aggregated2->getTaskList().end());

        outAggregated = OpenSoT::tasks::Aggregated::Ptr(
                    new OpenSoT::tasks::Aggregated( taskList,
                                                    aggregated1->getXSize()));

        //outAggregated->setLambda(aggregated1->getLambda());
    } else {
        outAggregated = OpenSoT::tasks::Aggregated::Ptr(
            new OpenSoT::tasks::Aggregated( aggregated1,
                                            aggregated2,
                                            aggregated1->getXSize()));
    }

    Eigen::MatrixXd W = outAggregated->getWeight();
    Eigen::MatrixXd W1 = aggregated1->getWeight();
    Eigen::MatrixXd W2 = aggregated2->getWeight();
    W.block(0,0,W1.rows(),W1.cols()) = W1;
    W.block(W1.rows(),W1.cols(),W2.rows(),W2.cols()) = W2;
    outAggregated->setWeight(W);
    outAggregated->getConstraints() = aggregated1->getConstraints();
    typedef std::list< OpenSoT::tasks::Aggregated::ConstraintPtr >::const_iterator it_c;
    for(it_c constraint = aggregated2->getConstraints().begin();
        constraint != aggregated2->getConstraints().end();
        ++constraint)
    {
        if(find(outAggregated->getConstraints().begin(),
                outAggregated->getConstraints().end(), *constraint)
           == outAggregated->getConstraints().end())
            outAggregated->getConstraints().push_back(*constraint);
    }

    return outAggregated;
}


OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task2)
{
    OpenSoT::solvers::iHQP::Stack stack;
    stack.push_back(task1);
    stack.push_back(task2);
    return OpenSoT::AutoStack::Ptr(
        new OpenSoT::AutoStack(stack));
}

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task)
{
    OpenSoT::solvers::iHQP::Stack outStack(stack->getStack());
    outStack.push_back(task);
    if(stack->getBoundsList().size() > 0)
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack,
                                       stack->getBoundsList()));
    else
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack));
}

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                    OpenSoT::AutoStack::Ptr stack)
{
    OpenSoT::solvers::iHQP::Stack outStack;
    outStack.push_back(task);
    outStack.insert(outStack.end(),
                    stack->getStack().begin(),
                    stack->getStack().end());
    if(stack->getBoundsList().size() > 0)
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack,
                                       stack->getBoundsList()));
    else
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack));

}

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack1,
                                    const OpenSoT::AutoStack::Ptr stack2)
{
    OpenSoT::solvers::iHQP::Stack outStack;
    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> outBounds;

    outStack.insert(outStack.end(),
                    stack1->getStack().begin(),
                    stack1->getStack().end());

    outStack.insert(outStack.end(),
                    stack2->getStack().begin(),
                    stack2->getStack().end());

    if(stack1->getBoundsList().size() > 0 ||
       stack2->getBoundsList().size() > 0) {
        outBounds.insert(outBounds.end(),
                         stack1->getBoundsList().begin(),
                         stack1->getBoundsList().end());
        typedef std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>::const_iterator it_b;
        for(it_b bound = stack2->getBoundsList().begin();
            bound != stack2->getBoundsList().end();
            ++bound)
            if(std::find(outBounds.begin(), outBounds.end(), *bound) == outBounds.end())
                outBounds.push_back(*bound);
    }

    if(!outBounds.empty())
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack, outBounds));
    else
        return OpenSoT::AutoStack::Ptr(
                new OpenSoT::AutoStack(outStack));
}


OpenSoT::tasks::Aggregated::TaskPtr operator<<( OpenSoT::tasks::Aggregated::TaskPtr task,
                                                const OpenSoT::constraints::Aggregated::ConstraintPtr constraint)
{
    task->getConstraints().push_back(constraint);
    return task;
}

OpenSoT::tasks::Aggregated::Ptr operator<<( OpenSoT::tasks::Aggregated::Ptr task,
                                            const OpenSoT::constraints::Aggregated::ConstraintPtr constraint)
{
    task->getConstraints().push_back(constraint);
    return task;
}

OpenSoT::AutoStack::Ptr operator<<( OpenSoT::AutoStack::Ptr autoStack,
                                    const OpenSoT::constraints::Aggregated::ConstraintPtr bound)
{
    // check both pointers are valid
    assert(autoStack && bound);

    if((autoStack->getBoundsList().size() == 0) ||
       (std::find(autoStack->getBoundsList().begin(),
                 autoStack->getBoundsList().end(),
                 bound) == autoStack->getBoundsList().end()))
        autoStack->getBoundsList().push_back(bound);

    return autoStack;
}
}

OpenSoT::AutoStack::AutoStack(const double x_size) :
    _stack(),
    _boundsAggregated(
        new OpenSoT::constraints::Aggregated(
            std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>(),
            x_size))
{

}

OpenSoT::AutoStack::AutoStack(OpenSoT::tasks::Aggregated::TaskPtr task):
    _boundsAggregated(
        new OpenSoT::constraints::Aggregated(
            std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>(),
            task->getXSize()))
{
    _stack.push_back(task);
}

OpenSoT::AutoStack::AutoStack(OpenSoT::solvers::iHQP::Stack stack) :
    _stack(stack),
    _boundsAggregated(
        new OpenSoT::constraints::Aggregated(
            std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>(),
            stack.front()->getXSize()))
{

}

OpenSoT::AutoStack::AutoStack(OpenSoT::solvers::iHQP::Stack stack,
                              std::list<OpenSoT::solvers::iHQP::ConstraintPtr> bounds) :
    _stack(stack),
    _boundsAggregated(
        new OpenSoT::constraints::Aggregated(
            bounds,
            bounds.front()->getXSize()))
{

}

void OpenSoT::AutoStack::update(const Eigen::VectorXd &state)
{
    _boundsAggregated->update(state);
    typedef std::vector<OpenSoT::tasks::Aggregated::TaskPtr>::iterator it_t;
    for(it_t task = _stack.begin();
        task != _stack.end();
        ++task)
        (*task)->update(state);
}

std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>& OpenSoT::AutoStack::getBoundsList()
{
    return _boundsAggregated->getConstraintsList();
}

void OpenSoT::AutoStack::setBoundsAggregationPolicy(const unsigned int aggregationPolicy)
{
    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>
        bounds = _boundsAggregated->getConstraintsList();
    _boundsAggregated.reset(
        new OpenSoT::constraints::Aggregated(
            bounds,
            bounds.front()->getXSize()));
}

OpenSoT::constraints::Aggregated::ConstraintPtr OpenSoT::AutoStack::getBounds()
{
    _boundsAggregated->generateAll();
    return _boundsAggregated;
}

OpenSoT::solvers::iHQP::Stack& OpenSoT::AutoStack::getStack()
{
    return _stack;
}


std::vector<OpenSoT::solvers::iHQP::TaskPtr> OpenSoT::AutoStack::flattenTask(
        OpenSoT::solvers::iHQP::TaskPtr task)
{
    std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector;
    if(!OpenSoT::tasks::Aggregated::isAggregated(task))
        task_vector.push_back(task);
    else
    {
        boost::shared_ptr<OpenSoT::tasks::Aggregated> aggregated =
                boost::dynamic_pointer_cast<OpenSoT::tasks::Aggregated>(task);
        std::list<OpenSoT::solvers::iHQP::TaskPtr> tasks_list = aggregated->getTaskList();

        std::list<OpenSoT::solvers::iHQP::TaskPtr>::iterator it;
        for(it = tasks_list.begin(); it != tasks_list.end(); it++)
        {
            if(!OpenSoT::tasks::Aggregated::isAggregated(task))
                task_vector.push_back(*it);
            else
            {
                std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector_tmp = flattenTask(*it);
                task_vector.insert(task_vector.begin(), task_vector_tmp.begin(), task_vector_tmp.end());
            }
        }
    }
    return task_vector;
}

OpenSoT::solvers::iHQP::TaskPtr OpenSoT::AutoStack::getOperationalSpaceTask(
        const std::string& base_link, const std::string& distal_link)
{
    std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector;
    for(unsigned int i = 0; i < _stack.size(); ++i)
    {
        std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector_tmp = flattenTask(_stack[i]);
        task_vector.insert(task_vector.begin(), task_vector_tmp.begin(), task_vector_tmp.end());
    }

    for(unsigned int i = 0; i < task_vector.size(); ++i)
    {
        boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> task_Cartesian =
                boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(task_vector[i]);
        if(task_Cartesian)
        {
            std::string _base_link = task_Cartesian->getBaseLink();
            std::string _distal_link = task_Cartesian->getDistalLink();
            if(_base_link.compare(base_link) == 0 && _distal_link.compare(distal_link) == 0)
                return task_vector[i];
        }

        boost::shared_ptr<OpenSoT::tasks::velocity::CoM> task_CoM =
                boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task_vector[i]);
        if(task_CoM)
        {
            std::string _base_link = task_CoM->getBaseLink();
            std::string _distal_link = task_CoM->getDistalLink();
            if(_base_link.compare(base_link) == 0 && _distal_link.compare(distal_link) == 0)
                return task_vector[i];
        }
    }
    return OpenSoT::solvers::iHQP::TaskPtr();
}

OpenSoT::solvers::iHQP::TaskPtr OpenSoT::AutoStack::getOperationalSpaceTask(
        const std::string& task_id)
{
    std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector;
    for(unsigned int i = 0; i < _stack.size(); ++i)
    {
        std::vector<OpenSoT::solvers::iHQP::TaskPtr> task_vector_tmp = flattenTask(_stack[i]);
        task_vector.insert(task_vector.begin(), task_vector_tmp.begin(), task_vector_tmp.end());
    }

    for(unsigned int i = 0; i < task_vector.size(); ++i)
    {
        std::string _task_id = task_vector[i]->getTaskID();
        if(_task_id.compare(task_id) == 0)
            return task_vector[i];
    }
    return OpenSoT::solvers::iHQP::TaskPtr();
}

void OpenSoT::AutoStack::log(XBot::MatLogger::Ptr logger)
{
    for(auto task : _stack)
        task->log(logger);
    if(_boundsAggregated)
        _boundsAggregated->log(logger);
}

bool OpenSoT::AutoStack::checkConsistency()
{
    for(auto task : _stack)
    {
        if(!task->checkConsistency())
            return false;
    }
    if(_boundsAggregated)
    {
        if(!_boundsAggregated->checkConsistency())
            return false;
    }

    return true;
}


