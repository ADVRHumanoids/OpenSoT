#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

using namespace yarp::math;
using namespace OpenSoT::solvers;

/// QPOases_sot ///
QPOases_sot::QPOases_sot(Stack &stack_of_tasks, const double eps_regularisation):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation)
{
    _qp_stack_of_tasks.reserve(_tasks.size());
    assert(prepareSoT());
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         boost::shared_ptr<constraints::Aggregated> &bounds, const double eps_regularisation):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
    _qp_stack_of_tasks.reserve(_tasks.size());
    assert(prepareSoT());
}

bool QPOases_sot::prepareSoT()
{
    bool prepared = true;
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        if(_bounds)
            _qp_stack_of_tasks.push_back(QPOasesTask(_tasks[i], _bounds, _epsRegularisation));
        else
            _qp_stack_of_tasks.push_back(QPOasesTask(_tasks[i], _epsRegularisation));

        if(i > 0)
            prepared = prepared && expandProblem(i);

        if(prepared)
            _qp_stack_of_tasks[i].printProblemInformation(i);
        else
            std::cout<<"ERROR Occurered preparing task "<<_qp_stack_of_tasks[i].getTaskID()<<std::endl;
    }
    return prepared;
}

bool QPOases_sot::expandProblem(unsigned int i)
{
    bool constraints_added = true;

    //I want to add all the constraints of the previous j tasks to task i
    for(unsigned int j = 0; j < i; ++j)
    {
        //1. Prepare new constraints from task j
            boost::shared_ptr<OpenSoT::constraints::BilateralConstraint> task_j_constraint(
                new OpenSoT::constraints::BilateralConstraint
                    (
                        _tasks[j]->getA(),
                        _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                        _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                     )
            );
        //2. Get constraints of task j
            std::list< boost::shared_ptr< OpenSoT::constraints::Aggregated::ConstraintType > > constraints_list =
                _tasks[j]->getConstraints();
            constraints_list.push_back(task_j_constraint);
            OpenSoT::constraints::Aggregated
                new_constraints_for_task_i(constraints_list, _tasks[j]->getXSize());

        //3. Add new constraints & bounds to problem i
            //3.1 ADD constraints to problem i
//                constraints_added = constraints_added &&
//                        _qp_stack_of_tasks[i].addConstraints(
//                            new_constraints_for_task_i.getAineq(),
//                            new_constraints_for_task_i.getbLowerBound(),
//                            new_constraints_for_task_i.getbUpperBound());
            constraints_added = constraints_added &&
                    _qp_stack_of_tasks[i].addConstraints(
                        task_j_constraint->getAineq(),
                        task_j_constraint->getbLowerBound(),
                        task_j_constraint->getbUpperBound());
    }
    return constraints_added;
}

unsigned int QPOases_sot::getNumberOfTasks()
{
    if(!(_qp_stack_of_tasks.size() == _tasks.size()))
    {
        std::cout<<"Stored # QP Problems does not mach stored # tasks"<<std::endl;
        std::cout<<"# TASKS: "<<_tasks.size()<<std::endl;
        std::cout<<"# QP PROBLEMS: "<<_qp_stack_of_tasks.size()<<std::endl;
        assert(_qp_stack_of_tasks.size() == _tasks.size());
    }

    return _qp_stack_of_tasks.size();
}

bool QPOases_sot::solve(Vector &solution)
{
    bool solved_task_i = true;
    bool expanded = true;
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        expanded = expanded && updateExpandedProblem(i);
        bool a = _qp_stack_of_tasks[i].solve(false);
        if(!a) std::cout<<"TASK "<<_qp_stack_of_tasks[i].getTaskID()<<std::endl;
        solved_task_i =  solved_task_i && a;
        solution = _qp_stack_of_tasks[i].getSolution();
        //_qp_stack_of_tasks[i].printProblemInformation(i);
        //std::cout<<"SOLUTION PROBLEM i: "<<solution.toString()<<std::endl;
    }
    return solved_task_i && expanded;
}

bool QPOases_sot::updateExpandedProblem(unsigned int i)
{
    //I want to update all the constraints of the previous j tasks to task i
    //1. I got all the updated constraints from task i
    OpenSoT::constraints::Aggregated
        constraints_task_i(_tasks[i]->getConstraints(),
                           _tasks[i]->getXSize());

    yarp::sig::Matrix A = constraints_task_i.getAineq();
    yarp::sig::Vector lA = constraints_task_i.getbLowerBound();
    yarp::sig::Vector uA = constraints_task_i.getbUpperBound();

    //2. I got all the updated constraints from the previous tasks
    for(unsigned int j = 0; j < i; ++j)
    {
        //2.1 Prepare new constraints from task j
        boost::shared_ptr<OpenSoT::constraints::BilateralConstraint> task_j_constraint(
            new OpenSoT::constraints::BilateralConstraint
                (
                    _tasks[j]->getA(),
                    _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                    _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                 )
        );
        //2.2 Get constraints & bounds of task j
        std::list< boost::shared_ptr< OpenSoT::constraints::Aggregated::ConstraintType > > constraints_list =
            _tasks[j]->getConstraints();
        constraints_list.push_back(task_j_constraint);
        OpenSoT::constraints::Aggregated
            updated_constraints_for_task_i(constraints_list,
                                           _tasks[j]->getXSize());

        //3. Stack updated constraints to constraints in task i
//        A = yarp::math::pile(A, updated_constraints_for_task_i.getAineq());
//        lA = yarp::math::cat(lA, updated_constraints_for_task_i.getbLowerBound());
//        uA = yarp::math::cat(uA, updated_constraints_for_task_i.getbUpperBound());
        A = yarp::math::pile(A, task_j_constraint->getAineq());
        lA = yarp::math::cat(lA, task_j_constraint->getbLowerBound());
        uA = yarp::math::cat(uA, task_j_constraint->getbUpperBound());
   }
    //3. Update constraints matrices of task j
    ///TO DO: check that matrices from point 2. and matrices in _qp_stack_of_tasks[i]
    /// has the same size!
   _qp_stack_of_tasks[i].updateConstraints(A, lA, uA);
   if(_bounds)
       _qp_stack_of_tasks[i].updateBounds(_bounds->getLowerBound(), _bounds->getUpperBound());
    return true;
}





bool QPOases_sot::setOptions(const unsigned int i, const qpOASES::Options &opt)
{
    if(i > _qp_stack_of_tasks.size())
    {
        std::cout<<"Index out of range!"<<std::endl;
        return false;
    }

    _qp_stack_of_tasks[i].setOptions(opt);
    return true;
}

bool QPOases_sot::getOptions(const unsigned int i, qpOASES::Options& opt)
{

    if(i > _qp_stack_of_tasks.size())
    {
        std::cout<<"Index out of range!"<<std::endl;
        return false;
    }

    opt = _qp_stack_of_tasks[i].getOptions();
    return true;
}
