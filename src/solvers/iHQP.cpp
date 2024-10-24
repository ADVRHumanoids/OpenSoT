#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <xbot2_interface/logger.h>
#include <OpenSoT/utils/AutoStack.h>


using namespace OpenSoT::solvers;

const std::string iHQP::_IHQP_CONSTRAINTS_PLUS_ = "+";
const std::string iHQP::_IHQP_CONSTRAINTS_OPTIMALITY_ = "_OPTIMALITY";

iHQP::iHQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation,
     const solver_back_ends be_solver):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds()),
    _epsRegularisation(eps_regularisation),
    _regularisation_task(stack_of_tasks.getRegularisationTask())
{
    for(unsigned int i = 0; i < stack_of_tasks.getStack().size(); ++i){
        _active_stacks.push_back(true);
        _be_solver.push_back(be_solver);
    }

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT!");
}

iHQP::iHQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation,
     const std::vector<solver_back_ends> be_solver):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds()),
    _epsRegularisation(eps_regularisation),
    _be_solver(be_solver),
    _regularisation_task(stack_of_tasks.getRegularisationTask())
{
    for(unsigned int i = 0; i < stack_of_tasks.getStack().size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT!");
}

iHQP::iHQP(Stack &stack_of_tasks, const double eps_regularisation,const solver_back_ends be_solver):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i){
        _active_stacks.push_back(true);
        _be_solver.push_back(be_solver);
    }

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT!");
}

iHQP::iHQP(Stack& stack_of_tasks, const double eps_regularisation,
     const std::vector<solver_back_ends> be_solver):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation),
    _be_solver(be_solver)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT!");
}

iHQP::iHQP(Stack &stack_of_tasks,
                         ConstraintPtr bounds,
                         const double eps_regularisation,const solver_back_ends be_solver):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i){
        _active_stacks.push_back(true);
        _be_solver.push_back(be_solver);
    }

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
}

iHQP::iHQP(Stack& stack_of_tasks,
            ConstraintPtr bounds,
            const double eps_regularisation,
            const std::vector<solver_back_ends> be_solver):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation),
    _be_solver(be_solver)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
}

iHQP::iHQP(Stack &stack_of_tasks,
                         ConstraintPtr bounds,
                         ConstraintPtr globalConstraints,
                         const double eps_regularisation,const solver_back_ends be_solver):
    Solver(stack_of_tasks, bounds, globalConstraints),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i){
        _active_stacks.push_back(true);
        _be_solver.push_back(be_solver);
    }

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
}

iHQP::iHQP(Stack& stack_of_tasks,
            ConstraintPtr bounds,
            ConstraintPtr globalConstraints,
            const double eps_regularisation,
            const std::vector<solver_back_ends> be_solver):
    Solver(stack_of_tasks, bounds, globalConstraints),
    _epsRegularisation(eps_regularisation),
    _be_solver(be_solver)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT(_be_solver))
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
}

void iHQP::computeCostFunction(const TaskPtr& task, Eigen::MatrixXd& H, Eigen::VectorXd& g)
{
//    H = task->getA().transpose() * task->getWeight() * task->getA();
//    g = -1.0 * task->getA().transpose() * task->getWeight() * task->getb();


    H.resize(task->getXSize(), task->getXSize());
    if(task->getWeight().isIdentity())
    {
        if(task->getA().size() != 0)
        {
            H.triangularView<Eigen::Upper>() = task->getATranspose()*task->getA();
            H = H.selfadjointView<Eigen::Upper>();
            g.noalias() = -1.0 * task->getATranspose() * task->getb();
            g += task->getc();
        }
        else
            g = task->getc();
    }
    else
    {
        if(task->getA().size() != 0)
        {
            H.triangularView<Eigen::Upper>() = task->getATranspose()*task->getWA();
            H = H.selfadjointView<Eigen::Upper>();
            g.noalias() = -1.0 * task->getATranspose() * task->getWb();
            g += task->getc();
        }
        else
            g = task->getc();
    }
    
    
}

void iHQP::computeOptimalityConstraint(  const TaskPtr& task, BackEnd::Ptr& problem,
                                                Eigen::MatrixXd& A, Eigen::VectorXd& lA, Eigen::VectorXd& uA)
{
    A = task->getA();
    lA.noalias() = task->getA()*problem->getSolution();
    uA = lA;
}

bool iHQP::prepareSoT(const std::vector<solver_back_ends> be_solver)
{   
    if(_regularisation_task)
    {
        XBot::Logger::info("User defined regularisation will be added to all levels");
        computeCostFunction(_regularisation_task, Hr, gr);
    }

    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        XBot::Logger::info("#USING BACK-END @LEVEL %i: %s\n", i, getBackEndName(i).c_str());
        computeCostFunction(_tasks[i], H, g);
        if(_regularisation_task)
        {
            H += Hr;
            g += gr;
        }

        OpenSoT::constraints::Aggregated constraints_task_i(_tasks[i]->getConstraints(), _tasks[i]->getXSize());
        if(_globalConstraints){
            constraints_task_i.getConstraintsList().push_back(_globalConstraints);
            constraints_task_i.generateAll();}
        else if(_bounds && _bounds->isConstraint())
        {
            constraints_task_i.getConstraintsList().push_back(_bounds);
            constraints_task_i.generateAll();
        }

        std::string constraints_str = constraints_task_i.getConstraintID();

        A.set(constraints_task_i.getAineq());
        lA.set(constraints_task_i.getbLowerBound());
        uA.set(constraints_task_i.getbUpperBound());
        if(i > 0)
        {
            Eigen::MatrixXd _tmp_A;
            Eigen::VectorXd _tmp_lA, _tmp_uA;
            for(unsigned int j = 0; j < i; ++j)
            {
                computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], _tmp_A, _tmp_lA, _tmp_uA);

                if( j == i-1)
                {
                    tmp_A.push_back(_tmp_A);
                    tmp_lA.push_back(_tmp_lA);
                    tmp_uA.push_back(_tmp_uA);
                }

                if(!constraints_str.compare("") == 0)
                    constraints_str = constraints_str + _IHQP_CONSTRAINTS_PLUS_;
                constraints_str = constraints_str + _tasks[j]->getTaskID() + _IHQP_CONSTRAINTS_OPTIMALITY_;

                A.pile(tmp_A[j]);
                lA.pile(tmp_lA[j]);
                uA.pile(tmp_uA[j]);
            }
        }

        if(_bounds && _bounds->isBound()){   // if it is a constraint, it has already been added in #74
            constraints_task_i.getConstraintsList().push_back(_bounds);
            constraints_task_i.generateAll();}
        l = constraints_task_i.getLowerBound();
        u = constraints_task_i.getUpperBound();

//        QPOasesBackEnd problem_i(_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
//                                 _epsRegularisation);

        BackEnd::Ptr problem_i = BackEndFactory(be_solver[i],_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
                                           _epsRegularisation);

        if(problem_i->initProblem(H, g, A.generate_and_get(), lA.generate_and_get(), uA.generate_and_get(), l, u)){
            _qp_stack_of_tasks.push_back(problem_i);
            std::string bounds_string = "";
            if(_bounds)
                bounds_string = _bounds->getConstraintID();
            _qp_stack_of_tasks[i]->printProblemInformation(i, _tasks[i]->getTaskID(),
                                                          constraints_str,
                                                          bounds_string);
            if(_regularisation_task)
                XBot::Logger::info("USER DEFINED REGULARISATION: %s\n", _regularisation_task->getTaskID().c_str());
        }
        else{
            XBot::Logger::error("ERROR: INITIALIZING STACK %i \n", i);
            return false;}


        constraints_task.push_back(constraints_task_i);
    }
    return true;
}

bool iHQP::solve(Eigen::VectorXd &solution)
{
    if(_regularisation_task)
        computeCostFunction(_regularisation_task, Hr, gr);


    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        if(_active_stacks[i])
        {
            computeCostFunction(_tasks[i], H, g);
            if(_regularisation_task)
            {
                H += Hr;
                g += gr;
            }
            if(!_qp_stack_of_tasks[i]->updateTask(H, g))
                return false;

            OpenSoT::constraints::Aggregated& constraints_task_i = constraints_task[i];
            constraints_task_i.generateAll();

            A.set(constraints_task_i.getAineq());
            lA.set(constraints_task_i.getbLowerBound());
            uA.set(constraints_task_i.getbUpperBound());
            if(i > 0)
            {

                ///TODO: new implementation, this permits to save some multiplications since
                /// we do not recompute always all the optimality (priority) constraints but only the one
                /// at the previous level. The problem arise if a level is not active so all the previous
                /// not active levels are set to zero up to the last active level which is computed
                //Here I compute the optimality (priority) constraint only for the previous active level
                for(unsigned int l = i-1; l >= 0; --l)
                {
                    if(_active_stacks[l]){
                        computeOptimalityConstraint(_tasks[l], _qp_stack_of_tasks[l], tmp_A[l], tmp_lA[l], tmp_uA[l]);
                        break;}
                    else
                    {
                        //Here we consider fake optimality constraints:
                        //
                        //    -1 <= 0x <= 1
                        tmp_A[l].setZero(_tasks[l]->getA().rows(), _tasks[l]->getA().cols());
                        tmp_lA[l].setConstant(_tasks[l]->getA().rows(), -1.0);
                        tmp_uA[l].setConstant(_tasks[l]->getA().rows(), 1.0);
                    }
                }
                ///


                for(unsigned int j = 0; j < i; ++j)
                {
                    ///TODO: old implementation keep it to check against the new implementation
//                    if(_active_stacks[j])
//                        computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A[j], tmp_lA[j], tmp_uA[j]);
//                    else
//                    {
//                        //Here we consider fake optimality constraints:
//                        //
//                        //    -1 <= 0x <= 1
//                        tmp_A[j].setZero(_tasks[j]->getA().rows(), _tasks[j]->getA().cols());
//                        tmp_lA[j].setConstant(_tasks[j]->getA().rows(), -1.0);
//                        tmp_uA[j].setConstant(_tasks[j]->getA().rows(), 1.0);
//                    }
                    ///
                    A.pile(tmp_A[j]);
                    lA.pile(tmp_lA[j]);
                    uA.pile(tmp_uA[j]);
                }
            }

            if(!_qp_stack_of_tasks[i]->updateConstraints(A.generate_and_get(),
                                    lA.generate_and_get(), uA.generate_and_get()))
                return false;


            if(constraints_task_i.hasBounds()) // bounds specified everywhere will work
            {
                if(!_qp_stack_of_tasks[i]->updateBounds(constraints_task_i.getLowerBound(), constraints_task_i.getUpperBound()))
                    return false;
            }

            if(!_qp_stack_of_tasks[i]->solve())
                return false;

            solution = _qp_stack_of_tasks[i]->getSolution();
            
        }
        else
        {
            //Here we do nothing
        }
    }
    return true;
}

bool iHQP::setOptions(const unsigned int i, const boost::any &opt)
{
    if(i > _qp_stack_of_tasks.size()){
        XBot::Logger::error("ERROR Index out of range! \n");
        return false;}

    _qp_stack_of_tasks[i]->setOptions(opt);
    return true;
}

bool iHQP::getOptions(const unsigned int i, boost::any& opt)
{

    if(i > _qp_stack_of_tasks.size()){
        XBot::Logger::error("ERROR Index out of range! \n");
        return false;}

    opt = _qp_stack_of_tasks[i]->getOptions();
    return true;
}

bool iHQP::getObjective(const unsigned int i, double& val)
{
    if(i > _qp_stack_of_tasks.size()){
        XBot::Logger::error("ERROR Index out of range! \n");
        return false;}

    val = _qp_stack_of_tasks[i]->getObjective();
    return true;
}

void iHQP::setActiveStack(const unsigned int i, const bool flag)
{
    if(i >= 0 && i < _active_stacks.size())
        _active_stacks[i] = flag;
}

void iHQP::activateAllStacks()
{
    _active_stacks.assign(_active_stacks.size(), true);
}


void iHQP::_log(XBot::MatLogger2::Ptr logger, const std::string& prefix)
{
    if(_regularisation_task)
    {
        if(Hr.rows() > 0)
            logger->add("Hr", Hr);
        if(gr.size() > 0)
            logger->add("gr", gr);
    }

    for(unsigned int i = 0; i < _qp_stack_of_tasks.size(); ++i)
        _qp_stack_of_tasks[i]->log(logger,i, prefix);
}

std::string iHQP::getBackEndName(const unsigned int i)
{
    if(i >= _be_solver.size())
    {
        XBot::Logger::error("Requested level %i BackEnd which does not exists!\n", i);
        return "";
    }
    return OpenSoT::solvers::whichBackEnd(_be_solver[i]);
}

bool iHQP::getBackEnd(const unsigned int i, BackEnd::Ptr& back_end)
{
    if(i >= _qp_stack_of_tasks.size())
    {
        XBot::Logger::error("Requested level %i BackEnd which does not exists!\n", i);
        return false;
    }
    back_end = _qp_stack_of_tasks[i];
    return true;
}

bool iHQP::setEpsRegularisation(const double eps, const unsigned int i)
{
    if(i >= _qp_stack_of_tasks.size())
    {
        XBot::Logger::error("Requested level %i BackEnd which does not exists!\n", i);
        return false;
    }

    return _qp_stack_of_tasks[i]->setEpsRegularisation(eps);
}

bool iHQP::setEpsRegularisation(const double eps)
{
    for(unsigned int i = 0; i < _qp_stack_of_tasks.size(); ++i)
    {
        if(!_qp_stack_of_tasks[i]->setEpsRegularisation(eps))
        {
            XBot::Logger::error("Problem setting eps regularisation in level %i", i);
            return false;
        }
    }
    return true;
}
