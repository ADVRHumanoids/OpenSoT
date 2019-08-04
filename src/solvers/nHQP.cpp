#include <OpenSoT/solvers/nHQP.h>




OpenSoT::solvers::nHQP::nHQP(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack & stack_of_tasks,
                             OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr bounds,
                             const double eps_regularisation,
                             const OpenSoT::solvers::solver_back_ends be_solver):
    Solver(stack_of_tasks, bounds)
{
    // nx = number of optimization variables
    const int nx = stack_of_tasks.front()->getXSize();

    // initialize number of free variables to nx
    // this value will decrease when going down the hierarchy
    int num_free_vars = nx;

    // iterate over hierarchy of tasks
    int i = 0;
    for(auto t : stack_of_tasks)
    {
        // the current layer does not have any dof to move
        // the optimization problem is ill-formed
        if(num_free_vars <= 0)
        {
            throw std::runtime_error("No free variables left at layer #" + std::to_string(i) + ": decrease the number of layers!");
        }

        // local constraints not supported
        if(t->getConstraints().size() > 0)
        {
            throw std::runtime_error("Local constraints not supported");
//            throw std::runtime_error("Local constraints not supported");
        }

        // equality constraints not supported
        if(bounds->getAeq().rows() > 0)
        {
            throw std::runtime_error("Equality constraints not supported");
        }

        printf("nHQP: free variables at layer #%d = %d \n", i++, num_free_vars);

        // compute number of constraints and bounds (TBD: consider equalities)
        int num_constr = bounds->getAineq().rows();
        int num_bounds = nx;

        // for all layers except the first, bounds must be turned into constraints
        if(num_free_vars != nx)
        {
            num_constr += num_bounds;
            num_bounds = 0;
        }

        // construct backend and initialize with dummy problem (always feasible)
        auto backend = BackEndFactory(be_solver,
                                      num_free_vars,
                                      num_constr,
                                      HessianType::HST_SEMIDEF,
                                      eps_regularisation);

        backend->initProblem( Eigen::MatrixXd::Identity(num_free_vars, num_free_vars),
                             Eigen::VectorXd::Zero(num_free_vars),
                             Eigen::MatrixXd::Zero(num_constr, num_free_vars),
                             -Eigen::VectorXd::Ones(num_constr),
                             Eigen::VectorXd::Ones(num_constr),
                             -Eigen::VectorXd::Ones(num_bounds),
                             Eigen::VectorXd::Ones(num_bounds));

        // construct task data and push it into a vector
        _data_struct.emplace_back(num_free_vars, t, bounds, backend);

        // compute free variables for next layer
        num_free_vars -= t->getTaskSize();

        // push some value inside vector of cumulated nullspaces
        _cumulated_nullspace.push_back(Eigen::MatrixXd::Identity(nx, nx));
    }

}

OpenSoT::solvers::nHQP::~nHQP()
{

}

bool OpenSoT::solvers::nHQP::solve(Eigen::VectorXd& solution)
{
    const int n_tasks = _tasks.size();
    const int n_x = _tasks.front()->getXSize();

    // initialize solution with zeros
    _solution.setZero(n_x);

    // iterate over the hierarchy
    for(int i = 0; i < n_tasks; i++)
    {
        // get i-th task data
        TaskData& data = _data_struct[i];

        // first layer, no nullspace to be considered (i.e. it would be the nx-by-nx identity)
        if(i == 0)
        {
            data.compute_cost(nullptr, _solution);
            data.compute_contraints(nullptr, _solution);
        }
        else // use nullspace basis computed at the previous step
        {
            data.compute_cost(&(_cumulated_nullspace[i]), _solution);
            data.compute_contraints(&(_cumulated_nullspace[i]), _solution);
        }

        // solve QP
        if(!data.update_and_solve())
        {
            return false;
        }

        // update solution according to 'solK = solK-1 + NK-1*xK_opt'
        _solution.noalias() += _cumulated_nullspace[i] * data.get_solution();

        // compute cumulated nullspace
        if(i < (n_tasks - 1))
        {
            if(!data.compute_nullspace())
            {
                throw std::runtime_error("Nullspace basis not available");
            }
            _cumulated_nullspace[i+1].noalias() = _cumulated_nullspace[i] * data.get_nullspace();
        }

    }

    solution = _solution;
    return true;
}

void OpenSoT::solvers::nHQP::_log(XBot::MatLogger::Ptr logger, const string & prefix)
{

}




void OpenSoT::solvers::nHQP::TaskData::regularize_A_b(double threshold)
{

    Eigen::VectorXd sv = svd.singularValues();
    b0 = svd.matrixU().transpose()*b0;

    const double sv_max = sv(0);

    for(int i = 0; i < b0.size(); i++)
    {
        if(i >= sv.size())
        {
            b0(i) = 0.0;
        }
        else if(sv(i) < threshold*sv_max)
        {
            b0(i) *= sv(i)/(threshold*sv_max);
            sv(i) = std::pow(threshold*sv_max, 2u)/(sv(i)+threshold/100.0);
        }
    }

    b0 = svd.matrixU()*b0;
    AN = svd.matrixU().leftCols(sv.size())*sv.asDiagonal()*svd.matrixV().transpose().topRows(sv.size());

}


void OpenSoT::solvers::nHQP::TaskData::compute_contraints(const Eigen::MatrixXd* N,
                                                          const Eigen::VectorXd& q0)
{

    if(constraints->getAeq().size() > 0)
    {
        throw std::runtime_error("Equality constraits not supported by nHQP solver");
    }

    Aineq.reset();
    lb.reset();
    ub.reset();

    if(!N)
    {
        Aineq.pile(constraints->getAineq());

        lb_bound = constraints->getLowerBound() - q0;
        ub_bound = constraints->getUpperBound() - q0;

        ub.pile(constraints->getbUpperBound());
        lb.pile(constraints->getbLowerBound());
    }
    else
    {
        Aineq.pile(constraints->getAineq() * (*N));
        Aineq.pile(*N);

        lb.pile(constraints->getbLowerBound() - constraints->getAineq()*q0);
        lb.pile(constraints->getLowerBound());

        ub.pile(constraints->getbUpperBound() - constraints->getAineq()*q0);
        ub.pile(constraints->getUpperBound());
    }

}

const Eigen::MatrixXd & OpenSoT::solvers::nHQP::TaskData::get_nullspace() const
{
    return N;
}


OpenSoT::solvers::nHQP::TaskData::TaskData(int num_free_vars,
                                           OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr a_task,
                                           OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr a_constraint,
                                           BackEnd::Ptr a_back_end):
    num_vars(num_free_vars),
    size(a_task->getTaskSize()),
    task(a_task),
    constraints(a_constraint),
    Aineq(num_free_vars), lb(1), ub(1),
    back_end(a_back_end)
{

}

void OpenSoT::solvers::nHQP::TaskData::compute_cost(const Eigen::MatrixXd* N, const Eigen::VectorXd& q0)
{

    /* || A(N*x + q0) - b || */

    if(!N)
    {
        AN = task->getA();
        b0 = task->getb();
    }
    else
    {
        AN.noalias() = task->getA() * (*N);
        b0.noalias() = task->getb() - task->getA() * q0;
    }

    svd.compute(AN, Eigen::ComputeFullU|Eigen::ComputeFullV);

    const double REG_THRESHOLD = 0.05;
    regularize_A_b(REG_THRESHOLD);

    H.noalias() =  AN.transpose() * task->getWeight() * AN;
    g.noalias() = -AN.transpose() * task->getWeight() * b0;
}

bool OpenSoT::solvers::nHQP::TaskData::update_and_solve()
{
    back_end->updateTask(H, g);


    back_end->updateConstraints(Aineq.generate_and_get(),
                                lb.generate_and_get(),
                                ub.generate_and_get());

    back_end->updateBounds(lb_bound, ub_bound);


    return back_end->solve();
}

const Eigen::VectorXd& OpenSoT::solvers::nHQP::TaskData::get_solution() const
{
    return back_end->getSolution();
}

bool OpenSoT::solvers::nHQP::TaskData::compute_nullspace()
{

    const int ns_dim = num_vars - size;

    if(ns_dim <= 0)
    {
        return false;
    }
    else
    {
        N = svd.matrixV().rightCols(ns_dim); // svd was computed during regularization
        return true;
    }

    return true;
}
