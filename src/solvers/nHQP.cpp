#include <OpenSoT/solvers/nHQP.h>




OpenSoT::solvers::nHQP::nHQP(OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack & stack_of_tasks,
                             OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr bounds,
                             const double eps_regularisation,
                             const OpenSoT::solvers::solver_back_ends be_solver):
    Solver(stack_of_tasks, bounds)
{
    // nx = number of optimization variables
    const int nx = stack_of_tasks.front()->getXSize();
    _solution.setZero(nx);

    // initialize number of free variables to nx
    // this value will decrease when going down the hierarchy
    int num_free_vars = nx;

    // first cumulated nullspace is eye(nx)
    _cumulated_nullspace.push_back(Eigen::MatrixXd::Identity(nx, nx));

    // iterate over hierarchy of tasks
    const int n_tasks = stack_of_tasks.size();
    for(int i = 0; i < n_tasks; i++)
    {
        auto t = stack_of_tasks[i];

        // update task (NB: with x = zeros(nx))
        t->update(_solution);

        // the current layer does not have any dof to move
        // the optimization problem is ill-formed
        if(num_free_vars <= 0)
        {
            throw std::runtime_error("[nHQP] No free variables left at layer #" + std::to_string(i) + ": decrease the number of layers!");
        }

        // local constraints not supported (TODO)
        if(t->getConstraints().size() > 0)
        {
            throw std::runtime_error("[nHQP] Local constraints not supported");
        }

        // equality constraints not supported (TODO)
        if(bounds->getAeq().rows() > 0)
        {
            throw std::runtime_error("[nHQP] Equality constraints not supported");
        }

        printf("[nHQP] Free variables at layer #%d = %d \n", i, num_free_vars);

        // compute number of constraints and bounds (TBD: consider equalities)
        int num_constr = bounds->getAineq().rows();
        int num_bounds = nx;

        // for all layers except the first, bounds must be turned into constraints
        if(num_free_vars != nx)
        {
            num_constr += num_bounds;
            num_bounds = 0;
        }

        // construct backend
        auto backend = BackEndFactory(be_solver,
                                      num_free_vars,
                                      num_constr,
                                      HessianType::HST_SEMIDEF,
                                      eps_regularisation);


        // construct task data and push it into a vector
        _data_struct.emplace_back(num_free_vars, t, bounds, backend);

        // if we are processing the last task, skip nullspace dim computation
        if(i == n_tasks - 1)
        {
            continue;
        }

        // compute cost without regularization to get nullspace dimension
        auto& data = _data_struct.back();
        if(i == 0)
        {
            data.compute_cost(nullptr,
                              _solution);
        }
        else // use nullspace basis computed at the previous step
        {
            data.compute_cost(&(_cumulated_nullspace[i]),
                              _solution);
        }

        // nullspace dimension
        const double SV_THRESH = 1e-6;
        int ns_dim = data.compute_nullspace_dimension(SV_THRESH);
        data.set_nullspace_dimension(ns_dim);

        // warn on rank deficient task
        if(ns_dim < num_free_vars - t->getTaskSize())
        {
            printf("[nHQP] layer #%d is rank deficient with tol = %.1e! (Rank = %d, Task Size = %d) \n",
                   i, SV_THRESH, num_free_vars - ns_dim, t->getTaskSize());
        }

        // compute cumulated nullspace
        if(!data.compute_nullspace())
        {
            throw std::runtime_error("Nullspace basis not available");
        }
        _cumulated_nullspace.emplace_back(_cumulated_nullspace[i] * data.get_nullspace());


        // compute free variables for next layer
        num_free_vars = ns_dim;


    }

}

OpenSoT::solvers::nHQP::~nHQP()
{

}

void OpenSoT::solvers::nHQP::setPerformAbRegularization(bool perform_A_b_regularization)
{
    for(auto& data : _data_struct)
        data.set_perform_A_b_regularization(perform_A_b_regularization);
}

void OpenSoT::solvers::nHQP::setPerformAbRegularization(int hierarchy_level, bool perform_A_b_regularization)
{
    if(hierarchy_level >= _data_struct.size())
        throw std::invalid_argument("hierarchy_level >= # layers");
    auto& data = _data_struct[hierarchy_level];
    data.set_perform_A_b_regularization(perform_A_b_regularization);
}

void OpenSoT::solvers::nHQP::setPerformSelectiveNullSpaceRegularization(bool perform_selective_null_space_regularization)
{
    for(auto& data : _data_struct)
        data.set_perform_selective_null_space_regularization(perform_selective_null_space_regularization);
}

void OpenSoT::solvers::nHQP::setPerformSelectiveNullSpaceRegularization(int hierarchy_level, bool perform_selective_null_space_regularization)
{
    if(hierarchy_level >= _data_struct.size())
        throw std::invalid_argument("hierarchy_level >= # layers");
    auto& data = _data_struct[hierarchy_level];
    data.set_perform_selective_null_space_regularization(perform_selective_null_space_regularization);
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

void OpenSoT::solvers::nHQP::setMinSingularValueRatio(double sv_min)
{
    setMinSingularValueRatio(std::vector<double>(_data_struct.size(), sv_min));
}

void OpenSoT::solvers::nHQP::setMinSingularValueRatio(std::vector<double> sv_min)
{
    if(sv_min.size() != _data_struct.size())
    {
        throw std::invalid_argument("[nHQP::setMinSingularValueRatio] sv_min.size() != # layers");
    }

    for(int i = 0; i < sv_min.size(); i++)
    {
        _data_struct[i].set_min_sv_ratio(sv_min[i]);
    }
}

void OpenSoT::solvers::nHQP::_log(XBot::MatLogger2::Ptr logger, const string & prefix)
{
    int i = 0;
    for(auto& t : _data_struct)
    {
        t.enable_logger(logger, prefix + "_layer_" + std::to_string(i++) + "_");
    }
}




void OpenSoT::solvers::nHQP::TaskData::regularize_A_b(double threshold)
{
    if(logger)
    {
        logger->add(log_prefix + "b0", b0);
    }

    Eigen::VectorXd sv = svd.singularValues(); // svd was computed during compute_cost
    b0 = svd.matrixU().transpose()*b0;

    if(logger)
    {
        logger->add(log_prefix + "sv", sv);
        logger->add(log_prefix + "b0_rot", b0);
        logger->add(log_prefix + "A", AN);
    }


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

    if(logger)
    {
        logger->add(log_prefix + "sv_reg", sv);
        logger->add(log_prefix + "b0_reg", b0);
        logger->add(log_prefix + "A_reg", AN);
    }

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

        lb_bound = constraints->getLowerBound();
        ub_bound = constraints->getUpperBound();

        ub.pile(constraints->getbUpperBound());
        lb.pile(constraints->getbLowerBound());
    }
    else
    {
        Aineq.pile(constraints->getAineq() * (*N));
        Aineq.pile(*N);

        lb.pile(constraints->getbLowerBound() - constraints->getAineq()*q0);
        lb.pile(constraints->getLowerBound() - q0);

        ub.pile(constraints->getbUpperBound() - constraints->getAineq()*q0);
        ub.pile(constraints->getUpperBound() - q0);
    }

}

const Eigen::MatrixXd & OpenSoT::solvers::nHQP::TaskData::get_nullspace() const
{
    return AN_nullspace;
}


OpenSoT::solvers::nHQP::TaskData::TaskData(int num_free_vars,
                                           OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr a_task,
                                           OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr a_constraint,
                                           BackEnd::Ptr a_back_end):
    task(a_task),
    constraints(a_constraint),
    min_sv_ratio(nHQP::DEFAULT_MIN_SV_RATIO),
    Aineq(num_free_vars), lb(1), ub(1),
    ns_dim(num_free_vars - a_task->getTaskSize()),
    back_end(a_back_end),
    back_end_initialized(false),
    perform_A_b_regularization(true),
    perform_selective_null_space_regularization(true)
{

}

void OpenSoT::solvers::nHQP::TaskData::set_min_sv_ratio(double sv)
{
    if(sv < 0.0)
    {
        throw std::invalid_argument("[nHQP::TaskData::set_min_sv_ratio] Minimum singular value threshold should respect 0 < s < 1");
    }

    if(sv > 1.0)
    {
        throw std::invalid_argument("[nHQP::TaskData::set_min_sv_ratio] Minimum singular value threshold should respect 0 < s < 1");
    }

    min_sv_ratio = sv;
}

void OpenSoT::solvers::nHQP::TaskData::compute_cost(const Eigen::MatrixXd* N,
                                                    const Eigen::VectorXd& q0)
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

    if(perform_A_b_regularization)
        regularize_A_b(min_sv_ratio);

    H.noalias() =  AN.transpose() * task->getWeight() * AN;
    g.noalias() = -AN.transpose() * task->getWeight() * b0;

    if(compute_nullspace()) // if there is some nullspace left..
    {
        if(perform_selective_null_space_regularization)
        {
            const double sv_max = svd.singularValues()[0];
            H.noalias() += sv_max * get_nullspace() * get_nullspace().transpose(); // add selective nullspace regularization
        }
    }
}

bool OpenSoT::solvers::nHQP::TaskData::update_and_solve()
{
    bool success = false;

    // solver is not initialized
    if(!back_end_initialized)
    {
        success = back_end->initProblem(H, g,
                                        Aineq.generate_and_get(),
                                        lb.generate_and_get(),
                                        ub.generate_and_get(),
                                        lb_bound,
                                        ub_bound
                                        );

        if(success)
        {
            back_end_initialized = true;
        }
    }
    else // solver was initialized already
    {

        back_end->updateTask(H, g);


        back_end->updateConstraints(Aineq.generate_and_get(),
                                    lb.generate_and_get(),
                                    ub.generate_and_get());

        back_end->updateBounds(lb_bound, ub_bound);


        success = back_end->solve();

    }

    if(logger)
    {
        logger->add(log_prefix + "solution", back_end->getSolution());
    }

    return success;
}

const Eigen::VectorXd& OpenSoT::solvers::nHQP::TaskData::get_solution() const
{
    return back_end->getSolution();
}

bool OpenSoT::solvers::nHQP::TaskData::enable_logger(XBot::MatLogger2::Ptr a_logger, std::string a_log_prefix)
{
    if(!logger)
    {
        logger = a_logger;
        log_prefix = a_log_prefix;
        return true;
    }

    return false;
}

bool OpenSoT::solvers::nHQP::TaskData::compute_nullspace()
{

    if(ns_dim <= 0)
    {
        return false;
    }
    else
    {
        AN_nullspace = svd.matrixV().rightCols(ns_dim); // svd was computed during compute_cost
        return true;
    }

    return true;
}

int OpenSoT::solvers::nHQP::TaskData::compute_nullspace_dimension(double threshold)
{
    int rank = (svd.singularValues().array() >= threshold).count(); // svd was computed during compute_cost
    return AN.cols() - rank;
}

void OpenSoT::solvers::nHQP::TaskData::set_nullspace_dimension(int a_ns_dim)
{
    ns_dim = a_ns_dim;
}

void OpenSoT::solvers::nHQP::TaskData::set_perform_A_b_regularization(bool perform_A_b_regularization_)
{
    perform_A_b_regularization = perform_A_b_regularization_;
}

void OpenSoT::solvers::nHQP::TaskData::set_perform_selective_null_space_regularization(bool perform_selective_null_space_regularization_)
{
    perform_selective_null_space_regularization = perform_selective_null_space_regularization_;
}



