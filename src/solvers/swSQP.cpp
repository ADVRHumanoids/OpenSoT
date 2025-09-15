#include <OpenSoT/solvers/swSQP.h>

using namespace OpenSoT::solvers;

swSQP::swSQP(OpenSoT::ocp::Ptr ocp):
    _ocp(ocp), _stats(ocp->getNumberOfNodes())
{
    _qp_solver = std::make_shared<hpipmOC>(ocp->getNumberOfNodes());

    _init();
}

void swSQP::computeDynamics(const unsigned int i, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b)
{
    A = _ocp->stage(i)->dynamics_derivative->getA() * _Mx[i].transpose();
    B = _ocp->stage(i)->dynamics_derivative->getA() * _Mu[i].transpose();
    b = - 1. *_ocp->stage(i)->dynamics_derivative->getb(); //this is negative because it comes from an OpenSoT Task ||Ax - b||!
}

void swSQP::computeQuadraticApproximation(const unsigned int i, Eigen::MatrixXd& H, Eigen::VectorXd& g)
{
    H.triangularView<Eigen::Upper>() = _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWA();
    H = H.selfadjointView<Eigen::Upper>();

    g = -1.0 * _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWb();
}

void swSQP::computeCost(const unsigned int i,
                        Eigen::MatrixXd& Q, Eigen::VectorXd& q,
                        Eigen::MatrixXd& R, Eigen::VectorXd& r,
                        Eigen::MatrixXd& S)
{
    Q = _Mx[i] * _H[i] * _Mx[i].transpose();
    q = _Mx[i] * _g[i]; //(_g[i].transpose() * _Mx[i].transpose()).transpose();

    if(_ocp->stage(i)->u)
    {
        R = _Mu[i] * _H[i] * _Mu[i].transpose();
        S = _Mu[i] * _H[i] * _Mx[i].transpose(); //(_Mx[i] * _H[i] * _Mu[i].transpose()).transpose();
        r = _Mu[i] * _g[i]; //(_g[i].transpose() * _Mu[i].transpose()).transpose();
    }
}

bool swSQP::solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    auto start = std::chrono::high_resolution_clock::now();

    _x0_candidate.resize(x0.size());
    _u0_candidate.resize(u0.size());

    _x0 = x0;
    _u0 = u0;

    Eigen::VectorXd dx0(_A[0].cols());
    dx0.setZero();

    for(unsigned int iter = 0; iter < _opt.max_iters; ++iter)
    {
        _stats.line_search_accepted = false;
        _stats.line_search_iters = 0;
        _stats.alpha = 1;

        auto iter_start = std::chrono::high_resolution_clock::now();

        _stats.iters = iter;

        //0) linearize ocp aorund x0, u0
        _ocp->update(_x0, _u0);

        _stats.cost = _ocp->cost();

        for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
        {
            _stats.stages_statistics[k].cost = _ocp->cost(k);

            // --- Dynamics (only for k < N) ---
            if(k < _ocp->getNumberOfNodes())
            {
                computeDynamics(k, _A[k], _B[k], _b[k]);

                _qp_solver->setStageDynamics(
                    k, _A[k], _B[k], -1.0 * _ocp->stage(k)->dynamics_derivative->getb());
            }

            // --- Cost (always) ---
            computeQuadraticApproximation(k, _H[k], _g[k]);

            computeCost(k, _Q[k], _q[k], _R[k], _r[k], _S[k]);

            _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);
        }


        //3) solve
        bool success = _qp_solver->solve(dx0);
        if(!success)
            return false;

        //4) check break criteria on QP solution
        bool exit = true; //I assume I can exit
        for(unsigned int i = 0; i < _qp_solver->getSolution().size(); ++i)
        {
            for(unsigned int j = 0; j < _qp_solver->getSolution()[i].x.size(); ++j)
            {
                exit = fabs(_qp_solver->getSolution()[i].x[j]) <= _opt.min_abs_delta_solution; // check is performed on states (not needed to do it also to controls)
                if(!exit) // if exit became false, I break this loop
                    break;
            }
            if(!exit) // if exit became false I break also outer loop
                break;
        }

        if(exit) // if exit remains true I return
        {
            auto iter_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> iter_elapsed = iter_end - iter_start;
            _stats.iter_time = iter_elapsed.count();

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            _stats.total_time = elapsed.count();

            if(_opt.verbose)
                std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;

            return true;
        }
        else
        {
            if(_opt.use_line_search)
            {
                if(line_search())
                {
                    auto iter_end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> iter_elapsed = iter_end - iter_start;
                    _stats.iter_time = iter_elapsed.count();

                    if(_opt.verbose)
                        std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;

                    dx0 = _qp_solver->getSolution()[0].x;
                }
                else //not improving solution found, return
                {

                    auto iter_end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> iter_elapsed = iter_end - iter_start;
                    _stats.iter_time = iter_elapsed.count();

                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = end - start;
                    _stats.total_time = elapsed.count();

                    if(_opt.verbose)
                        std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;


                    return true;
                }
            }
            else
            {
                auto iter_end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> iter_elapsed = iter_end - iter_start;
                _stats.iter_time = iter_elapsed.count();

                if(_opt.verbose)
                    std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;

                dx0 = _qp_solver->getSolution()[0].x;
                //4) Newton Step
                 for(unsigned int k = 0; k < _x0.size(); ++k)
                 {
                    if(_ocp->stage(k)->state_space)
                    {
                        _x0_candidate[k].resize(_x0[k].size());
                        _ocp->stage(k)->state_space->integrate(_x0[k], _qp_solver->getSolution()[k].x, _x0_candidate[k]);
                        _x0[k] = _x0_candidate[k];

                    }
                    else
                        throw std::runtime_error("_ocp->stage(k)->state_space is not defined for stage " + to_string(k));

                 }
                 for(unsigned int k = 0; k < _u0.size(); ++k)
                 {
                     _u0[k] += _qp_solver->getSolution()[k].u;
                 }
            }
        }

    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    _stats.total_time = elapsed.count();
    if(_opt.verbose)
        std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;

    return true;
}

bool swSQP::line_search()
{
    _x0_candidate.resize(_x0.size());
    _u0_candidate.resize(_u0.size());

    double alpha = 1.;
    double initial_merit = _ocp->cost();

    double merit_der = 0.;
    for(unsigned int i = 0; i <= _ocp->getNumberOfNodes(); ++i)
    {
        merit_der += _ocp->stage(i)->der(_qp_solver->getSolution()[i].x, _qp_solver->getSolution()[i].u);
    }

    _stats.line_search_iters = 1;
    while(alpha >= _opt.alpha_min)
    {
        //4) Newton Step
        for(unsigned int k = 0; k < _x0.size(); ++k)
        {
            if(_ocp->stage(k)->state_space)
                {
                    _x0_candidate[k].resize(_x0[k].size());
                    _ocp->stage(k)->state_space->integrate(_x0[k], alpha*_qp_solver->getSolution()[k].x, _x0_candidate[k]);
                }
            else
                throw std::runtime_error("_ocp->stage(k)->state_space is not defined for stage " + to_string(k));

        }

        for(unsigned int k = 0; k < _u0.size(); ++k)
        {
            _u0_candidate[k] = _u0[k] + alpha * _qp_solver->getSolution()[k].u;
        }

        //0) linearize ocp aorund _x0_candidate, _u0_candidate
        _ocp->update(_x0_candidate, _u0_candidate);

        double merit = _ocp->cost();

        if(merit < initial_merit + _opt.beta * alpha * merit_der) //Armijo's rule
        {
            //take step
            _x0 = _x0_candidate;
            _u0 = _u0_candidate;

            _stats.alpha = alpha;
            _stats.line_search_accepted = true;
            return true;
        }
        else
        {
            alpha = alpha/2.;
        }

        _stats.line_search_iters += 1;
    }
    return false;
}

void swSQP::_init()
{
    for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
    {
        _Mx.push_back(_ocp->stage(k)->dx->getM());

        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes())
        {
            _Mu.push_back(_ocp->stage(k)->du->getM());

            Eigen::MatrixXd A, B;
            Eigen::VectorXd b;
            computeDynamics(k, A, B, b);

            _A.push_back(A);
            _B.push_back(B);
            _b.push_back(b);

            _qp_solver->setStageDynamics(k, A, B, b);
        }

        // --- Cost (always) ---
        _H.push_back(Eigen::MatrixXd(_ocp->stage(k)->stack->getStack()[0]->getA().cols(), _ocp->stage(k)->stack->getStack()[0]->getA().cols()));
        Eigen::VectorXd g;
        computeQuadraticApproximation(k, _H[k], g);
        _g.push_back(g);


        Eigen::MatrixXd R, Q, S;
        Eigen::VectorXd r, q;
        computeCost(k, Q, q, R, r, S);
        _Q.push_back(Q);
        _q.push_back(q);
        _R.push_back(R);
        _r.push_back(r);
        _S.push_back(S);

        _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);
    }
}

