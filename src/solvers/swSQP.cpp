#include <OpenSoT/solvers/swSQP.h>

using namespace OpenSoT::solvers;

swSQP::swSQP(OpenSoT::ocp::Ptr ocp):
    _ocp(ocp), _stats(ocp->getNumberOfNodes())
{
    _qp_solver = std::make_shared<hpipmOC>(ocp->getNumberOfNodes());

init();
}

void swSQP::computeDynamics(const unsigned int i, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b)
{
    A = _ocp->stage(i)->dynamics_derivative->getA() * _Mx[i].transpose();
    B = _ocp->stage(i)->dynamics_derivative->getA() * _Mu[i].transpose();
    b = - 1. *_ocp->stage(i)->dynamics_derivative->getb(); //this is negative because it comes from an OpenSoT Task ||Ax - b||!
}

void swSQP::computeCost(const unsigned int i,
                        Eigen::MatrixXd& Q, Eigen::VectorXd& q,
                        Eigen::MatrixXd& R, Eigen::VectorXd& r,
                        Eigen::MatrixXd& S)
{
    // calculating the quadratic approximation
    _H[i].triangularView<Eigen::Upper>() = _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWA();
    _H[i] = _H[i].selfadjointView<Eigen::Upper>();

    _g[i] = - _ocp->stage(i)->stack->getStack()[0]->getA().transpose() * _ocp->stage(i)->stack->getStack()[0]->getWb();


    // computing state and control cost matrices from the quadratic approximation
    Q = _Mx[i] * _H[i] * _Mx[i].transpose();
    q = _Mx[i] * _g[i];

    if(_ocp->stage(i)->u)
    {
        R = _Mu[i] * _H[i] * _Mu[i].transpose();
        S = _Mu[i] * _H[i] * _Mx[i].transpose();
        r = _Mu[i] * _g[i];
    }
}

void swSQP::computeConstraints(const unsigned int i,
                               Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::VectorXd& dl, Eigen::VectorXd& du)
{
    OpenSoT::constraints::Aggregated::ConstraintPtr constraints = _ocp->stage(i)->stack->getBounds();

    //Do not make sense to check bnounds since bounds in the non-linear problem are constraints
    if(constraints->getAineq().rows() > 0) //there are constraints
    {
        C = constraints->getAineq() * _Mx[i].transpose();
        if(_ocp->stage(i)->u)
            D = constraints->getAineq() * _Mu[i].transpose();

        dl = constraints->getbLowerBound();
        du = constraints->getbUpperBound();
    }
}



void swSQP::linearize()
{
    for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
    {
        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes()-1)
        {
            computeDynamics(k, _A[k], _B[k], _b[k]);
            _qp_solver->setStageDynamics(k, _A[k], _B[k], _b[k]);
        }

        // --- Cost (always) ---
        computeCost(k, _Q[k], _q[k], _R[k], _r[k], _S[k]);
        _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);

        // --- Constraints (always) ---
        computeConstraints(k, _C[k], _D[k], _dl[k], _du[k]);
        _qp_solver->setConstraint(k, _C[k], _D[k], _dl[k], _du[k]);
    }


}

bool swSQP::solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    auto start = std::chrono::high_resolution_clock::now();

    _x0_candidate = x0;
    _u0_candidate = u0;

    _x0 = x0;
    _u0 = u0;

    _ocp->update(_x0_candidate, _u0_candidate);

    if (_opt.line_search_strategy!=0)
    {
        _prev_cost = _ocp->cost();
        _prev_defect = _ocp->dynamics_defect();
        _prev_viol =  _ocp->constraint_violation();
    }

    for (uint i = 0; i < _ocp->getNumberOfNodes() && _opt.line_search_strategy==1 ; i++)
    {
        dcost_dw[i] = _ocp->stage(i)->stage_dcost_dw();
        dviol_dw[i] = _ocp->stage(i)->stage_dviolation_dw();
        ddefect_dw[i] = _ocp->stage(i)->stage_ddefect_dw();
    }

    _dx0.setZero();

    for(unsigned int iter = 1; iter <= _opt.max_iters; ++iter)
    {
        _stats.iters = iter;
        _stats.alpha = 1.;
        _stats.line_search_iters = 1;
        _stats.line_search_accepted = false;

        auto iter_start = std::chrono::high_resolution_clock::now();

        // relinarize and update qp
        linearize();

        // solve
        if (!_qp_solver->solve(_dx0))
        {
            std::cout<< "nosolve"<< std::endl;
            return false;
        }

        // first update
        step(_stats.alpha);
        
    
        while(_opt.line_search_strategy!=0 && _stats.alpha >= _opt.alpha_min)
        {
            _ocp->update(_x0_candidate, _u0_candidate);
            if((this->*ls_function)())
            {
                _stats.line_search_accepted=true;
                break;
            }
            _stats.alpha /= 2.;
            _stats.line_search_iters++;
            step(_stats.alpha);

            
        }
        if (_opt.line_search_strategy==0)
            _ocp->update(_x0_candidate, _u0_candidate);

        _x0 = _x0_candidate;
        _u0 = _u0_candidate;
        

        // check break criteria on QP solution
        if (break_criteria())
        {
            std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - iter_start;
            _stats.iter_time = iter_elapsed.count();
            break;
        } 

        _prev_cost = _ocp->cost();
        _prev_defect = _ocp->dynamics_defect();
        _prev_viol = _ocp->constraint_violation();

        if(_opt.verbose)
        {
            update_statistics();
            std::chrono::duration<double> iter_elapsed = std::chrono::high_resolution_clock::now() - iter_start;
            _stats.iter_time = iter_elapsed.count();
            std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;
        }

    }

    // _ocp->update(_x0, _u0);

    if(_opt.verbose)
    {
        update_statistics();   
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
        _stats.total_time = elapsed.count();

        std::cout<<_stats.toOSS().str()<<"\n"<<std::endl;
    }

    return true;
}

bool swSQP::break_criteria()
{
    double max_dw = -INFINITY;
    for(unsigned int i = 0; i < _ocp->getNumberOfNodes() ; ++i)
        max_dw = std::max(max_dw, (_x0[i] - _x0_candidate[i]).cwiseAbs().maxCoeff());
        // #TODO: USE THE MANIFOLD OMINUS

    return max_dw <= _opt.min_abs_delta_solution && _ocp->constraint_violation() <= _opt.min_abs_delta_solution;

}

void swSQP::step(double alpha)
{
    for(unsigned int k = 0; k < _x0_candidate.size(); ++k)
    {
        if(_ocp->stage(k)->state_space)
        {
            _ocp->stage(k)->state_space->plus(_x0[k], alpha*_qp_solver->getSolution()[k].x, _x0_candidate[k]);
        }
        if (k < _u0_candidate.size())
            _u0_candidate[k] = _u0[k] + alpha * _qp_solver->getSolution()[k].u;

    }
}

bool swSQP::ls_merit()
{
    double merit = _ocp->cost() + _ocp->constraint_violation() + _ocp->dynamics_defect();

    
    double merit_der = 0.;

    for(unsigned int i = 0; i < _ocp->getNumberOfNodes(); ++i)
    {
        // std::cout<< dcost_dw[i].rows() <<"----"<< dcost_dw[i].cols()<< std::endl;
        // std::cout<< dviol_dw[i].rows() <<"----"<< dviol_dw[i].cols()<< std::endl;
        // std::cout<< ddefect_dw[i].rows() <<"----"<< ddefect_dw[i].cols()<< std::endl;
        // std::cout<< _qp_solver->getSolution()[i].x.rows() <<",,"<< _qp_solver->getSolution()[i].x.cols()<< std::endl;
        // std::cout<< _Mx[i].rows() <<",,"<< _Mx[i].cols()<< std::endl;
        // std::cout<< _qp_solver->getSolution()[i].u.rows() <<",,,"<< _qp_solver->getSolution()[i].u.cols()<< std::endl;
        // std::cout<< _Mu[i].rows() <<",,,"<< _Mu[i].cols()<< std::endl;

        merit_der += (dcost_dw[i].transpose() * _Mx[i].transpose() * _qp_solver->getSolution()[i].x)[0];
        merit_der += (dviol_dw[i].transpose() * _Mx[i].transpose() * _qp_solver->getSolution()[i].x)[0];
        merit_der += (ddefect_dw[i].transpose() * _Mx[i].transpose() * _qp_solver->getSolution()[i].x)[0];

        if(i<_ocp->getNumberOfNodes()-1)
        {
            merit_der += (dcost_dw[i].transpose() * _Mu[i].transpose() * _qp_solver->getSolution()[i].u)[0];
            merit_der += (dviol_dw[i].transpose() * _Mu[i].transpose() * _qp_solver->getSolution()[i].u)[0];
            merit_der += (ddefect_dw[i].transpose() * _Mu[i].transpose() * _qp_solver->getSolution()[i].u)[0];
        }
    }

    if(merit < _prev_cost + _prev_viol + _prev_defect + _opt.beta * _stats.alpha * merit_der)
        return  true;

    return false;
}

bool swSQP::ls_filter()
{
    if (_ocp->cost() <  _prev_cost || _ocp->constraint_violation() < _prev_viol || _ocp->dynamics_defect()< _prev_defect)  
        return true;

    return false;
}


void swSQP::update_statistics()
{
    _stats.cost = _ocp->cost();
    _stats.constraint_violation = _ocp->constraint_violation();
    for (uint i = 0; i < _ocp->getNumberOfNodes(); i++)
    {
        _stats.stages_statistics[i].cost = _ocp->stage(i)->stage_cost();
        _stats.stages_statistics[i].constraint_violation = _ocp->stage(i)->stage_constraint_violation();
    }
    
}


void swSQP::init()
{
    _stats.line_search_accepted = false;
    _stats.line_search_iters = 0;
    _stats.alpha = 1;

    dcost_dw.resize(_ocp->getNumberOfNodes());
    dviol_dw.resize(_ocp->getNumberOfNodes());
    ddefect_dw.resize(_ocp->getNumberOfNodes());


    if(_opt.line_search_strategy == 1)
        ls_function = &swSQP::ls_merit;
    if(_opt.line_search_strategy == 2)
        ls_function = &swSQP::ls_filter;


    for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
    {
        _Mx.push_back(_ocp->stage(k)->dx->getM());

        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes()-1)
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
        _dx0.setZero(_A[0].cols()); //initial delta state constraint (_dx0 = 0)

        // --- Cost (always) ---
        _H.push_back(Eigen::MatrixXd(_ocp->stage(k)->stack->getStack()[0]->getA().cols(), _ocp->stage(k)->stack->getStack()[0]->getA().cols()));
        Eigen::VectorXd g;
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

        // --- Constraints (always) ---
        Eigen::MatrixXd C, D;
        Eigen::VectorXd dl, du;
        computeConstraints(k, C, D, dl, du);
        _C.push_back(C);
        _D.push_back(D);
        _dl.push_back(dl);
        _du.push_back(du);

        _qp_solver->setConstraint(k, _C[k], _D[k], _dl[k], _du[k]);
   
    }
    if(_opt.verbose)
            std::cout<<"Solver inited"<<std::endl;
}

