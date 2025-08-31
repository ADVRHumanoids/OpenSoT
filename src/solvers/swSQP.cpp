#include <OpenSoT/solvers/swSQP.h>

using namespace OpenSoT::solvers;

swSQP::swSQP(OpenSoT::ocp::Ptr ocp):
    _ocp(ocp)
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
    _x0 = x0;
    _u0 = u0;

    Eigen::VectorXd dx0(_x0[0].size());
    dx0.setZero();

    for(unsigned int iter = 0; iter < _opt.max_iters; ++iter)
    {
        std::cout<<"iter: "<<iter<<std::endl;
        //0) linearize ocp aorund x0, u0
        _ocp->update(_x0, _u0);

        for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
        {
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

        //4) Newton Step
        for(unsigned int k = 0; k < _x0.size(); ++k)
        {
            _x0[k] += _qp_solver->getSolution()[k].x;
        }
        dx0 = _qp_solver->getSolution()[0].x;
        for(unsigned int k = 0; k < _u0.size(); ++k)
        {
            _u0[k] += _qp_solver->getSolution()[k].u;
        }

        //5) check break criteria
        bool break_ = true;
        for(unsigned int i = 0; i < _qp_solver->getSolution().size(); ++i)
        {
            for(unsigned int j = 0; j < _qp_solver->getSolution()[i].x.size(); ++j)
            {
                break_ = fabs(_qp_solver->getSolution()[i].x[j]) <= _opt.min_abs_delta_solution;
                if(!break_)
                    break;
            }
            if(!break_)
                break;
        }

        if(break_)
            return true;

    }
    return true;
}

void swSQP::_init()
{
    for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
    {
        _Mx.push_back(_ocp->stage(k)->x->getM());

        // --- Dynamics (only for k < N) ---
        if(k < _ocp->getNumberOfNodes())
        {
            _Mu.push_back(_ocp->stage(k)->u->getM());

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

