#include <OpenSoT/solvers/swSQP.h>

using namespace OpenSoT::solvers;

swSQP::swSQP(OpenSoT::ocp::Ptr ocp):
    _ocp(ocp)
{
    _qp_solver = std::make_shared<hpipmOC>(ocp->getNumberOfNodes());

    _init();
}

bool swSQP::solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0)
{
    _x0 = x0;
    _u0 = u0;

    Eigen::VectorXd dx0(_x0[0].size());
    dx0.setZero();

    for(unsigned int iter = 0; iter < _opt.max_iters; ++iter)
    {
        //0) linearize ocp aorund x0, u0
        _ocp->update(_x0, _u0);

        //1) update dynamics
        for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
        {
            _A[k] = _ocp->stage(k)->dynamics_derivative->getA() * _ocp->stage(k)->x->getM().transpose();
            _B[k] = _ocp->stage(k)->dynamics_derivative->getA() * _ocp->stage(k)->u->getM().transpose();

            _qp_solver->setStageDynamics(k, _A[k], _B[k], - 0. *_ocp->stage(k)->dynamics_derivative->getb());
        }

        //2) update cost
        for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
        {
            Eigen::MatrixXd Mx = _ocp->stage(k)->x->getM();

            _H[k].triangularView<Eigen::Upper>() = _ocp->stage(k)->stack->getStack()[0]->getA().transpose() * _ocp->stage(k)->stack->getStack()[0]->getWA();
            _H[k] = _H[k].selfadjointView<Eigen::Upper>();

            _g[k] = -1. * _ocp->stage(k)->stack->getStack()[0]->getA().transpose() * _ocp->stage(k)->stack->getStack()[0]->getWb();


            _Q[k] = Mx * _H[k] *Mx.transpose();
            _q[k] = (_g[k].transpose() * Mx.transpose()).transpose();


            Eigen::MatrixXd Mu;
            if(_ocp->stage(k)->u)
            {
                Mu = _ocp->stage(k)->u->getM();

                _R[k] = Mu * _H[k] * Mu.transpose();
                _S[k] = (Mx * _H[k] * Mu.transpose()).transpose();

                _r[k] = (_g[k].transpose() * Mu.transpose()).transpose();
            }

            _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);
        }


        //3) solve
        //Eigen::VectorXd x0_flatten(_x0);
        //flatten(_x0, x0_flatten);
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
    }
    return true;
}

void swSQP::_init()
{
    //1) Initialize dynamics
    for(unsigned int k = 0; k < _ocp->getNumberOfNodes(); ++k)
    {
        _A.push_back(_ocp->stage(k)->dynamics_derivative->getA() * _ocp->stage(k)->x->getM().transpose());
        _B.push_back(_ocp->stage(k)->dynamics_derivative->getA() * _ocp->stage(k)->u->getM().transpose());

        _qp_solver->setStageDynamics(k, _A[k], _B[k], - 0.*_ocp->stage(k)->dynamics_derivative->getb());
    }

    //2) initialize cost
    for(unsigned int k = 0; k <= _ocp->getNumberOfNodes(); ++k)
    {
        Eigen::MatrixXd Mx = _ocp->stage(k)->x->getM();

        _H.push_back(Eigen::MatrixXd(_ocp->stage(k)->stack->getStack()[0]->getA().cols(), _ocp->stage(k)->stack->getStack()[0]->getA().cols()));
        _H[k].triangularView<Eigen::Upper>() = _ocp->stage(k)->stack->getStack()[0]->getA().transpose() * _ocp->stage(k)->stack->getStack()[0]->getWA();
        _H[k] = _H[k].selfadjointView<Eigen::Upper>();

        _g.push_back(-1. * _ocp->stage(k)->stack->getStack()[0]->getA().transpose() * _ocp->stage(k)->stack->getStack()[0]->getWb());

        _Q.push_back(Mx * _H[k] *Mx.transpose());

        _q.push_back((_g[k].transpose() * Mx.transpose()).transpose());


        Eigen::MatrixXd Mu;
        Eigen::MatrixXd Foo;
        _R.push_back(Foo);
        _S.push_back(Foo);
        Eigen::VectorXd foo;
        _r.push_back(foo);
        if(_ocp->stage(k)->u)
        {
            Mu = _ocp->stage(k)->u->getM();

            _R[k] = Mu * _H[k] * Mu.transpose();
            _S[k] = Mx * _H[k] * Mu.transpose();
            _r[k] = (_g[k].transpose() * Mu.transpose()).transpose();
        }

        _qp_solver->setFullCost(k, _R[k], _Q[k], _S[k], _r[k], _q[k]);
    }

}

