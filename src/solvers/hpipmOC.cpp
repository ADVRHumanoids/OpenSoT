#include <OpenSoT/solvers/hpipmOC.h>

using namespace OpenSoT::solvers;

hpipmOC::hpipmOC(const unsigned int Ns):
    _Ns(Ns)
{
    //These are taken from examples in hpipm-cpp
    _solver_settings.mode = hpipm::HpipmMode::Balance;
    _solver_settings.iter_max = 30;
    _solver_settings.alpha_min = 1e-8;
    _solver_settings.mu0 = 1e2;
    _solver_settings.tol_stat = 1e-04;
    _solver_settings.tol_eq = 1e-04;
    _solver_settings.tol_ineq = 1e-04;
    _solver_settings.tol_comp = 1e-04;
    _solver_settings.reg_prim = 1e-12;
    _solver_settings.warm_start = 0;
    _solver_settings.pred_corr = 1;
    _solver_settings.ric_alg = 0;
    _solver_settings.split_step = 1;

    _qp.resize(Ns+1);

    _WxAx.resize(Ns+1);
    _Wxbx.resize(Ns+1);

    _WuAu.resize(Ns);
    _Wubu.resize(Ns);

    _solution.resize(Ns+1);
}

hpipmOC::~hpipmOC()
{

}

bool hpipmOC::solve(const Eigen::VectorXd& x0)
{
    if(!_solver)
    {
        _solver = std::make_shared<hpipm::OcpQpIpmSolver>(_qp, _solver_settings);
    }

    _solve_status = _solver->solve(x0, _qp, _solution);
    if(_solve_status == hpipm::HpipmStatus::Success)
        return true;
    return false;
}

void hpipmOC::setStageDynamics(const unsigned int i, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& b)
{
    _qp[i].A = A;
    _qp[i].B = B;
    _qp[i].b = b;
}

 void hpipmOC::setBoundsX(const unsigned int i, const std::vector<int>& idxbx, const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx)
{
    _qp[i].idxbx = idxbx;
    _qp[i].lbx = lbx;
    _qp[i].ubx = ubx;
}

void hpipmOC::setBoundsU(const unsigned int i, const std::vector<int>& idxbu, const Eigen::VectorXd& lbu, const Eigen::VectorXd& ubu)
{
    _qp[i].idxbu = idxbu;
    _qp[i].lbu = lbu;
    _qp[i].ubu = ubu;
}

void hpipmOC::setConstraint(const unsigned int i, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const Eigen::VectorXd& min, const Eigen::VectorXd& max)
{
    _qp[i].C = C;
    _qp[i].D = D;
    _qp[i].lg = min;
    _qp[i].ug = max;
}

void hpipmOC::setFullCost(const unsigned int i, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& S, const Eigen::VectorXd& r, const Eigen::VectorXd& q)
{
    _qp[i].R = R;
    _qp[i].Q = Q;
    _qp[i].S = S;
    _qp[i].r = r;
    _qp[i].q = q;
}

void hpipmOC::setCost(const unsigned int i, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, const Eigen::VectorXd& r, const Eigen::VectorXd& q)
{
    _qp[i].R = R;
    _qp[i].Q = Q;
    _qp[i].S = Eigen::MatrixXd::Zero(R.rows(), Q.rows());
    _qp[i].r = r;
    _qp[i].q = q;
}

void hpipmOC::setLSCost(const unsigned int i,
                        const Eigen::MatrixXd& Ax, const Eigen::MatrixXd& Wx, const Eigen::VectorXd& bx,
                        const Eigen::MatrixXd& Au, const Eigen::MatrixXd& Wu, const Eigen::VectorXd& bu)
{
    _qp[i].Q.resize(Ax.cols(), Ax.cols());
    if(Wx.isIdentity())
    {
        _WxAx[i] = Ax;
        _Wxbx[i] = bx;
    }
    else
    {
        _WxAx[i] = Wx*Ax;
        _Wxbx[i] = Wx*bx;
    }
    _qp[i].Q.triangularView<Eigen::Upper>() = _WxAx[i];
    _qp[i].Q = _qp[i].Q.selfadjointView<Eigen::Upper>();

    _qp[i].q = -1.0 * Ax.transpose() * _Wxbx[i];

    if(Au.rows() > 0)
    {
        _qp[i].R.resize(Au.cols(), Au.cols());
        if(Wu.isIdentity())
        {
            _WuAu[i] = Au;
            _Wubu[i] = bu;
        }
        else
        {
            _WuAu[i] = Wu*Au;
            _Wubu[i] = Wu*bu;
        }
        _qp[i].R.triangularView<Eigen::Upper>() = _WuAu[i];
        _qp[i].R = _qp[i].R.selfadjointView<Eigen::Upper>();

        _qp[i].r = -1.0 * Au.transpose() * _Wubu[i];

        _qp[i].S = Eigen::MatrixXd(_qp[i].R.rows(), _qp[i].Q.rows());
    }
    else
        _qp[i].S = Eigen::MatrixXd(0,0);
}








