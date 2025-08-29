#ifndef _WB_SOT_SOLVERS_HPIPM_BE_H_
#define _WB_SOT_SOLVERS_HPIPM_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <memory>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Piler.h>
#include <hpipm-cpp/hpipm-cpp.hpp>


using namespace OpenSoT::utils;

namespace OpenSoT{
namespace solvers{

class hpipmOC{
public:
    typedef std::shared_ptr<hpipmOC> Ptr;

    /**
     * @brief hpipmOC
     * @param Ns number of stage (Ns controls, Ns+1 states)
     */
    hpipmOC(const unsigned int Ns);

    ~hpipmOC();

    hpipm::OcpQpIpmSolverSettings& getOptions() {return _solver_settings;}


    /**
     * @brief setStageDynamics Ax + Bu + b at stage i (i.e., if i == 0, we are setting dynamics to compute x1)
     * @param i stage
     * @param A matrix
     * @param B matrix
     * @param b vector
     */
    void setStageDynamics(const unsigned int i, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& b);

    /**
     * @brief setBoundsX add bounds on state variables at stage i
     *
     *      lbx <= x <= ubx
     *
     * @param i stage
     * @param idxbx indices to be considered
     * @param lbx lower bounds
     * @param ubx upper bounds
     */
    void setBoundsX(const unsigned int i, const std::vector<int>& idxbx, const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx);

    /**
     * @brief setBoundsU add bounds on control variables at stage i
     *
     *      lbu <= u <= ubu
     *
     *
     * @param i stage
     * @param idxbu indices to be considered
     * @param lbu lower bounds
     * @param ubu upper bounds
     */
    void setBoundsU(const unsigned int i, const std::vector<int>& idxbu, const Eigen::VectorXd& lbu, const Eigen::VectorXd& ubu);

    /**
     * @brief setFullCost sets the cost matrix as:
     *
     *                -         -   - -
     *  1            | R   S   r | | u |
     *  -  [u' x' 1] | S'  Q   q | | x | = 0.5 u'Ru + 0.5 x'Qx + u'Sx + r'u + q'x + ...
     *  2            | r'  q'  1 | | 1 |
     *                -         -   - -
     *
     *  at stage i
     *
     * @param i
     * @param R
     * @param Q
     * @param S
     * @param r
     * @param q
     */
    void setFullCost(const unsigned int i, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& S, const Eigen::VectorXd& r, const Eigen::VectorXd& q);

    /**
     * @brief setCost sets the cost matrix as:
     *
     *                -         -   - -
     *  1            | R   0   r | | u |
     *  -  [u' x' 1] | 0   Q   q | | x | = 0.5 u'Ru + 0.5 x'Qx + r'u + q'x + ...
     *  2            | r'  q'  1 | | 1 |
     *                -         -   - -
     *
     *  at stage i
     *
     * @param i
     * @param R
     * @param Q
     * @param r
     * @param q
     */
    void setCost(const unsigned int i, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, const Eigen::VectorXd& r, const Eigen::VectorXd& q);

    /**
     * @brief setLSCost sets the cost matrix as:
     *
     * Given : A1 = Ax
     *         b1 = bx
     *         W1 = Wx
     *         A2 = Au
     *         b2 = bu
     *         W2 = Wu
     *
     *  1                   1
     *  - ||A2u - b2||_W2 + - ||A1x - b1||_W1 =
     *  2                   2
     *
     *
     *  =   0.5 u'A2'W2A2u + 0.5 x'A1'W1A1x - W2A2u - W1A1x +  + ... =
     *  =   0.5 u'Ru       + 0.5 x'Qx       + r'u   + q'x + ...
     *
     *  at stage i
     *
     * @param i
     * @param A1
     * @param W1
     * @param b1
     * @param A2
     * @param W2
     * @param b2
     */
    void setLSCost(const unsigned int i,
                   const Eigen::MatrixXd& Ax, const Eigen::MatrixXd& Wx, const Eigen::VectorXd& bx,
                   const Eigen::MatrixXd& Au = Eigen::MatrixXd(0,0), const Eigen::MatrixXd& Wu = Eigen::MatrixXd(0,0), const Eigen::VectorXd& bu = Eigen::VectorXd(0));



    /**
     * @brief setConstraint set the inequality constraint:
     *
     *      min <= Cx + Du <= max
     *
     * @param i
     * @param D
     * @param C
     * @param min
     * @param max
     */
    void setConstraint(const unsigned int i, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const Eigen::VectorXd& min, const Eigen::VectorXd& max);


    /**
     * @brief solve
     * @param x0 initial state
     * @return true if success
     */
    bool solve(const Eigen::VectorXd& x0);

    const std::vector<hpipm::OcpQpSolution>& getSolution() const {return _solution;}


private:
    hpipm::OcpQpIpmSolverSettings _solver_settings;
    std::vector<hpipm::OcpQp> _qp;
    std::vector<hpipm::OcpQpSolution> _solution;



    std::shared_ptr<hpipm::OcpQpIpmSolver> _solver;

    hpipm::HpipmStatus _solve_status;

    unsigned int _Ns;

    std::vector<Eigen::MatrixXd> _WxAx, _WuAu;
    std::vector<Eigen::VectorXd> _Wxbx, _Wubu;
};

}
}

#endif
