#ifndef _WB_SOT_SOLVERS_SWSQP_H_
#define _WB_SOT_SOLVERS_SWSQP_H_

#include <OpenSoT/solvers/hpipmOC.h>
#include <OpenSoT/utils/oc.h>

namespace OpenSoT{
namespace solvers{

class swSQP{
public:
    typedef std::shared_ptr<swSQP> Ptr;

    struct stage_statistics
    {
        double cost;
    };

    struct statistics
    {
        statistics(const unsigned int Ns)
        {
            stages_statistics.resize(Ns + 1);
        }

        std::vector<stage_statistics> stages_statistics;
        int iters;
        double cost;

        const std::ostringstream& toOSS()
        {
            _oss<<"swSQP Stats: "<<std::endl;
            _oss<<" iters: "<<iters<<std::endl;
            _oss<<" cost: "<<cost<<std::endl;

            for(unsigned int i = 0; i < stages_statistics.size(); ++i)
            {
                _oss<<" stage "<<i<<" stats:"<<std::endl;
                _oss<<"     cost: "<<stages_statistics[i].cost<<std::endl;
            }

            return _oss;
        }

    private:
        std::ostringstream _oss;
    };

    struct options
    {
        options()
        {
            max_iters = 100;
            min_abs_delta_solution = 1e-7;
        }

        /**
         * @brief max_iters maximum number of iterations of solve
         */
        unsigned int max_iters;

        /**
         * @brief min_abs_delta_solution minimum absolute delta solution allowed for increment solution
         */
        double min_abs_delta_solution;

        const std::ostringstream& toOSS()
        {
            _oss<<"swSQP Options: "<<max_iters<<std::endl;
            _oss<<" max_iters: "<<max_iters<<std::endl;
            _oss<<" min_abs_delta_solution: "<<min_abs_delta_solution<<std::endl;

            return _oss;
        }

    private:
        std::ostringstream _oss;
    };

    swSQP(OpenSoT::ocp::Ptr ocp);


    options& getOptions(){return _opt;}

    bool solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0);

    const std::vector<Eigen::VectorXd>& getStateSolution() const { return _x0;}
    const std::vector<Eigen::VectorXd>& getControlSolution() const { return _u0;}

    void printStatistics(){ std::cout<<_stats.toOSS().str()<<std::endl; }


private:
    void _init();

    void computeDynamics(const unsigned int i, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b);
    void computeQuadraticApproximation(const unsigned int i, Eigen::MatrixXd& H, Eigen::VectorXd& g);
    void computeCost(const unsigned int i,
                     Eigen::MatrixXd& Q, Eigen::VectorXd& q,
                     Eigen::MatrixXd& R, Eigen::VectorXd& r,
                     Eigen::MatrixXd& S);

    hpipmOC::Ptr _qp_solver;
    OpenSoT::ocp::Ptr _ocp;

    options _opt;
    statistics _stats;

    std::vector<Eigen::MatrixXd> _Mx, _Mu;

    // stores dynamics in the horizon
    std::vector<Eigen::MatrixXd> _A;
    std::vector<Eigen::MatrixXd> _B;
    std::vector<Eigen::VectorXd> _b;

    // stores cost in the horizon
    std::vector<Eigen::MatrixXd> _H, _Q, _R, _S;
    std::vector<Eigen::VectorXd> _g, _q, _r;


    std::vector<Eigen::VectorXd> _x0, _u0;


};

}
}

#endif
