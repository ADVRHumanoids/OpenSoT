#ifndef _WB_SOT_SOLVERS_SWSQP_H_
#define _WB_SOT_SOLVERS_SWSQP_H_

#include <OpenSoT/solvers/hpipmOC.h>
#include <OpenSoT/oc/oc.h>
#include <chrono>
#include <limits>

namespace OpenSoT{
namespace solvers{

class swSQP{
public:
    typedef std::shared_ptr<swSQP> Ptr;

    struct stage_statistics
    {
        double cost;
        double constraint_violation;
    };

    struct statistics
    {
        statistics(const unsigned int Ns)
        {
            stages_statistics.resize(Ns);
        }

        std::vector<stage_statistics> stages_statistics;
        int iters;
        double cost;
        double constraint_violation;
        double alpha;
        int line_search_iters;
        bool line_search_accepted;
        double iter_time;
        double total_time = std::numeric_limits<double>::quiet_NaN();

        const std::ostringstream& toOSS()
        {
            _oss.str("");
            _oss.clear();
            _oss<< std::boolalpha;

            _oss<<"=== swSQP Statistics ==="<<std::endl;
            _oss << "  iter              : " << iters << std::endl;
            _oss << "  cost              : " << cost << std::endl;
            _oss << "  sum-constr-viol   : " << constraint_violation << std::endl;
            _oss << "  iter time         : " << iter_time << std::endl;
            _oss << "  total time        : " << total_time << std::endl;
            _oss << " === LineSearch Statistics === " << std::endl;
            _oss << "   accepted         : " << line_search_accepted << std::endl;
            _oss << "   alpha            : " << alpha << std::endl;
            _oss << "   ls iters         : " << line_search_iters << std::endl;


            _oss << "=== swSQP Stage Statistics ===" << std::endl;

            // Column headers
            _oss << std::setw(10) << "Stage"
                << std::setw(15) << "Cost"
                << std::setw(25) << "Constr-Viol"
                << std::endl;

            // Print a separator line (optional)
            _oss << std::string(50, '-') << std::endl;

            // Print one row per stage
            for (size_t i = 0; i < stages_statistics.size(); ++i) {
                const auto& s = stages_statistics[i];
                _oss << std::setw(10) << i
                    << std::setw(15) << s.cost
                    << std::setw(25) << s.constraint_violation
                    << std::endl;
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
            verbose = false;
            alpha_min = 0.125;
            beta = 1e-4;
            line_search_strategy = 0;
        }

        //termination criteria
        unsigned int max_iters;
        double min_abs_delta_solution;

        /// LineSearch
        double alpha_min;
        uint line_search_strategy;
        double beta; // multiply merit derivative in Armijo's condition in line search

        bool verbose;

        const std::ostringstream& toOSS()
        {
            _oss.str("");
            _oss.clear();

            _oss << "=== swSQP Options ===" << std::endl;
            _oss << "  verbose                : " << verbose << std::endl;
            _oss << "  max_iters              : " << max_iters << std::endl;
            _oss << "  min_abs_delta_solution : " << min_abs_delta_solution << std::endl;
            _oss << "  alpha_min              : " << alpha_min << std::endl;
            _oss << "  beta                   : " << beta << std::endl;
            _oss << "  line_search_strategy   : " << line_search_strategy << std::endl;

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

    hpipmOC::Ptr getQPSolver() { return _qp_solver; }

    void init();

private:
    

    void computeDynamics(const unsigned int i, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b);
    void computeCost(const unsigned int i,
                     Eigen::MatrixXd& Q, Eigen::VectorXd& q,
                     Eigen::MatrixXd& R, Eigen::VectorXd& r,
                     Eigen::MatrixXd& S);
    void computeConstraints(const unsigned int i,
                            Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::VectorXd& dl, Eigen::VectorXd& du);


    void linearize(); // update linearization/quadritization matrices
    void update_statistics();
    void step(double alpha); //step of the solver
    bool break_criteria(); // sqp solvers breaking criteria

    double _prev_cost;  // total cost
    double _prev_defect; // total gap violation
    double _prev_viol; // total constraint violation 

    bool (swSQP::*ls_function)();
    bool ls_filter(); // filter line search implementation
    bool ls_merit(); // merit line search implementation

    std::vector<Eigen::VectorXd> dcost_dw; // cost gradient 
    std::vector<Eigen::VectorXd> ddefect_dw; //defect gradient 
    std::vector<Eigen::VectorXd> dviol_dw;  //violations gradient


    hpipmOC::Ptr _qp_solver;
    OpenSoT::ocp::Ptr _ocp;

    options _opt;
    statistics _stats;

    std::vector<Eigen::MatrixXd> _Mx, _Mu;

    // stores dynamics in the horizon
    std::vector<Eigen::MatrixXd> _A, _B;
    std::vector<Eigen::VectorXd> _b;

    // stores cost in the horizon
    std::vector<Eigen::MatrixXd> _H, _Q, _R, _S;
    std::vector<Eigen::VectorXd> _g, _q, _r;

    // stores constraints in the horizon
    std::vector<Eigen::MatrixXd> _D, _C;
    std::vector<Eigen::VectorXd> _dl, _du;

    std::vector<Eigen::VectorXd> _x0, _u0;
    std::vector<Eigen::VectorXd> _x0_candidate, _u0_candidate;

    Eigen::VectorXd _dx0; //initial delta state constraint (_dx0 = 0)
    

};

}
}

#endif
