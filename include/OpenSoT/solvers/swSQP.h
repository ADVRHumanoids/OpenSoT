#ifndef _WB_SOT_SOLVERS_SWSQP_H_
#define _WB_SOT_SOLVERS_SWSQP_H_

#include <OpenSoT/solvers/hpipmOC.h>
#include <OpenSoT/utils/oc.h>

namespace OpenSoT{
namespace solvers{

class swSQP{
public:
    typedef std::shared_ptr<swSQP> Ptr;

    struct options
    {
        options()
        {
            max_iters = 100;
        }

        unsigned int max_iters;
    };

    swSQP(OpenSoT::ocp::Ptr ocp);


    options& getOptions(){return _opt;}

    bool solve(const std::vector<Eigen::VectorXd>& x0, const std::vector<Eigen::VectorXd>& u0);

    const std::vector<Eigen::VectorXd>& getStateSolution() const { return _x0;}
    const std::vector<Eigen::VectorXd>& getControlSolution() const { return _u0;}


private:
    void _init();

    /**
     * @brief flatten assumes all vectors in vecs have the same size!
     * @param vecs
     * @param vec
     * @return
     */
    void flatten(const std::vector<Eigen::VectorXd>& vecs, Eigen::VectorXd& vec)
    {
        const std::size_t m = vecs[0].size();   // size of each subvector
        const std::size_t N = vecs.size();      // number of subvectors

        vec.resize(N * m);

        for(std::size_t i = 0; i < N; ++i) {
            vec.segment(i * m, m) = vecs[i];
        }
    }

    hpipmOC::Ptr _qp_solver;
    OpenSoT::ocp::Ptr _ocp;

    options _opt;

    // stores dynamics in the horizon
    std::vector<Eigen::MatrixXd> _A;
    std::vector<Eigen::MatrixXd> _B;

    // stores cost in the horizon
    std::vector<Eigen::MatrixXd> _H, _Q, _R, _S;
    std::vector<Eigen::VectorXd> _g, _q, _r;


    std::vector<Eigen::VectorXd> _x0, _u0;
};

}
}

#endif
