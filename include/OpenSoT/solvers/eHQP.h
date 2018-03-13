#ifndef __SOLVERS_PSEUDOINVERSE__
#define __SOLVERS__PSEUDOINVERSE__

#include <OpenSoT/Solver.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Macros.h>
#include <OpenSoT/tasks/Aggregated.h>

#if EIGEN_MINOR_VERSION <= 0
#include <Eigen/LU>
#endif

namespace OpenSoT{
    namespace solvers{
    struct stack_level
    {
        Eigen::MatrixXd _P;
        Eigen::MatrixXd _JP;
        Eigen::MatrixXd _JPpinv;
        Eigen::JacobiSVD<Eigen::MatrixXd> _JPsvd;
#if EIGEN_MINOR_VERSION <= 0
        Eigen::FullPivLU<Eigen::MatrixXd> _FPL;
#endif
        /**
         * @brief _WChol is used to handle weights for each task.
         * We compute W = LL' and then we multiply L'A and L'b
         */
        Eigen::LLT<Eigen::MatrixXd> _WChol;
    };
    /**
     * @brief The eHQP class implements an equality Hierarchical QP solver as the one used in:
     * "Prioritized Multi-Task Motion Control of Redundant Robots under Hard Joint Constraints"
     * by Fabrizio Flacco, Alessandro De Luca and Oussama Khatib
     */
    class eHQP : public OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        int _x_size;
        std::vector<stack_level> _stack_levels;

        /**
         * @brief getDampedPinv computes the weighted, damped pseudoinverse of A
         *        The pseudoinverse is damped using one of the methods listed in 
         *        "Deo and Walker, 1995, Overview of damped least-squares methods 
         *        for inverse kinematics of robot manipulators", and used e.g. in
         *        "The Tasks Priority Matrix: a new tool for hierarchical redundancy 
         *        resolution". The pseudoinversion is computed by performing a "thin"
         *        SVD decomposition \f$J = U \Sigma V^T\f$ with 
         *        \f$J\in\mathcal{R}^{m\timesn}\f$, 
         *        \f$U\in\mathcal{R}^{m\timesr}\f$ unitary,
         *        \f$V\in\mathcal{R}^{n\timesr}\f$ unitary,
         *        \f$\Sigma\in\mathcal{R}^{r\timesr}\f$ diagonal, and with
         *        \f$r\f$ the number of singular values greater than \f$\text{threshold}\f$.
         *        Then \f$J^\dagger = V \Sigma^\dagger U^T\f$ with \f$\Sigma^\dagger\f$
         *        obtained by computing the inverse of every element in the diagonal,
         *        eventually after adding a term deriving from Tikhonov regularization.
         *        The matrix is damped using the minimum singular value that is an index of
         *        manipulability of the robot.
         *        The JacobiSVD method from Eigen is used, which uses QR decomposition
         *        by Householder transformations with column pivoting.
         *        The SVD decomposition is also used the compute the projectors, since
         *        we have that \f$AA^\dagger=U_1U_1^T\f$
         *
         */
        #if EIGEN_MINOR_VERSION <= 0
        Eigen::MatrixXd getDampedPinv(  const Eigen::MatrixXd& J,
                                const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                const Eigen::FullPivLU<Eigen::MatrixXd>& fpl) const;
        #else
        Eigen::MatrixXd getDampedPinv(  const Eigen::MatrixXd& J,
                                        const Eigen::JacobiSVD<Eigen::MatrixXd>& svd) const;
        #endif
                                        
        /** @brief sigma_min is the minimum value which is accepted for 
         *                   a singular value before regularization is enabled */
        double sigma_min;

        void printProblemInformation(const int problem_number, const std::string& problem_id,
                                     const std::string& constraints_id, const std::string& bounds_id);

    public:
        typedef boost::shared_ptr<eHQP> Ptr;
        /**
         * @brief creates a pseudoinverse solver for the current Stack
         */
        eHQP(Stack& stack);
        
        /**
        * @brief solve solve the optimization problem by minimizing the error in the weighted least squares sense
        * @param solution the argmin of the optimization
        * @return  true if solved/solvable
        */
        bool solve(Eigen::VectorXd& solution);
        
        
        double getSigmaMin() const;
        
        void setSigmaMin(const double& sigma_min);        
    };
}
}

#endif
