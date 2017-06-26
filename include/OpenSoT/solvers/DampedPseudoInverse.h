#ifndef __SOLVERS_PSEUDOINVERSE__
#define __SOLVERS__PSEUDOINVERSE__

#include <OpenSoT/Solver.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/src/Core/util/Macros.h>

#if EIGEN_MINOR_VERSION <= 0
#include <Eigen/LU>
#endif

namespace OpenSoT
{
    
    class DampedPseudoInverse : public OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        Eigen::MatrixXd _W, _Wchol;
        Eigen::LLT<Eigen::MatrixXd> _WcholSolver;
        std::vector<Eigen::MatrixXd> _P, _JP, _JPpinv; // occupy more memory, avoid reallocating during execution
        std::vector<Eigen::JacobiSVD<Eigen::MatrixXd>> _JPsvd;
        int _x_size;

#if EIGEN_MINOR_VERSION <= 0
        std::vector<Eigen::FullPivLU<Eigen::MatrixXd>> _FPL;
#endif

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
         *        The JacobiSVD method from Eigen is used, which uses QR decomposition
         *        by Householder transformations with column pivoting.
         *        The SVD decomposition is also used the compute the projectors, since
         *        we have that \f$AA^\dagger=U_1U_1^T\f$
         * @param threshold is the rank of \f$r = \text{rank}\left(J^TJ\right)\f$ is
         *                  the number of singular values \f$\sigma_i\f$ such that 
         *                  \f$\sigma_i > \text{threshold}\f$,which defaults to
         *                  Eigen::NumTraits<double>::epsilon()
         * @param lambda_max is the maximum regularization term that is applied, when the
         *                   smallest singular value of J is \f$\sigma \eq \text{threshold}\f$
         * 
         */
        #if EIGEN_MINOR_VERSION <= 0
        Eigen::MatrixXd getDampedPinv(  const Eigen::MatrixXd& J,
                                const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                const Eigen::FullPivLU<Eigen::MatrixXd>& fpl,
                                double threshold = Eigen::NumTraits< double >::epsilon(),
                                double lambda_max = 1e-6) const;
        #else
        Eigen::MatrixXd getDampedPinv(  const Eigen::MatrixXd& J,
                                        const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                        double threshold = Eigen::NumTraits< double >::epsilon(),
                                        double lambda_max = 1e-6) const;
        #endif
                                        
        /** @brief sigma_min is the minimum value which is accepted for 
         *                   a singular value before regularization is enabled */
        double sigma_min;

    public:
        /**
         * @brief creates a pseudoinverse solver for the current Stack
         */
        DampedPseudoInverse(Stack& stack);
        
        /**
        * @brief solve solve the optimization problem by minimizing the error in the weighted least squares sense
        * @param solution the argmin of the optimization
        * @return  true if solved/solvable
        */
        bool solve(Eigen::VectorXd& solution);
        
                /**
         * @brief setWeight sets the error metric for the PseudoInverse
         * @param W the distance metric according to which we will manimize the squared error. It needs to be a positive definite matrix.
         *          Notice it needs to be of compatible dimensions with the \f$x\f$ vector,
         *          that is \f$\mathbb{R}^{n\timesn}\f$
         */
        void setWeight(const Eigen::MatrixXd& W);
        
        double getSigmaMin() const;
        
        void setSigmaMin(const double& sigma_min);
        
        /**
         * @brief getWeight returns the metric that is currently used for the PseudoInverse
         */
        Eigen::MatrixXd getWeight() const;
    };

}

#endif
