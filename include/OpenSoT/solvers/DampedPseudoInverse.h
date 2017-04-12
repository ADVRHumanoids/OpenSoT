#ifndef __SOLVERS_PSEUDOINVERSE__
#define __SOLVERS__PSEUDOINVERSE__

#include <OpenSoT/Solver.h>
#include <Eigen/Core>

namespace OpenSoT
{
    
    class DampedPseudoInverse : public OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        Eigen::MatrixXd _W;
        std::vector<Eigen::MatrixXd> _P, _Jpinv; // occupy more memory, avoid reallocating during execution
        int _x_size;

        
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
         * @param threshold is the rank of \f$r = \text{rank}\left(J^TJ\right)\f$ is
         *                  the number of singular values \f$\sigma_i\f$ such that 
         *                  \f$\sigma_i > \text{threshold}\f$,which defaults to
         *                  Eigen::NumTraits<double>::epsilon()
         * @param sigma_min is the minimum value which is accepted for a singular value
         *                  before regularization is enabled
         * @param lambda_max is the maximum regularization term that is applied, when the
         *                   smallest singular value of J is \f$\sigma \eq \text{threshold}\f$
         * 
         */
        Eigen::MatrixXd getDampedPinv(  const Eigen::MatrixXd& J,
                                        double threshold = Eigen::NumTraits< double >::epsilon(),
                                        double sigma_min = 1e-12,
                                        double lambda_max = 1e-6) const;
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
         * @param W the distance metric according to which we will manimize the squared error.
         *          Notice it needs to be of compatible dimensions with the \f$x\f$ vector,
         *          that is \f$\mathbb{R}^{n\timesn}\f$
         */
        void setWeight(const Eigen::MatrixXd& W);
        
        /**
         * @brief getWeight returns the metric that is currently used for the PseudoInverse
         */
        Eigen::MatrixXd getWeight() const;
    };

}

#endif