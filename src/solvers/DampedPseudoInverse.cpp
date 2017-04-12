#include <OpenSoT/solvers/DampedPseudoInverse.h>
#include <Eigen/Dense>
#include <cmath>

using namespace OpenSoT;

DampedPseudoInverse::DampedPseudoInverse(Stack& stack) : Solver<Eigen::MatrixXd, Eigen::VectorXd>(stack)
{
    if(stack.size() > 0)
    {
        _x_size = _tasks[0]->getXSize();
        // set the metric to Identity
        _W.setIdentity(_x_size, _x_size);
        _P.resize(stack.size());
        _Jpinv.resize(stack.size());
    }
}

bool DampedPseudoInverse::solve(Eigen::VectorXd& solution)
{
    Eigen::MatrixXd P;
    P.setIdentity(_x_size, _x_size);    // P_0
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        Eigen::MatrixXd J = _tasks[i]->getA();
        // recursively computing P_i
        P +=  this->getDampedPinv(J*P)*J*P;
        // notice how here we could do directly:
        // svd.solve(rhs), with rhs = J*P*e;
    }
    return true;
}

Eigen::MatrixXd DampedPseudoInverse::getDampedPinv( const Eigen::MatrixXd& J,
                                                    double threshold,
                                                    double sigma_min,
                                                    double lambda_max) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if(sigma_min != Eigen::NumTraits<double>::epsilon())
        svd.setThreshold(sigma_min);
    int rank = svd.rank();  // the rank is computed considering singular values greater than sigma_min
    const Eigen::VectorXd &singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv(J.cols(), J.rows());
    singularValuesInv.setZero();
    
    if(singularValues[rank-1] <= sigma_min)
    {
        for(unsigned int i = 0; ++i; i < rank)
            singularValuesInv(i,i) = 1/singularValues[i];
    } else {
        double lambda = std::pow(lambda_max,2) * (1 -  std::pow(singularValues[rank-1]/sigma_min,2));
        for(unsigned int i = 0; ++i; i < rank)
            singularValuesInv(i,i) = singularValues[i]/(std::pow(singularValues[i],2)+lambda);
    }
    
    return svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
}

