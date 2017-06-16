#include <OpenSoT/solvers/DampedPseudoInverse.h>
#include <Eigen/Dense>
#include <cmath>

using namespace OpenSoT;

DampedPseudoInverse::DampedPseudoInverse(Stack& stack) : Solver<Eigen::MatrixXd, Eigen::VectorXd>(stack), sigma_min(Eigen::NumTraits<double>::epsilon())
{
    if(stack.size() > 0)
    {
        _x_size = _tasks[0]->getXSize();
        // set the metric to Identity
        _W.setIdentity(_x_size, _x_size);
        _Wchol = _W;
        _P.resize(stack.size());
        _P[0].setIdentity(_x_size, _x_size);
        _JP.resize(stack.size());
        _JPpinv.resize(stack.size());
        
        for(unsigned int i = 0; i < stack.size(); ++i)
        {
            _JPsvd.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>(
                stack[i]->getTaskSize(),_x_size, 
                Eigen::ComputeThinU | Eigen::ComputeThinV));
        }
        
        this->setSigmaMin(1e-12);
    }
}

bool DampedPseudoInverse::solve(Eigen::VectorXd& solution)
{
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        _JP[i] = _tasks[i]->getA()*_P[i];
        _JPsvd[i].compute(_JP[i]);
        
        _JPpinv[i] = this->getDampedPinv(_JP[i], _JPsvd[i]);
        solution += _JPpinv[i] * _tasks[i]->getLambda() * _tasks[i]->getb();
        if(i < _tasks.size()-1)
            // recursively computing P_i+1
            _P[i+1] = _P[i] + _JPsvd[i].matrixV()*_JPsvd[i].matrixV().transpose();
        // notice how here we could do directly:
        // svd.solve(rhs), with rhs = J*P*e;
    }
    return true;
}

Eigen::MatrixXd DampedPseudoInverse::getDampedPinv( const Eigen::MatrixXd& J,
                                                    const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                    double threshold,
                                                    double lambda_max) const
{
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

void DampedPseudoInverse::setWeight(const Eigen::MatrixXd& W)
{
    _W = W;
    _WcholSolver.compute(_W);
    _Wchol = _WcholSolver.matrixL();
}

Eigen::MatrixXd DampedPseudoInverse::getWeight() const
{
    return _W;
}


double DampedPseudoInverse::getSigmaMin() const
{
    return sigma_min;
}

void DampedPseudoInverse::setSigmaMin(const double& sigma_min)
{
    
    for(unsigned int i = 0; i < _JPsvd.size(); ++i)
    {
        if(sigma_min != Eigen::NumTraits<double>::epsilon() ||
            sigma_min != this->sigma_min)
            _JPsvd[i].setThreshold(sigma_min);
    }
    
    this->sigma_min = sigma_min;
}

