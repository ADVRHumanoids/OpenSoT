#include <OpenSoT/solvers/DampedPseudoInverse.h>
#include <iostream>
#include <cmath>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;

DampedPseudoInverse::DampedPseudoInverse(Stack& stack) : Solver<Eigen::MatrixXd, Eigen::VectorXd>(stack), sigma_min(Eigen::NumTraits<double>::epsilon())
{
    if(stack.size() > 0)
    {
        _x_size = _tasks[0]->getXSize();
        // We reserve some memory
        // this goes from 0 to stack.size() !!!
        stack_level lvl;
        for(unsigned int i = 0; i <= stack.size(); ++i)
        {
            if(i == 0)
            {
                lvl._P = Eigen::MatrixXd::Identity(_x_size, _x_size);
            }
            else
            {
                lvl._P = Eigen::MatrixXd::Identity(_x_size, _x_size);
                lvl._JP = Eigen::MatrixXd::Identity(
                    _tasks[i-1]->getA().rows(), _x_size);
                lvl._JPpinv = Eigen::MatrixXd::Identity(
                    _x_size, _tasks[i-1]->getA().rows());

                lvl._JPsvd = Eigen::JacobiSVD<Eigen::MatrixXd>(
                            _tasks[i-1]->getA().rows(),_x_size,
                            Eigen::ComputeThinU | Eigen::ComputeThinV);

                #if EIGEN_MINOR_VERSION <= 0
                lvl._FPL = Eigen::FullPivLU<Eigen::MatrixXd>(
                            _tasks[i-1]->getA().rows(), _x_size);

                #endif


            }
            _stack_levels.push_back(lvl);

            if(i < stack.size())
                printProblemInformation(i, _tasks[i]->getTaskID(),
                                    "NONE", "NONE");
        }

        this->setSigmaMin(1e-12);

        for(unsigned int i = 0; i < stack.size(); ++i)
            printProblemInformation(i, _tasks[i]->getTaskID(),
                                    "NONE", "NONE");
    }
}

bool DampedPseudoInverse::solve(Eigen::VectorXd& solution)
{
    solution.setZero(solution.size());
    for(unsigned int i = 1; i <= _tasks.size(); ++i)
    {
        _stack_levels[i]._WChol.compute(_tasks[i-1]->getWeight());

        _stack_levels[i]._JP = _stack_levels[i]._WChol.matrixL().transpose() * _tasks[i-1]->getA()*_stack_levels[i-1]._P;
        _stack_levels[i]._JPsvd.compute(_stack_levels[i]._JP);

#if EIGEN_MINOR_VERSION <= 0
        _stack_levels[i]._FPL.compute(_stack_levels[i]._JP);
        _stack_levels[i]._JPpinv = this->getDampedPinv(
                    _stack_levels[i]._JP, _stack_levels[i]._JPsvd,
                    _stack_levels[i]._FPL);
#endif

         solution += _stack_levels[i]._JPpinv * (
                     _stack_levels[i]._WChol.matrixL().transpose() * _tasks[i-1]->getb()-
                 _stack_levels[i]._WChol.matrixL().transpose() * _tasks[i-1]->getA()*solution
                 );



        _stack_levels[i]._P = _stack_levels[i-1]._P -
                _stack_levels[i]._JPsvd.matrixV() * _stack_levels[i]._JPsvd.matrixV().transpose();
    }
    return true;
}

#if EIGEN_MINOR_VERSION <= 0
Eigen::MatrixXd DampedPseudoInverse::getDampedPinv(  const Eigen::MatrixXd& J,
                        const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                        const Eigen::FullPivLU<Eigen::MatrixXd>& fpl) const
{
    int rank = fpl.rank();
    Eigen::MatrixXd singularValuesInv(J.cols(), J.cols());
    singularValuesInv.setZero(J.rows(), J.rows());

    double lambda = svd.singularValues().minCoeff();


    if(svd.singularValues().minCoeff() >= sigma_min)
    {
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1./svd.singularValues()[i];
    } else {
        //double lambda = std::pow(lambda_max,2) * (1. -  std::pow(svd.singularValues()[rank-1]/sigma_min,2));
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) =
                    svd.singularValues()[i]/(std::pow(svd.singularValues()[i],2)+lambda*lambda);
    }


    return svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
}
#else
Eigen::MatrixXd DampedPseudoInverse::getDampedPinv( const Eigen::MatrixXd& J,
                                                    const Eigen::JacobiSVD<Eigen::MatrixXd>& svd) const
{
    int rank = svd.rank();
    Eigen::MatrixXd singularValuesInv(J.cols(), J.rows());
    singularValuesInv.setZero();

    double lambda = svd.singularValues().minCoeff();


    if(svd.singularValues().minCoeff() >= sigma_min)
    {
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1./svd.singularValues()[i];
    } else {
        //double lambda = std::pow(lambda_max,2) * (1. -  std::pow(svd.singularValues()[rank-1]/sigma_min,2));
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) =
                    svd.singularValues()[i]/(std::pow(svd.singularValues()[i],2)+lambda*lambda);
    }

    return svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
}
#endif

double DampedPseudoInverse::getSigmaMin() const
{
    return sigma_min;
}

#if EIGEN_MINOR_VERSION <= 0
void DampedPseudoInverse::setSigmaMin(const double& sigma_min)
{
    if(sigma_min > 0)
    {
        for(unsigned int i = 0; i < _stack_levels.size(); ++i)
            _stack_levels[i]._FPL.setThreshold(sigma_min);

        this->sigma_min = sigma_min;
    }
}
#else
void DampedPseudoInverse::setSigmaMin(const double& sigma_min)
{
    if(sigma_min > 0)
    {
        // for(unsigned int i = 0; i < _JPsvd.size(); ++i)
        //    _JPsvd[i].setThreshold(sigma_min);
        for(unsigned int i = 0; i < _stack_levels.size(); ++i)
            _stack_levels[i]._JPsvd.setThreshold(sigma_min);


        this->sigma_min = sigma_min;
    }
}
#endif

void DampedPseudoInverse::printProblemInformation(const int problem_number, const std::string& problem_id,
                    const std::string& constraints_id, const std::string& bounds_id)
{
    std::cout<<std::endl;
    if(problem_number == -1)
        std::cout<<GREEN<<"PROBLEM ID: "<<DEFAULT<<problem_id<<std::endl;
    else
        std::cout<<GREEN<<"PROBLEM "<<problem_number<<" ID: "<<DEFAULT<<problem_id<<std::endl;
    std::cout<<GREEN<<"sigman _min: "<<DEFAULT<<this->sigma_min<<std::endl;
    std::cout<<GREEN<<"CONSTRAINTS ID: "<<DEFAULT<<constraints_id<<std::endl;
    std::cout<<GREEN<<"     # OF CONSTRAINTS: "<<DEFAULT<<0<<std::endl;
    std::cout<<GREEN<<"BOUNDS ID: "<<DEFAULT<<bounds_id<<std::endl;
    std::cout<<GREEN<<"     # OF BOUNDS: "<<DEFAULT<<0<<std::endl;
    std::cout<<GREEN<<"# OF VARIABLES: "<<DEFAULT<<_x_size<<std::endl;
    std::cout<<std::endl;
}
