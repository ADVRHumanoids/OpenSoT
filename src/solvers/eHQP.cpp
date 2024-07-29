#include <OpenSoT/solvers/eHQP.h>
#include <iostream>
#include <cmath>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;

eHQP::eHQP(Stack& stack) : Solver<Eigen::MatrixXd, Eigen::VectorXd>(stack), sigma_min(Eigen::NumTraits<double>::epsilon())
{
    //if(stack.size() > 0)
    //{
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
                if(_tasks[i-1]->getHessianAtype() == HessianType::HST_ZERO){
                    std::stringstream error_ss;
                    error_ss << "Task "<<i-1<<" has Hessian Type HST_ZERO which is not handled by eHQP, aborting!"<<std::endl;
                    throw std::runtime_error(error_ss.str());}

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
                printProblemInformation(i, _tasks[i]->getTaskID(),"NONE", "NONE");
        }

        this->setSigmaMin(1e-12);

        for(unsigned int i = 0; i < stack.size(); ++i)
            printProblemInformation(i, _tasks[i]->getTaskID(), "NONE", "NONE");
    //}
}

bool eHQP::solve(Eigen::VectorXd& solution)
{
    solution.setZero(_x_size);
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
#else
        _stack_levels[i]._JPpinv = this->getDampedPinv(_stack_levels[i]._JP,
                                                       _stack_levels[i]._JPsvd);
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
Eigen::MatrixXd eHQP::getDampedPinv(  const Eigen::MatrixXd& J,
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
Eigen::MatrixXd eHQP::getDampedPinv( const Eigen::MatrixXd& J,
                                                    const Eigen::JacobiSVD<Eigen::MatrixXd>& svd) const
{
    int rank = svd.rank();
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
#endif

double eHQP::getSigmaMin() const
{
    return sigma_min;
}

#if EIGEN_MINOR_VERSION <= 0
void eHQP::setSigmaMin(const double& sigma_min)
{
    if(sigma_min > 0)
    {
        for(unsigned int i = 0; i < _stack_levels.size(); ++i)
            _stack_levels[i]._FPL.setThreshold(sigma_min);

        this->sigma_min = sigma_min;
    }
}
#else
void eHQP::setSigmaMin(const double& sigma_min)
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

void eHQP::printProblemInformation(const int problem_number, const std::string& problem_id,
                                      const std::string& constraints_id, const std::string& bounds_id)
{
    std::cout<<std::endl;
    if(problem_number == -1)
        XBot::Logger::info("PROBLEM ID: %s \n", problem_id.c_str());
    else
        XBot::Logger::info("PROBLEM %i ID: %s \n", problem_number, problem_id.c_str());

    XBot::Logger::info("CONSTRAINTS ID: %s \n", constraints_id.c_str());
    XBot::Logger::info("    # OF CONSTRAINTS: %i \n", 0);

    XBot::Logger::info("BOUNDS ID: %s \n", bounds_id.c_str());
    XBot::Logger::info("    # OF BOUNDS: %i \n", 0);

    XBot::Logger::info("# OF VARIABLES: %i \n", _x_size);

    XBot::Logger::info("sigman _min: %f", this->sigma_min);

    XBot::Logger::info("\n");
}

