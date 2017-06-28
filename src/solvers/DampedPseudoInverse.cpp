#include <OpenSoT/solvers/DampedPseudoInverse.h>
#include <iostream>
#include <cmath>


using namespace OpenSoT::solvers;

DampedPseudoInverse::DampedPseudoInverse(Stack& stack) : Solver<Eigen::MatrixXd, Eigen::VectorXd>(stack), sigma_min(Eigen::NumTraits<double>::epsilon())
{
    if(stack.size() > 0)
    {
        _x_size = _tasks[0]->getXSize();
        // set the metric to Identity
//        _W.setIdentity(_x_size, _x_size);
//        _Wchol = _W;
//        _P.resize(stack.size());
//        _P[0].setIdentity(_x_size, _x_size);
//        _JP.resize(stack.size());
//        _JPpinv.resize(stack.size());

//        #if EIGEN_MINOR_VERSION <= 0
//            _FPL.resize(stack.size());
//        #endif

        //1 We create the aggregated tasks
        // This goes from 0 to stack.size()-1 !!!
        for(unsigned int i = 0; i < stack.size(); ++i)
        {
            if(i == 0)
                _aggregated_tasks_vector.push_back(stack[i]);
            else
                _aggregated_tasks_vector.push_back(
                    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr(
                        new OpenSoT::tasks::Aggregated(_aggregated_tasks_vector[i-1], stack[i], _x_size)));
        }

        // 2 We reserve some memory
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
                    _aggregated_tasks_vector[i-1]->getA().rows(), _x_size);
                lvl._JPpinv = Eigen::MatrixXd::Identity(
                    _x_size, _aggregated_tasks_vector[i-1]->getA().rows());

                lvl._JPsvd = Eigen::JacobiSVD<Eigen::MatrixXd>(
                            _aggregated_tasks_vector[i-1]->getA().rows(),_x_size,
                            Eigen::ComputeThinU | Eigen::ComputeThinV);

                #if EIGEN_MINOR_VERSION <= 0
                lvl._FPL = Eigen::FullPivLU<Eigen::MatrixXd>(
                            _aggregated_tasks_vector[i-1]->getA().rows(), _x_size);
                #endif

            }
            _stack_levels.push_back(lvl);
        }


        
//        for(unsigned int i = 0; i < stack.size(); ++i)
//        {
//            _JPsvd.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>(
//                stack[i]->getTaskSize(),_x_size,
//                Eigen::ComputeThinU | Eigen::ComputeThinV));

//            #if EIGEN_MINOR_VERSION <= 0
//                _FPL.push_back(Eigen::FullPivLU<Eigen::MatrixXd>(
//                    stack[i]->getTaskSize(),_x_size));
//            #endif
//        }
        
        this->setSigmaMin(1e-12);
    }
}

bool DampedPseudoInverse::solve(Eigen::VectorXd& solution)
{
    solution.setZero(solution.size());
    for(unsigned int i = 1; i <= _tasks.size(); ++i)
    {

        _stack_levels[i]._JP = _tasks[i-1]->getA()*_stack_levels[i-1]._P;
        _stack_levels[i]._JPsvd.compute(_stack_levels[i]._JP);

#if EIGEN_MINOR_VERSION <= 0
        _stack_levels[i]._FPL.compute(_stack_levels[i]._JP);
        _stack_levels[i]._JPpinv = this->getDampedPinv(
                    _stack_levels[i]._JP, _stack_levels[i]._JPsvd,
                    _stack_levels[i]._FPL,1e-3);
#else

#endif


         solution += _stack_levels[i]._JPpinv * (_tasks[i-1]->getLambda() *
                     _tasks[i-1]->getb() -
                 _tasks[i-1]->getA()*solution);



        _stack_levels[i]._P = _stack_levels[i-1]._P -
                _stack_levels[i]._JPpinv * _stack_levels[i]._JP;


//        _JP[i] = _tasks[i]->getA()*_P[i];
//        _JPsvd[i].compute(_JP[i]);
//        #if EIGEN_MINOR_VERSION <= 0
//            _FPL[i].compute(_JP[i]);
//            _JPpinv[i] = this->getDampedPinv(_JP[i], _JPsvd[i], _FPL[i]);
//        #else
//            _JPpinv[i] = this->getDampedPinv(_JP[i], _JPsvd[i]);
//        #endif



        //solution += _JPpinv[i] * (_tasks[i]->getLambda() * _tasks[i]->getb() - _tasks[i]->getA()*solution);
        //if(i < _tasks.size()-1){
            // recursively computing P_i+1
            //_P[i+1] = _P[i] + _JPsvd[i].matrixV()*_JPsvd[i].matrixV().transpose();
          //  _P[i+1] = _P[i] - _JPpinv[i]*_JP[i];}
        // notice how here we could do directly:
        // svd.solve(rhs), with rhs = J*P*e;
    }
    return true;
}

#if EIGEN_MINOR_VERSION <= 0
Eigen::MatrixXd DampedPseudoInverse::getDampedPinv(  const Eigen::MatrixXd& J,
                        const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                        const Eigen::FullPivLU<Eigen::MatrixXd>& fpl,
                        double threshold,
                        double lambda_max) const
{
    int rank = fpl.rank();
    Eigen::MatrixXd singularValuesInv(J.cols(), J.rows());
    singularValuesInv.setZero();

    double lambda = svd.singularValues().minCoeff();


    if(svd.singularValues().minCoeff() >= sigma_min)
    {
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1./svd.singularValues()[i];
    } else {
        //double lambda = std::pow(lambda_max,2) * (1. -  std::pow(svd.singularValues()[rank-1]/sigma_min,2));
        for(unsigned int i = 0; i < rank; ++i){
            singularValuesInv(i,i) =
                    svd.singularValues()[i]/(std::pow(svd.singularValues()[i],2)+lambda*lambda);
        }
    }

    return svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
}
#else
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
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = 1/singularValues[i];
    } else {
        double lambda = std::pow(lambda_max,2) * (1 -  std::pow(singularValues[rank-1]/sigma_min,2));
        for(unsigned int i = 0; i < rank; ++i)
            singularValuesInv(i,i) = singularValues[i]/(std::pow(singularValues[i],2)+lambda);
    }
    
    return svd.matrixV()* singularValuesInv *svd.matrixU().transpose();
}
#endif

//void DampedPseudoInverse::setWeight(const Eigen::MatrixXd& W)
//{
//    _W = W;
//    _WcholSolver.compute(_W);
//    _Wchol = _WcholSolver.matrixL();
//}

//Eigen::MatrixXd DampedPseudoInverse::getWeight() const
//{
//    return _W;
//}


double DampedPseudoInverse::getSigmaMin() const
{
    return sigma_min;
}

#if EIGEN_MINOR_VERSION <= 0
void DampedPseudoInverse::setSigmaMin(const double& sigma_min)
{

    for(unsigned int i = 0; i < _stack_levels.size(); ++i)
    {
        if(sigma_min != Eigen::NumTraits<double>::epsilon() ||
            sigma_min != this->sigma_min)
            _stack_levels[i]._FPL.setThreshold(sigma_min);
    }

    this->sigma_min = sigma_min;
}
#else
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
#endif
