#ifndef __OPENSOT_UTILS_REGULARIZE_JACOBIAN_H__
#define __OPENSOT_UTILS_REGULARIZE_JACOBIAN_H__

#include <XBotInterface/MatLogger.hpp>
#include <lapack_svd/lapack_svd.h>

namespace OpenSoT {
    
    class JacobianRegularizer
    {
        
    public:
        
        JacobianRegularizer():
            _min_singular_value(1e-3)
        {
            
        }
        
        
        JacobianRegularizer(int rows, int cols, double min_singular_value, std::string name = ""):
            _svd(rows, cols),
            _min_singular_value(min_singular_value),
            _US(rows, rows),
            _logger(XBot::MatLogger::getLogger("/tmp/jacob_regularizer_log_" + name))
        {
            _logger->createVectorVariable("sv", std::min(rows,cols), 1, 1e4);
        }
        
        void regularize(Eigen::MatrixXd& J)
        {
            _svd.compute(J);
            
            _US = _svd.matrixU();
            
            for(int i = 0; i < _svd.singularValues().size(); i++)
            {
                double sigma = std::max(_min_singular_value, _svd.singularValues()[i]);
                _US.col(i) *= sigma;
            }
            
            J.noalias() = _US * _svd.matrixV().transpose().topRows(_US.cols());
            
            _logger->add("sv", _svd.singularValues());

            
        }
        
        ~JacobianRegularizer()
        {
            _logger->flush();
        }
        
        const Eigen::MatrixXd& matrixUSUt() 
        {
            _US = _svd.matrixU();
            
            for(int i = 0; i < _svd.singularValues().size(); i++)
            {
                double sigma = _svd.singularValues()[i];
                _US.col(i) *= sigma;
            }
            
            _USUt = _US*_svd.matrixU().transpose();
            
            return _USUt;
        }
        
        
    private:
        
        double _min_singular_value;
        LapackSvd _svd;
        Eigen::MatrixXd _US, _USUt;
        XBot::MatLogger::Ptr _logger;
        
    };
    
    
} 

#endif