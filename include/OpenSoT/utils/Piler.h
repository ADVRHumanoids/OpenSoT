#include <Eigen/Dense>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

namespace OpenSoT { namespace utils {
   
    class MatrixPiler {
        
    public:
        
        MatrixPiler(int cols = -1);
        
        void reset();
        
        template <typename Derived>
        void pile(const Eigen::MatrixBase<Derived>& matrix);

        template <typename Derived>
        void set(const Eigen::MatrixBase<Derived>& matrix);
        
        const Eigen::MatrixXd& generate_and_get();
        
    private:
        
        int _cols;
        int _current_row;
        
        Eigen::MatrixXd _mat;
        
    };
    
} }


OpenSoT::utils::MatrixPiler::MatrixPiler(int cols):
    _cols(cols),
    _current_row(0)
{
    _mat.resize(0, _cols);
}

template <typename Derived>
void OpenSoT::utils::MatrixPiler::pile(const Eigen::MatrixBase<Derived>& matrix)
{
    if(matrix.cols() != _cols){
        throw std::runtime_error("matrix.cols() != _cols");
    }
    
    int rows_needed = _current_row + matrix.rows();
    
    if( rows_needed > _mat.rows() ){
        Logger::info("PilerHelper: expanding to %d x %d \n", rows_needed, _cols);
        _mat.conservativeResize(rows_needed, _cols);
    }
    
    _mat.block(_current_row, 0, matrix.rows(), matrix.cols()) = matrix;
    
    _current_row += matrix.rows();
    
}


template <typename Derived>
void OpenSoT::utils::MatrixPiler::set(const Eigen::MatrixBase<Derived>& matrix)
{
    _cols = matrix.cols();

    _mat = matrix;

    _current_row = _mat.rows();

}

void OpenSoT::utils::MatrixPiler::reset()
{
    _current_row = 0;
}

const Eigen::MatrixXd& OpenSoT::utils::MatrixPiler::generate_and_get()
{
    if(_current_row != _mat.rows()){
        _mat.conservativeResize(_current_row, _cols);
        Logger::info() << "Doing a conservativeResize!" << Logger::endl();
    }
    return _mat;
}

