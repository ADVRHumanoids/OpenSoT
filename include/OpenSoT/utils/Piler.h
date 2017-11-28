#include <Eigen/Dense>

namespace OpenSoT { namespace utils {
   
    class PilerHelper {
        
    public:
        
        PilerHelper(int cols = -1);
        
        void reset();
        
        template <typename Derived>
        void pile(const Eigen::MatrixBase<Derived>& matrix);
        
        const Eigen::MatrixXd& generate_and_get();
        
    private:
        
        int _cols;
        int _current_row;
        
        Eigen::MatrixXd _mat;
        
    };
    
} }


OpenSoT::utils::PilerHelper::PilerHelper(int cols):
    _cols(cols),
    _current_row(0)
{
    _mat.resize(0, _cols);
}

template <typename Derived>
void OpenSoT::utils::PilerHelper::pile(const Eigen::MatrixBase<Derived>& matrix)
{
    if(matrix.cols() != _cols){
        throw std::runtime_error("matrix.cols() != _cols");
    }
    
    int rows_needed = _current_row + matrix.rows();
    
    if( rows_needed > _mat.rows() ){
        printf("PilerHelper: expanding to %d x %d \n", rows_needed, _cols);
        _mat.conservativeResize(rows_needed, _cols);
    }
    
    _mat.block(_current_row, 0, matrix.rows(), matrix.cols()) = matrix;
    
    _current_row += matrix.rows();
    
}

void OpenSoT::utils::PilerHelper::reset()
{
    _current_row = 0;
}

const Eigen::MatrixXd& OpenSoT::utils::PilerHelper::generate_and_get()
{
    _mat.conservativeResize(_current_row, _cols);
    return _mat;
}

