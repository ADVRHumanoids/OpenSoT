#ifndef _OPENSOT_UTILS_PILER_H_
#define _OPENSOT_UTILS_PILER_H_

#include <Eigen/Dense>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

namespace OpenSoT { namespace utils {
    /**
     * @brief The MatrixPiler class implements real-time safe matrix/vector piling
     */
    class MatrixPiler {
        
    public:
        /**
         * @brief MatrixPiler constructor, the matrix starts with 0 rows
         * @param cols number of columns
         */
        MatrixPiler(const int cols = 0);
        
        /**
         * @brief reset the number of rows of the matrix to 0
         */
        void reset();

        /**
         * @brief reset the number of rows of the matrix to 0 and set new number of columns
         * @param cols new nnumber of columns
         */
        void reset(const int cols);
        
        template <typename Derived>
        /**
         * @brief pile a new matrix
         * @param matrix to pile
         */
        void pile(const Eigen::MatrixBase<Derived>& matrix);

        template <typename Derived>
        /**
         * @brief set reset the matrix and initialize with new matrix
         * @param matrix for initialization
         */
        void set(const Eigen::MatrixBase<Derived>& matrix);
        
        /**
         * @brief generate_and_get
         * @return the actual matrix
         */
        Eigen::Block<Eigen::MatrixXd> generate_and_get();

        /**
         * @brief cols
         * @return number of current columns
         */
        int cols() const {return _mat.cols();}

        /**
         * @brief rows
         * @return number of current rows
         */
        int rows() const {return _current_row;}

        /**
         * @brief operator [] to index vector
         * @param i index
         * @return value at i
         */
        double& operator[](const int i){return _mat(i);}

        /**
         * @brief operator () to index matrix
         * @param i row
         * @param j column
         * @return value at (i,j)
         */
        double& operator()(const int i, const int j){return _mat(i,j);}
        
    private:
        
        int _cols;
        int _current_row;
        
        Eigen::MatrixXd _mat;
        
    };
    
} }




inline OpenSoT::utils::MatrixPiler::MatrixPiler(const int cols):
    _cols(cols),
    _current_row(0)
{
    _mat.resize(0, _cols);
}

template <typename Derived>
inline void OpenSoT::utils::MatrixPiler::pile(const Eigen::MatrixBase<Derived>& matrix)
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
inline void OpenSoT::utils::MatrixPiler::set(const Eigen::MatrixBase<Derived>& matrix)
{
    if(_cols == matrix.cols())
    {
        reset();
        pile(matrix);
    }
    else
    {
       _cols = matrix.cols();
       _mat = matrix;
       _current_row = _mat.rows();
    }

}

inline void OpenSoT::utils::MatrixPiler::reset()
{
    _current_row = 0;
}

inline void OpenSoT::utils::MatrixPiler::reset(const int cols)
{
    if(_cols == cols)
        reset();
    else
    {
        _cols = cols;
        _current_row = 0;
        _mat.resize(0, _cols);
    }
}

inline Eigen::Block<Eigen::MatrixXd> OpenSoT::utils::MatrixPiler::generate_and_get()
{
//    if(_current_row != _mat.rows()){
//        _mat.conservativeResize(_current_row, _cols);
//        Logger::info() << "Doing a conservativeResize!" << Logger::endl();
//    }
    return _mat.block(0,0,_current_row, _cols);
}

#endif
