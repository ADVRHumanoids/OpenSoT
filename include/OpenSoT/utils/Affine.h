/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * 
*/

#ifndef __OPENSOT_AFFINEHELPER_H__
#define __OPENSOT_AFFINEHELPER_H__

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <type_traits>
#include <memory>


namespace OpenSoT { 

// /* Forward declarations */
// template <typename DerivedM1, typename DerivedM2, typename DerivedQ1, typename DerivedQ2>
// class AffineAffineSum;
// 
// template <typename DerivedMatrix, typename DerivedM, typename DerivedQ>
// class MatrixAffineProduct;
// 
// template <typename DerivedVector, typename DerivedM, typename DerivedQ>
// class AffineVectorSum;


/* Base class for AffineHelpers */


/**
 * @brief This class models an affine mapping between an "input variable" x,
 * and an "output variable" y. The underlying math is:
 * 
 * y = M*x + q
 * 
 * This is useful to make a task/constraint aware
 * about the formatting of the optimization vector w.r.t. to its own variable
 * of interest
 * 
 * Example: in the context of an optimization problem where x = joint acceleration,
 * a torque constraint must know that y = B*x + h. The task can be used also in problems
 * where x = tau, by passing the affine mapping y = x.
 * 
 */

template <typename DerivedM, typename DerivedQ>
class AffineHelperBase {
    
public:
    
    typedef typename std::remove_reference<DerivedM>::type * PtrM;
    typedef typename std::remove_reference<DerivedQ>::type * PtrQ;
    typedef typename std::remove_reference<DerivedM>::type const * ConstPtrM;
    typedef typename std::remove_reference<DerivedQ>::type const * ConstPtrQ;
    
   
    AffineHelperBase() = default;
    AffineHelperBase(int input_size, int output_size)
    {
        setZero(input_size, output_size);
    }
    AffineHelperBase(const DerivedM& M, const DerivedQ& q): _M(M), _q(q) {
        check_consistency();
    }
    
    template <typename OtherM, typename OtherQ>
    AffineHelperBase(const AffineHelperBase<OtherM, OtherQ>& other)
    {
        *this = other;
    }
    
    template <typename OtherM, typename OtherQ>
    AffineHelperBase<DerivedM, DerivedQ>& operator=(const AffineHelperBase<OtherM, OtherQ>& other)
    {
        _M.noalias() = other.getM();
        _q.noalias() = other.getq();
        check_consistency();
        return *this;
    }
    
    template <typename OtherM, typename OtherQ>
    AffineHelperBase<DerivedM, DerivedQ>& operator+=(const AffineHelperBase<OtherM, OtherQ>& other);
    
    const DerivedM& getM() const { return _M; }
    const DerivedQ& getq() const { return _q; }
    
    int getInputSize() const { return _M.cols(); }
    int getOutputSize() const { return _M.rows(); }
    
    void setZero(int input_size, int output_size)
    {
        _M.setZero(output_size, input_size);
        _q.setZero(output_size);
        check_consistency();
    }
    
    void setZero()
    {
        _M.setZero(_M.rows(), _M.cols());
        _q.setZero(_q.rows());
    }
    
    static AffineHelperBase<DerivedM, DerivedQ> Identity(int size)
    {
        DerivedM m = Eigen::MatrixBase<DerivedM>::Identity(size, size);
        DerivedQ q = Eigen::MatrixBase<DerivedQ>::Zero(size, 1);
        
        return AffineHelperBase(m, q);
    }
    
    static AffineHelperBase<DerivedM, DerivedQ> Zero(int input_size, int output_size)
    {
        DerivedM m = Eigen::MatrixBase<DerivedM>::Zero(output_size, input_size);
        DerivedQ q = Eigen::MatrixBase<DerivedQ>::Zero(output_size, 1);
        
        return AffineHelperBase(m, q);
    }
    
    
    auto segment(int start_idx, int size) const -> AffineHelperBase<decltype(std::declval<ConstPtrM>()->block(0,0,0,0)), decltype(std::declval<ConstPtrQ>()->segment(0,0))>
    {
        return AffineHelperBase<decltype(std::declval<ConstPtrM>()->block(0,0,0,0)), decltype(std::declval<ConstPtrQ>()->segment(0,0))>
                            (_M.block(start_idx, 0, size, _M.cols()), 
                             _q.segment(start_idx, size)
                             );
    }
    
    auto head(int size) -> decltype( this->segment(0,0) )
    {
        return segment(0, size);
    }
    
    auto tail(int size) -> decltype( this->segment(0,0) )
    {
        return segment(_M.rows() - size, size);
    }
 

    
    template <typename Derived>
    void getValue(const Eigen::VectorXd& x, Eigen::MatrixBase<Derived>& value)
    {
        value.noalias() = _M*x;
        value += _q;
    }
    
    virtual void update () {}
    
protected:
    
    AffineHelperBase<DerivedM, DerivedQ>& self() { return *this; }
    const AffineHelperBase<DerivedM, DerivedQ>& self() const { return *this; }
    
    void check_consistency()
    {
        if(_M.rows() != _q.rows()){
            throw std::runtime_error("_M.rows() != _q.rows()");
        }
    }
    
    DerivedM _M;
    DerivedQ _q;
    
};

typedef AffineHelperBase<Eigen::MatrixXd, Eigen::VectorXd> AffineHelper;


/**
 * @brief This class manages the serialization of the optimization vector x
 * from an ordered set of variables yi. Then, it allows to get affine mappings
 * corresponding to all defined variables yi for using inside tasks/constraints.
 * 
 */
class OptvarHelper {
    
public:
    
    typedef std::vector<std::pair<std::string, int>> VariableVector;
    
    OptvarHelper(VariableVector name_size_pairs);
    
    AffineHelper getVariable(std::string name) const;
    
    std::vector<AffineHelper> getAllVariables() const;
    
    int getSize() const;
    
private:
    
    struct VarInfo {
        int start_idx;
        int size;
        std::string name;
    };
    
    std::vector<VarInfo> _vars;
    std::map<std::string, VarInfo> _vars_map;
    int _size;
    
};















/*** IMPL ***/


template <typename DerivedM1, typename DerivedM2, 
          typename DerivedQ1, typename DerivedQ2>
inline auto operator+(const AffineHelperBase<DerivedM1, DerivedQ1>& lhs, 
                      const AffineHelperBase<DerivedM2, DerivedQ2>& rhs) ->
                      AffineHelperBase<decltype(lhs.getM()+rhs.getM()), decltype(lhs.getq()+rhs.getq())>
{
    return AffineHelperBase<decltype(lhs.getM()+rhs.getM()), decltype(lhs.getq()+rhs.getq())>(lhs.getM()+rhs.getM(),
                                                                                              lhs.getq()+rhs.getq());
}



template <typename DerivedM1, typename DerivedQ1,
          typename DerivedVector>
inline auto operator+(const AffineHelperBase<DerivedM1, DerivedQ1>& lhs,
                      const Eigen::MatrixBase<DerivedVector>& vector) ->
                      AffineHelperBase<decltype(lhs.getM()), decltype(lhs.getq()+vector)>
{
    return AffineHelperBase<decltype(lhs.getM()), decltype(lhs.getq()+vector)>(lhs.getM(), lhs.getq() + vector);
}


template <typename DerivedM1, typename DerivedQ1,
          typename DerivedVector>
inline auto operator-(const AffineHelperBase<DerivedM1, DerivedQ1>& lhs,
                      const Eigen::MatrixBase<DerivedVector>& vector) ->
                      AffineHelperBase<decltype(lhs.getM()), decltype(lhs.getq()-vector)>
{
    return AffineHelperBase<decltype(lhs.getM()), decltype(lhs.getq()-vector)>(lhs.getM(), lhs.getq() - vector);
}



template <typename DerivedMatrix, typename DerivedM, typename DerivedQ>
inline auto operator*(const DerivedMatrix& matrix, 
                      const AffineHelperBase<DerivedM, DerivedQ>& affine) -> 
                      AffineHelperBase<decltype(matrix*affine.getM()), decltype(matrix*affine.getq())>
{   
    
    
    return AffineHelperBase<decltype(matrix*affine.getM()), decltype(matrix*affine.getq())> (matrix*affine.getM(), matrix*affine.getq());
}


template <typename DerivedM1, typename DerivedM2, 
          typename DerivedQ1, typename DerivedQ2>
inline AffineHelper operator/(const AffineHelperBase<DerivedM1, DerivedQ1>& lhs, 
                              const AffineHelperBase<DerivedM2, DerivedQ2>& rhs)
{
    if(lhs.getInputSize() != rhs.getInputSize()){
        std::stringstream ss;
        ss << "lhs.getInputSize() != rhs.getInputSize(): " << lhs.getInputSize() << " != " << rhs.getInputSize();
        throw std::invalid_argument(ss.str());
    }
    
    int input_size = lhs.getInputSize();
    int output_size = lhs.getOutputSize() + rhs.getOutputSize();
    
    Eigen::MatrixXd M3(output_size, input_size);
    Eigen::VectorXd q3(output_size);
    
    M3 << lhs.getM(), rhs.getM();
    q3 << lhs.getq(), rhs.getq();
    
    return AffineHelper(M3, q3);
    
}




inline std::ostream& operator<<(std::ostream& os, const AffineHelper& affine)
{
    os << "M:\n" << affine.getM() << "\n\nq:\n" << affine.getq();
    return os;
}









}

// // tau = S*(B*qddot + h - J*F)
// 
// int main(){
//     
//     XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(XBot::Utils::getXBotConfig());
//     
//     std::vector<std::string> contact_links = { "r_sole", "l_sole" };
//     
//     OpenSoT::OptvarHelper optvarhelper;
//     optvarhelper.add("qddot", 50);
//     optvarhelper.add("f1", 6);
//     optvarhelper.add("f2", 6);
//     optvarhelper.add("slack_1", 1);
//     
//     OpenSoT::AffineHelper qddot = optvarhelper.getVar("qddot");
//     std::vector<OpenSoT::AffineHelper> forces = { optvarhelper.getVar("f1"), optvarhelper.getVar("f2") };
//     
//     OpenSoT::Torque tau(model, contact_links, qddot, forces, optvarhelper);
//     
//     
//     
//     // A*tau - b
//     
//     Eigen::MatrixXd A(3, tau.getOutputSize());
//     Eigen::VectorXd b(3);
//     
//     OpenSoT::AffineHelper task;
//     task = A*tau - b;
//     
//     Eigen::MatrixXd _A = task.getM();
//     Eigen::VectorXd _b = b - task.getq();
//     
// 
// }



#endif
