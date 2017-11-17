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
#include <memory>


namespace OpenSoT { 

/* Forward declarations */
template <typename DerivedM1, typename DerivedM2, typename DerivedQ1, typename DerivedQ2>
class AffineAffineSum;

template <typename DerivedMatrix, typename DerivedM, typename DerivedQ>
class MatrixAffineProduct;

template <typename DerivedVector, typename DerivedM, typename DerivedQ>
class AffineVectorSum;


/* Base class for AffineHelpers */

template <typename DerivedM, typename DerivedQ>
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
class AffineHelperBase {
    
public:
    
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
    AffineAffineSum<DerivedM, OtherM, DerivedQ, OtherQ> operator+(const AffineHelperBase<OtherM, OtherQ>& other) const;
    
    template <typename OtherM, typename OtherQ>
    AffineHelperBase<DerivedM, DerivedQ>& operator+=(const AffineHelperBase<OtherM, OtherQ>& other);
    
    template <typename DerivedVector>
    AffineVectorSum<DerivedVector, DerivedM, DerivedQ> operator+(const DerivedVector& vector) const;
    
    template <typename DerivedVector>
    AffineVectorSum<DerivedVector, DerivedM, DerivedQ> operator-(const DerivedVector& vector) const
    {
        return operator+(-vector);
    }
    
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
    
    void getValue(const Eigen::VectorXd& x, Eigen::VectorXd& value)
    {
        value.noalias() = _M*x + _q;
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
    
    AffineHelper getVar(std::string name) const;
    
    int getSize() const;
    
private:
    
    struct VarInfo {
        int start_idx;
        int size;
    };
    
    std::vector<VarInfo> _vars;
    std::map<std::string, VarInfo> _vars_map;
    int _size;
    
};















/*** IMPL ***/

namespace internal {
    
    template <typename Derived1, typename Derived2>
    using Sum = Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<double>, const Derived1, const Derived2>;
    
    template <typename Derived1, typename Derived2>
    using Product = Eigen::Product<Derived1, Derived2, 0>;
}

template <typename DerivedM1, typename DerivedM2, typename DerivedQ1, typename DerivedQ2>
class AffineAffineSum : public AffineHelperBase< internal::Sum<DerivedM1, DerivedM2>, internal::Sum<DerivedQ1, DerivedQ2> > {
    
public:
    
    AffineAffineSum(const AffineHelperBase<DerivedM1, DerivedQ1>& aff1,
                    const AffineHelperBase<DerivedM2, DerivedQ2>& aff2):
        AffineHelperBase< internal::Sum<DerivedM1, DerivedM2>, internal::Sum<DerivedQ1, DerivedQ2> >(aff1.getM() + aff2.getM(), aff1.getq() + aff2.getq())
    {
        if(aff1.getInputSize() != aff2.getInputSize()){
            throw std::runtime_error("aff1.getInputSize != aff2.getInputSize");
        }
        
        if(aff1.getOutputSize() != aff2.getOutputSize()){
            throw std::runtime_error("aff1.getOutputSize != aff2.getOutputSize");
        }
    }
    
};


template <typename DerivedMatrix, typename DerivedM, typename DerivedQ>
class MatrixAffineProduct : public AffineHelperBase< internal::Product<DerivedMatrix, DerivedM>, internal::Product<DerivedMatrix, DerivedQ> > {
    
public:
    
    MatrixAffineProduct(const DerivedMatrix& matrix, 
                        const AffineHelperBase<DerivedM, DerivedQ>& affine):
        AffineHelperBase< internal::Product<DerivedMatrix, DerivedM>, internal::Product<DerivedMatrix, DerivedQ> >(matrix*affine.getM(), matrix*affine.getq())
    {
        
    }
    
};

template <typename DerivedVector, typename DerivedM, typename DerivedQ>
class AffineVectorSum : public AffineHelperBase< const DerivedM&, internal::Sum<DerivedVector, DerivedQ> > {
    
public:
    
    AffineVectorSum(const DerivedVector& vector, 
                        const AffineHelperBase<DerivedM, DerivedQ>& affine):
        AffineHelperBase< const DerivedM&, internal::Sum<DerivedVector, DerivedQ> >(affine.getM(), vector + affine.getq())
    {
        
    }
    
};

template <typename DerivedM, typename DerivedQ>
template <typename OtherM, typename OtherQ>
inline AffineAffineSum<DerivedM, OtherM, DerivedQ, OtherQ> AffineHelperBase<DerivedM, DerivedQ>::operator+(const AffineHelperBase<OtherM, OtherQ>& other) const
{
    AffineAffineSum<DerivedM, OtherM, DerivedQ, OtherQ> sum(*this, other);
    
    return sum;
}

template <typename DerivedM, typename DerivedQ>
template <typename DerivedVector>
inline AffineVectorSum<DerivedVector, DerivedM, DerivedQ> AffineHelperBase<DerivedM, DerivedQ>::operator+(const DerivedVector& vector) const
{
    AffineVectorSum<DerivedVector, DerivedM, DerivedQ> sum(vector, *this);
    
    return sum;
}


template <typename DerivedMatrix, typename DerivedM, typename DerivedQ>
inline MatrixAffineProduct<DerivedMatrix, DerivedM, DerivedQ> operator*(const DerivedMatrix& matrix, const AffineHelperBase<DerivedM, DerivedQ>& affine)
{
    MatrixAffineProduct<DerivedMatrix, DerivedM, DerivedQ> prod(matrix, affine);
    
    return prod;
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