#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/Constraint.h>

using namespace OpenSoT;
namespace py = pybind11;

template <class MatrixType, class VectorType>
class pyTaskTrampoline : public Task<MatrixType, VectorType> {
public:
    using Task<MatrixType, VectorType>::Task;
    typedef Task<MatrixType, VectorType> TMV;

    void _update() override {
        PYBIND11_OVERLOAD_PURE(void, TMV, _update);
    }
};

template<typename MatrixType, typename VectorType>
void pyTask(py::module& m, const std::string& className) {
    py::class_<Task<MatrixType, VectorType>, std::shared_ptr<Task<MatrixType, VectorType>>, pyTaskTrampoline<MatrixType, VectorType>>(m, className.c_str())
        .def(py::init<const std::string, const unsigned int>())
        .def("getWeightIsDiagonalFlag", &Task<MatrixType, VectorType>::getWeightIsDiagonalFlag)
        .def("setWeightIsDiagonalFlag", &Task<MatrixType, VectorType>::setWeightIsDiagonalFlag)
        .def("setActive", &Task<MatrixType, VectorType>::setActive)
        .def("isActive", &Task<MatrixType, VectorType>::isActive)
        .def("getA", &Task<MatrixType, VectorType>::getA)
        .def("getHessianAtype", &Task<MatrixType, VectorType>::getHessianAtype)
        .def("getb", &Task<MatrixType, VectorType>::getb)
        .def("getWA", &Task<MatrixType, VectorType>::getWA)
        .def("getATranspose", &Task<MatrixType, VectorType>::getATranspose)
        .def("getWb", &Task<MatrixType, VectorType>::getWb)
        .def("getc", &Task<MatrixType, VectorType>::getc)
        .def("getWeight", &Task<MatrixType, VectorType>::getWeight)
        .def("setWeight", py::overload_cast<const MatrixType&>(&Task<MatrixType, VectorType>::setWeight))
        .def("setWeight", py::overload_cast<const double&>(&Task<MatrixType, VectorType>::setWeight))
        .def("getLambda", &Task<MatrixType, VectorType>::getLambda)
        .def("setLambda", &Task<MatrixType, VectorType>::setLambda)
        .def("getConstraints", &Task<MatrixType, VectorType>::getConstraints, py::return_value_policy::reference_internal)
        .def("getXSize", &Task<MatrixType, VectorType>::getXSize)
        .def("getTaskSize", &Task<MatrixType, VectorType>::getTaskSize)
        .def("update", &Task<MatrixType, VectorType>::update)
        .def("getTaskID", &Task<MatrixType, VectorType>::getTaskID)
        .def("getActiveJointsMask", &Task<MatrixType, VectorType>::getActiveJointsMask)
        .def("setActiveJointsMask", &Task<MatrixType, VectorType>::setActiveJointsMask)
        .def("log", &Task<MatrixType, VectorType>::log)
        .def("computeCost", &Task<MatrixType, VectorType>::computeCost)
        .def("checkConsistency", &Task<MatrixType, VectorType>::checkConsistency);
}

template<typename MatrixType, typename VectorType>
void pyConstraint(py::module& m, const std::string& className) {
    py::class_<Constraint<MatrixType, VectorType>, std::shared_ptr<Constraint<MatrixType, VectorType>>> (m, className.c_str())
            .def(py::init<const std::string&, const unsigned int>())
            .def("getXSize", &Constraint<MatrixType, VectorType>::getXSize)
            .def("getLowerBound", &Constraint<MatrixType, VectorType>::getLowerBound, py::return_value_policy::reference)
            .def("getUpperBound", &Constraint<MatrixType, VectorType>::getUpperBound, py::return_value_policy::reference)
            .def("getAeq", &Constraint<MatrixType, VectorType>::getAeq, py::return_value_policy::reference)
            .def("getbeq", &Constraint<MatrixType, VectorType>::getbeq, py::return_value_policy::reference)
            .def("getAineq", &Constraint<MatrixType, VectorType>::getAineq, py::return_value_policy::reference)
            .def("getbLowerBound", &Constraint<MatrixType, VectorType>::getbLowerBound, py::return_value_policy::reference)
            .def("getbUpperBound", &Constraint<MatrixType, VectorType>::getbUpperBound, py::return_value_policy::reference)
            .def("isEqualityConstraint", &Constraint<MatrixType, VectorType>::isEqualityConstraint)
            .def("isInequalityConstraint", &Constraint<MatrixType, VectorType>::isInequalityConstraint)
            .def("isUnilateralConstraint", &Constraint<MatrixType, VectorType>::isUnilateralConstraint)
            .def("isBilateralConstraint", &Constraint<MatrixType, VectorType>::isBilateralConstraint)
            .def("hasBounds", &Constraint<MatrixType, VectorType>::hasBounds)
            .def("isBound", &Constraint<MatrixType, VectorType>::isBound)
            .def("isConstraint", &Constraint<MatrixType, VectorType>::isConstraint)
            .def("getConstraintID", &Constraint<MatrixType, VectorType>::getConstraintID)
            .def("update", &Constraint<MatrixType, VectorType>::update)
            .def("log", &Constraint<MatrixType, VectorType>::log)
            .def("checkConsistency", &Constraint<MatrixType, VectorType>::checkConsistency);
}




