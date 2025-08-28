// NOTE: Do not call .update() from Python subclass __init__.
//       Use a @classmethod factory if you need immediate initialization.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <memory>

#include <OpenSoT/Task.h>
#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/AutoStack.h>

using namespace OpenSoT;
namespace py = pybind11;

// -----------------------------
// Trampolines
// -----------------------------

template <class MatrixType, class VectorType>
class pyTaskTrampoline : public Task<MatrixType, VectorType> {
public:
    using Base = Task<MatrixType, VectorType>;
    using Base::Base; // inherit constructors

protected:
    void _update() override {
        PYBIND11_OVERRIDE_PURE(void, Base, _update);
    }

public:
    // Lift protected members to public so Python can access them (as in your original)
    using Base::_A;
    using Base::_b;
    using Base::_c;
    using Base::_W;
};

template <class MatrixType, class VectorType>
class pyConstraintTrampoline : public Constraint<MatrixType, VectorType> {
public:
    using Base = Constraint<MatrixType, VectorType>;
    using Base::Base; // inherit constructors

protected:
    void _update() override {
        PYBIND11_OVERRIDE_PURE(void, Base, _update);
    }

public:

    using Base::_Aineq;
    using Base::_bLowerBound;
    using Base::_bUpperBound;

    using Base::_lowerBound;
    using Base::_upperBound;
};

// -----------------------------
// Bind helpers
// -----------------------------

template<typename MatrixType, typename VectorType>
void pyTask(py::module& m, const std::string& className) {
    using Base = Task<MatrixType, VectorType>;
    using Trmp = pyTaskTrampoline<MatrixType, VectorType>;

    py::class_<Base, Trmp, std::shared_ptr<Base>>(m, className.c_str())
        .def(py::init<const std::string&, const unsigned int>())

        .def("getWeightIsDiagonalFlag", &Base::getWeightIsDiagonalFlag)
        .def("setWeightIsDiagonalFlag", &Base::setWeightIsDiagonalFlag)
        .def("setActive", &Base::setActive)
        .def("isActive", &Base::isActive)
        .def("getA", &Base::getA)
        .def("getHessianAtype", &Base::getHessianAtype)
        .def("getb", &Base::getb)
        .def("getWA", &Base::getWA)
        .def("getATranspose", &Base::getATranspose)
        .def("getWb", &Base::getWb)
        .def("getc", &Base::getc)
        .def("getWeight", &Base::getWeight)
        .def("setWeight", py::overload_cast<const MatrixType&>(&Base::setWeight))
        .def("setWeight", py::overload_cast<const double&>(&Base::setWeight))
        .def("getLambda", &Base::getLambda)
        .def("setLambda", &Base::setLambda)
        .def("getConstraints", &Base::getConstraints, py::return_value_policy::reference_internal)
        .def("getXSize", &Base::getXSize)
        .def("getTaskSize", &Base::getTaskSize)

        // Public non-virtual entry point that calls _update()
        .def("update", &Base::update)

        .def("getTaskID", &Base::getTaskID)
        .def("getActiveJointsMask", &Base::getActiveJointsMask)
        .def("setActiveJointsMask", &Base::setActiveJointsMask)
        .def("log", &Base::log)
        .def("computeCost", &Base::computeCost)
        .def("checkConsistency", &Base::checkConsistency)

        .def("__add__", [](const std::shared_ptr<Base>& task1, const std::shared_ptr<Base>& task2) {
            return task1 + task2;
        })
        .def("__mod__", [](const std::shared_ptr<Base>& task, const std::list<unsigned int>& rowIndices) {
            return task % rowIndices;
        })

        .def("__getitem__", [](const std::shared_ptr<Base>& task, const size_t i) {
            std::list<unsigned int> indices;
            indices.push_back(static_cast<unsigned int>(i));
            return task % indices;
        })

        .def("__getitem__", [](const std::shared_ptr<Base>& task, py::slice slice) {
            size_t start, stop, step, slicelength;
            if (!slice.compute(static_cast<size_t>(task->getA().rows()), &start, &stop, &step, &slicelength))
                throw py::error_already_set();

            std::list<unsigned int> slice_vector;
            for (size_t i = 0; i < slicelength; ++i) {
                unsigned int id = static_cast<unsigned int>(start + i * step);
                slice_vector.push_back(id);
            }
            return task % slice_vector;
        })

        // Expose lifted members from trampoline
        .def_readwrite("_A", &Trmp::_A)
        .def_readwrite("_b", &Trmp::_b)
        .def_readwrite("_c", &Trmp::_c)
        .def_readwrite("_W", &Trmp::_W)

        .def("__rmul__", [](const std::shared_ptr<Base>& task, const float& w) {
            return w * task;
        })
        .def("__truediv__", [](const std::shared_ptr<Base>& task1, const std::shared_ptr<Base>& task2) {
            return task1 / task2;
        })
        .def("__truediv__", [](const std::shared_ptr<Base>& task, const OpenSoT::AutoStack::Ptr stack) {
            return task / stack;
        })
        .def("__lshift__", [](const std::shared_ptr<Base>& task, const std::shared_ptr<Constraint<MatrixType, VectorType>>& constraint) {
            return task << constraint;
        })
        .def("__lshift__", [](const std::shared_ptr<Base>& task1, const std::shared_ptr<Base>& task2) {
            return task1 << task2;
        });
}

template<typename MatrixType, typename VectorType>
void pyConstraint(py::module& m, const std::string& className) {
    using Base = Constraint<MatrixType, VectorType>;
    using Trmp = pyConstraintTrampoline<MatrixType, VectorType>;

    py::class_<Base, Trmp, std::shared_ptr<Base>>(m, className.c_str())
        .def(py::init<const std::string&, const unsigned int>())

        .def("getXSize", &Base::getXSize)
        .def("getLowerBound", &Base::getLowerBound, py::return_value_policy::reference)
        .def("getUpperBound", &Base::getUpperBound, py::return_value_policy::reference)
        .def("getAineq", &Base::getAineq, py::return_value_policy::reference)
        .def("getbLowerBound", &Base::getbLowerBound, py::return_value_policy::reference)
        .def("getbUpperBound", &Base::getbUpperBound, py::return_value_policy::reference)
        .def("isInequalityConstraint", &Base::isInequalityConstraint)
        .def("isUnilateralConstraint", &Base::isUnilateralConstraint)
        .def("isBilateralConstraint", &Base::isBilateralConstraint)
        .def("hasBounds", &Base::hasBounds)
        .def("isBound", &Base::isBound)
        .def("isConstraint", &Base::isConstraint)
        .def("getConstraintID", &Base::getConstraintID)

        // Public non-virtual entry point that calls _update()
        .def("update", &Base::update)

        .def("log", &Base::log)
        .def("checkConsistency", &Base::checkConsistency)
        .def("getActiveJointsMask", &Base::getActiveJointsMask)
        .def("setActiveJointsMask", &Base::setActiveJointsMask)

        // Expose lifted members from trampoline
        .def_readwrite("_Aineq", &Trmp::_Aineq)
        .def_readwrite("_bLowerBound", &Trmp::_bLowerBound)
        .def_readwrite("_bUpperBound", &Trmp::_bUpperBound)
        .def_readwrite("_lowerBound", &Trmp::_lowerBound)
        .def_readwrite("_upperBound", &Trmp::_upperBound)

        .def("__mod__", [](const std::shared_ptr<Base>& constraint, const std::list<unsigned int>& rowIndices) {
            return constraint % rowIndices;
        });
}
