// py_autostack.cpp

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <iostream>
#include <typeinfo>
#include <memory>
#include <cxxabi.h>

namespace py = pybind11;
using namespace OpenSoT;

// helper to demangle type names
// static std::string demangle(const char* name) {
//     int status = 0;
//     std::unique_ptr<char, void(*)(void*)> res{
//         abi::__cxa_demangle(name, nullptr, nullptr, &status),
//         std::free
//     };
//     return (status == 0 && res) ? std::string(res.get()) : std::string(name);
// }


void pyAutostack(py::module& m) {

    py::class_<AutoStack, AutoStack::Ptr>(m, "AutoStack")
    .def(py::init<int>(), py::arg("x_size"))
        .def(py::init<tasks::Aggregated::TaskPtr>(), py::arg("task"))
        .def(py::init<tasks::Aggregated::TaskPtr, std::list<constraints::Aggregated::ConstraintPtr>>(),
             py::arg("task"), py::arg("bounds"))
        .def(py::init<solvers::iHQP::Stack>(), py::arg("stack"))
        .def(py::init<solvers::iHQP::Stack, std::list<constraints::Aggregated::ConstraintPtr>>(),
             py::arg("stack"), py::arg("bounds"))

        .def("update", &AutoStack::update)
        // .def("update", [](AutoStack::Ptr stack) {
        //     try {
        //         std::cerr << "=== AutoStack::update() diagnostic ===\n";
        //         auto &cstack = stack->getStack();
        //         std::cerr << "iHQP::Stack size: " << cstack.size() << "\n";
        //         for (size_t i = 0; i < cstack.size(); ++i) {
        //             auto taskPtr = cstack[i];
        //             std::cerr << "[" << i << "] iHQP::TaskPtr addr: " << taskPtr.get();
        //             try {
        //                 const std::type_info &ti = typeid(*taskPtr);
        //                 std::cerr << ", dynamic type: " << demangle(ti.name()) << "\n";
        //             } catch (...) {
        //                 std::cerr << ", dynamic type: <typeid failed>\n";
        //             }
        //         }
        //         std::cerr << "=== end diagnostic list ===\n";

        //         stack->update();
        //         std::cerr << "AutoStack::update() completed without throwing.\n";
        //     } catch (const std::exception &e) {
        //         std::cerr << "AutoStack::update() threw std::exception: " << e.what() << "\n";
        //         throw;
        //     } catch (...) {
        //         std::cerr << "AutoStack::update() threw unknown exception\n";
        //         throw;
        //     }
        // })

        .def("log", &AutoStack::log)
        .def("checkConsistency", &AutoStack::checkConsistency)
        .def("getStack", &AutoStack::getStack, py::return_value_policy::reference_internal)
        .def("getBoundsList", &AutoStack::getBoundsList, py::return_value_policy::reference_internal)
        .def("setRegularisationTask", &AutoStack::setRegularisationTask)
        .def("getRegularisationTask", &AutoStack::getRegularisationTask, py::return_value_policy::reference_internal)
        .def("setBoundsAggregationPolicy", &AutoStack::setBoundsAggregationPolicy,
             py::arg("aggregationPolicy") = constraints::Aggregated::EQUALITIES_TO_INEQUALITIES |
                                            constraints::Aggregated::UNILATERAL_TO_BILATERAL)
        .def("getBounds", &AutoStack::getBounds, py::return_value_policy::reference_internal)
        .def("getTask", &AutoStack::getTask, py::return_value_policy::reference_internal)

        // operator overloads with shared_ptr
        .def("__lshift__", [](AutoStack::Ptr stack, constraints::Aggregated::ConstraintPtr bound) -> AutoStack::Ptr { return stack << bound; })
        .def("__lshift__", [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr task) -> AutoStack::Ptr { return stack << task; })

        .def("__truediv__", [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr task) -> AutoStack::Ptr { return stack / task; })
        .def("__truediv__", [](tasks::Aggregated::TaskPtr task1, tasks::Aggregated::TaskPtr task2) -> OpenSoT::AutoStack::Ptr { return task1 / task2; })
        .def("__truediv__", [](tasks::Aggregated::TaskPtr task, AutoStack::Ptr stack) -> AutoStack::Ptr { return task / stack; })
        .def("__truediv__", [](AutoStack::Ptr stack1, AutoStack::Ptr stack2) -> AutoStack::Ptr { return stack1 / stack2; });

    // Free functions (operators)
    m.def("mul", [](const Eigen::MatrixXd& W, tasks::Aggregated::TaskPtr task) -> tasks::Aggregated::TaskPtr { return W * task; });
    m.def("mul", [](double w, tasks::Aggregated::TaskPtr task) -> tasks::Aggregated::TaskPtr { return w * task; });
    m.def("mul", [](double w, tasks::Aggregated::Ptr task) -> tasks::Aggregated::Ptr { return w * task; });

    m.def("sub", [](tasks::Aggregated::TaskPtr task, const std::list<unsigned int>& rows) -> tasks::Aggregated::TaskPtr { return task % rows; });
    m.def("sub", [](constraints::Aggregated::ConstraintPtr constraint, const std::list<unsigned int>& rows) -> constraints::Aggregated::ConstraintPtr { return constraint % rows; });

    m.def("sum", [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) -> tasks::Aggregated::TaskPtr { return t1 + t2; });
    m.def("sum", [](tasks::Aggregated::Ptr agg, tasks::Aggregated::TaskPtr t) -> tasks::Aggregated::Ptr { return agg + t; });
    m.def("sum", [](tasks::Aggregated::TaskPtr t, tasks::Aggregated::Ptr agg) -> tasks::Aggregated::Ptr { return t + agg; });
    m.def("sum", [](tasks::Aggregated::Ptr agg1, tasks::Aggregated::Ptr agg2) -> tasks::Aggregated::Ptr { return agg1 + agg2; });

    m.def("hard", [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) -> OpenSoT::AutoStack::Ptr { return t1 / t2; });
    m.def("hard", [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr t) -> AutoStack::Ptr { return stack / t; });
    m.def("hard", [](tasks::Aggregated::TaskPtr t, AutoStack::Ptr stack) -> AutoStack::Ptr { return t / stack; });
    m.def("hard", [](AutoStack::Ptr stack1, AutoStack::Ptr stack2) -> AutoStack::Ptr { return stack1 / stack2; });

    m.def("subj", [](tasks::Aggregated::TaskPtr t, constraints::Aggregated::ConstraintPtr c) -> tasks::Aggregated::TaskPtr { return t << c; });
    m.def("subj", [](tasks::Aggregated::Ptr t, constraints::Aggregated::ConstraintPtr c) -> tasks::Aggregated::Ptr { return t << c; });
    m.def("subj", [](AutoStack::Ptr stack, constraints::Aggregated::ConstraintPtr c) -> AutoStack::Ptr { return stack << c; });
    m.def("subj", [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) -> tasks::Aggregated::TaskPtr { return t1 << t2; });
    m.def("subj", [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr t) -> AutoStack::Ptr { return stack << t; });
}
