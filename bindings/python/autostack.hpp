// py_autostack_bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/utils/AutoStack.h>

#include <cxxabi.h>
#include <iostream>
#include <typeinfo>
#include <memory>

// helper to demangle type names
static std::string demangle(const char* name) {
    int status = 0;
    std::unique_ptr<char, void(*)(void*)> res{
        abi::__cxa_demangle(name, NULL, NULL, &status),
        std::free
    };
    return (status == 0 && res) ? std::string(res.get()) : std::string(name);
}

namespace py = pybind11;
using namespace OpenSoT;

void pyAutostack(py::module& m) {

    py::class_<AutoStack, AutoStack::Ptr>(m, "AutoStack")
    .def(py::init<int>())
        .def(py::init<tasks::Aggregated::TaskPtr>(),
             py::keep_alive<1,2>()) // keep the input task alive while AutoStack exists
        .def(py::init<tasks::Aggregated::TaskPtr, std::list<constraints::Aggregated::ConstraintPtr>>(),
             py::keep_alive<1,2>()) // keep the aggregated task (arg 2) alive while AutoStack (1) lives
        .def(py::init<solvers::iHQP::Stack>())
        .def(py::init<solvers::iHQP::Stack, std::list<constraints::Aggregated::ConstraintPtr>>())
        //.def("update", &AutoStack::update)

        .def("update", [](AutoStack::Ptr stack) {
            try {
                std::cerr << "=== AutoStack::update() diagnostic ===\n";
                // Try to get the underlying iHQP::Stack
                auto &cstack = stack->getStack(); // iHQP::Stack&

                // The internal iHQP::Stack type is usually a vector/list of TaskPtr
                std::cerr << "iHQP::Stack size: " << cstack.size() << "\n";

                for (size_t i = 0; i < cstack.size(); ++i) {
                    auto taskPtr = cstack[i]; // iHQP::TaskPtr
                    // Print address
                    std::cerr << "[" << i << "] iHQP::TaskPtr addr: " << taskPtr.get();

                    // Print dynamic type name
                    try {
                        const std::type_info &ti = typeid(*taskPtr);
                        std::string name = demangle(ti.name());
                        std::cerr << ", dynamic type: " << name;
                    } catch (...) {
                        std::cerr << ", dynamic type: <typeid failed>";
                    }

                    // print newline
                    std::cerr << "\n";
                }

                std::cerr << "=== end diagnostic list ===\n";

                // Now call the real update (this may throw)
                stack->update();

                std::cerr << "AutoStack::update() completed without throwing.\n";
            } catch (const std::exception &e) {
                std::cerr << "AutoStack::update() threw std::exception: " << e.what() << "\n";
                throw; // rethrow to Python (so you still see the Python traceback)
            } catch (...) {
                std::cerr << "AutoStack::update() threw unknown exception\n";
                throw;
            }
        })

        .def("log", &AutoStack::log)
        .def("checkConsistency", &AutoStack::checkConsistency)
        .def("getStack", &AutoStack::getStack, py::return_value_policy::reference_internal)
        .def("getBoundsList", &AutoStack::getBoundsList, py::return_value_policy::reference_internal)
        .def("setRegularisationTask", &AutoStack::setRegularisationTask, py::keep_alive<1,2>()) // keep reg task alive while stack lives
        .def("getRegularisationTask", &AutoStack::getRegularisationTask)
        .def("setBoundsAggregationPolicy", &AutoStack::setBoundsAggregationPolicy,
             py::arg("aggregationPolicy") = constraints::Aggregated::EQUALITIES_TO_INEQUALITIES |
                                            constraints::Aggregated::UNILATERAL_TO_BILATERAL)
        .def("getBounds", &AutoStack::getBounds)
        .def("getTask", &AutoStack::getTask)

        // AutoStack operators: ensure returned AutoStack / Aggregated keeps args alive
        .def("__lshift__",
             [](AutoStack::Ptr stack, constraints::Aggregated::ConstraintPtr bound) { return stack << bound; },
             py::keep_alive<0,1>(), // keep returned (0) alive while 'stack' (1) is alive (and vice-versa)
             py::keep_alive<0,2>()) // keep bound (2) alive while returned (0) lives
        .def("__lshift__",
             [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr task) { return stack << task; },
             py::keep_alive<0,1>(),
             py::keep_alive<0,2>())

        .def("__truediv__",
             [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr task) { return stack / task; },
             py::keep_alive<0,1>(),
             py::keep_alive<0,2>())
        .def("__truediv__",
             [](tasks::Aggregated::TaskPtr task1, tasks::Aggregated::TaskPtr task2) { return task1 / task2; },
             py::keep_alive<0,1>(),
             py::keep_alive<0,2>())
        .def("__truediv__",
             [](tasks::Aggregated::TaskPtr task, AutoStack::Ptr stack) { return task / stack; },
             py::keep_alive<0,1>(),
             py::keep_alive<0,2>())
        .def("__truediv__",
             [](AutoStack::Ptr stack1, AutoStack::Ptr stack2) { return stack1 / stack2; },
             py::keep_alive<0,1>(),
             py::keep_alive<0,2>());

    // Free functions
    // mul: keep the returned Aggregated (0) alive while the task arg (2) is alive
    m.def("mul",
          [](const Eigen::MatrixXd& W, tasks::Aggregated::TaskPtr task) { return W * task; },
          py::keep_alive<0,2>());

    m.def("mul",
          [](double w, tasks::Aggregated::TaskPtr task) { return w * task; },
          py::keep_alive<0,2>());

    m.def("mul",
          [](double w, tasks::Aggregated::Ptr task) { return w * task; },
          py::keep_alive<0,2>());

    // sub: returns a SubTask/SubConstraint that holds pointer to original -> keep original alive
    m.def("sub",
          [](tasks::Aggregated::TaskPtr task, const std::list<unsigned int>& rows) { return task % rows; },
          py::keep_alive<0,1>());

    m.def("sub",
          [](constraints::Aggregated::ConstraintPtr constraint, const std::list<unsigned int>& rows) { return constraint % rows; },
          py::keep_alive<0,1>());

    // sum overloads: result aggregates tasks; keep inputs alive
    m.def("sum",
          [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) { return t1 + t2; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("sum",
          [](tasks::Aggregated::Ptr agg, tasks::Aggregated::TaskPtr t) { return agg + t; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("sum",
          [](tasks::Aggregated::TaskPtr t, tasks::Aggregated::Ptr agg) { return t + agg; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("sum",
          [](tasks::Aggregated::Ptr agg1, tasks::Aggregated::Ptr agg2) { return agg1 + agg2; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    // hard (/) overloads: keep args alive while returned object lives
    m.def("hard",
          [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) { return t1 / t2; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("hard",
          [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr t) { return stack / t; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("hard",
          [](tasks::Aggregated::TaskPtr t, AutoStack::Ptr stack) { return t / stack; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("hard",
          [](AutoStack::Ptr stack1, AutoStack::Ptr stack2) { return stack1 / stack2; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    // subj (<<) overloads: keep both args alive while returned object lives
    m.def("subj",
          [](tasks::Aggregated::TaskPtr t, constraints::Aggregated::ConstraintPtr c) { return t << c; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("subj",
          [](tasks::Aggregated::Ptr t, constraints::Aggregated::ConstraintPtr c) { return t << c; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("subj",
          [](AutoStack::Ptr stack, constraints::Aggregated::ConstraintPtr c) { return stack << c; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("subj",
          [](tasks::Aggregated::TaskPtr t1, tasks::Aggregated::TaskPtr t2) { return t1 << t2; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());

    m.def("subj",
          [](AutoStack::Ptr stack, tasks::Aggregated::TaskPtr t) { return stack << t; },
          py::keep_alive<0,1>(), py::keep_alive<0,2>());
}
