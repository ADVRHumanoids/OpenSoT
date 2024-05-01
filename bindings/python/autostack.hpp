#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/utils/AutoStack.h>

namespace py = pybind11;
using namespace OpenSoT;

void pyAutostack(py::module& m) {
    // Expose AutoStack class
    py::class_<AutoStack, AutoStack::Ptr>(m, "AutoStack")
        .def(py::init<const int>())
        .def(py::init<OpenSoT::tasks::Aggregated::TaskPtr>())
        .def(py::init<OpenSoT::tasks::Aggregated::TaskPtr, std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>>())
        .def(py::init<OpenSoT::solvers::iHQP::Stack>())
        .def(py::init<OpenSoT::solvers::iHQP::Stack, std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>>())
        .def("update", &AutoStack::update)
        .def("log", &AutoStack::log)
        .def("checkConsistency", &AutoStack::checkConsistency)
        .def("getStack", &AutoStack::getStack)
        .def("getBoundsList", &AutoStack::getBoundsList)
        .def("setRegularisationTask", &AutoStack::setRegularisationTask)
        .def("getRegularisationTask", &AutoStack::getRegularisationTask)
        .def("setBoundsAggregationPolicy", &AutoStack::setBoundsAggregationPolicy)
        .def("getBounds", &AutoStack::getBounds)
        .def("getTask", &AutoStack::getTask);


    m.def("mul", [](const Eigen::MatrixXd& W, OpenSoT::tasks::Aggregated::TaskPtr task) {
        return W * task;
    });

    m.def("mul", [](const double w, OpenSoT::tasks::Aggregated::TaskPtr task) {
        return w * task;
    });

    m.def("mul", [](const double w, OpenSoT::tasks::Aggregated::Ptr task) {
        return w * task;
    });

    m.def("sub", [](OpenSoT::tasks::Aggregated::TaskPtr task, const std::list<unsigned int>& rowIndices) {
        return task % rowIndices;
    });

    m.def("sub", [](OpenSoT::constraints::Aggregated::ConstraintPtr constraint, const std::list<unsigned int>& rowIndices) {
        return constraint % rowIndices;
    });

    m.def("sum", [](OpenSoT::tasks::Aggregated::TaskPtr task1, OpenSoT::tasks::Aggregated::TaskPtr task2) {
        return task1 + task2;
    });

    m.def("sum", [](OpenSoT::tasks::Aggregated::Ptr aggregated, OpenSoT::tasks::Aggregated::TaskPtr task) {
        return aggregated + task;
    });

    m.def("sum", [](OpenSoT::tasks::Aggregated::TaskPtr task, OpenSoT::tasks::Aggregated::Ptr aggregated) {
        return task + aggregated;
    });

    m.def("sum", [](OpenSoT::tasks::Aggregated::Ptr aggregated1, OpenSoT::tasks::Aggregated::Ptr aggregated2) {
        return aggregated1 + aggregated2;
    });

    m.def("hard", [](OpenSoT::tasks::Aggregated::TaskPtr task1, OpenSoT::tasks::Aggregated::TaskPtr task2) {
        return task1 / task2;
    });

    m.def("hard", [](OpenSoT::AutoStack::Ptr stack, OpenSoT::tasks::Aggregated::TaskPtr task) {
        return stack / task;
    });

    m.def("hard", [](OpenSoT::tasks::Aggregated::TaskPtr task, OpenSoT::AutoStack::Ptr stack) {
        return task / stack;
    });

    m.def("hard", [](OpenSoT::AutoStack::Ptr stack1, OpenSoT::AutoStack::Ptr stack2) {
        return stack1 / stack2;
    });

    m.def("subj", [](OpenSoT::tasks::Aggregated::TaskPtr task, OpenSoT::constraints::Aggregated::ConstraintPtr constraint) {
        return task << constraint;
    });

    m.def("subj", [](OpenSoT::tasks::Aggregated::Ptr task, OpenSoT::constraints::Aggregated::ConstraintPtr constraint) {
        return task << constraint;
    });

    m.def("subj", [](OpenSoT::AutoStack::Ptr stack, OpenSoT::constraints::Aggregated::ConstraintPtr bound) {
        return stack << bound;
    });

    m.def("subj", [](OpenSoT::tasks::Aggregated::TaskPtr task1, OpenSoT::tasks::Aggregated::TaskPtr task2) {
        return task1 << task2;
    });

    m.def("subj", [](OpenSoT::AutoStack::Ptr stack, OpenSoT::tasks::Aggregated::TaskPtr constraint) {
        return stack << constraint;
    });
}
