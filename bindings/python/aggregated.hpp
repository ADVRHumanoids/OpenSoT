// py_aggregated_bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>

namespace py = pybind11;


void pyAggregatedTask(py::module& m) {
    // Aggregated Task
    py::class_<OpenSoT::tasks::Aggregated,
               std::shared_ptr<OpenSoT::tasks::Aggregated>,
               OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AggregatedTask")
        // ctor(list<TaskPtr>, x_size) -- keep the list (arg 2) alive while returned Aggregated (1) lives
        .def(py::init<const std::list<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>>&, const unsigned int>(),
             py::keep_alive<1, 2>())

        // ctor(TaskPtr, x_size) -- keep the TaskPtr (arg 2) alive while returned Aggregated (1) lives
        .def(py::init<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>, const unsigned int>(),
             py::keep_alive<1, 2>())

        // ctor(TaskPtr, TaskPtr, x_size) -- keep both TaskPtr args alive while returned Aggregated (1) lives
        .def(py::init<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>,
                      std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>, const unsigned int>(),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>())

        // expose update() (public entry that calls the virtual _update())
        //.def("update", &OpenSoT::tasks::Aggregated::update)

        .def("getOwnConstraints", &OpenSoT::tasks::Aggregated::getOwnConstraints, py::return_value_policy::reference_internal)
        .def("getAggregatedConstraints", &OpenSoT::tasks::Aggregated::getAggregatedConstraints, py::return_value_policy::reference_internal)
        .def("getTaskList", &OpenSoT::tasks::Aggregated::getTaskList, py::return_value_policy::reference_internal)
        .def("setLambda", &OpenSoT::tasks::Aggregated::setLambda)
        .def("setWeight", &OpenSoT::tasks::Aggregated::setWeight);
}


void pyAggregatedConstraint(py::module& m) {
    // AggregationPolicy enum
    py::enum_<OpenSoT::constraints::Aggregated::AggregationPolicy>(m, "AggregationPolicy")
        .value("EQUALITIES_TO_INEQUALITIES", OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES)
        .value("UNILATERAL_TO_BILATERAL", OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL)
        .export_values();

    // Aggregated Constraint
    py::class_<OpenSoT::constraints::Aggregated,
               std::shared_ptr<OpenSoT::constraints::Aggregated>,
               OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AggregatedConstraint")
        // ctor(list<ConstraintPtr>, x_size, aggregationPolicy)
        .def(py::init<const std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr>&,
                      const unsigned int, const unsigned int>(),
             py::arg(), py::arg(), py::arg("aggregationPolicy") =
             OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES |
             OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL,
             py::keep_alive<1, 2>())

        // ctor(ConstraintPtr, ConstraintPtr, x_size, aggregationPolicy)
        .def(py::init<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr,
                      OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr,
                      const unsigned int, const unsigned int>(),
             py::arg(), py::arg(), py::arg(), py::arg("aggregationPolicy") =
             OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES |
             OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL,
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>())

        // expose update()
        //.def("update", &OpenSoT::constraints::Aggregated::update)

        .def("getConstraintsList", &OpenSoT::constraints::Aggregated::getConstraintsList, py::return_value_policy::reference_internal);
}
