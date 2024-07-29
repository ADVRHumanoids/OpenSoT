#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>

namespace py = pybind11;

void pyAggregatedTask(py::module& m) {
    py::class_<OpenSoT::tasks::Aggregated, std::shared_ptr<OpenSoT::tasks::Aggregated>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AggregatedTask")
        .def(py::init<const std::list<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>>&, const unsigned int>())
        .def(py::init<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>, const unsigned int>())
        .def(py::init<std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>,
             std::shared_ptr<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>, const unsigned int>())
        .def("_update", &OpenSoT::tasks::Aggregated::_update)
        .def("getOwnConstraints", &OpenSoT::tasks::Aggregated::getOwnConstraints, py::return_value_policy::reference_internal)
        .def("getAggregatedConstraints", &OpenSoT::tasks::Aggregated::getAggregatedConstraints, py::return_value_policy::reference_internal)
        .def("getTaskList", &OpenSoT::tasks::Aggregated::getTaskList, py::return_value_policy::reference_internal)
        .def("setLambda", &OpenSoT::tasks::Aggregated::setLambda)
        .def("setWeight", &OpenSoT::tasks::Aggregated::setWeight);
}


void pyAggregatedConstraint(py::module& m) {
    py::enum_<OpenSoT::constraints::Aggregated::AggregationPolicy>(m, "AggregationPolicy")
            .value("EQUALITIES_TO_INEQUALITIES", OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES)
            .value("UNILATERAL_TO_BILATERAL", OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL)
            .export_values();

    py::class_<OpenSoT::constraints::Aggregated, std::shared_ptr<OpenSoT::constraints::Aggregated>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AggregatedConstraint")
        .def(py::init<const std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr>&,
             const unsigned int, const unsigned int>(),
             py::arg(), py::arg(), py::arg("aggregationPolicy") =
            OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES |
            OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL)
        .def(py::init<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr,
             OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, const unsigned int, const unsigned int>(),
             py::arg(), py::arg(), py::arg(), py::arg("aggregationPolicy") =
            OpenSoT::constraints::Aggregated::AggregationPolicy::EQUALITIES_TO_INEQUALITIES |
            OpenSoT::constraints::Aggregated::AggregationPolicy::UNILATERAL_TO_BILATERAL)
        .def("update", &OpenSoT::constraints::Aggregated::update)
        .def("getConstraintsList", &OpenSoT::constraints::Aggregated::getConstraintsList, py::return_value_policy::reference_internal);
}
