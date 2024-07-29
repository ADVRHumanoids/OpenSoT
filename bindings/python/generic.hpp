#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/GenericTask.h>

namespace py = pybind11;
using namespace OpenSoT::constraints;
using namespace OpenSoT::tasks;

// Define bindings
void pyGenericConstraint(py::module& m) {
    py::enum_<GenericConstraint::Type>(m, "ConstraintType")
        .value("BOUND", GenericConstraint::Type::BOUND)
        .value("CONSTRAINT", GenericConstraint::Type::CONSTRAINT)
        .export_values();

    py::class_<GenericConstraint, std::shared_ptr<GenericConstraint>, Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "GenericConstraint")
        .def(py::init<std::string, const Eigen::VectorXd&, const Eigen::VectorXd&, int>())
        .def(py::init<std::string, const AffineHelper&, const Eigen::VectorXd&, const Eigen::VectorXd&, GenericConstraint::Type>())
        .def("setConstraint", &GenericConstraint::setConstraint)
        .def("setBounds", &GenericConstraint::setBounds)
        .def("update", &GenericConstraint::update)
        .def("getType", &GenericConstraint::getType);
}

void pyGenericTask(py::module& m) {
    py::class_<GenericTask, std::shared_ptr<GenericTask>, Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "GenericTask")
        .def(py::init<const std::string&, const Eigen::MatrixXd&, const Eigen::VectorXd&>())
        .def(py::init<const std::string&, const Eigen::MatrixXd&, const Eigen::VectorXd&, const AffineHelper&>())
        .def("setA", &GenericTask::setA)
        .def("setb", &GenericTask::setb)
        .def("setAb", &GenericTask::setAb)
        .def("setc", &GenericTask::setc)
        .def("setHessianType", &GenericTask::setHessianType);
}


