#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/SubConstraint.h>

namespace py = pybind11;
using namespace OpenSoT;

void pySubConstraint(py::module& m) {
    py::class_<SubConstraint, std::shared_ptr<SubConstraint>, Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "SubConstraint")
       .def(py::init<std::shared_ptr<Constraint<Eigen::MatrixXd, Eigen::VectorXd>>, const std::list<unsigned int>&>())
       .def("update", &SubConstraint::update);
}


void pySubTask(py::module& m) {
    py::class_<SubTask, std::shared_ptr<SubTask>, Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "SubTask")
        .def(py::init<std::shared_ptr<Task<Eigen::MatrixXd, Eigen::VectorXd>>, const std::list<unsigned int>&>())
        .def("setWeight", &SubTask::setWeight)
        .def("getConstraints", &SubTask::getConstraints)
        .def("getTaskSize", &SubTask::getTaskSize)
        .def("getActiveJointsMask", &SubTask::getActiveJointsMask)
        .def("setActiveJointsMask", &SubTask::setActiveJointsMask)
        .def_static("isSubTask", &SubTask::isSubTask)
        .def_static("asSubTask", &SubTask::asSubTask)
        .def("getTask", &SubTask::getTask);
}
