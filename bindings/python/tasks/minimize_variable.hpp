#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/MinimizeVariable.h>

namespace py = pybind11;
using namespace OpenSoT::tasks;

Eigen::VectorXd get_reference(const MinimizeVariable& am)
{
    Eigen::VectorXd ref;
    am.getReference(ref);
    return ref;
}

void pyMinimizeVariable(py::module& m) {
    py::class_<MinimizeVariable, std::shared_ptr<MinimizeVariable>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "MinimizeVariable")
        .def(py::init<std::string, const OpenSoT::AffineHelper&>())
        .def("setReference", &MinimizeVariable::setReference)
        .def("getReference", get_reference);
}
