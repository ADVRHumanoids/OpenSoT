#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/common/utils.h>
#include <OpenSoT/variables/Torque.h>

namespace py = pybind11;
using namespace OpenSoT::variables;

void pyTorqueVariable(py::module& m) {
    py::class_<Torque, std::shared_ptr<Torque>, OpenSoT::AffineHelper>(m, "Torque")
    .def(py::init<XBot::ModelInterface::Ptr, const OpenSoT::AffineHelper&, std::vector<std::string>, std::vector<OpenSoT::AffineHelper>>(),
         py::arg("model"),
         py::arg("qddot_var"),
         py::arg("contact_links") = std::vector<std::string>(),
         py::arg("force_vars") = std::vector<OpenSoT::AffineHelper>())
        .def("update", &Torque::update);
}
