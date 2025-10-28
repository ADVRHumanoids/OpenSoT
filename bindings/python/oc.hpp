#include <OpenSoT/oc/oc.h>

// py_ocp.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/oc/Manifolds.h>
#include <OpenSoT/oc/EulerSE3.h>
#include <OpenSoT/oc/EulerVector.h>
#include <OpenSoT/oc/SE3Task.h>
#include <OpenSoT/oc/TorquesTask.h>
#include <OpenSoT/oc/TorquesConstraint.h>

namespace py = pybind11;

using OpenSoT::ocp;
using Stage = OpenSoT::ocp::Stage;


// Opaque vector types so we can bind them as Python list-like containers
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<OpenSoT::AffineHelper>>);
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<Stage>>);


struct PyStateSpaceRepresentation : OpenSoT::Space
{
    using Space::Space;

    void plus(const Eigen::VectorXd &x0,
              const Eigen::VectorXd &dx0,
              Eigen::VectorXd &x1) override
    {
        PYBIND11_OVERRIDE_PURE(
            void,       // return type
            Space,      // parent class
            sum,        // function name
            x0, dx0, x1 // arguments
        );
    }
};

void pyopensot_oc(py::module &m)
{

    py::class_<OpenSoT::oc::EulerSE3, OpenSoT::oc::EulerSE3::Ptr, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "EulerSE3")
        .def(py::init<const XBot::ModelInterface &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const double>());

    py::class_<OpenSoT::oc::EulerVector, OpenSoT::oc::EulerVector::Ptr, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "EulerVector")
        .def(py::init<const XBot::ModelInterface &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const AffineHelper &, const double>());

    py::class_<OpenSoT::oc::TorquesTask, OpenSoT::oc::TorquesTask::Ptr, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "TorquesTask")
        .def(py::init<XBot::ModelInterface &, const AffineHelper &, const AffineHelper &>());

    py::class_<OpenSoT::oc::DynamicsConstraint, OpenSoT::oc::DynamicsConstraint::Ptr, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "DynamicsConstraint")
        .def(py::init<XBot::ModelInterface &, const AffineHelper &, const AffineHelper &>())
        .def("getTorqueLimit", &OpenSoT::oc::DynamicsConstraint::getTorqueLimit)
        .def("setTorqueLimit", &OpenSoT::oc::DynamicsConstraint::setTorqueLimit);

    py::class_<OpenSoT::oc::SE3Task, OpenSoT::oc::SE3Task::Ptr, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "SE3Task")
        .def(py::init<const std::string &, const XBot::ModelInterface &, const AffineHelper &, const std::string &>())
        .def("getError", &OpenSoT::oc::SE3Task::getError)
        .def("setReference", &OpenSoT::oc::SE3Task::setReference)
        .def("getReference", &OpenSoT::oc::SE3Task::getReference)
        .def("getDistalFrame", &OpenSoT::oc::SE3Task::getDistalFrame);

    py::class_<OpenSoT::Space, OpenSoT::Space::Ptr, PyStateSpaceRepresentation>(m, "Space")
        .def(py::init<unsigned int, unsigned int>(), py::arg("nq"), py::arg("nv"))
        .def("nq", &OpenSoT::Space::nq)
        .def("nv", &OpenSoT::Space::nv)
        .def("plus", &OpenSoT::Space::plus, py::arg("x0"), py::arg("dx0"), py::arg("x1"));

    // ---------------- Derived: VectorSpace ----------------
    py::class_<OpenSoT::VectorSpace, OpenSoT::Space, OpenSoT::VectorSpace::Ptr>(m, "VectorSpace")
        .def(py::init<unsigned int>(), py::arg("dimension"))
        .def("plus", [](OpenSoT::VectorSpace &self, const Eigen::VectorXd &x0, const Eigen::VectorXd &dx0) -> Eigen::VectorXd
             {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.plus(x0, dx0, x1);
            return x1; }, py::arg("x0"), py::arg("dx0"));

    // ---------------- Derived: SE3Space ----------------
    py::class_<OpenSoT::SE3Space, OpenSoT::Space, OpenSoT::SE3Space::Ptr>(m, "SE3Space")
        .def(py::init<>())
        .def("plus", [](OpenSoT::SE3Space &self, const Eigen::VectorXd &x0, const Eigen::VectorXd &dx0) -> Eigen::VectorXd
             {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.plus(x0, dx0, x1);
            return x1; }, py::arg("x0"), py::arg("dx0"));

    // ---------------- Composite: CompositeSpace ----------------
    py::class_<OpenSoT::CompositeSpace, OpenSoT::Space, OpenSoT::CompositeSpace::Ptr>(m, "CompositeSpace")
        .def(py::init<const std::vector<OpenSoT::Space::Ptr> &>(), py::arg("representations"))
        .def("getSpaces", &OpenSoT::CompositeSpace::getSpaces)
        .def("plus", [](OpenSoT::CompositeSpace &self, const Eigen::VectorXd &x0, const Eigen::VectorXd &dx0) -> Eigen::VectorXd
             {
                              Eigen::VectorXd x1(x0.size());
                              x1.setZero();
                              self.plus(x0, dx0, x1);
                              return x1; }, py::arg("x0"), py::arg("dx0"));

    // Expose vector<stage::Ptr> as a Python list-like container (the horizon)
    py::bind_vector<std::vector<std::shared_ptr<Stage>>>(m, "StagePtrVector");

    // Bind vector of AffineHelper as Python list
    py::bind_vector<std::vector<std::shared_ptr<OpenSoT::AffineHelper>>>(m, "AffineHelperVector")
        .def("append", [](std::vector<std::shared_ptr<OpenSoT::AffineHelper>> &v, const std::shared_ptr<OpenSoT::AffineHelper> &val)
             { v.push_back(val); });

    // Bind stage
    py::class_<Stage, std::shared_ptr<Stage>>(m, "Stage")
        .def(py::init<>())
        .def("isFinalStage", &Stage::isFinalStage)
        .def("update", &Stage::update)
        .def("stage_cost", &Stage::stage_cost)
        .def_readwrite("model", &Stage::model)
        .def_readwrite("variables", &Stage::variables)
        .def_readwrite("x", &Stage::x)
        .def_readwrite("xdot", &Stage::xdot)
        .def_readwrite("u", &Stage::u)
        .def_readwrite("dx", &Stage::dx)
        .def_readwrite("du", &Stage::du)
        .def_readwrite("q", &Stage::q)
        .def_readwrite("v", &Stage::v)
        .def_readwrite("a", &Stage::a)
        .def_readwrite("stack", &Stage::stack)
        .def_readwrite("dynamics_derivative", &Stage::dynamics_derivative)
        .def_readwrite("state_space", &Stage::state_space);

    // Bind ocp
    py::class_<ocp, std::shared_ptr<ocp>>(m, "OCP")
        .def(py::init<>())
        .def("addStage", &ocp::addStage, py::arg("stage"))
        .def("stage", [](ocp &self, unsigned int i) -> std::shared_ptr<Stage>
             {
                 // bounds check via .at() for nicer Python IndexError
                 return self.getHorizon().at(i); }, py::arg("i"), py::return_value_policy::reference_internal)
        .def("getHorizon", (ocp::horizon & (ocp::*)()) & ocp::getHorizon, py::return_value_policy::reference_internal)
        .def("getNumberOfNodes", &ocp::getNumberOfNodes)

        .def("cost", py::overload_cast<>(&ocp::cost))

        .def("update", &ocp::update);
}
