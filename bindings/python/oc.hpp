#include <OpenSoT/utils/oc.h>

// py_ocp.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/oc.h>
#include <OpenSoT/solvers/swSQP.h>

namespace py = pybind11;

using OpenSoT::ocp;
using Stage = OpenSoT::ocp::Stage;

using namespace OpenSoT::solvers;

// Opaque vector types so we can bind them as Python list-like containers
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<OpenSoT::AffineHelper>>);
PYBIND11_MAKE_OPAQUE(std::vector<std::shared_ptr<Stage>>);

std::string print_opt(swSQP::options& opt)
{
    std::string str;
    str = opt.toOSS().str();
    return str;
}

struct PyStateSpaceRepresentation : OpenSoT::Space {
    using Space::Space;

    void integrate(const Eigen::VectorXd& x0,
             const Eigen::VectorXd& dx0,
             Eigen::VectorXd& x1) override {
        PYBIND11_OVERRIDE_PURE(
            void,                         // return type
            Space,     // parent class
            sum,                          // function name
            x0, dx0, x1                   // arguments
            );
    }
};



void pyopensot_oc(py::module& m) {
    py::class_<OpenSoT::Space, OpenSoT::Space::Ptr, PyStateSpaceRepresentation>(m, "Space")
        .def(py::init<unsigned int, unsigned int>(), py::arg("nq"), py::arg("nv"))
        .def("nq",  &OpenSoT::Space::nq)
        .def("nv",  &OpenSoT::Space::nv)
        .def("integrate", &OpenSoT::Space::integrate, py::arg("x0"), py::arg("dx0"), py::arg("x1"));


    // ---------------- Derived: VectorSpace ----------------
    py::class_<OpenSoT::VectorSpace, OpenSoT::Space, OpenSoT::VectorSpace::Ptr>(m, "VectorSpace")
        .def(py::init<unsigned int>(), py::arg("dimension"))
        .def("integrate", [](OpenSoT::VectorSpace& self, const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0) -> Eigen::VectorXd {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.integrate(x0, dx0, x1);
            return x1;}, py::arg("x0"), py::arg("dx0"));

    // ---------------- Derived: R3 ----------------
    py::class_<OpenSoT::R3, OpenSoT::Space, OpenSoT::R3::Ptr>(m, "R3")
        .def(py::init<std::shared_ptr<XBot::ModelInterface>, const std::string&, const std::string&>(), py::arg("model"), py::arg("base"), py::arg("distal"))
        .def("integrate", [](OpenSoT::VectorSpace& self, const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0) -> Eigen::VectorXd {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.integrate(x0, dx0, x1);
            return x1;}, py::arg("x0"), py::arg("dx0"));

    // ---------------- Derived: RobotSpace ----------------
    py::class_<OpenSoT::RobotSpace, OpenSoT::Space, OpenSoT::RobotSpace::Ptr>(m, "RobotSpace")
        .def(py::init<std::shared_ptr<XBot::ModelInterface>>(), py::arg("model"))
        .def("integrate", [](OpenSoT::VectorSpace& self, const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0) -> Eigen::VectorXd {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.integrate(x0, dx0, x1);
            return x1;}, py::arg("x0"), py::arg("dx0"));

    // ---------------- Derived: QuaternionSpace ----------------
    py::class_<OpenSoT::QuaternionSpace, OpenSoT::Space, OpenSoT::QuaternionSpace::Ptr>(m, "QuaternionSpace")
        .def(py::init<>())
        .def("integrate", [](OpenSoT::QuaternionSpace& self, const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0) -> Eigen::VectorXd {
            Eigen::VectorXd x1(x0.size());
            x1.setZero();
            self.integrate(x0, dx0, x1);
            return x1;}, py::arg("x0"), py::arg("dx0"));

    // ---------------- Composite: CompositeSpace ----------------
    py::class_<OpenSoT::CompositeSpace, OpenSoT::Space, OpenSoT::CompositeSpace::Ptr>(m, "CompositeSpace")
        .def(py::init<const std::vector<OpenSoT::Space::Ptr>&>(), py::arg("representations"))
        .def("getSpaces", &OpenSoT::CompositeSpace::getSpaces)
        .def("integrate", [](OpenSoT::CompositeSpace& self, const Eigen::VectorXd& x0, const Eigen::VectorXd& dx0) -> Eigen::VectorXd {
                              Eigen::VectorXd x1(x0.size());
                              x1.setZero();
                              self.integrate(x0, dx0, x1);
                              return x1;}, py::arg("x0"), py::arg("dx0"));


    // Expose vector<stage::Ptr> as a Python list-like container (the horizon)
    py::bind_vector<std::vector<std::shared_ptr<Stage>>>(m, "StagePtrVector");

    // Bind vector of AffineHelper as Python list
    py::bind_vector<std::vector<std::shared_ptr<OpenSoT::AffineHelper>>>(m, "AffineHelperVector")
        .def("append", [](std::vector<std::shared_ptr<OpenSoT::AffineHelper>>& v, const std::shared_ptr<OpenSoT::AffineHelper>& val) {
            v.push_back(val);});

    // Bind stage
    py::class_<Stage, std::shared_ptr<Stage>>(m, "Stage")
        .def(py::init<>())
        .def("isFinalStage", &Stage::isFinalStage)
        .def("update", &Stage::update)
        .def("cost", &Stage::cost)
        .def_readwrite("model", &Stage::model)
        .def_readwrite("variables", &Stage::variables)
        .def_readwrite("x", &Stage::x)
        .def_readwrite("u", &Stage::u)
        .def_readwrite("dx", &Stage::dx)
        .def_readwrite("du", &Stage::du)
        .def_readwrite("q", &Stage::q)
        .def_readwrite("v", &Stage::v)
        .def_readwrite("stack", &Stage::stack)
        .def_readwrite("dynamics_derivative", &Stage::dynamics_derivative)
        .def_readwrite("state_space", &Stage::state_space);

    // Bind ocp
    py::class_<ocp, std::shared_ptr<ocp>>(m, "OCP")
        .def(py::init<>())
        .def("addStage", &ocp::addStage, py::arg("stage"))
        .def("stage",
             [](ocp& self, unsigned int i) -> std::shared_ptr<Stage> {
                 // bounds check via .at() for nicer Python IndexError
                 return self.getHorizon().at(i);
             },
             py::arg("i"),
             py::return_value_policy::reference_internal)
        .def("getHorizon",
             (ocp::horizon& (ocp::*)()) &ocp::getHorizon,
             py::return_value_policy::reference_internal)
        .def("getNumberOfNodes", &ocp::getNumberOfNodes)

        .def("cost", py::overload_cast<>(&ocp::cost))
        .def("cost", py::overload_cast<unsigned int>(&ocp::cost))

        .def("update", &ocp::update);

        // Bind swSQP::options
        py::class_<swSQP::options>(m, "swSQPOptions")
            .def(py::init<>())
            .def("print", print_opt)
            .def_readwrite("verbose", &swSQP::options::verbose)
            .def_readwrite("max_iters", &swSQP::options::max_iters)
            .def_readwrite("alpha_min", &swSQP::options::alpha_min)
            .def_readwrite("beta", &swSQP::options::beta)
            .def_readwrite("min_abs_delta_solution", &swSQP::options::min_abs_delta_solution)
            .def_readwrite("use_line_search", &swSQP::options::use_line_search);


        // Bind swSQP
        py::class_<swSQP, swSQP::Ptr>(m, "swSQP")
            .def(py::init<OpenSoT::ocp::Ptr>(), py::arg("ocp"))
            .def("solve", &swSQP::solve)
            .def("getStateSolution", &swSQP::getStateSolution)
            .def("getControlSolution", &swSQP::getControlSolution)
            .def("getOptions", (swSQP::options& (swSQP::*)()) &swSQP::getOptions, py::return_value_policy::reference_internal);

}
