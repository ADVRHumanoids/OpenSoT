#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/solvers/nHQP.h>
#include <OpenSoT/solvers/BackEndFactory.h>

namespace py = pybind11;
using namespace OpenSoT;

template <class MatrixType, class VectorType>
class pySolverTrampoline : public Solver<MatrixType, VectorType> {
public:
    using Solver<MatrixType, VectorType>::Solver;
    typedef Solver<MatrixType, VectorType> SMV;

    bool solve(VectorType& solution) override {
        PYBIND11_OVERLOAD_PURE(bool, SMV, solve, solution);
    }
};

template<typename MatrixType, typename VectorType>
VectorType solve(Solver<MatrixType, VectorType>& solver)
{
    VectorType solution;
    solver.solve(solution);
    return solution;
}


template<typename MatrixType, typename VectorType>
void pySolver(py::module& m, const std::string& className) {
    py::enum_<OpenSoT::solvers::solver_back_ends>(m, "solver_back_ends")
            .value("qpOASES", OpenSoT::solvers::solver_back_ends::qpOASES)
            .value("OSQP", OpenSoT::solvers::solver_back_ends::OSQP)
            .value("GLPK", OpenSoT::solvers::solver_back_ends::GLPK)
            .value("eiQuadProg", OpenSoT::solvers::solver_back_ends::eiQuadProg)
            .value("ODYS", OpenSoT::solvers::solver_back_ends::ODYS)
            .value("qpSWIFT", OpenSoT::solvers::solver_back_ends::qpSWIFT)
            .value("proxQP", OpenSoT::solvers::solver_back_ends::proxQP)
            .export_values();


    py::class_<Solver<MatrixType, VectorType>, std::shared_ptr<Solver<MatrixType, VectorType>>, pySolverTrampoline<MatrixType, VectorType>>(m, className.c_str())
        .def(py::init<std::vector<typename Solver<MatrixType, VectorType>::TaskPtr>&>())
        .def(py::init<std::vector<typename Solver<MatrixType, VectorType>::TaskPtr>&, typename Solver<MatrixType, VectorType>::ConstraintPtr>())
        .def(py::init<std::vector<typename Solver<MatrixType, VectorType>::TaskPtr>&, typename Solver<MatrixType, VectorType>::ConstraintPtr, typename Solver<MatrixType, VectorType>::ConstraintPtr>())
        .def("solve", solve<MatrixType, VectorType>)
        .def("getSolverID", &Solver<MatrixType, VectorType>::getSolverID)
        .def("setSolverID", &Solver<MatrixType, VectorType>::setSolverID)
        .def("log", &Solver<MatrixType, VectorType>::log);
}

void pyeHQP(py::module& m) {
    py::class_<solvers::eHQP, std::shared_ptr<solvers::eHQP>, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>>(m, "eHQP")
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&>())
        .def("solve", solve<Eigen::MatrixXd, Eigen::VectorXd>)
        .def("getSigmaMin", &solvers::eHQP::getSigmaMin)
        .def("setSigmaMin", &solvers::eHQP::setSigmaMin);
}

void pyiHQP(py::module& m) {
    py::class_<solvers::iHQP, std::shared_ptr<solvers::iHQP>, Solver<Eigen::MatrixXd, Eigen::VectorXd>>(m, "iHQP")
        .def(py::init<OpenSoT::AutoStack&, double, OpenSoT::solvers::solver_back_ends>(),
             py::arg(), py::arg("eps_regularisation") = DEFAULT_EPS_REGULARISATION, py::arg("be_solver") = OpenSoT::solvers::solver_back_ends::qpOASES)
        .def(py::init<OpenSoT::AutoStack&, double, std::vector<OpenSoT::solvers::solver_back_ends>>())
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, double, OpenSoT::solvers::solver_back_ends>(),
             py::arg(), py::arg("eps_regularisation") = DEFAULT_EPS_REGULARISATION, py::arg("be_solver") = OpenSoT::solvers::solver_back_ends::qpOASES)
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, double, std::vector<OpenSoT::solvers::solver_back_ends>>())
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, double, OpenSoT::solvers::solver_back_ends>(),
             py::arg(), py::arg(), py::arg("eps_regularisation") = DEFAULT_EPS_REGULARISATION, py::arg("be_solver") = OpenSoT::solvers::solver_back_ends::qpOASES)
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, double, std::vector<OpenSoT::solvers::solver_back_ends>>())
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, double, OpenSoT::solvers::solver_back_ends>(),
             py::arg(), py::arg(), py::arg(), py::arg("eps_regularisation") = DEFAULT_EPS_REGULARISATION, py::arg("be_solver") = OpenSoT::solvers::solver_back_ends::qpOASES)
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, double, std::vector<OpenSoT::solvers::solver_back_ends>>())
        .def("solve", solve<Eigen::MatrixXd, Eigen::VectorXd>)
        .def("getNumberOfTasks", &solvers::iHQP::getNumberOfTasks)
        //.def("setOptions", &solvers::iHQP::setOptions) //TODO!
        //.def("getOptions", &solvers::iHQP::getOptions) //TODO!
        .def("getObjective", &solvers::iHQP::getObjective)
        .def("setActiveStack", &solvers::iHQP::setActiveStack)
        .def("activateAllStacks", &solvers::iHQP::activateAllStacks)
        .def("getBackEndName", &solvers::iHQP::getBackEndName)
        .def("setEpsRegularisation", py::overload_cast<const double, const unsigned int>(&solvers::iHQP::setEpsRegularisation))
        .def("setEpsRegularisation", py::overload_cast<const double>(&solvers::iHQP::setEpsRegularisation))
        .def("getBackEnd", &solvers::iHQP::getBackEnd);
}

void pynHQP(py::module& m) {
    py::class_<solvers::nHQP, std::shared_ptr<solvers::nHQP>, Solver<Eigen::MatrixXd, Eigen::VectorXd>>(m, "nHQP")
       .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, double, OpenSoT::solvers::solver_back_ends>(),
            py::arg(), py::arg(), py::arg(), py::arg("be_solver") = OpenSoT::solvers::solver_back_ends::qpOASES)
       .def("solve", solve<Eigen::MatrixXd, Eigen::VectorXd>)
       .def("setMinSingularValueRatio", py::overload_cast<double>(&solvers::nHQP::setMinSingularValueRatio))
       .def("setMinSingularValueRatio", py::overload_cast<std::vector<double>>(&solvers::nHQP::setMinSingularValueRatio))
       .def("setPerformAbRegularization", py::overload_cast<int, bool>(&solvers::nHQP::setPerformAbRegularization))
       .def("setPerformAbRegularization", py::overload_cast<bool>(&solvers::nHQP::setPerformAbRegularization))
       .def("setPerformSelectiveNullSpaceRegularization", py::overload_cast<int, bool>(&solvers::nHQP::setPerformSelectiveNullSpaceRegularization))
       .def("setPerformSelectiveNullSpaceRegularization", py::overload_cast<bool>(&solvers::nHQP::setPerformSelectiveNullSpaceRegularization));
}

