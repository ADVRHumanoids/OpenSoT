#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <OpenSoT/solvers/HCOD.h>

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


void pyHCOD(py::module& m) {
    py::class_<solvers::HCOD, std::shared_ptr<solvers::HCOD>, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>>(m, "HCOD")
    .def(py::init<OpenSoT::AutoStack&, const double>())
        .def(py::init<OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack&, OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr, const double>())
        .def("solve", solve<Eigen::MatrixXd, Eigen::VectorXd>)
        .def("setDisableWeightsComputation", &solvers::HCOD::setDisableWeightsComputation)
        .def("getDisableWeightsComputation", &solvers::HCOD::getDisableWeightsComputation)
        .def("setDamping", &solvers::HCOD::setDamping)
        .def("printSOT", &solvers::HCOD::printSOT);
}
