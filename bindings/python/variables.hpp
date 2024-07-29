#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/utils/Affine.h>

namespace py = pybind11;
using namespace OpenSoT;

inline AffineHelper diff(const AffineHelper& lhs, const AffineHelper& rhs)
{
    return lhs-rhs;
}

inline AffineHelper diff(const AffineHelper& lhs, const Eigen::VectorXd& v)
{
    return lhs-v;
}

inline AffineHelper sum(const AffineHelper& lhs, const AffineHelper& rhs)
{
    return lhs+rhs;
}

inline AffineHelper sum(const AffineHelper& lhs, const Eigen::VectorXd& v)
{
    return lhs+v;
}

inline AffineHelper mul(const Eigen::MatrixXd& M, const AffineHelper& lhs)
{
    return M*lhs;
}

inline AffineHelper div(const AffineHelper& lhs, const AffineHelper& rhs)
{
    return lhs/rhs;
}

inline std::string print(const AffineHelper& affine)
{
    std::ostringstream os;
    os<< "M:\n" << affine.getM() << "\n\nq:\n" << affine.getq();
    return os.str();
}

class OptvarHelperWrapper
{
    public:
        OptvarHelperWrapper(const py::dict& vars)
        {
            OptvarHelper::VariableVector vv;

            for (std::pair<py::handle, py::handle> item : vars)
            {
                auto key = item.first.cast<std::string>();
                auto value = item.second.cast<int>();
                vv.push_back(std::pair<std::string, int>(key, value));
            }

            _optvar = std::make_shared<OptvarHelper>(vv);
        }

        AffineHelper getVariable(const std::string& name) const
        {
            return _optvar->getVariable(name);
        }

        int getSize() const
        {
            return _optvar->getSize();
        }

        std::vector<AffineHelper> getAllVariables() const
        {
            return _optvar->getAllVariables();
        }

    private:
        std::shared_ptr<OptvarHelper> _optvar;
};

Eigen::VectorXd get_value(const AffineHelper& var, const Eigen::VectorXd& x)
{
    Eigen::VectorXd val;
    var.getValue(x, val);
    return val;
}

void pyAffineHelper(py::module& m, const std::string& className) {
    py::class_<AffineHelper>(m, className.c_str())
        .def(py::init<>())
        .def(py::init<int, int>())
        .def(py::init<const Eigen::MatrixXd&, const Eigen::VectorXd&>())
        .def("setM", &AffineHelper::setM)
        .def("setq", &AffineHelper::setq)
        .def("set", &AffineHelper::set)
        .def("getM", &AffineHelper::getM)
        .def("getq", &AffineHelper::getq)
        .def("getInputSize", &AffineHelper::getInputSize)
        .def("getOutputSize", &AffineHelper::getOutputSize)
        .def("setZero", py::overload_cast<>(&AffineHelper::setZero))
        .def("setZero", py::overload_cast<int, int>(&AffineHelper::setZero))
        .def("update", &AffineHelper::update)
        .def("getValue", get_value)
        .def("__sub__", [](const AffineHelper &a, const AffineHelper &b) { return diff(a, b); })
        .def("__add__", [](const AffineHelper &a, const AffineHelper &b) { return sum(a, b); })
        .def("__add__", [](const AffineHelper &a, const Eigen::VectorXd &v) { return sum(a, v); })
        .def("__sub__", [](const AffineHelper &a, const Eigen::VectorXd &v) { return diff(a, v); })
        .def("__mul__", [](const Eigen::MatrixXd &m, const AffineHelper &a) { return mul(m, a); })
        .def("__truediv__", [](const AffineHelper &a, const AffineHelper &b) { return div(a, b); })
        .def("__str__", [](const AffineHelper &a) { return print(a); });
}

void pyOptvarHelperWrapper(py::module& m, const std::string& className) {
    py::class_<OptvarHelperWrapper>(m, className.c_str())
        .def(py::init<py::dict>())
         .def("getVariable", &OptvarHelperWrapper::getVariable)
         .def("getAllVariables", &OptvarHelperWrapper::getAllVariables)
         .def("getSize", &OptvarHelperWrapper::getSize);
}


