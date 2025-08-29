#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/utils/AffineUtils.h>

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

inline AffineHelper matmul(const Eigen::MatrixXd& M, const AffineHelper& lhs)
{
    return M*lhs;
}

inline AffineHelper mul(const double s, const AffineHelper& lhs)
{
    return s*lhs;
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

inline AffineHelper subVariable(const AffineHelper& affine, const std::vector<size_t>& slice)
{
    Eigen::MatrixXd M;
    M.setZero(slice.size(), affine.getM().rows());

    unsigned int row = 0;
    for(size_t index : slice)
    {
        M(row, index) = 1.;
        row++;
    }

    return M*affine;
}

inline AffineHelper subVariable(const AffineHelper& affine, const size_t& id)
{
    std::vector<size_t> slice;
    slice.push_back(id);
    return subVariable(affine, slice);
}



class OptvarHelperWrapper
{
    public:
        OptvarHelperWrapper(const py::list& vars)
        {
            OptvarHelper::VariableVector vv;

            for (size_t i = 0; i < vars.size(); ++i)
            {
                vv.push_back(vars[i].cast<std::pair<std::string, int>>());
            }

            _optvar = std::make_shared<OptvarHelper>(vv);
        }

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


struct PyAffineHeplerTrampoline : public AffineHelper {
    using AffineHelper::AffineHelper;  // Inherit constructors

    void update() override {
        PYBIND11_OVERRIDE(
            void,          // Return type
            AffineHelper,   // C++ parent class
            update        // Name of the function
            );
    }

public:
    // Lift protected members to public so Python can access them (as in your original)
    using AffineHelper::_M;
    using AffineHelper::_q;
};



void pyAffineHelper(py::module& m, const std::string& className) {
    py::class_<AffineHelper, std::shared_ptr<AffineHelper>, PyAffineHeplerTrampoline>(m, className.c_str())
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


        .def("getValue", [](AffineHelperBase<Eigen::MatrixXd, Eigen::VectorXd>& self, const Eigen::VectorXd& x) -> const Eigen::VectorXd& { return self.getValue(x);})
        .def("getValue", [](AffineHelperBase<Eigen::MatrixXd, Eigen::VectorXd>& self) -> const Eigen::VectorXd& { return self.getValue();})

        .def("__sub__", [](const AffineHelper &a, const AffineHelper &b) { return diff(a, b); })
        .def("__add__", [](const AffineHelper &a, const AffineHelper &b) { return sum(a, b); })
        .def("__add__", [](const AffineHelper &a, const Eigen::VectorXd &v) { return sum(a, v); })
        .def("__sub__", [](const AffineHelper &a, const Eigen::VectorXd &v) { return diff(a, v); })
        .def("__rmatmul__", [](const AffineHelper &a, const Eigen::MatrixXd &m) { return matmul(m, a);}, py::is_operator())
        .def("__matmul__", [](const AffineHelper &a, const Eigen::MatrixXd &m) { return matmul(m, a); }, py::is_operator())
        .def("__mul__", [](const AffineHelper &a, const double s) { return mul(s, a); }, py::is_operator())
        .def("__rmul__", [](const AffineHelper &a, const double s) { return mul(s, a); }, py::is_operator())
        .def("__truediv__", [](const AffineHelper &a, const AffineHelper &b) { return div(a, b); })
        .def("__str__", [](const AffineHelper &a) { return print(a); })

        // Expose lifted members from trampoline
        .def_readwrite("_M", &PyAffineHeplerTrampoline::_M)
        .def_readwrite("_q", &PyAffineHeplerTrampoline::_q)

        .def("__getitem__", [](const AffineHelper& a, const size_t i) {
            if(i >= a.getM().rows())
                throw py::index_error();
            return subVariable(a, i);
        })

        .def("__getitem__", [](const AffineHelper& a, py::slice slice) {
            size_t start, stop, step, slicelength;
            if (!slice.compute(a.getM().rows(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();

            std::vector<size_t> slice_vector;
            slice_vector.reserve(slicelength);
            for(size_t i = 0; i < slicelength; ++i)
            {
                unsigned int id = start + i * step;
                if(id >= a.getM().rows())
                    throw py::index_error();
                slice_vector.push_back(id);
            }
            return subVariable(a, slice_vector);
        })

        .def("update", &AffineHelper::update)

        .def_static("pile", &AffineHelper::pile<Eigen::MatrixXd, Eigen::VectorXd>)
        .def_static("Identity", &AffineHelper::Identity)
        .def_static("Zero", &AffineHelper::Zero)

        .attr("__array_priority__") = 1000.0;

    py::class_<AffineUtils::AffineTask, std::shared_ptr<AffineUtils::AffineTask>, Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AffineTask")
        .def_static("toAffine", &AffineUtils::AffineTask::toAffine, py::arg("task"), py::arg("var"));

    py::class_<AffineUtils::AffineConstraint, std::shared_ptr<AffineUtils::AffineConstraint>, Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AffineConstraint")
        .def_static("toAffine", &AffineUtils::AffineConstraint::toAffine, py::arg("constraint"), py::arg("var"));

}

void pyOptvarHelperWrapper(py::module& m, const std::string& className) {
    py::class_<OptvarHelperWrapper>(m, className.c_str())
        .def(py::init<py::dict>())
        .def(py::init<py::list>())
        .def("getVariable", &OptvarHelperWrapper::getVariable)
        .def("getAllVariables", &OptvarHelperWrapper::getAllVariables)
        .def("getSize", &OptvarHelperWrapper::getSize);
}

