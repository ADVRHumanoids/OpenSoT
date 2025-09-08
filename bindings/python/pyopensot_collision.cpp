#include "constraints/velocity_collision.hpp"

PYBIND11_MODULE(pyopensot_collision, m) {
    auto m_c = m.def_submodule("constraints");
    auto m_cv = m_c.def_submodule("velocity");
    pyVelocityCollisionAvoidance(m_cv);
}
