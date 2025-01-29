#include "variables.hpp"
#include "basic.hpp"
#include "generic.hpp"
#include "aggregated.hpp"
#include "sub.hpp"
#include "autostack.hpp"
#include "solver.hpp"
#include "tasks/velocity.hpp"
#include "tasks/acceleration.hpp"
#include "constraints/velocity.hpp"
#include "constraints/acceleration.hpp"
#include "constraints/force.hpp"

PYBIND11_MODULE(pyopensot, m) {
    pyTask<Eigen::MatrixXd, Eigen::VectorXd>(m, "Task");
    pyConstraint<Eigen::MatrixXd, Eigen::VectorXd>(m, "Constraint");
    pyAffineHelper(m, "AffineHelper");
    pyOptvarHelperWrapper(m, "OptvarHelper");
    pyGenericTask(m);
    pyGenericConstraint(m);
    pyAggregatedTask(m);
    pyAggregatedConstraint(m);
    pySubTask(m);
    pySubConstraint(m);
    pyAutostack(m);

    pySolver<Eigen::MatrixXd, Eigen::VectorXd>(m, "Solver");
    pyeHQP(m);
    pyiHQP(m);
    pynHQP(m);

    auto m_t = m.def_submodule("tasks");

    auto m_tv = m_t.def_submodule("velocity");
    pyVelocityPostural(m_tv);
    pyVelocityCartesian(m_tv);
    pyVelocityAngularMomentum(m_tv);
    pyVelocityCoM(m_tv);
    pyVelocityGaze(m_tv);
    pyVelocityManipulability(m_tv);
    pyVelocityMinimumEffort(m_tv);
    pyVelocityCartesianAdmittance(m_tv);

    auto m_ta = m_t.def_submodule("acceleration");
    pyAccelerationPostural(m_ta);
    pyAccelerationCartesian(m_ta);
    pyAccelerationAngularMomentum(m_ta);
    pyAccelerationCoM(m_ta);
    pyDynamicFeasibility(m_ta);

    auto m_c = m.def_submodule("constraints");

    auto m_cv = m_c.def_submodule("velocity");
    pyVelocityJointLimits(m_cv);
    pyVelocityLimits(m_cv);
    pyVelocityOmniWheels4X(m_cv);

    auto m_ca = m_c.def_submodule("acceleration");
    pyAccelerationJointLimits(m_ca);
    pyTorqueLimits(m_ca);
    pyAVelocityLimits(m_ca);

    auto m_cf = m_c.def_submodule("force");
    pyForceCoP(m_cf);
    pyForceFrictionCone(m_cf);
    pyForceNormalTorque(m_cf);
    pyWrenchLimits(m_cf);
}
