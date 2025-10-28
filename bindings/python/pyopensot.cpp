#include "variables.hpp"
#include "basic.hpp"
#include "generic.hpp"
#include "aggregated.hpp"
#include "sub.hpp"
#include "autostack.hpp"
#include "solver.hpp"
#include "tasks/velocity.hpp"
#include "tasks/acceleration.hpp"
#include "tasks/minimize_variable.hpp"
#include "constraints/velocity.hpp"
#include "constraints/acceleration.hpp"
#include "constraints/force.hpp"
#include "variables/torque.hpp"
#include "oc.hpp"

#ifdef OPENSOT_COMPILE_COLLISION
    #include "constraints/velocity_collision.hpp"
    #include "tasks/velocity_collision.hpp"
#endif

#ifdef OPENSOT_SOTH_FRONT_END
    #include "solver_hcod.hpp"
#endif

#ifdef HPIPM_CPP_FOUND
    #include "solver_hpipmoc.hpp"
#endif


PYBIND11_MODULE(pyopensot, m) {
    pyTask<Eigen::MatrixXd, Eigen::VectorXd>(m, "Task");
    pyConstraint<Eigen::MatrixXd, Eigen::VectorXd>(m, "Constraint");
    pyAggregatedTask(m);
    pyAggregatedConstraint(m);
    pySubTask(m);
    pySubConstraint(m);
    pyAffineHelper(m, "AffineHelper");
    pySubVariable(m, "SubVariable");
    pyOptvarHelperWrapper(m, "OptvarHelper");
    pyGenericTask(m);
    pyGenericConstraint(m);
    pyAutostack(m);

    pySolver<Eigen::MatrixXd, Eigen::VectorXd>(m, "Solver");
    pyeHQP(m);
    pyiHQP(m);
    pynHQP(m);

#ifdef OPENSOT_SOTH_FRONT_END
    pyHCOD(m);
#endif

#ifdef HPIPM_CPP_FOUND
    pyHPIPMOC(m);
#endif

    auto m_t = m.def_submodule("tasks");
    pyMinimizeVariable(m_t);

    auto m_tv = m_t.def_submodule("velocity");
    pyVelocityPostural(m_tv);
    pyVelocityCartesian(m_tv);
    pyVelocityAngularMomentum(m_tv);
    pyVelocityCoM(m_tv);
    pyVelocityGaze(m_tv);
    pyVelocityManipulability(m_tv);
    pyVelocityMinimumEffort(m_tv);

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

#ifdef OPENSOT_COMPILE_COLLISION
    pyVelocityCollisionAvoidance(m_cv);
    pyVelocityCollisionAvoidanceTask(m_tv);
#endif

    auto m_ca = m_c.def_submodule("acceleration");
    pyAccelerationJointLimits(m_ca);
    pyTorqueLimits(m_ca);
    pyAVelocityLimits(m_ca);

    auto m_cf = m_c.def_submodule("force");
    pyForceCoP(m_cf);
    pyForceFrictionCone(m_cf);
    pyForceNormalTorque(m_cf);
    pyWrenchLimits(m_cf);

    auto m_v = m.def_submodule("variables");
    pyTorqueVariable(m_v);

    auto m_oc = m.def_submodule("oc");
    pyopensot_oc(m_oc);

}


