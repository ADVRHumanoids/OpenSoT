from xbot2_interface import pyxbot2_interface as xbi
import pyopensot as pysot
# import pyopensot.oc as
import numpy as np
import unittest
from scipy.spatial.transform import Rotation as R
from pyopensot.tasks.velocity import Cartesian


np.set_printoptions(2, linewidth=200)

utest = unittest.TestCase()

with open("/home/forest_ws/code/OpenSoT/bindings/python/examples/floating_frame/floating_frame.urdf", "r") as f: # TODO: Change the absolute path
    urdf_string = f.read()


model = xbi.ModelInterface2(urdf_string)

print(f"model.nq: {model.nq}")
print(f"model.nv: {model.nv}")

q_val = np.array([0., 0., 0., 0., 0., 1., 0.])
qdot_val = np.array([0., 0., 0., 0., 0., 0.])

model.setJointPosition(q_val)
model.setJointVelocity(qdot_val)
model.update()

# controls are velocities

vars = list()
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))

variables = pysot.OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")


dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))

dvariables = pysot.OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")


dt = 0.1

dSE3 = pysot.oc.EulerSE3(model, dq, dqdot, dt)

print(f"dSE3.getA():\n{dSE3.getA()}")
print(f"dSE3.getA().shape:\n{dSE3.getA().shape}")

utest.assertEqual(dSE3.getA().shape[0], dq.getOutputSize())
utest.assertEqual(dSE3.getA().shape[1], dq.getInputSize())


dx = dq
du = dqdot


dSE3_dx = dSE3.getA()@dx.getM().transpose()
dSE3_du = dSE3.getA()@du.getM().transpose()

print(f"dSE3_dx:\n{dSE3_dx}")
print(f"dSE3_du:\n{dSE3_du}")

cartesian_task = Cartesian("Cartesian", model, "base_link", "world")
cartesian_task.setLambda(1)
cartesian_task.setWeight(1e0 * np.eye(6))

T, _ = cartesian_task.getReference()
pose_ref = T.copy()

q_rand = np.array([1., 1., 0., 0., 0., 1., 0.]) #random_pose(-2., 2.)
pose_ref.translation = q_rand[0:3]
pose_ref.linear = R.from_quat(q_rand[3:]).as_matrix()
cartesian_task.setReference(pose_ref.copy())

cartesian_task.update()


print(f"cartesian_task.getA():\n{cartesian_task.getA()}")
print(f"cartesian_task.getb():\n{cartesian_task.getb()}")

print(np.linalg.inv(cartesian_task.getA())@cartesian_task.getb())


exit()

utest.assertEqual(dSE3_dx.shape[0], dx.getOutputSize())
utest.assertEqual(dSE3_dx.shape[1], dx.getOutputSize())
utest.assertEqual(dSE3_du.shape[0], du.getOutputSize())
utest.assertEqual(dSE3_du.shape[1], du.getOutputSize())


# controls are accelerations

vars.append(("qddot", model.nv))
variables = pysot.OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")



dvars.append(("dqddot", model.nv))

dvariables = pysot.OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")



dSE3 = pysot.oc.EulerSE3(model, dq, dqdot, dt)

print(f"dSE3.getA():\n{dSE3.getA()}")
print(f"dSE3.getA().shape:\n{dSE3.getA().shape}")

utest.assertEqual(dSE3.getA().shape[0], dq.getOutputSize())
utest.assertEqual(dSE3.getA().shape[1], dq.getInputSize())

dx = pysot.AffineHelper.pile(dq, dqdot)
du = dqddot

dSE3_dx = dSE3.getA()@dx.getM().transpose()
dSE3_du = dSE3.getA()@du.getM().transpose()

print(f"dSE3_dx:\n{dSE3_dx}")
print(f"dSE3_du:\n{dSE3_du}")

utest.assertEqual(dSE3_dx.shape[0], dq.getOutputSize())
utest.assertEqual(dSE3_dx.shape[1], dx.getOutputSize())
utest.assertEqual(dSE3_du.shape[0], du.getOutputSize())
utest.assertEqual(dSE3_du.shape[1], du.getOutputSize())

# SE3xRn

class dynamics_derivative(pysot.Task):
    """
    This carries the derivative of the linear dynamics computed from euler.
    """
    def __init__(self, name, df):
        super().__init__(name, df.getInputSize())
        self.df = df
        self._W = np.eye(df.getOutputSize())

    def _update(self):
        self.lin = self.df
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

    @classmethod
    def create(cls, name, df):
        obj = cls(name, df)
        obj.update()
        return obj

def euler(x, xdot, dt):
    return x + dt * xdot  # x1 = x0 + dt * xdot0


dRn = dynamics_derivative.create(f"dRn", euler(dqdot, dqddot, dt))
print(f"dRn.getA():\n{dRn.getA()}")
print(f"dRn.getA().shape:\n{dRn.getA().shape}")
dRn_dx = dRn.getA()@dx.getM().transpose()
dRn_du = dRn.getA()@du.getM().transpose()
print(f"dRn_dx:\n{dRn_dx}")
print(f"dRn_du:\n{dRn_du}")



dSE3xRn = dSE3 + dRn
print(f"dSE3xRn.getA():\n{dSE3xRn.getA()}")
print(f"dSE3xRn.getA().shape:\n{dSE3xRn.getA().shape}")
dSE3xRn_dx = dSE3xRn.getA()@dx.getM().transpose()
dSE3xRn_du = dSE3xRn.getA()@du.getM().transpose()
print(f"dSE3xRn_dx:\n{dSE3xRn_dx}")
print(f"dSE3xRn_du:\n{dSE3xRn_du}")





