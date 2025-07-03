from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
import os
import time

urdf_path = os.getcwd() + '/panda.urdf'
urdf = open(urdf_path, 'r').read()
model = xbi.ModelInterface2(urdf)

q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()

qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)

dqmax = model.getVelocityLimits()
dt = 1./100.
dqlims = VelocityLimits(model, dqmax, dt)

p = Postural(model)

c = Cartesian("Cartesian", model, "panda_link7", "world")
c.setLambda(0.1)

manip = Manipulability(model, c)
mineffort = MinimumEffort(model)

ref = c.getActualPose().copy()

s = pysot.hard(pysot.sub(c, [0,1,2]), pysot.sub(c, [3,4,5]))
s = pysot.hard(s, p)
#s = pysot.hard(s, manip)
#s = pysot.hard(s, mineffort)
s = pysot.subj(s, qlims)
#s = pysot.subj(s, dqlims) # removing dqlims improve tracking!

s.update()

qref, dqref = p.getReference()
print(f"qref: {qref}")
print(f"dqref: {dqref}")

pose_ref, vel_ref = c.getReference()
print(f"pose_ref: {pose_ref}")
print(f"vel_ref: {vel_ref}")

#solver = pysot.eHQP(s.getStack())
solver = pysot.iHQP(s)
#solver = pysot.nHQP(s.getStack(), s.getBounds(), 1e-3)

t = 0.
alpha = 0.01
for i in range(1000):
    model.setJointPosition(q)
    model.update()

    pose_ref.translation[0] += alpha * np.sin(2.*3.1415 * t)
    pose_ref.translation[1] += alpha * np.cos(2.*3.1415 * t)
    t = t + alpha
    if t > 1.:
        t = 0
    c.setReference(pose_ref)

    s.update()

    dq = solver.solve()
    q += dq








