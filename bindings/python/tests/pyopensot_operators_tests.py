from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
import unittest
import os

utest = unittest.TestCase()

urdf_path = os.getcwd() + '/panda.urdf'
urdf = open(urdf_path, 'r').read()
model = xbi.ModelInterface2(urdf)

q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()

C = Cartesian("Cartesian", model, "panda_link7", "world")
Cp = C%[0,1,2]
Co = C%[3,4,5]
CC = Cp + Co

for cc, c in zip(C.getA(), CC.getA()):
    utest.assertEqual(cc.tolist(), c.tolist())
utest.assertEqual(C.getb().tolist(), CC.getb().tolist())

p = Postural(model)

D = C + p%[0]
for i in range(0, 6):
    utest.assertEqual(D.getA()[i,:].tolist(), C.getA()[i,:].tolist())
utest.assertEqual(D.getA()[6,:].tolist(), p.getA()[0,:].tolist())
utest.assertEqual(D.getb()[0:6].tolist(), C.getb().tolist())
utest.assertEqual(D.getb()[6], p.getb()[0])

C2 = Cartesian("Cartesian2", model, "panda_link4", "world")
C3 = Cartesian("Cartesian3", model, "panda_link2", "world")

A1 = C + C2 + C3
A2 = A1 + p

for i in range(0, 6):
    utest.assertEqual(A2.getA()[i,:].tolist(), C.getA()[i,:].tolist())
utest.assertEqual(A2.getb()[0:6].tolist(), C.getb().tolist())
for i in range(6, 12):
    utest.assertEqual(A2.getA()[i,:].tolist(), C2.getA()[i-6,:].tolist())
utest.assertEqual(A2.getb()[6:12].tolist(), C2.getb().tolist())
for i in range(12, 18):
    utest.assertEqual(A2.getA()[i,:].tolist(), C3.getA()[i-12,:].tolist())
utest.assertEqual(A2.getb()[12:18].tolist(), C3.getb().tolist())
for i in range(18, 25):
    utest.assertEqual(A2.getA()[i,:].tolist(), p.getA()[i-18,:].tolist())
utest.assertEqual(A2.getb()[18:25].tolist(), p.getb().tolist())

T = 2*C
for i in range(0, 6):
    utest.assertEqual(T.getWeight()[i,:].tolist(), C.getWeight()[i,:].tolist())

W = C.getWeight()
W = 3*W
T = pysot.mul(W,C)
for i in range(0, 6):
    utest.assertEqual(T.getWeight()[i,:].tolist(), C.getWeight()[i,:].tolist())

qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
ql = qlims%[0,2,5]
print(ql.getAineq())
utest.assertEqual(ql.getbLowerBound()[0], qlims.getLowerBound()[0])
utest.assertEqual(ql.getbLowerBound()[1], qlims.getLowerBound()[2])
utest.assertEqual(ql.getbLowerBound()[2], qlims.getLowerBound()[5])
utest.assertEqual(ql.getbUpperBound()[0], qlims.getUpperBound()[0])
utest.assertEqual(ql.getbUpperBound()[1], qlims.getUpperBound()[2])
utest.assertEqual(ql.getbUpperBound()[2], qlims.getUpperBound()[5])

S = C/C2
utest.assertEqual(S.getStack()[0].getTaskID(), C.getTaskID())
utest.assertEqual(S.getStack()[1].getTaskID(), C2.getTaskID())

S1 = (C2 + C3) / p
utest.assertEqual(S1.getStack()[0].getTaskID(), (C2 + C3).getTaskID())
utest.assertEqual(S1.getStack()[1].getTaskID(), p.getTaskID())

S = S / p
utest.assertEqual(S.getStack()[0].getTaskID(), C.getTaskID())
utest.assertEqual(S.getStack()[1].getTaskID(), C2.getTaskID())
utest.assertEqual(S.getStack()[2].getTaskID(), p.getTaskID())

S2 = (C+C3) / S1
for t in S2.getStack():
    print(t.getTaskID())
utest.assertEqual(S2.getStack()[0].getTaskID(), (C+C3).getTaskID())
utest.assertEqual(S2.getStack()[1].getTaskID(), (C2+C3).getTaskID())
utest.assertEqual(S2.getStack()[2].getTaskID(), p.getTaskID())

SS = C/C2/C3
utest.assertEqual(SS.getStack()[0].getTaskID(), C.getTaskID())
utest.assertEqual(SS.getStack()[1].getTaskID(), C2.getTaskID())
utest.assertEqual(SS.getStack()[2].getTaskID(), C3.getTaskID())

SSS = pysot.hard(SS, S1)
utest.assertEqual(SSS.getStack()[0].getTaskID(), C.getTaskID())
utest.assertEqual(SSS.getStack()[1].getTaskID(), C2.getTaskID())
utest.assertEqual(SSS.getStack()[2].getTaskID(), C3.getTaskID())
utest.assertEqual(SSS.getStack()[3].getTaskID(), (C2+C3).getTaskID())
utest.assertEqual(SSS.getStack()[4].getTaskID(), p.getTaskID())


TC = C<<qlims
utest.assertEqual(TC.getConstraints()[0].getConstraintID(), qlims.getConstraintID())

dqmax = model.getVelocityLimits()
dt = 1./100.
dqlims = VelocityLimits(model, dqmax, dt)

TCC = (C2+C3)<<qlims<<dqlims
utest.assertEqual(TCC.getConstraints()[0].getConstraintID(), qlims.getConstraintID())
utest.assertEqual(TCC.getConstraints()[1].getConstraintID(), dqlims.getConstraintID())

TTC = C3<<C2
utest.assertEqual(TTC.getTaskID(), C3.getTaskID())
utest.assertEqual(TTC.getConstraints()[0].getConstraintID(), C2.getTaskID())

S1C = S1<<qlims
utest.assertEqual(S1C.getBoundsList()[0].getConstraintID(), qlims.getConstraintID())

S2C = (C2/C3)<<dqlims<<qlims
utest.assertEqual(S2C.getBoundsList()[0].getConstraintID(), dqlims.getConstraintID())
utest.assertEqual(S2C.getBoundsList()[1].getConstraintID(), qlims.getConstraintID())

S23 = (C2/C3)<<p
utest.assertEqual(S23.getBoundsList()[0].getConstraintID(), p.getTaskID())

S4 = (C/C2)/(C3/p)
S11 = C/C2
S12 = C3/p
S14 = S11/S12
for i in range(0,4):
    utest.assertEqual(S4.getStack()[i].getTaskID(), S14.getStack()[i].getTaskID())
