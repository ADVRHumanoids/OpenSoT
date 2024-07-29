from pyopensot import AffineHelper, OptvarHelper
import numpy as np
import unittest

M1 = np.array([[1, 2, 3],[4, 5, 6]])
q1 = np.array([1, 2])
V1 = AffineHelper(M1, q1)
print(f"V1: {V1}")

M2 = 2 * np.array([[1, 2, 3],[4, 5, 6]])
q2 = 2 * np.array([1, 2])
V2 = AffineHelper(M2, q2)

V3 = V1 - V2
print(f"M3: {V3.getM()}")
print(f"q3: {V3.getq()}")

utest = unittest.TestCase()
utest.assertTrue((V3.getM() == (M1 - M2)).all())
utest.assertTrue((V3.getq() == (q1 - q2)).all())

v = np.array([10, 22])
V4 = V1 - v
print(f"M4: {V4.getM()}")
print(f"q4: {V4.getq()}")
utest.assertTrue((V4.getM() == M1).all())
utest.assertTrue((V4.getq() == (q1 - v)).all())

V5 = V1 + V2
utest.assertTrue((V5.getM() == (M1 + M2)).all())
utest.assertTrue((V5.getq() == (q1 + q2)).all())

variables_vec = dict()
variables_vec["qddot"] = 23;
variables_vec["f1"] = 3;
variables_vec["f2"] = 3;

variables = OptvarHelper(variables_vec)
qddot = variables.getVariable("qddot")
print(f"qddot: {qddot}")
f1 = variables.getVariable("f1")
print(f"qddot: {f1}")
print(f"size: {variables.getSize()}")
vv = variables.getAllVariables()
for v in vv:
    print(v)
