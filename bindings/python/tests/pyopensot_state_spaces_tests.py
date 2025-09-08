from pyopensot_oc import *
import unittest
import numpy as np


R3 = VectorSpace(3)

utest = unittest.TestCase()
utest.assertTrue(R3.nq() == 3)
utest.assertTrue(R3.nv() == 3)

print(f"R3.nq: {R3.nq()}")
print(f"R3.nv: {R3.nv()}")

Quaternion = QuaternionSpace()
utest.assertTrue(Quaternion.nq() == 4)
utest.assertTrue(Quaternion.nv() == 3)

print(f"SO3.nq: {Quaternion.nq()}")
print(f"SO3.nv: {Quaternion.nv()}")
#

SE3 = CompositeSpace([R3, Quaternion])
print(f"SE3.nq: {SE3.nq()}")
print(f"SE3.nv: {SE3.nv()}")
utest.assertTrue(SE3.nq() == 7)
utest.assertTrue(SE3.nv() == 6)
#
RSS = CompositeSpace([SE3, VectorSpace(10)])
print(f"RSS.nq: {RSS.nq()}")
print(f"RSS.nv: {RSS.nv()}")
utest.assertTrue(RSS.nq() == 7+10)
utest.assertTrue(RSS.nv() == 6+10)



x = np.array([1,1,1])
print(f"x: {x}")
dx = np.array([1,2,3])
print(f"dx: {dx}")

x1 = R3.integrate(x, dx)
print(f"integrated x: {x1}")
utest.assertTrue((x1 == x+dx).all())

R7 = CompositeSpace([R3, VectorSpace(4)])
x = np.array([1,1,1, 1,2,3,4])
print(f"x: {x}")
dx = np.array([1,2,3, 4,5,6,7])
print(f"dx: {dx}")
x1 = R7.integrate(x, dx)
print(f"integrated x: {x1}")
utest.assertTrue((x1 == x+dx).all())


# ------------------------

QuatSpace = QuaternionSpace()

q = np.array([1,0,0,0]) 
print(q)
dw = np.array([0, np.pi, 0])
print(dw)
QuatSpace.integrate(q,dw)


print("[Quat,R3]")
SE3 = CompositeSpace([Quaternion, R3])
x = np.array([1,0,0,0, 0,2,1])
print(f"x: {x}")
dx = np.array([0, 0.1, 0, 1,1,1])
print(f"dx: {dx}")
x1 = SE3.integrate(x, dx)
print(f"integrated x: {x1}")

print("[R3,Quat]")
SE3 = CompositeSpace([R3, Quaternion])
x = np.array([0,2,1, 0,0,0,0])
print(f"x: {x}")
dx = np.array([1,1,1, 0, 0, np.pi])
print(f"dx: {dx}")
x1 = SE3.integrate(x, dx)
print(f"integrated x: {x1}")