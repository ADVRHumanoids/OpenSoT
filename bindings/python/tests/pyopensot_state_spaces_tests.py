from pyopensot_oc import *
import unittest
import numpy as np


# R3 = VectorSpace(3)

# utest = unittest.TestCase()
# utest.assertTrue(R3.nq() == 3)
# utest.assertTrue(R3.nv() == 3)

# print(f"R3.nq: {R3.nq()}")
# print(f"R3.nv: {R3.nv()}")


SE3 = SE3Space()
x0 = np.array([0,0,0,0,0,0,1])
dx = np.array([0,0,0,0,0,np.pi])

print(SE3.integrate(x0, dx))

