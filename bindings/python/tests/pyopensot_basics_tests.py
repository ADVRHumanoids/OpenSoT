from pyopensot import GenericTask, GenericConstraint, AffineHelper, ConstraintType
import numpy as np
import unittest

M = np.array([[1, 2, 3],[4, 5, 6]])
q = np.array([1, 2])
v = AffineHelper(M, q)
print(f"v: {v}")

min = np.array([-10., -5.])
max = np.array([10., 5.])
c = GenericConstraint("foo", v, max, min, ConstraintType.CONSTRAINT)
c.update()

print(f"c.getAineq(): {c.getAineq()}")
print(f"c.getbLowerBound(): {c.getbLowerBound()}")
print(f"c.getbUpperBound(): {c.getbUpperBound()}")

utest = unittest.TestCase()
utest.assertTrue((c.getAineq() == v.getM()).all())
utest.assertTrue((c.getbLowerBound() == min - v.getq()).all())
utest.assertTrue((c.getbUpperBound() == max - v.getq()).all())

min *= 0.
max *= 0.
c.setBounds(max, min)
c.update()
print(f"c.getAineq(): {c.getAineq()}")
print(f"c.getbLowerBound(): {c.getbLowerBound()}")
print(f"c.getbUpperBound(): {c.getbUpperBound()}")

utest.assertTrue((c.getAineq() == v.getM()).all())
utest.assertTrue((c.getbLowerBound() == min - v.getq()).all())
utest.assertTrue((c.getbUpperBound() == max - v.getq()).all())

M = np.array([[1, 0, 0],[0, 1, 0], [0, 0, 1]])
q = np.array([0, 0, 0])
v = AffineHelper(M, q)

A = np.random.rand(3,3)
b = np.random.rand(3,1)
print(f"A: {A}")
print(f"b: {b}")

t = GenericTask("foo", A, b, v)
t.update()
print(f"t.getA(): {t.getA()}")
print(f"t.getb(): {t.getb()}")

A = np.identity(3)
b = np.array([0,0,0])
t.setAb(A, b)
t.update()
print(f"t.getA(): {t.getA()}")
print(f"t.getb(): {t.getb()}")

