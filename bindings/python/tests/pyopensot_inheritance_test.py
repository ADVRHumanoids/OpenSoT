import unittest
import numpy as np
from pyopensot import Task, Constraint, AffineHelper, ConstraintType


class foo_var(AffineHelper):
    def __init__(self):
        super().__init__(3, 1)
        self._M = np.zeros((1,3))
        self._M[0,1] = 1.
        self._q = np.array([0.]).reshape(-1,1)

        self.setM(self._M)
        self.setq(self._q)

    def update(self):
        self._M[0,1] += 1.
        self._q[0] += 1
        self.setM(self._M)
        self.setq(self._q)

class foo_task(Task):
    def __init__(self, variable):
        super().__init__("fooTask", variable.getInputSize())
        self.variable = variable
        self._W = np.eye(variable.getOutputSize())
        self.update()


    def _update(self):
        self.variable.update()
        self._A = self.variable.getM()
        self._b = -self.variable.getq()

class foo_constraint(Constraint):
    def __init__(self, variable, lims):
        super().__init__("fooConstraint", variable.getInputSize())
        self.variable = variable
        self.lims = lims
        self.update()

    def _update(self):
        self._Aineq = self.variable.getM()
        self._bLowerBound = -self.variable.getq() -self.lims
        self._bUpperBound = -self.variable.getq() + self.lims



foo = foo_var()
M = foo.getM()
q = foo.getq()
print(f"foo size: {foo.getOutputSize()} x {foo.getInputSize()}")

foo.update()
M[0,1] += 1.
q[0] += 1.

utest = unittest.TestCase()
utest.assertTrue((foo.getM() == M).all())
utest.assertTrue((foo.getq() == q).all())

fooTask = foo_task(foo)
utest.assertTrue((fooTask.getA() == foo.getM()).all())
utest.assertTrue((fooTask.getb() == -foo.getq()).all())
print(f"fooTask.W: {fooTask.getWeight()}")

fooTask.update()
utest.assertTrue((fooTask.getA() == foo.getM()).all())
utest.assertTrue((fooTask.getb() == -foo.getq()).all())

print(f"fooTask.getA(): {fooTask.getA()}")
print(f"fooTask.getb(): {fooTask.getb()}")

lims = 10.
fooConstraint = foo_constraint(foo, lims)
utest.assertTrue((fooConstraint.getAineq() == foo.getM()).all())
utest.assertTrue((fooConstraint.getbLowerBound() == -foo.getq() - lims).all())
utest.assertTrue((fooConstraint.getbUpperBound() == -foo.getq() + lims).all())

fooTask.update()
utest.assertTrue((fooTask.getA() == foo.getM()).all())
utest.assertTrue((fooTask.getb() == -foo.getq()).all())
fooConstraint.update()
utest.assertTrue((fooConstraint.getAineq() == foo.getM()).all())
utest.assertTrue((fooConstraint.getbLowerBound() == -foo.getq() - lims).all())
utest.assertTrue((fooConstraint.getbUpperBound() == -foo.getq() + lims).all())


