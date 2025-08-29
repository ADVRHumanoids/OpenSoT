import unittest
import numpy as np
from pyopensot import Task, Constraint, AffineHelper, ConstraintType, AffineHelper, OptvarHelper


class foo_var(AffineHelper):
    def __init__(self):
        super().__init__(3, 1)
        self.M = np.zeros((1,3))
        self.M[0,1] = 1.
        self.q = np.array([0.]).reshape(-1,1)

        self._M = self.M
        self._q = self.q

    def update(self):
        self.M[0,1] += 1.
        self.q[0] += 1
        self._M = self.M
        self._q = self.q


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


class T(Task):
    def __init__(self, n):
        super().__init__("T", n)   # n is the x-size
    def _update(self):
        self._A = np.zeros((1, self.getXSize()))
        self._b = np.zeros((1, ))

t = T(5)
t.update()   # should NOT throw
print(t.getA().shape, t.getb().shape)  # (1,5) (1,)


vars = list()
vars.append(("x", 4))
vars.append(("u", 2))

variables = OptvarHelper(vars)
x = variables.getVariable("x")
u = variables.getVariable("u")

w = np.array([1., 2., 3., 4., 5., 6.])

class min_var(Task):
    def __init__(self, variable):
        super().__init__("min_var", variable.getInputSize())
        self.variable = variable
        self._W = np.eye(variable.getOutputSize())
        self.update()

    def _update(self):
        self.lin =  self.variable + self.variable.getValue()
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

u.getValue(w)
minu = min_var(u)

print(f"minu.getb(): {minu.getb()}")
utest.assertTrue((minu.getb() == -w[4:]).all())

w = np.array([1., 2., 3., 4., 9., 10.])

u.getValue(w)
minu.update()

print(f"minu.getb(): {minu.getb()}")
utest.assertTrue((minu.getb() == -w[4:]).all())



vars = list()
vars.append(("q", 2))
vars.append(("qdot", 2))
vars.append(("qddot", 2))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")


x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

def euler(x, xdot, dt):
    return x + dt * xdot  # x1 = x0 + dt * xdot0

f = euler(x, xdot, 0.01)
f.getValue(w)

class dynamics_derivative(Task):
    def __init__(self, f):
        super().__init__("ddyn", f.getInputSize())
        self.f = f
        self._W = np.eye(f.getOutputSize())
        self.update()

    def _update(self):
        self.lin = self.f + self.f.getValue()
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

ddyn = dynamics_derivative(f)

print(f"ddyn.getA(): {ddyn.getA()}")
print(f"ddyn.getb(): {ddyn.getb()}")
print(f"f.getValue(): {f.getValue()}")
utest.assertTrue((ddyn.getb() == -f.getValue()).all())

w = np.array([1., 2., 3., 4., 13., 14.])
f.getValue(w)
ddyn.update()
print(f"ddyn.getA(): {ddyn.getA()}")
print(f"ddyn.getb(): {ddyn.getb()}")
print(f"f.getValue(): {f.getValue()}")
utest.assertTrue((ddyn.getb() == -f.getValue()).all())