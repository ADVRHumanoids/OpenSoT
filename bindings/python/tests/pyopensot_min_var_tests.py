from pyopensot.tasks import MinimizeVariable
import pyopensot as pysot
import numpy as np

var_dict = {}
var_dict["a"] = 3
var_dict["b"] = 3
variables = pysot.OptvarHelper(var_dict)

min_a = MinimizeVariable("a", variables.getVariable("a"))
min_b = MinimizeVariable("b", variables.getVariable("b"))
min_b.setReference(np.array([1,2,3]))

stack = pysot.AutoStack(variables.getSize())
stack /= (min_a + min_b)
solver = pysot.iHQP(stack)

stack.update()

x = solver.solve()
a_val = variables.getVariable("a").getValue(x)
b_val = variables.getVariable("b").getValue(x)

print(f"a ref: {min_a.getReference()}")
print(f"a: {a_val}")
print(f"b ref: {min_b.getReference()}")
print(f"b: {b_val}")

min_a.setReference(np.array([1,2,3]))

stack.update()

x = solver.solve()
a_val = variables.getVariable("a").getValue(x)
b_val = variables.getVariable("b").getValue(x)

print(f"a ref: {min_a.getReference()}")
print(f"a: {a_val}")
print(f"b ref: {min_b.getReference()}")
print(f"b: {b_val}")


