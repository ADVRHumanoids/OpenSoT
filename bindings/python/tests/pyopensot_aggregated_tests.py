from pyopensot import GenericTask, GenericConstraint, ConstraintType, AggregatedTask, AggregatedConstraint, AffineHelper, AggregationPolicy, SubConstraint
import numpy as np
import unittest

M1 = np.array([[1, 2, 3]])
q1 = np.array([0])

c1 = GenericConstraint("c1",
                       AffineHelper(M1, q1),
                       np.array([1]), np.array([-1]),
                       ConstraintType.CONSTRAINT)
c1.update()

M2 = np.array([[4, 5, 6]])
q2 = np.array([0])

c2 = GenericConstraint("c2",
                       AffineHelper(M2, q2),
                       2*np.array([1]), 2*np.array([-1]),
                       ConstraintType.CONSTRAINT)
c2.update()

c3 = AggregatedConstraint(c1, c2, 3)

print(f"c3.getAineq(): {c3.getAineq()}")
print(f"c3.getbLowerBound(): {c3.getbLowerBound()}")
print(f"c3.getbUpperBound(): {c3.getbUpperBound()}")

c1.setBounds(4*np.array([1]), 4*np.array([-1]))
c3.update()
print(f"c3.getbLowerBound(): {c3.getbLowerBound()}")
print(f"c3.getbUpperBound(): {c3.getbUpperBound()}")

clist = []
clist.append(c1)
clist.append(c2)
clist.append(c3)
c4 = AggregatedConstraint(clist, 3)
print(f"c4.getAineq(): {c4.getAineq()}")
print(f"c4.getbLowerBound(): {c4.getbLowerBound()}")
print(f"c4.getbUpperBound(): {c4.getbUpperBound()}")

constraints = c4.getConstraintsList()
for c in constraints:
    print(c.getConstraintID())

t1 = GenericTask("t1", np.array([[1,2,3],[4,5,6]]), np.array([[1],[2]]))
t2 = GenericTask("t2", np.array([[7,8,9],[10,11,12], [13,14,15]]), np.array([[3],[4],[5]]))
t3 = AggregatedTask(t1,t2,3)
t3.update()
print(f"t3.getA(): {t3.getA()}")
print(f"t3.getb(): {t3.getb()}")
t2.setAb(0.*np.array([[7,8,9],[10,11,12], [13,14,15]]), 0.*np.array([[3],[4],[5]]))
t3.update()
print(f"t3.getA(): {t3.getA()}")
print(f"t3.getb(): {t3.getb()}")
tlist = []
tlist.append(t1)
tlist.append(t2)
tlist.append(t3)
t4 = AggregatedTask(tlist, 3)
tasks = t4.getTaskList()
for t in tasks:
    print(t.getTaskID())


print(f"c4.getA(): {c4.getAineq()}")
print(f"c4.getbUpperBound(): {c4.getbUpperBound()}")
print(f"c4.getbLowerBound(): {c4.getbLowerBound()}")
idx = [0, 2, 3]
sc4 = SubConstraint(c4, idx)
print(f"sc4.getAineq(): {sc4.getAineq()}")
print(f"sc4.getbUpperBound(): {sc4.getbUpperBound()}")
print(f"sc4.getbLowerBound(): {sc4.getbLowerBound()}")

c1.setBounds(np.array([100]), np.array([-100]))
sc4.update()
print(f"sc4.getAineq(): {sc4.getAineq()}")
print(f"sc4.getbUpperBound(): {sc4.getbUpperBound()}")
print(f"sc4.getbLowerBound(): {sc4.getbLowerBound()}")

