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

Aineq = np.vstack([M1, M2])
bLowerBound = np.array([-1, -2])
bUpperBound = np.array([1, 2])
print(f"Aineq(): {Aineq}")
print(f"bLowerBound(): {bLowerBound}")
print(f"bUpperBound(): {bUpperBound}")

utest = unittest.TestCase()
utest.assertTrue((c3.getAineq() == Aineq).all())
utest.assertTrue((c3.getbLowerBound() == bLowerBound).all())
utest.assertTrue((c3.getbUpperBound() == bUpperBound).all())



c1.setBounds(4*np.array([1]), 4*np.array([-1]))
c3.update()
print(f"c1.getbLowerBound(): {c1.getbLowerBound()}")
print(f"c1.getbUpperBound(): {c1.getbUpperBound()}")

print(f"c3.getbLowerBound(): {c3.getbLowerBound()}")
print(f"c3.getbUpperBound(): {c3.getbUpperBound()}")

utest.assertTrue((c1.getbLowerBound() == c3.getbLowerBound()[0]).all())
utest.assertTrue((c1.getbUpperBound() == c3.getbUpperBound()[0]).all())

#
clist = []
clist.append(c1)
clist.append(c2)

M4 = np.array([[7, 8, 9]])
q4 = np.array([0])

c4 = GenericConstraint("c4",
                       AffineHelper(M4, q4),
                       4*np.array([1]), 4*np.array([-1]),
                       ConstraintType.CONSTRAINT)
c4.update()
clist.append(c4)

c5 = AggregatedConstraint(clist, 3)
print(f"c5.getAineq(): {c5.getAineq()}")
print(f"c5.getbLowerBound(): {c5.getbLowerBound()}")
print(f"c5.getbUpperBound(): {c5.getbUpperBound()}")

utest.assertTrue((clist[0].getAineq() == c5.getAineq()[0,:]).all())
utest.assertTrue((clist[0].getbLowerBound() == c5.getbLowerBound()[0]).all())
utest.assertTrue((clist[0].getbUpperBound() == c5.getbUpperBound()[0]).all())

utest.assertTrue((clist[1].getAineq() == c5.getAineq()[1,:]).all())
utest.assertTrue((clist[1].getbLowerBound() == c5.getbLowerBound()[1]).all())
utest.assertTrue((clist[1].getbUpperBound() == c5.getbUpperBound()[1]).all())

utest.assertTrue((clist[2].getAineq() == c5.getAineq()[2,:]).all())
utest.assertTrue((clist[2].getbLowerBound() == c5.getbLowerBound()[2]).all())
utest.assertTrue((clist[2].getbUpperBound() == c5.getbUpperBound()[2]).all())

c4.setBounds(12*np.array([1]), 12*np.array([-1]))
c5.update()
print(f"c5.getbLowerBound(): {c5.getbLowerBound()}")
print(f"c5.getbUpperBound(): {c5.getbUpperBound()}")
utest.assertTrue((clist[2].getbLowerBound() == c5.getbLowerBound()[2]).all())
utest.assertTrue((clist[2].getbUpperBound() == c5.getbUpperBound()[2]).all())


#
constraints = c5.getConstraintsList()
for c in constraints:
    print(c5.getConstraintID())
#


t1 = GenericTask("t1", np.array([[1,2,3],[4,5,6]]), np.array([[1],[2]]))
t2 = GenericTask("t2", np.array([[7,8,9],[10,11,12], [13,14,15]]), np.array([[3],[4],[5]]))
t3 = AggregatedTask(t1,t2,3)
t3.update()
print(f"t3.getA(): {t3.getA()}")
print(f"t3.getb(): {t3.getb()}")

utest.assertTrue((t1.getA() == t3.getA()[0:2, :]).all())
utest.assertTrue((t1.getb() == t3.getb()[0:2]).all())

utest.assertTrue((t2.getA() == t3.getA()[2:5, :]).all())
utest.assertTrue((t2.getb() == t3.getb()[2:5]).all())

t2.setAb(0.*np.array([[7,8,9],[10,11,12], [13,14,15]]), 0.*np.array([[3],[4],[5]]))
t3.update()
print(f"t3.getA(): {t3.getA()}")
print(f"t3.getb(): {t3.getb()}")

utest.assertTrue((t1.getA() == t3.getA()[0:2, :]).all())
utest.assertTrue((t1.getb() == t3.getb()[0:2]).all())

utest.assertTrue((t2.getA() == t3.getA()[2:5, :]).all())
utest.assertTrue((t2.getb() == t3.getb()[2:5]).all())


tlist = []
tlist.append(t1)
tlist.append(t2)

t4 = GenericTask("t4", np.array([[7,8,9]]), np.array([10]))
tlist.append(t4)
t5 = AggregatedTask(tlist, 3)
t5.update()
tasks = t5.getTaskList()
for t in tasks:
    print(t.getTaskID())
#
#
print(f"t5.getA(): {t5.getA()}")
print(f"t5.getb(): {t5.getb()}")

utest.assertTrue((t1.getA() == t5.getA()[0:2, :]).all())
utest.assertTrue((t1.getb() == t5.getb()[0:2]).all())

utest.assertTrue((t2.getA() == t5.getA()[2:5, :]).all())
utest.assertTrue((t2.getb() == t5.getb()[2:5]).all())

utest.assertTrue((t4.getA() == t5.getA()[5:6, :]).all())
utest.assertTrue((t4.getb() == t5.getb()[5:6]).all())

t1.setb(np.array([[10],[20]]))
t5.update()

print(f"t1.getA(): {t1.getA()}")
print(f"t1.getb(): {t1.getb()}")
print(f"t5.getA(): {t5.getA()}")
print(f"t5.getb(): {t5.getb()}")

utest.assertTrue((t1.getA() == t5.getA()[0:2, :]).all())
utest.assertTrue((t1.getb() == t5.getb()[0:2]).all())


idx = [0, 2]
sc5 = SubConstraint(c5, idx)
print(f"sc5.getAineq(): {sc5.getAineq()}")
print(f"sc5.getbUpperBound(): {sc5.getbUpperBound()}")
print(f"sc5.getbLowerBound(): {sc5.getbLowerBound()}")

for i in range(2):
    utest.assertTrue((sc5.getAineq()[i,:] == c5.getAineq()[idx[i],:]).all())
    utest.assertTrue((sc5.getbLowerBound()[i] == c5.getbLowerBound()[idx[i]]).all())
    utest.assertTrue((sc5.getbUpperBound()[i] == c5.getbUpperBound()[idx[i]]).all())

#
c1.setBounds(np.array([100]), np.array([-100]))
sc5.update()
print(f"sc5.getAineq(): {sc5.getAineq()}")
print(f"sc5.getbUpperBound(): {sc5.getbUpperBound()}")
print(f"sc5.getbLowerBound(): {sc5.getbLowerBound()}")

utest.assertTrue((sc5.getbLowerBound()[0] == c1.getbLowerBound()).all())
utest.assertTrue((sc5.getbUpperBound()[0] == c1.getbUpperBound()).all())
#
c1_joint_mask = c1.getActiveJointsMask()
print(f"c1_joint_mask: {c1_joint_mask}")
c1_joint_mask[1] = False
c1.setActiveJointsMask(c1_joint_mask)
print(f"c1_joint_mask: {c1.getActiveJointsMask()}")
#
c5.update()
print(f"c5.getAineq(): {c5.getAineq()}")
utest.assertTrue((c5.getAineq()[0,1] == False).all())
#
c5.setActiveJointsMask(c1_joint_mask)
c5.update()
print(f"c5.getAineq(): {c5.getAineq()}")
utest.assertTrue((c5.getAineq()[:,1] == False).all())


