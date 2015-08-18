import yarp
import pYTask
import PyKDL as kdl
from example_trajectories import *
import time

yarp.Network.init()

waist = pYTask.CartesianTask(  'huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::waist',
                               'world', 'Body_Hip')

l_wrist = pYTask.CartesianTask('huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::l_wrist',
                               'world', 'Body_LWP')

r_wrist = pYTask.CartesianTask('huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::r_wrist',
                               'world', 'Body_RWP')

com = pYTask.CoMTask('huboplus_klampt_controller',
                     'huboplus',
                     'CoM')

postural = pYTask.PosturalTask('huboplus_klampt_controller',
                               'huboplus',
                               'Postural')

ee_pose = l_wrist.getActualPose()
center = kdl.Frame(ee_pose)
center.M.DoRotY(np.pi/2.0)
center.p[2] -= 0.075
duration = 10.0
dT = 0.01
traj_l = CircularTrajectory(duration, ee_pose, center)

ee_pose = r_wrist.getActualPose()
ee_desired = kdl.Frame(ee_pose)
ee_desired.p[2] += 0.2
traj_r = LinearTrajectory(duration/2.0, ee_pose, ee_desired, loop=True)

t = 0
while t <= duration:
    des_l = traj_l.Pos(t)
    l_wrist.setReference(des_l)
    des_r = traj_r.Pos(t)
    r_wrist.setReference(des_r)
    if float(int(t)) == t:
        print "l/r wrist traj @t", t
    time.sleep(1.1*dT)
    t += dT


com_position = com.getActualPosition()
com_pose_fake = kdl.Frame()
com_pose_fake.p = com_position
center = kdl.Frame(com_pose_fake)
center.p[0] += 0.03
duration = 10.0
dT = 0.01
traj_com = CircularTrajectory(duration, com_pose_fake, center)

time.sleep(6.0)
t = 0
print "Moving CoM in circle."
print "Initial CoM position:", com_position
des_com = com_pose_fake
while t <= duration:
    des_com = traj_com.Pos(t)
    com.setReference(des_com.p)
    if float(int(t)) == t:
        print "CoM traj @t", t
    time.sleep(1.2*dT)
    t+=dT
print "Final CoM reference:", des_com.p

#com.help()
#p = com.getActualPosition()
#print p
#p[0] -= 0.45
#com.setReference(p)

#postural.help()
#p = postural.getActualPosture()
#print p
#p['HPY'] -= 0.02
#postural.setReference(p)
