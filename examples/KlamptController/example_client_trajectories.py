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


# Defining trajectory constants
duration = 20.0
dT = 0.01
settling_time = 5.0
realtime_factor = 0.2

move_waist = True
move_hands = True
move_l_hand = True
move_r_hand = True
move_com = True

if move_waist:
    # Setting up Waist Trajectory
    waist_pose = waist.getActualPose()
    waist_pose_desired = kdl.Frame(waist_pose)
    waist_pose_desired.p[2] -= 0.15
    traj_waist = LinearTrajectory(duration/2.0, waist_pose, waist_pose_desired, loop=True)
    print "Moving waist up and down."
    print "Initial Waist position:\n", waist_pose.p
    t = 0
    des_waist = None
    while t <= duration:
        des_waist = traj_waist.Pos(t)
        waist.setReference(des_waist)
        if np.abs(int(t) - t) < dT:
            print "waist traj @t", t, "\n", des_waist.p
        time.sleep(dT/realtime_factor)
        t+=dT
    time.sleep(settling_time)
    print "Final waist reference:\n", des_waist.p
    print "Final waist position:\n", waist.getActualPose().p
    time.sleep(6.0)

if move_hands:
    # Setting up l_wrist / r_wrist reference
    duration = 30.0
    print "Moving l_wrist in circle, r_wrist in up-down line"
    ee_pose = l_wrist.getActualPose()
    center = kdl.Frame(ee_pose)
    center.M = kdl.Rotation.Identity()
    center.M.DoRotY(np.pi/2.0)
    center.p[2] += 0.05
    traj_l = CircularTrajectory(duration, ee_pose, center)
    print "Initial l_wrist  pose:\n", ee_pose

    ee_pose = r_wrist.getActualPose()
    ee_desired = kdl.Frame(ee_pose)
    ee_desired.p[2] += 0.2
    traj_r = LinearTrajectory(duration/2.0, ee_pose, ee_desired, loop=True)
    print "Initial r_wrist  pose:\n", ee_pose

    t = 0
    des_l = None
    des_r = None
    while t <= duration:
        des_l = traj_l.Pos(t)
        if move_l_hand:
            l_wrist.setReference(des_l)
        des_r = traj_r.Pos(t)
        if move_r_hand:
            r_wrist.setReference(des_r)
        if np.abs(int(t) - t) < dT:
            print "l/r wrist traj @t", t, "\nl:", des_l.p, "\nr:", des_r.p
        time.sleep(dT/realtime_factor)
        t += dT
    time.sleep(settling_time)
    print "Final l_wrist  reference:\n", des_l
    print "Final l_wrist pose:\n", l_wrist.getActualPose()
    print "Final r_wrist  reference:\n", des_r
    print "Final r_wrist pose:\n", r_wrist.getActualPose()
    time.sleep(6.0)

if move_com:
    # Setting up com trajectory
    duration = 20.0
    com_position = com.getActualPosition()
    com_pose_fake = kdl.Frame()
    com_pose_fake.p = com_position
    center = kdl.Frame(com_pose_fake)
    center.p[0] += 0.03
    duration = 10.0
    dT = 0.01
    traj_com = CircularTrajectory(duration, com_pose_fake, center)

    time.sleep(6.0)
    print "Moving CoM in circle."
    print "Initial CoM position: ", com_position

    t = 0
    des_com = com_pose_fake
    while t <= duration:
        des_com = traj_com.Pos(t)
        com.setReference(des_com.p)
        if np.abs(int(t) - t) < dT:
            print "CoM traj @t", t, ": ", des_com.p
        time.sleep(dT/realtime_factor)
        t+=dT
    time.sleep(settling_time)
    print "Final CoM reference:", des_com.p
    print "Final CoM position:", com.getActualPosition()

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
