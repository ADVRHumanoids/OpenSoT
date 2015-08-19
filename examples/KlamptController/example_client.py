import yarp
import pYTask
import PyKDL as kdl
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

#l_wrist.help()
p = l_wrist.getActualPose()
print p
p.p[2] += 0.1
l_wrist.setReference(p)

#r_wrist.help()
p = r_wrist.getActualPose()
print p
p.p[2] += 0.1
r_wrist.setReference(p)

#com.help()
p = com.getActualPosition()
print p
p[0] -= 0.01
com.setReference(p)

#postural.help()
p = postural.getActualPosture()
print p
p['HPY'] -= 0.01
postural.setReference(p)
