import yarp
import pYTask
import PyKDL as kdl

yarp.Network.init()
l_wrist = pYTask.CartesianTask('huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::l_wrist',
                               'world', 'Body_LWP')

com = pYTask.CoMTask('huboplus_klampt_controller',
                     'huboplus',
                     'CoM')

postural = pYTask.PosturalTask('huboplus_klampt_controller',
                               'huboplus',
                               'Postural')

#l_wrist.help()
p = l_wrist.getActualPose()
print p
p.p[1] -= 0.20
l_wrist.setReference(p)

#com.help()
p = com.getActualPosition()
print p
p[0] -= 0.45
com.setReference(p)

#postural.help()
p = postural.getActualPosture()
print p
p['HPY'] -= 0.02
postural.setReference(p)
