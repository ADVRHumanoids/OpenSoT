import yarp
import pYTask
import PyKDL as kdl

yarp.Network.init()
l_wrist = pYTask.CartesianTask('huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::l_wrist',
                               'world', 'l_wrist')

com = pYTask.CoMTask('huboplus_klampt_controller',
                     'huboplus',
                     'CoM')

postural = pYTask.PosturalTask('huboplus_klampt_controller',
                               'huboplus',
                               'Postural')

#l_wrist.help()
p = l_wrist.getActualPose()
print p
p.p[0] += 0.01
l_wrist.setReference(p)

#com.help()
p = com.getActualPosition()
print p
p.p[0] += 0.01
com.setReference(p)

#postural.help()
p = postural.getActualPosture()
print p
p['HPY']+=0.2
postural.setReference(p)
