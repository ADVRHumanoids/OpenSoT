import yarp
import pYTask
import PyKDL as kdl

yarp.Network.init()
l_wrist = pYTask.CartesianTask('huboplus_klampt_controller',
                               'huboplus',
                               'cartesian::l_wrist')

com = pYTask.CoMTask('huboplus_klampt_controller',
                     'huboplus',
                     'CoM')

postural = pYTask.PosturalTask('huboplus_klampt_controller',
                               'huboplus',
                               'Postural')

#l_wrist.help()
print l_wrist.getActualPose()

#com.help()
print com.getActualPosition()

#postural.help()
print postural.getActualPosture()
