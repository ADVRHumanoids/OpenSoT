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
l_wrist.setLambda(0)
r_wrist.setLambda(0)
waist.setLambda(0)
com.setLambda(0)

p = postural.getActualPosture()
print p

# moving left arm

for joint in ["LSP", "LSR", "LSY", "LEP", "LWY", "LWP"]:
    print "Moving joint", joint
    p[joint] += 0.1
    postural.setReference(p)
    time.sleep(2)
