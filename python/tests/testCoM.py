#!/usr/bin/python

import yarp
import numpy as np

print "This Test creates Trajectories for left and right arm end-effectors"

yarp.Network.init()

CoM = yarp.BufferedPortBottle()
CoM.open("/desired/CoM/position/ref:o")
yarp.Network.connect("/desired/CoM/position/ref:o", "/sot_VelKinCon/com/set_ref:i")

com_x_0 = 0.021886
com_y_0 = -0.125690
com_z_0 = 0.525539

com_x_f = 0.021886
com_y_f = -0.015690
com_z_f = 0.45

t_start = yarp.Time.now()
while(1):
    t = yarp.Time.now() - t_start
    l = 0.5 * (-np.cos( 0.6*t%(2 * np.pi) ) + 1)
        
    com_x = com_x_0# + l*(com_x_f - com_x_0)
    com_y = com_y_0# + l*(com_y_f - com_y_0)
    com_z = com_z_0 + l*(com_z_f - com_z_0)
    
    bottle_CoM = CoM.prepare()
    bottle_CoM.clear()
    
    bottle_tmp0 = bottle_CoM.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("l_sole")        
    
    bottle_tmp = bottle_CoM.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(com_x)
    bottle_tmp.addDouble(com_y)
    bottle_tmp.addDouble(com_z)
    bottle_tmp.addDouble(0.0)
    bottle_tmp.addDouble(0.0)
    bottle_tmp.addDouble(0.0)

    CoM.write()
    
    yarp.Time.delay(0.01)

