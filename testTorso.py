#!/usr/bin/python

import yarp
import numpy as np

print "This Test is used to check motion of the torso for a task involving left and right arm."

yarp.Network.init()

right_arm = yarp.BufferedPortBottle()
right_arm.open("/desired/right_arm/position/ref:o")
yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/right_arm/set_ref:i")

  
#right_arm_x_f = 0.203
#right_arm_y_f = -0.18
#right_arm_z_f = 0.61
right_arm_x_f = 0.11
right_arm_y_f = -0.18
right_arm_z_f = 0.51

right_arm_qx_f = -0.477
right_arm_qy_f = -0.4
right_arm_qz_f = 0.589
right_arm_qw_f = 0.516

bottle_right = right_arm.prepare()
bottle_right.clear()    

bottle_tmp0 = bottle_right.addList()
bottle_tmp0.addString("frame")
bottle_tmp0.addString("world")        
    
bottle_tmp = bottle_right.addList()
bottle_tmp.addString("data")
bottle_tmp.addDouble(right_arm_x_f)
bottle_tmp.addDouble(right_arm_y_f)
bottle_tmp.addDouble(right_arm_z_f)
bottle_tmp.addDouble(right_arm_qx_f)
bottle_tmp.addDouble(right_arm_qy_f)
bottle_tmp.addDouble(right_arm_qz_f)
bottle_tmp.addDouble(right_arm_qw_f)
    
right_arm.write()
