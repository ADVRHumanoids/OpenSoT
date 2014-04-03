#!/usr/bin/python

import yarp
import numpy as np

print "This Test creates Trajectories for left and left arm end-effectors considering fixed Position desired"

yarp.Network.init()

left_arm = yarp.BufferedPortBottle()
left_arm.open("/desired/left_arm/position/ref:o")
yarp.Network.connect("/desired/left_arm/position/ref:o", "/sot_VelKinCon/left_arm/set_ref:i")

  
left_arm_x_f = 0.117103
left_arm_y_f = 0.191341
left_arm_z_f = 0.519536

left_arm_qx_f = 0.0
left_arm_qy_f = -0.7071
left_arm_qz_f = 0.0
left_arm_qw_f = 0.7071

bottle_left = left_arm.prepare()
bottle_left.clear()    

bottle_tmp0 = bottle_left.addList()
bottle_tmp0.addString("frame")
bottle_tmp0.addString("world")        
    
bottle_tmp = bottle_left.addList()
bottle_tmp.addString("data")
bottle_tmp.addDouble(left_arm_x_f)
bottle_tmp.addDouble(left_arm_y_f)
bottle_tmp.addDouble(left_arm_z_f)
bottle_tmp.addDouble(left_arm_qx_f)
bottle_tmp.addDouble(left_arm_qy_f)
bottle_tmp.addDouble(left_arm_qz_f)
bottle_tmp.addDouble(left_arm_qw_f)
    
left_arm.write()

