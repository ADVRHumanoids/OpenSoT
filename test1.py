#!/usr/bin/python

import yarp
import numpy as np

print "This Test creates Trajectories for left and right arm end-effectors considering fixed Orientation desired"

yarp.Network.init()

left_arm = yarp.BufferedPortBottle()
left_arm.open("/desired/left_arm/position/ref:o")
yarp.Network.connect("/desired/left_arm/position/ref:o", "/sot_VelKinCon/left_arm/set_ref:i")

right_arm = yarp.BufferedPortBottle()
right_arm.open("/desired/right_arm/position/ref:o")
yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/right_arm/set_ref:i")

left_arm_x_0 = 0.119692
left_arm_y_0 = 0.189703
left_arm_z_0 = 0.519815
left_arm_R_0 = 21.5
left_arm_P_0 = -63.04
left_arm_Y_0 = -22.06

left_arm_R_f = 0.0
left_arm_P_f = -90.0
left_arm_Y_f = 0.0


right_arm_x_0 = 0.119692
right_arm_y_0 = -0.189703
right_arm_z_0 = 0.519815
right_arm_R_0 = -21.5
right_arm_P_0 = -63.04
right_arm_Y_0 = 22.06

right_arm_R_f = 0.0
right_arm_P_f = 0.0
right_arm_Y_f = 0.0

r = 0.08

t_start = yarp.Time.now()
while(1):
    t = yarp.Time.now() - t_start
    l = 0.5 * (-np.cos( 0.4*t%(2 * np.pi) ) + 1)
            
    left_arm_R = left_arm_R_0 + l*(left_arm_R_f - left_arm_R_0)
    left_arm_P = left_arm_P_0 + l*(left_arm_P_f - left_arm_P_0)
    left_arm_Y = left_arm_Y_0 + l*(left_arm_Y_f - left_arm_Y_0)  
    
    right_arm_R = right_arm_R_0 + l*(right_arm_R_f - right_arm_R_0)
    right_arm_P = right_arm_P_0 + l*(right_arm_P_f - right_arm_P_0)
    right_arm_Y = right_arm_Y_0 + l*(right_arm_Y_f - right_arm_Y_0)  
    
    bottle_left = left_arm.prepare()
    bottle_left.clear()
    
    bottle_tmp0 = bottle_left.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("world")        
    
    bottle_tmp = bottle_left.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(left_arm_x_0)
    bottle_tmp.addDouble(left_arm_y_0)
    bottle_tmp.addDouble(left_arm_z_0)
    bottle_tmp.addDouble(left_arm_R)
    bottle_tmp.addDouble(left_arm_P)
    bottle_tmp.addDouble(left_arm_Y)

    bottle_right = right_arm.prepare()
    bottle_right.clear()
    
    bottle_tmp0 = bottle_right.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("world")      
    
    bottle_tmp = bottle_right.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(right_arm_x_0)
    bottle_tmp.addDouble(right_arm_y_0)
    bottle_tmp.addDouble(right_arm_z_0)
    bottle_tmp.addDouble(right_arm_R)
    bottle_tmp.addDouble(right_arm_P)
    bottle_tmp.addDouble(right_arm_Y)
    
    left_arm.write()
    right_arm.write()
    yarp.Time.delay(0.01)

