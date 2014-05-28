#!/usr/bin/python

import yarp
import numpy as np

print "This Test creates Trajectories for left and right arm end-effectors considering fixed Position desired"

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

left_arm_x_f = 0.119692
left_arm_y_f = 0.189703
left_arm_z_f = 0.719815


right_arm_x_0 = 0.119692
right_arm_y_0 = -0.189703
right_arm_z_0 = 0.519815
right_arm_R_0 = -21.5
right_arm_P_0 = -63.04
right_arm_Y_0 = 22.06

r = 0.08

t_start = yarp.Time.now()
while(1):
    t = yarp.Time.now() - t_start
    l = 0.5 * (-np.cos( 0.4*t%(2 * np.pi) ) + 1)
        
    left_arm_x = left_arm_x_0 + l*(left_arm_x_f - left_arm_x_0)
    left_arm_y = left_arm_y_0 + l*(left_arm_y_f - left_arm_y_0)
    left_arm_z = left_arm_z_0 + l*(left_arm_z_f - left_arm_z_0)
    
    right_arm_x = right_arm_x_0
    right_arm_y = right_arm_y_0 + r*( np.cos(0.5*t%(2 * np.pi)) - 1 )
    right_arm_z = right_arm_z_0 + r*( np.sin(0.5*t%(2 * np.pi)) )

    bottle_left = left_arm.prepare()
    bottle_left.clear()
    
    bottle_tmp0 = bottle_left.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("world")        
    
    bottle_tmp = bottle_left.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(left_arm_x)
    bottle_tmp.addDouble(left_arm_y)
    bottle_tmp.addDouble(left_arm_z)
    bottle_tmp.addDouble(left_arm_R_0)
    bottle_tmp.addDouble(left_arm_P_0)
    bottle_tmp.addDouble(left_arm_Y_0)

    bottle_right = right_arm.prepare()
    bottle_right.clear()
    
    bottle_tmp0 = bottle_right.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("world")      
    
    bottle_tmp = bottle_right.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(right_arm_x)
    bottle_tmp.addDouble(right_arm_y)
    bottle_tmp.addDouble(right_arm_z)
    bottle_tmp.addDouble(right_arm_R_0)
    bottle_tmp.addDouble(right_arm_P_0)
    bottle_tmp.addDouble(right_arm_Y_0)
    
    left_arm.write()
    right_arm.write()
    yarp.Time.delay(0.01)

