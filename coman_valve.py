#!/usr/bin/python

# Copyright: (C) 2010 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

import yarp

yarp.Network.init()

left_arm = yarp.BufferedPortBottle()
left_arm.open("/desired/left_arm/position/ref:o")
yarp.Network.connect("/desired/left_arm/position/ref:o", "/sot_VelKinCon/coman/left_arm/position/ref:i")

right_arm = yarp.BufferedPortBottle()
right_arm.open("/desired/right_arm/position/ref:o")
yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/coman/right_arm/position/ref:i")

# initial posture
bottle_left = left_arm.prepare()
bottle_left.clear()
bottle_left.addDouble(0.0)
bottle_left.addDouble(0.2)
bottle_left.addDouble(0.5)

bottle_right = right_arm.prepare()
bottle_right.clear()
bottle_right.addDouble(0.0)
bottle_right.addDouble(-0.2)
bottle_right.addDouble(0.5)

left_arm.write()
right_arm.write()

yarp.Time.delay(3.0)

bottle_left = left_arm.prepare()
bottle_left.clear()
bottle_left.addDouble(0.3)
bottle_left.addDouble(0.2)
bottle_left.addDouble(0.5)

bottle_right = right_arm.prepare()
bottle_right.clear()
bottle_right.addDouble(0.3)
bottle_right.addDouble(-0.2)
bottle_right.addDouble(0.5)

left_arm.write()
right_arm.write()

yarp.Time.delay(3.0)

#second posture
bottle_left = left_arm.prepare()
bottle_left.clear()
bottle_left.addDouble(0.5)
bottle_left.addDouble(0.2)
bottle_left.addDouble(0.6)

bottle_right = right_arm.prepare()
bottle_right.clear()
bottle_right.addDouble(0.5)
bottle_right.addDouble(-0.2)
bottle_right.addDouble(0.6)

left_arm.write()
right_arm.write()

yarp.Time.delay(3.0)

#closing on valve
bottle_left = left_arm.prepare()
bottle_left.clear()
bottle_left.addDouble(0.6)
bottle_left.addDouble(0.2)
bottle_left.addDouble(0.6)

bottle_right = right_arm.prepare()
bottle_right.clear()
bottle_right.addDouble(-0.25)
bottle_right.addDouble(-0.1)
bottle_right.addDouble(0.6)

left_arm.write()
right_arm.write()

yarp.Time.delay(4.0)

bottle_left = left_arm.prepare()
bottle_left.clear()
bottle_left.addDouble(0.6)
bottle_left.addDouble(-0.2)
bottle_left.addDouble(0.5)

bottle_right = right_arm.prepare()
bottle_right.clear()
bottle_right.addDouble(-0.25)
bottle_right.addDouble(0.2)
bottle_right.addDouble(0.6)

left_arm.write()
right_arm.write()

yarp.Time.delay(3.0)

left_arm.close()
right_arm.close()


yarp.Network.fini();
