#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""paramsTuner.py: tunes parameters for module, using paramHelp.
For now it supports just scalar parameters."""

__author__      = "Alessio Rocchi"
__email__       = "alessio.rocchi@iit.it"
__license__ = "LGPL"
__version__ = "2.1"
__copyright__   = "Copyright 2014, Walkman Consortium"

import sys
import yarp
import numpy
from gi.repository import Gtk

class paramsTuner:
    def onDeleteWindow(self, *args):
        Gtk.main_quit()

    def onValueChangedInt(self, widget):
        # widget sits in an hbox which sits in a vbox which sits in an expander
        expander = widget.get_parent().get_parent().get_parent()
        self.setInt(expander.get_label(),int(widget.get_value()))

    def onValueChangedDouble(self, widget):
        # widget sits in a vbox which sits in an expander
        expander = widget.get_parent().get_parent().get_parent()
        self.setDouble(expander.get_label(),widget.get_value())

    def onValueChangedBool(self, widget):
        # widget sits in a vbox which sits in a vbox which sits in an expander
        expander = widget.get_parent().get_parent()
        self.setBool(expander.get_label(),widget.get_active())

    def setDouble(self, name, value):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["set",name])
        cmd.addDouble(value)
        self.paramPort.write(cmd,ans)
        return ans

    def setInt(self, name, value):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["set",name])
        cmd.addInt(value)
        self.paramPort.write(cmd,ans)
        return ans

    def setBool(self, name, value):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["set",name])
        if value == True:
            cmd.addInt(1)
        else:
            cmd.addInt(0)
        self.paramPort.write(cmd,ans)
        return ans

    def getDouble(self, name):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["get",name])
        self.paramPort.write(cmd,ans)
        if ans.get(0).isDouble():
            return ans.get(0).asDouble()
        else:
            raise Exception('error accessing',name,':not a double')

    def getInt(self, name):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["get",name])
        self.paramPort.write(cmd,ans)
        if ans.get(0).isInt():
            return ans.get(0).asInt()
        else:
            raise Exception('error accessing',name,':not an int')

    def getBool(self, name):
        cmd = yarp.Bottle()
        ans = yarp.Bottle()
        cmd.clear()
        ans.clear()
        map(cmd.addString,["get",name])
        self.paramPort.write(cmd,ans)
        if ans.get(0).isInt():
            return ans.get(0).asInt() == 1
        else:
            raise Exception('error accessing',name,':not an int')

################ @TODO FUTURE IMPLEMENTATION: add cycling #####################

    def onValuesChangedBool(self, *args):
        print "unimplemented"

    def onValuesChangedDouble(self, data):
        print "unimplemented"

    def onValuesChangedInt(self, *args):
        print "unimplemented"

    def setDoubles(self, values):
        print "unimplemented"

    def setInts(self, values):
        print "unimplemented"

    def setBools(self, values):
        print "unimplemented"
#################################################################################

    def expanderContainsBool(self, widget):
        tentativeCheckButton = widget.get_children()[0].get_children()[0]
        if type(tentativeCheckButton) is Gtk.CheckButton:
            return tentativeCheckButton
        else:
            return False

    def expanderContainsDouble(self, widget):
        try:
            tentativeSpinButton = widget.get_children()[0].get_children()[0].get_children()[1]
            if type(tentativeSpinButton) is Gtk.SpinButton:
                step_increment = tentativeSpinButton.get_adjustment().get_step_increment()
                if numpy.round(step_increment) != step_increment:
                    return tentativeSpinButton
            return False
        except:
            return False

    def expanderContainsInt(self, widget):
        try:
            tentativeSpinButton = widget.get_children()[0].get_children()[0].get_children()[1]
            if type(tentativeSpinButton) is Gtk.SpinButton:
                step_increment = tentativeSpinButton.get_adjustment().get_step_increment()
                if numpy.round(step_increment) == step_increment:
                    return tentativeSpinButton
            return False
        except:
            return False

    def init_parameters(self):
        objs = self.builder.get_objects()
        for obj in objs:
            if type(obj) is Gtk.Expander:
                checkbox = self.expanderContainsBool(obj)
                if checkbox:
                    # @TODO we should cycle here
                    checkbox.set_active(self.getBool(obj.get_label()))
                    print "Initializing Checkbox",obj.get_label()

                spinbutton = self.expanderContainsDouble(obj)
                if spinbutton:
                    # @TODO we should cycle here
                    spinbutton.set_value(self.getDouble(obj.get_label()))
                    print "Initializing Double SpinBox",obj.get_label()

                spinbutton = self.expanderContainsInt(obj)
                if spinbutton:
                    # @TODO we should cycle here
                    spinbutton.set_value(self.getInt(obj.get_label()))
                    print "Initializing Int SpinBox",obj.get_label()


    def __init__(self, paramPort):
        self.paramPort = paramPort

        self.rf = yarp.ResourceFinder()
        self.rf.setVerbose(True)
        self.rf.setDefaultContext("sot_velkincon")

        self.builder = Gtk.Builder()
        self.builder.add_from_file(self.rf.findFileByName('tuner.xml'))

        self.window = self.builder.get_object("parameterTuning")
        self.init_parameters()
        self.builder.connect_signals(self)


if __name__ == "__main__":

    if(len(sys.argv) != 2):
        print 'Example usage: python showdata.py "moduleName"'
        exit(1)
    moduleName = sys.argv[1]
    yarp.Network.init()
    streamer = yarp.RpcClient()

    if not streamer.open('/paramsTuner/'+moduleName+'/rpc:o'):
        print 'Error opening rpc port for module ', moduleName
        exit(1)
    else:
        print 'rpc port for ', moduleName, ' succesfully opened'

    if not yarp.Network.connect('/paramsTuner/'+moduleName+'/rpc:o','/'+moduleName+'/rpc'):
        print 'Error connecting ', '/paramsTuner/'+moduleName+'/rpc:o', ' to ', '/'+moduleName+'/rpc'
        exit(1)
    else:
        print 'Succesfully connected to ', moduleName

    tuner = paramsTuner(streamer)
    tuner.window.show_all()
    Gtk.main()

