################################################################################
# showdata.py
#
# Display analog data from YARP paramHelper
# Assumes stream parameters are of constant size
#
################################################################################

import sys
import yarp
import numpy as np
from collections import deque
from matplotlib import pyplot as plt
import matplotlib

# class that holds analog data for N samples
class AnalogData:
    # constr
    def __init__(self, maxLen, data):
        self.t_start = yarp.Time.now()
        self.data = list()
        self.time = deque([0.0]*maxLen)
        for dataIterator in range(data.size()):
            self.data.append( list() )
            for elementIterator in range(data.get(dataIterator).asList().size()):
                elementValue = data.get(dataIterator).asList().get(elementIterator).asDouble()
                self.data[dataIterator].append(deque([0.0]*maxLen))
        self.maxLen = maxLen
        self.dataSize = data.size()
        self.add(data)

    # ring buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.appendleft(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert(data.size() == self.dataSize)
        timeValue = yarp.Time.now()-self.t_start
        for dataIterator in range(data.size()):
            elementsList = self.data[dataIterator]
            self.addToBuf(self.time, timeValue)
            for elementIterator in range(data.get(dataIterator).asList().size()):
                elementDeque = elementsList[elementIterator]
                elementValue = data.get(dataIterator).asList().get(elementIterator).asDouble()
                self.addToBuf(elementDeque, elementValue)
    
# plot class
class AnalogPlot:
    # constr
    def __init__(self, analogData):

        # set plot to animated
        plt.switch_backend('TkAgg')
        mng = plt.get_current_fig_manager()
        
        plt.ion()

        self.nrows = np.int(np.floor(np.sqrt(analogData.dataSize)))
        self.ncols = np.int(np.ceil(analogData.dataSize/self.nrows))
        self.fig, self.axes = plt.subplots(nrows = self.nrows, ncols = self.ncols, sharex=True, squeeze=True, num="streamPlotter", figsize=(14,8))
        
        mng.window.resizable(width=True,height=True)

        rowsIterator = 0
        colsIterator = 0
        for dataIterator in range(analogData.dataSize):
            data = analogData.data[dataIterator]
            dataAxes = self.axes[rowsIterator,colsIterator]
            for elementIterator in range(len(data)):
                ydata = data[elementIterator]
                #self.axes[rowsIterator,colsIterator].plot(analogData.time,ydata)
                dataAxes.plot(ydata)
            dataAxes.set_ylim(top=.1,bottom=-.1)
            colsIterator = np.mod(colsIterator + 1, self.ncols)
            if colsIterator == 0:
                rowsIterator = rowsIterator + 1

        plt.draw()
        #self.fig.canvas.draw()
        

    # update plot
    def update(self, analogData):
        rowsIterator = 0
        colsIterator = 0
        for dataIterator in range(analogData.dataSize):
            data = analogData.data[dataIterator]
            xdata = analogData.time
            now = xdata[0]
            dataAxes = self.axes[rowsIterator,colsIterator]
            for elementIterator in range(len(data)):
                ydata =  data[elementIterator]
                elementLine = dataAxes.lines[elementIterator]
                elementLine.set_ydata(ydata)
                #elementLine.set_xdata(xdata)

            dataAxes.set_xlim(left=100,right=0.0)
            #dataAxes.set_ylim(bottom=np.min(data), top=np.max(data), auto=True)
            #dataAxes.set_ylim(auto=True)
            colsIterator = np.mod(colsIterator + 1, self.ncols)
            if colsIterator == 0:
                rowsIterator = rowsIterator + 1
        
        #plt.show()
        plt.draw()
        #self.fig.canvas.draw()
        

# main() function
def main():
    # expects 1 arg - serial port string
    if(len(sys.argv) != 2):
        print 'Example usage: python showdata.py "/moduleName/stream:o"'
        exit(1)

    yarp.Network.init()
    
    streamerPort = sys.argv[1];
    streamer = yarp.Port()
    if not streamer.open('/showData'+streamerPort):
        print 'Error opening port ', streamerPort
        exit(1)
    else:
        print 'Port ', streamerPort, ' succesfully opened'
    
    if not yarp.Network.connect(streamerPort,'/showData'+streamerPort):
        print 'Error connecting ', '/showData' + streamerPort, ' to ', streamerPort
        exit(1)
    else:
        print 'Succesfully connected to ', streamerPort


    print 'plotting data...'

    # getting first Bottle
    data = yarp.Bottle()
    if not streamer.read(data):
        print 'Error reading data from port'
        exit(1)

    analogData = AnalogData(16384,data)
    analogPlot = AnalogPlot(analogData)
            
    while True:
        try:
            
            data = yarp.Bottle()
            if not streamer.read(data):
                print 'Error reading data from port'
                exit(1)
            #else:
                #print yarp.Time.now()
            
            for dataIterator in range(data.size()):
                if not data.get(dataIterator).isList():
                    print '---- Element is not list: are you using paramHelper?'
                    exit(1)

            analogData.add(data)
            analogPlot.update(analogData)

        except KeyboardInterrupt:
            print 'exiting'
            break
    
    # close port
    streamer.close()

# call main
if __name__ == '__main__':
    main()