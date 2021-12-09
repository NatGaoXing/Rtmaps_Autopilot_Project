#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
import utm
import math
import time
from rtmaps.base_component import BaseComponent # base class 
import os, sys
location = os.path.dirname(os.path.realpath(__file__))


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor
        self.distPts = 5

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY) # define input
        self.add_input("recording", rtmaps.types.ANY)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")
        self.trajectoryFile = None
        self.previousX = 0.0
        self.previousY = 0.0
        self.trajectoryFile = open("./traj.csv", "r")
        self.trajectoryFile.close()

# Core() is called every time you have a new input
    def Core(self):

        self.IMU = self.inputs["in"].ioelt.data # create an ioelt from the input
        self.recording = self.inputs["recording"].ioelt.data
        if self.recording == 1 : #if we want to record

            #if record file is closed we open it
            if self.trajectoryFile.closed == True:
                self.trajectoryFile = open("./traj.csv", "w")

            #UTM Conversion
            UTmconv = utm.from_latlon(self.IMU[0], self.IMU[1])
            x = UTmconv[0]
            y = UTmconv[1]

            #If the point is further than self.distPts meters from the previous one,we save it
            if math.sqrt((x-self.previousX)**2+(y-self.previousY)**2) > self.distPts:

                self.trajectoryFile.write(str(x))
                self.previousX = x
                self.trajectoryFile.write(" ")
                self.trajectoryFile.write(str(y))
                self.previousY = y
                self.trajectoryFile.write(" ")
                self.trajectoryFile.write("0.0")
                self.trajectoryFile.write("\n")
        else :
            if self.trajectoryFile.closed == False :
                self.trajectoryFile.close()

# Death() will be called once at diagram execution shutdown
    def Death(self):
        if self.recording == 1:
            #record final position (twice bc assay issues)
            UTmconv = utm.from_latlon(self.IMU[0], self.IMU[1])
            x = UTmconv[0]
            y = UTmconv[1]
            self.trajectoryFile.write(str(x))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write(str(y))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write("0.0")
            self.trajectoryFile.write("\n")
            self.trajectoryFile.write(str(x))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write(str(y))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write("0.0")
            self.trajectoryFile.write("\n")
            self.trajectoryFile.close()
        else :
            self.trajectoryFile.close()
        pass
