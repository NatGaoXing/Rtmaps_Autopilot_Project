
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class
import time

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor

    def Dynamic(self):

        self.add_input("SpeedKmh", rtmaps.types.ANY)  #Actual speed
        self.add_input("SpeedCmd", rtmaps.types.ANY)  #Desired speed
        self.add_output("Accel", rtmaps.types.AUTO) #Throttle value to send to the simulator
        self.add_output("Break", rtmaps.types.AUTO) #Breaking value to send to the simulator

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["Accel"].write(0.0) #Start by writing a command of 0
        self.outputs["Break"].write(0.0)
        pass

    # Core() is called every time you have a new input
    def Core(self):
        #To modify depending on the real model
        #The higher these values are, the higher the system will accelerate / break
        accelFactor = 0.15
        breakFactor = 0.01

        #Read inputs
        speedKmh = self.inputs["SpeedKmh"].ioelt.data
        speedCmd = self.inputs["SpeedCmd"].ioelt.data

        #If the speed is smaller than the command
        if speedCmd > speedKmh :
            #Accelerate depending on the difference between the actual speed and the command (+1 otherwise the command won't be reached)
            accel = accelFactor* (abs(speedKmh-speedCmd)+1)
            breakval = 0.0
        elif  speedCmd < speedKmh : #If the speed is higher
            #Same but with breaking
            accel = 0.0
            breakval = breakFactor* (abs(speedKmh-speedCmd)+1)
        else :
            #else the speeds are the same, we don't need to do anything
            accel =0.0
            breakval=0.0

        #output the final values
        self.outputs["Accel"].write(accel)
        self.outputs["Break"].write(breakval)

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass