
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

        self.add_input("SpeedKmh", rtmaps.types.ANY)  # Accélérer_ou_Ralentir
        self.add_input("SpeedCmd", rtmaps.types.ANY)  # Accélérer_ou_Ralentir
        self.add_output("Accel", rtmaps.types.AUTO)
        self.add_output("Break", rtmaps.types.AUTO)  

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["Accel"].write(0.0)
        self.outputs["Break"].write(0.0)
        pass

    # Core() is called every time you have a new input
    def Core(self):
        #Modify depending on the real model
        accelFactor = 0.15
        breakFactor = 0.01

        speedKmh = self.inputs["SpeedKmh"].ioelt.data  # create an ioelt from the input
        speedCmd = self.inputs["SpeedCmd"].ioelt.data 
        if speedCmd > speedKmh :
            accel = accelFactor* (abs(speedKmh-speedCmd)) + 0.1
            breakval = 0.0
        elif  speedCmd < speedKmh :
            accel = 0.0
            breakval = breakFactor* (abs(speedKmh-speedCmd))
        else :
            accel =0.0
            breakval=0.0

        #Speed output
        self.outputs["Accel"].write(accel )
        self.outputs["Break"].write(breakval)

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass