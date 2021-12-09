#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy
import utm
from rtmaps.base_component import BaseComponent # base class 


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("IMU", rtmaps.types.ANY) # define input
        self.add_output("Easting", rtmaps.types.AUTO) # define output
        self.add_output("Nording", rtmaps.types.AUTO)  # define output

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")

# Core() is called every time you have a new input
    def Core(self):
        IMU = self.inputs["IMU"].ioelt.data # create an ioelt from the input

        UTmconv = utm.from_latlon(IMU[0], IMU[1])
        x = UTmconv[0]
        y = UTmconv[1]

        self.outputs["Nording"].write(y) # and write it to the output
        self.outputs["Easting"].write(x)  # and write it to the output

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
