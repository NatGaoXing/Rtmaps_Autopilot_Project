#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import math
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY) # define input
        self.add_output("out", rtmaps.types.AUTO) # define output

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")

# Core() is called every time you have a new input
    def Core(self):
        out = self.inputs["in"].ioelt.data# create an ioelt from the input
        out[2] = math.radians(out[2]+180)
        self.outputs["out"].write(np.float64(out[2])) # and write it to the output

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
