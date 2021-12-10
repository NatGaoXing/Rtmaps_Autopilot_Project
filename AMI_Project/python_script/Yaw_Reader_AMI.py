import rtmaps.types
import numpy as np
import math
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("out", rtmaps.types.AUTO)

    def Birth(self):
        print("Python Birth")

    def Core(self):
        out = self.inputs["in"].ioelt.data
        out[2] = math.radians(out[2]+90)
        self.outputs["out"].write(np.float64(out[2]))

    def Death(self):
        pass
