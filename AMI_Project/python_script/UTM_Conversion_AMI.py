import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy

import utm
from rtmaps.base_component import BaseComponent


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

    def Dynamic(self):
        self.add_input("IMU", rtmaps.types.ANY)
        self.add_output("East", rtmaps.types.AUTO)
        self.add_output("North", rtmaps.types.AUTO)

    def Birth(self):
        print("Python Birth")

    def Core(self):
        IMU = self.inputs["IMU"].ioelt.data

        UTM_conv = utm.from_latlon(IMU[0], IMU[1])
        x = UTM_conv[0]
        y = UTM_conv[1]

        self.outputs["East"].write(x)
        self.outputs["North"].write(y)

    def Death(self):
        pass
