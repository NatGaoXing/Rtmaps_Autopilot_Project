import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent


class rtmaps_python(BaseComponent):
    def __init__(self):  # TODO
        BaseComponent.__init__(self)

    def Dynamic(self):  # TODO
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("a", rtmaps.types.AUTO)
        self.add_output("delta", rtmaps.types.AUTO)

    def Birth(self):  # TODO
        print("Python Birth")

    def Core(self):  # TODO
        out = self.inputs["in"].ioelt
        self.outputs["a"].write(out)
        self.outputs["delta"].write(out)

    def Death(self):  # TODO
        pass
