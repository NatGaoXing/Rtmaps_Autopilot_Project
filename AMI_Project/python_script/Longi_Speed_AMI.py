#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import math
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent # base class


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):

        self.add_input("TargetY", rtmaps.types.ANY)
        self.add_input("TargetX", rtmaps.types.ANY)
        self.add_input("obstacle_info", rtmaps.types.ANY)
        self.add_output("speed", rtmaps.types.AUTO)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["speed"].write(0.)
        print("Python Birth")
        self.fin = False

# Core() is called every time you have a new input
    def Core(self):
        max_speed = 30.
        maneuver_speed = 10.
        input = self.inputs["obstacle_info"].ioelt # create an ioelt from the input
        breakval = input.data[0]
        is_obstacle = input.data[1]
        X_list = self.inputs["TargetX"].ioelt.data
        Y_list = self.inputs["TargetY"].ioelt.data

        speed = 0.
        radius = 7
        i = 0
        if not self.fin:
            try :
                while (math.sqrt(X_list[i] ** 2 + Y_list[i] ** 2) < radius) :
                    i += 1
            except :
                print("FIN")
                self.fin = True
                speed = 0.
                self.outputs["speed"].write(speed)
        if not self.fin:
            x = X_list[i]
            y = Y_list[i]
            angle = math.atan2(-x, y)
            speed = max_speed / (abs(angle) * 3 )
            if breakval > 0:
                speed =0.

            elif is_obstacle == 1 and speed > maneuver_speed:
                speed = maneuver_speed
            elif speed > max_speed:
                speed = max_speed
            self.outputs["speed"].write(speed) # and write it to the output

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
