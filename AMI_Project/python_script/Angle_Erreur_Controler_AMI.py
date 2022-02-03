import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy
import math
from rtmaps.base_component import BaseComponent

from Point_To_Line import *
from simple_pid import PID


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        #Define PID :Settings, sample time, limits
        self.pid = PID(0.7, 0.0, 0.0, setpoint=0)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-180, 180)

    def Dynamic(self):
        self.add_input("TargetY", rtmaps.types.ANY) #Target points
        self.add_input("TargetX", rtmaps.types.ANY)
        self.add_output("Lat_cmd", rtmaps.types.AUTO) #Lateral command
        self.add_output("targetpoints", rtmaps.types.AUTO) #Target point for the 3D viewer

    def Birth(self):
        print("Python Birth")
        self.outputs["Lat_cmd"].write(0.0)

    def Core(self):
        #Radius of the purepursuit method
        radius = 5

        #Read the target points
        X_list = self.inputs["TargetX"].ioelt.data
        Y_list = self.inputs["TargetY"].ioelt.data
        

        # if there is only one point (car), use a point 1m in front
        if len(X_list) < 2:
            x = 0.0
            y = 1.0
        else:  # else find the first one at least "radius" meters away
            i = 0
            try:
                while (math.sqrt(X_list[i] ** 2 + Y_list[i] ** 2) < radius):
                    i+=1
            except: #if such point does not exist, take the last one
                i-=1
                pass
            #take the point
            x = X_list[i]
            y = Y_list[i]
            
        #angle with targeted point
        angle = math.atan2(-x, y)

        #apply pid
        command = self.pid(math.degrees(-angle))

        #write the command
        self.outputs["Lat_cmd"].write(command)

        #display the targeted point
        self.outputs["targetpoints"].write([y,-x,0.0])

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
