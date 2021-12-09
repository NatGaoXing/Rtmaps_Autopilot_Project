
# This is a template code. Please save it in a proper .py file.
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

        self.add_input("A", rtmaps.types.ANY)  # Accélérer_ou_Ralentir
        self.add_input("Stop", rtmaps.types.ANY)  # Stop
        self.add_output("V_smooth", rtmaps.types.AUTO)  # vitesse_smoother

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        # print("Python Birth")
        self.V_fast = 1.5 #Straight line speed
        self.V_slow = 0.6 #Small turn speed
        self.accel = 0.02 #  m/sec-2 /10

        #variables init
        self.V_smooth = 0.0
        self.savedTime = time.thread_time_ns()

        self.trajectoryFile = open("./LogLongSpeed.csv", "w")
        pass

    # Core() is called every time you have a new input
    def Core(self):
        Stop = self.inputs["Stop"].ioelt.data  # create an ioelt from the input
        A = self.inputs["A"].ioelt.data  # create an ioelt from the input

        if time.time_ns() - self.savedTime > 100000 : #Triggers every 10ms
            self.savedTime = time.time_ns()
            if A ==1: #if command set to fast, accelerate u
                self.V_smooth = self.V_smooth + self.accel
            elif A == -1: #if command set to slow, accelerate or accelerait
                if self.V_smooth < self.V_slow:
                    self.V_smooth = self.V_smooth + self.accel
                else :
                    self.V_smooth = self.V_smooth - self.accel
            elif A ==-2 : #if command set to stop, stop
                self.V_smooth = self.V_smooth - self.accel
            else :
                print("Issue with speed command")

        #Speed boundaries
        if self.V_smooth > self.V_fast:
            self.V_smooth = self.V_fast
        if self.V_smooth < 0.0:
            self.V_smooth = 0.0

        #If emergency stop, we stop instantly
        if(Stop==1) :
            self.V_smooth=0.0

        #Speed log
        self.trajectoryFile.write(str(self.V_smooth))
        self.trajectoryFile.write("\n")

        #Speed output
        self.outputs["V_smooth"].write(self.V_smooth)

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        self.trajectoryFile.close()
        pass