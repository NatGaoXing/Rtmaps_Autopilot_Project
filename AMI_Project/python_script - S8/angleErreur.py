#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy
import math
from rtmaps.base_component import BaseComponent # base class
from distancePointToLine import *
from simple_pid import PID


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor
        self.mult = 1.2
        self.coefLine = 0
        self.pid = PID(2.0,0.0,0.0      , setpoint = 0)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-math.pi,math.pi)
    def Dynamic(self):
        self.add_input("TargetY", rtmaps.types.ANY)  # define input
        self.add_input("TargetX", rtmaps.types.ANY) # define input
        self.add_input("Fin", rtmaps.types.ANY)  # define input
        self.add_output("Lat_cmd", rtmaps.types.AUTO) # define output
        self.add_output("A",rtmaps.types.AUTO)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")
        self.outputs["Lat_cmd"].write(0.0)
        self.i=0
        self.trajectoryFile = open("./LogLatCommand.csv", "w")
# Core() is called every time you have a new input
    def Core(self):
        #inputs
        X_list = self.inputs["TargetX"].ioelt.data # create an ioelt from the input
        Y_list = self.inputs["TargetY"].ioelt.data  # create an ioelt from the input
        Fin = self.inputs["Fin"].ioelt.data

        #if there is only one point (robot), we take the one in front of us
        if len(X_list) < 2 :
            x=0.0
            y=1.0
        #else we tacke the first one
        else :
            x=X_list[1]
            y=Y_list[1]
        if len(X_list) > 1 : #if there are enough points, we calculate the distance to the line formed by the two next
            pointDroite1 = np.array([X_list[min(len(X_list)-2,3)],Y_list[min(len(X_list)-2,3)],0.0])
            pointDroite2 = np.array([X_list[min(len(X_list)-1,4)], Y_list[min(len(X_list)-1,4)], 0.0])
            pointToProcess = np.array([0.0, 0.0, 0.0])
            distancePtLine = getDistancePoint2Line(pointDroite1, pointDroite2, pointToProcess)
        else :
            distancePtLine = 0.0
        #sum of angle with next point and previous line
        angle = math.atan2(-x,y)   +distancePtLine*self.coefLine

        #apply pid
        command = self.pid(-angle)

        #log command
        self.trajectoryFile.write(str(command))
        self.trajectoryFile.write("\n")


        if Fin==1 : #if it's the end, stop turning and send command to stop
            self.outputs["Lat_cmd"].write(0.0)
            self.outputs["A"].write(-2)
        else : #else write command and determine wether we should go fast or not
            self.outputs["Lat_cmd"].write(-command/math.pi) # y etant l'axe en face du robot
            x_sp=X_list[len(X_list)-1]
            y_sp=Y_list[len(Y_list) - 1]
            #to determine the desired speed, we sum the angle with the next point and with the one further
            if abs(math.atan2(-x_sp,y_sp))+abs(math.atan2(-x,y))>2*math.pi/2-0.2 :
                self.outputs["A"].write(-2)
            elif abs(math.atan2(-x_sp,y_sp))+abs(math.atan2(-x,y))>2*math.pi/3-0.2 :
                self.outputs["A"].write(-1)
            else :
                self.outputs["A"].write(1)
    # Death() will be called once at diagram execution shutdown
    def Death(self):
        self.trajectoryFile.close()
        pass
