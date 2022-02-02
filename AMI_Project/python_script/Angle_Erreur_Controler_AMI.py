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
        self.trajectoryFile = None
        self.i = 0
        # self.multi = 1.2
        self.coefLine = 0
        self.pid = PID(0.7, 0.0, 0.0, setpoint=0)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-180, 180)

    def Dynamic(self):
        self.add_input("TargetY", rtmaps.types.ANY)
        self.add_input("TargetX", rtmaps.types.ANY)
        self.add_input("Fin", rtmaps.types.ANY)
        self.add_output("Lat_cmd", rtmaps.types.AUTO)
        self.add_output("A", rtmaps.types.AUTO)
        self.add_output("targetpoints", rtmaps.types.AUTO)

    def Birth(self):
        print("Python Birth")
        self.trajectoryFile = open("./LogLatCommand.csv", "w")
        self.outputs["Lat_cmd"].write(0.0)

    def Core(self):
        X_list = self.inputs["TargetX"].ioelt.data
        Y_list = self.inputs["TargetY"].ioelt.data
        Fin = self.inputs["Fin"].ioelt.data
        
        radius = 5
        
        
        # if there is only one point (robot), we take the one in front of us
        if len(X_list) < 2:
            x = 0.0
            y = 1.0
        else:  # else we take the first one
            i = 0
            try:
                while (math.sqrt(X_list[i] ** 2 + Y_list[i] ** 2) < radius):
                    i+=1
            except:
                i-=1
                pass
            x = X_list[i]
            y = Y_list[i]
        """
        # if there are enough points, we calculate the distance to the line formed by the two next
        if len(X_list) > 1:
            point_droite_1 = np.array([X_list[min(len(X_list) - 2, 3)], Y_list[min(len(X_list) - 2, 3)], 0.0])
            point_droite_2 = np.array([X_list[min(len(X_list) - 1, 4)], Y_list[min(len(X_list) - 1, 4)], 0.0])
            point_to_process = np.array([0.0, 0.0, 0.0])

            distancePtLine = getDistancePoint2Line(point_droite_1, point_droite_2, point_to_process)
        else:"""
            
        # sum of angle with next point and previous line
        angle = math.atan2(-x, y)# + distancePtLine * self.coefLine

        # apply pid
        command = self.pid(math.degrees(-angle))

        # log command
        self.trajectoryFile.write(str(command))
        self.trajectoryFile.write("\n")

        # if it's the end, stop turning and send command to stop
        if Fin == 1:
            self.outputs["Lat_cmd"].write(0.0)
            self.outputs["A"].write(-2)
        else:  # else write command and determine whether we should go fast or not
            self.outputs["Lat_cmd"].write(command)  # y is the longitudinal axis
            x_sp = X_list[len(X_list) - 1]
            y_sp = Y_list[len(Y_list) - 1]

            # to determine the desired speed, we sum the angle with the next point and with the one further
            if abs(math.atan2(-x_sp, y_sp)) + abs(math.atan2(-x, y)) > 2 * math.pi / 2 - 0.2:
                self.outputs["A"].write(-2)
            elif abs(math.atan2(-x_sp, y_sp)) + abs(math.atan2(-x, y)) > 2 * math.pi / 3 - 0.2:
                self.outputs["A"].write(-1)
            else:
                self.outputs["A"].write(1)
                
        self.outputs["targetpoints"].write([y,-x,0.0])

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        self.trajectoryFile.close()
        pass
