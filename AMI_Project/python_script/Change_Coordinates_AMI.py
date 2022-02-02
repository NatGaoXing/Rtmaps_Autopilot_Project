import math
import time
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy
from scipy.spatial.transform import Rotation as Rot
from rtmaps.base_component import BaseComponent
import os
import sys


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

        
        self.obstacle = 0

    def Dynamic(self):
        self.add_input("dataRobot", rtmaps.types.AUTO)
        self.add_input("stop", rtmaps.types.AUTO)
        self.add_output("targetX", rtmaps.types.AUTO, 4)
        self.add_output("targetY", rtmaps.types.AUTO, 4)
        self.add_output("targetXY", rtmaps.types.AUTO, 1000)
        
        self.add_output("fin", rtmaps.types.AUTO)

    def Birth(self):
        print("Python Birth")
        self.i = 0
        self.t_obstacle=0
        self.y_bound_decal = 0

        self.outputs["fin"].write(0)

    def Core(self):
        # Trajectory Read
        trajectory = np.genfromtxt("./traj.csv", delimiter=" ", invalid_raise=False)
        trajectoryTab = np.array(trajectory).reshape(-1, 3).astype(np.float64)

        # Robot position read
        UTM_X = self.inputs["dataRobot"].ioelt.data[1]
        UTM_Y = self.inputs["dataRobot"].ioelt.data[2]
        yaw = self.inputs["dataRobot"].ioelt.data[0]

        # Fix bc sometimes the first stop info is sent too late
        try:
            stop = self.inputs["stop"].ioelt.data
        except:
            stop = 0

        # Transition Matrix creation
        H_src = np.eye(4)
        H_src[0:2, 3] = UTM_X, UTM_Y
        rMatrix = Rot.from_euler("ZYX", np.array([(math.pi / 2 - yaw), 0.0, 0.0]), degrees=False).as_matrix()
        H_src[0:3, 0:3] = rMatrix
        H_src = np.linalg.inv(H_src)

        list_x = []
        list_y = []

        # Change of the trajectory coordinate system with the matrix
        for j in range(self.i, len(trajectoryTab)):  # i:len ?
            pointMonde = np.array([trajectoryTab[j, 0], trajectoryTab[j, 1], trajectoryTab[j, 2], 1.0])
            pointMonde = H_src.dot(pointMonde)
            list_y.append(-pointMonde[0])
            list_x.append(pointMonde[1])
            
            
        # Management of an obstacle
        if stop == 1:
            print("Obstacle !")
            self.outputs["fin"].write(1)
            time.sleep(2)
        elif self.i < len(list_x) - 1:
            self.outputs["fin"].write(0)

        # If we are close to the targeted point or the one after.
        if list_y[self.i]<1 or math.sqrt((list_x[self.i]) ** 2 + (list_y[self.i]) ** 2) < 2.5 or math.sqrt((list_x[min(self.i + 1, len(list_x) - 1)]) ** 2 + (list_y[min(self.i + 1, len(list_x) - 1)]) ** 2) < 2.5: #comparaison avec nouvelle liste x_test
            # And not at the end, we move forward in the list
            if self.i < len(list_x) - 2:
                self.i = self.i + 1
            # If we're at the end, we stop
            else:
                self.outputs["fin"].write(1)
        # output the 4 next points
        
        self.outputs["targetX"].write(list_x[self.i:self.i + 4])  # and write it to the output !!!new list x_test!!!
        self.outputs["targetY"].write(list_y[self.i:self.i + 4])

        # Output points for 3D viewer
        list_xy = [0.0, 0.0, 0.0]
        for i in range(len(list_x)):
            list_xy.append(list_y[i])
            list_xy.append(-list_x[i])
            list_xy.append(0.0)
        self.outputs["targetXY"].write(list_xy)

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
