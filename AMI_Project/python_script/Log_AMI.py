import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
import utm
import math
import time
from rtmaps.base_component import BaseComponent

import os
import sys
import csv
location = os.path.dirname(os.path.realpath(__file__))


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

        self.previousY = 0.0
        self.previousX = 0.0
        self.trajectoryFile = None
        self.distPts = 5

        self.IMU = None
        self.recording = 0

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)

    def Birth(self):
        print("Python Birth")
        self.trajectoryFile = open("./log.csv", "w")
        self.trajectoryFile.close()

    def Core(self):
        self.IMU = self.inputs["in"].ioelt.data

        trajectory = np.genfromtxt("./traj.csv", delimiter=" ", invalid_raise=False)
        trajectoryTab = np.array(trajectory).reshape(-1, 3).astype(np.float64)
        writer = csv.writer(self.trajectoryFile)


        if not self.trajectoryFile.closed:
            pass
        else:  # if record file is closed we open it
            self.trajectoryFile = open("./log.csv", "w")

        # UTM Conversion
        UTM_conv = utm.from_latlon(self.IMU[0], self.IMU[1])
        x = UTM_conv[0]
        y = UTM_conv[1]

        # If the point is further than self.distPts meters from the previous one,we save it
        if math.sqrt((x - self.previousX) ** 2 + (y - self.previousY) ** 2) > self.distPts:
            self.previousX = x
            self.previousY = y
            point = [x, y]
            prevDist = 1000
            for i in range(len(trajectory)):

                if (math.sqrt((trajectory[i][0] - x) ** 2 + (trajectory[i][1] - y) ** 2) < prevDist):
                    corresp_point = trajectory[i]
                    prevDist = math.sqrt((corresp_point[0] - x) ** 2 + (corresp_point[1] - y) ** 2)

            rows = ([x, y, corresp_point[0] ,corresp_point[1]])

            with open("./log.csv", "a") as f:
                writer = csv.writer(f)
                writer.writerow(rows)

    def Death(self):
        self.trajectoryFile.close()
        pass
