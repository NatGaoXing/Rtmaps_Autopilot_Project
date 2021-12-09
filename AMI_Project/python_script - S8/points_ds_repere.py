#This is a template code. Please save it in a proper .py file.
import math
import time
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy
from scipy.spatial.transform import Rotation as R #NOTE : pip install scipy==1.5.2
from rtmaps.base_component import BaseComponent # base class
import os, sys
location = os.path.dirname(os.path.realpath(__file__))


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("dataRobot", rtmaps.types.AUTO)
        self.add_input("stop", rtmaps.types.AUTO)
        self.add_output("targetX", rtmaps.types.AUTO,4)  # define outpu
        self.add_output("targetY", rtmaps.types.AUTO,4)  # define output
        self.add_output("targetXY", rtmaps.types.AUTO,1000)  # define output
        self.add_output("fin", rtmaps.types.AUTO)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")

        self.outputs["fin"].write(0)
        self.i = 0


# Core() is called every time you have a new input
    def Core(self):
        #Trajectory Read
        trajectory = np.genfromtxt("./traj.csv", delimiter=" ", invalid_raise = False)
        trajectoryTab = np.array(trajectory).reshape(-1, 3).astype(np.float64)

        #Robot position read
        UTMX = self.inputs["dataRobot"].ioelt.data[0] # create an ioelt from the input
        UTMY = self.inputs["dataRobot"].ioelt.data[1]  # create an ioelt from the input
        yaw = self.inputs["dataRobot"].ioelt.data[2]  # create an ioelt from the input

        #Fix bc sometimes the first stop info is sent too late
        try:
            stop = self.inputs["stop"].ioelt.data
        except :
            stop = 0

        #Transition Matrix creation
        Hsrc = np.eye(4)
        Hsrc[0:2, 3] = UTMX,UTMY
        rMatrix = R.from_euler("ZYX", np.array([(math.pi / 2 - yaw), 0.0, 0.0]), degrees=False).as_matrix()
        Hsrc[0:3, 0:3] = rMatrix
        Hsrc = np.linalg.inv(Hsrc)

        list_x = []
        list_y = []

        #Change of the trajectory coordinate system with the matrix
        for j in range(self.i, len(trajectoryTab)):#i:len ?
            pointMonde = np.array([trajectoryTab[j, 0], trajectoryTab[j, 1], trajectoryTab[j, 2], 1.0])
            pointMonde = Hsrc.dot(pointMonde)
            list_y.append(-pointMonde[0])
            list_x.append(pointMonde[1])

        #Management of an obstacle
        if stop == 1 :
            print("Obstacle !")
            self.outputs["fin"].write(1)
            time.sleep(2)
        elif self.i < len(list_x) -1:
            self.outputs["fin"].write(0)

        #If we are close to the targeted point or the one after..
        if math.sqrt((list_x[self.i]) ** 2 + (list_y[self.i]) ** 2) < 1 or math.sqrt((list_x[min(self.i+1,len(list_x)-1)]) ** 2 + (list_y[min(self.i+1, len(list_x)-1)]) ** 2) < 1 :
            #..And not at the end, we move forward in the list
            if self.i < len(list_x) - 2:
                self.i = self.i + 1
                #print(str(self.i))
                #print(len(list_x))
            #If we're at the end, we stop
            else:
                self.outputs["fin"].write(1)
        #output the 4 next points
        self.outputs["targetX"].write(list_x[self.i:self.i+4])  # and write it to the output
        self.outputs["targetY"].write(list_y[self.i:self.i+4])

        #Output points for 3D viewer
        list_xy= []
        list_xy.append(0.0)
        list_xy.append(0.0)
        list_xy.append(0.0)
        for i in range (len(list_x)) :
            list_xy.append(list_y[i])
            list_xy.append(-list_x[i])
            list_xy.append(0.0)
        self.outputs["targetXY"].write(list_xy)

    # Death() will be called once at diagram execution shutdown
    def Death(self):

        pass
