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

        self.add_input("TargetY", rtmaps.types.ANY) #Coordinates of the next targeted points
        self.add_input("TargetX", rtmaps.types.ANY)
        self.add_input("obstacle_info", rtmaps.types.ANY) #Information from the obstacle detection module : Need to stop and/or obstacle avoidance

        self.add_output("speed", rtmaps.types.AUTO) # Target speed

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["speed"].write(0.) #Write 0 at the beginning
        print("Python Birth")
        self.fin = False #At the start, the end is not reached

# Core() is called every time you have a new input
    def Core(self):
        #Variables definition
        max_speed = 30. #Speed to reach in a straight line
        maneuver_speed = 10. #Speed to reach while avoiding an obstacles
        radius = 7 #Distance to the point used to predict an incoming curve

        input = self.inputs["obstacle_info"].ioelt # get info about the obstacle
        breakval = input.data[0] #Need to stop
        is_obstacle = input.data[1] #Avoidance maneuver
        X_list = self.inputs["TargetX"].ioelt.data #List of points
        Y_list = self.inputs["TargetY"].ioelt.data

        speed = 0. #initial speed
        i = 0 #useful to find the point used to predict the curve

        if not self.fin: #If the end is not reached
            try : #Try to find a point at least 'radius' meters away
                while (math.sqrt(X_list[i] ** 2 + Y_list[i] ** 2) < radius) :
                    i += 1
            except : #If it does not exist, we have reached the end
                print("END REACHED")
                self.fin = True
                speed = 0.
                self.outputs["speed"].write(speed)
        if not self.fin: #Re-check if the end was not reached
            #take the point 'radius " meters away
            x = X_list[i]
            y = Y_list[i]

            #calculate the angle with the point
            angle = math.atan2(-x, y)

            #determine a appropriate speed based on this angle
            speed = max_speed / (abs(angle) * 3 )

            #Need to stop, then stop
            if breakval > 0:
                speed =0.
            #Else, if there is an avoiding maneuver and the speed if higher than the maneauver speed,
            elif is_obstacle == 1 and speed > maneuver_speed:
                speed = maneuver_speed
            #Finally, make sure than the speed will not be higher than the max speed
            elif speed > max_speed:
                speed = max_speed

            self.outputs["speed"].write(speed) # and write the final speed to the output

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
