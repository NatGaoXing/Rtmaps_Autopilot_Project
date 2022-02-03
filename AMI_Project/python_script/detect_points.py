


#This is a template code. Please save it in a proper .py file.
import numpy as np
import time
import rtmaps.types
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 


def detect_points(array_in,bounding_box): # function we use to detect the points inside bounding boxes
	x_min = bounding_box[0,0] # extract the values of the box
	x_max = bounding_box[0,1]
	y_min = bounding_box[1,0]
	y_max = bounding_box[1,1]
	z_min = bounding_box[2,0]
	z_max = bounding_box[2,1]
	braking = 0.0
	y_bound_min = 99.0 # init at a high value for the min
	y_bound_max = -99.0 # and at a low value for the max
	x_bound = 99.0
	points = array_in.reshape(int(np.size(array_in)/3),3) # reshape the xyz points to x,y,z for easier use
	points_size = np.size(points, axis=0)
	points_in_bound = [0.0, 0.0, 0.0]
	for i in range(0,points_size): # for loop to check every points
		if points[i,0] > x_min and points[i,0] < x_max and points[i,1] > y_min and points[i,1] < y_max and points[i,2] > z_min and points[i,2] < z_max: # condition to check if they're inside the bounding box
			points_in_bound.append(points[i,0]) # add the points inside the box to another array
			points_in_bound.append(points[i,1])
			points_in_bound.append(points[i,2])
			if points[i,0] < x_bound:
				x_bound = points[i,0] # define a new x closer point
			if points[i,1] < y_bound_min:
				y_bound_min = points[i,1] # define a new y min value
			if points[i,1] > y_bound_max:
				y_bound_max = points[i,1] # define a new y max value
			braking = (x_max - x_bound)*5/x_max # the lower the distance to the obstacle, the higher the braking value !!! the value was used before the integration of longi_speed block !!!
			if braking > 1:
				braking = 1	
	
	return float(braking), y_bound_min, y_bound_max, x_bound, points_in_bound


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY) # define input for the lidar points
        self.add_input("Dodge_1_Stop_0", rtmaps.types.ANY) # define input of the will to dodge or stop at obstacle
        self.add_output("out", rtmaps.types.AUTO) # define output for the braking value
        self.add_output("decalage_out", rtmaps.types.AUTO) # define output for the size of the object to dodge
        self.add_output("obstacle_out", rtmaps.types.AUTO) # define output for the detection of an obstacle
        self.add_output("points_in_bound_right_out", rtmaps.types.AUTO,10000) # define output for the points inside the right bounding box
        self.add_output("points_in_bound_front_out", rtmaps.types.AUTO,10000) # define output for the points inside the right bounding box

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["out"].write(0.0)
        self.t_obstacle = 0
        self.y_bound_decal = 0
        self.obstacle = 0

# Core() is called every time you have a new input
    def Core(self):
        input_array = self.inputs["in"].ioelt.data # create an ioelt from the incoming lidar points
        Dodge_1_Stop_0 = float(self.inputs["Dodge_1_Stop_0"].ioelt.data) # create an ioelt from the input
		
        array_in = np.array(input_array) # create an numpy array
        bounding_box = np.array([[0,10],[-2.,2.],[-1,2]]) # define the bounding box in front of the car
        bounding_box_right = np.array([[-3,5],[-4,-1],[-1,2]]) # define the bounding box on the right of the car
        out_right, y_bound_min_out, y_bound_max_out, x_bound_out, points_in_bound_right = detect_points(array_in,bounding_box_right) # detect points in front of the car
        out, y_bound_min_out, y_bound_max_out, x_bound_out, points_in_bound_front = detect_points(array_in,bounding_box )# and on the right of the car
		
        if out > 0: # means an obstacle has been detected
        	obstacle_in_bound = 1.0
        else:
        	obstacle_in_bound = 0.0
		
        if Dodge_1_Stop_0 == 1: # if asked to dodge, out becomes 0.0 so we don't brake
        	out = 0.0
        else:
        	out = out

        t_new = time.time() # time of the execution of the code
        if obstacle_in_bound > 0.0:
            self.obstacle = 1
            self.t_obstacle = time.time() # time of the obstacle detection
            if y_bound_max_out - y_bound_min_out > self.y_bound_decal : # if the size measured is bigger than the one detected on the previous frames
                self.y_bound_decal = y_bound_max_out - y_bound_min_out # then we replace it
            
        if (out_right > 0.0 and self.obstacle == 1): # if obstacle on the right, keep the variables on dodge state
            self.obstacle = 1.
            self.t_obstacle = time.time()
            self.y_bound_decal = self.y_bound_decal
        
        if t_new - self.t_obstacle > 0.5: # if 0.5 secs has elapsed since the lasr detection, go back to init state
            self.obstacle = 0.
            self.y_bound_decal = 0.
        
        if self.obstacle == 1 and Dodge_1_Stop_0 == 1:
            decalage = 1.5 + self.y_bound_decal*1. # definition of the distance to dodge
        else:
            decalage = 0.
		
        self.outputs["out"].write(out) # and write it to the output
        self.outputs["decalage_out"].write(decalage) # and write it to the output
        self.outputs["obstacle_out"].write(self.obstacle) # and write it to the output
        self.outputs["points_in_bound_right_out"].write(points_in_bound_right) # and write it to the output
        self.outputs["points_in_bound_front_out"].write(points_in_bound_front) # and write it to the output
		
        

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass