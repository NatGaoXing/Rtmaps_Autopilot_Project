


#This is a template code. Please save it in a proper .py file.
import numpy as np
import time
import rtmaps.types
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 


def detect_points(array_in,bounding_box):
	x_min = bounding_box[0,0]
	x_max = bounding_box[0,1]
	y_min = bounding_box[1,0]
	y_max = bounding_box[1,1]
	z_min = bounding_box[2,0]
	z_max = bounding_box[2,1]
	braking = 0.0
	y_bound_min = 99.0
	y_bound_max = -99.0
	x_bound = 99.0
	points = array_in.reshape(int(np.size(array_in)/3),3)
	points_size = np.size(points, axis=0)
	points_in_bound = [0.0, 0.0, 0.0]
	for i in range(0,points_size):
		if points[i,0] > x_min and points[i,0] < x_max and points[i,1] > y_min and points[i,1] < y_max and points[i,2] > z_min and points[i,2] < z_max:
			points_in_bound.append(points[i,0])
			points_in_bound.append(points[i,1])
			points_in_bound.append(points[i,2])
			if points[i,0] < x_bound:
				x_bound = points[i,0]
			if points[i,1] < y_bound_min:
				y_bound_min = points[i,1]
			if points[i,1] > y_bound_max:
				y_bound_max = points[i,1]
			braking = (x_max - points[i,0])*0.001/x_max #plus la distance entre le début de la bounding box et le point détecté est grande, plus le freinage est fort
			if braking > 1:
				braking = 1	
	
	return float(braking), y_bound_min, y_bound_max, x_bound, points_in_bound


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY) # define input
        self.add_input("Dodge_1_Stop_0", rtmaps.types.ANY) # define input
        self.add_output("out", rtmaps.types.AUTO) # define output
        self.add_output("decalage_out", rtmaps.types.AUTO) # define output
        self.add_output("obstacle_out", rtmaps.types.AUTO) # define output
        self.add_output("points_in_bound_right_out", rtmaps.types.AUTO,10000) # define output
        self.add_output("points_in_bound_front_out", rtmaps.types.AUTO,10000) # define output

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["out"].write(0.0)
        self.t_obstacle = 0
        self.y_bound_decal = 0
        self.obstacle = 0

# Core() is called every time you have a new input
    def Core(self):
        input_array = self.inputs["in"].ioelt.data # create an ioelt from the input
        Dodge_1_Stop_0 = float(self.inputs["Dodge_1_Stop_0"].ioelt.data) # create an ioelt from the input
		
        array_in = np.array(input_array) #[ligne,colonne]
        bounding_box = np.array([[0,10],[-2.,2.],[-1,2]])
        bounding_box_right = np.array([[-3,5],[-5,-1],[-1,2]])
        out_right, y_bound_min_out, y_bound_max_out, x_bound_out, points_in_bound_right = detect_points(array_in,bounding_box_right)
        out, y_bound_min_out, y_bound_max_out, x_bound_out, points_in_bound_front = detect_points(array_in,bounding_box)
		
        if out > 0:
        	obstacle_in_bound = 1.0
        else:
        	obstacle_in_bound = 0.0
		
        if Dodge_1_Stop_0 == 1:
        	out = 0.0
        else:
        	out = out * 5000
		
		#start of old change coordinates
        t_new = time.time()
		
        if obstacle_in_bound > 0.0:
            self.obstacle = 1
            self.t_obstacle = time.time()
            if y_bound_max_out - y_bound_min_out > self.y_bound_decal :
                self.y_bound_decal = y_bound_max_out - y_bound_min_out
                print(self.y_bound_decal)
            
        if (out_right > 0.0 and self.obstacle == 1):
            self.obstacle = 1.
            self.t_obstacle = time.time()
            self.y_bound_decal = self.y_bound_decal
        
        if t_new - self.t_obstacle > 0.5:
            self.obstacle = 0.
            self.y_bound_decal = 0.
        
        if self.obstacle == 1 and Dodge_1_Stop_0 == 1:
            decalage = 1.5 + self.y_bound_decal*1.
        else:
            decalage = 0.
		#end of old change coordinates
		
        self.outputs["out"].write(out) # and write it to the output
        self.outputs["decalage_out"].write(decalage) # and write it to the output
        self.outputs["obstacle_out"].write(self.obstacle) # and write it to the output
        self.outputs["points_in_bound_right_out"].write(points_in_bound_right) # and write it to the output
        self.outputs["points_in_bound_front_out"].write(points_in_bound_front) # and write it to the output
		
        

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass