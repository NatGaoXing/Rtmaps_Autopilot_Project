import numpy as np


def getDistancePoint2Line(point_droite1, point_droite2, point_to_process):
	if point_droite1.shape[0] != 3:
		print("Three Points are needed.")
		return None

	if np.array_equal(point_droite1, point_droite2):
		print("Two different points are needed.")
		return None
		
	p2_less_1 = point_droite2 - point_droite1
	p1_less_0 = point_droite1 - point_to_process
	
	numerator = np.cross(p2_less_1, p1_less_0)
	norm_Up = np.linalg.norm(numerator)
	norm_Down = np.linalg.norm(p2_less_1)

	if norm_Down:
		return norm_Up/norm_Down
	else:
		return 0.0


if __name__ == "__main__":
	pointDroite1 = np.array([0.0, 1.0, 2.0])
	pointDroite2 = np.array([8.0, 1.0, 2.0])
	pointToProcess = np.array([9.0, 1.0, 15.0])

	distance = getDistancePoint2Line(pointDroite1, pointDroite2, pointToProcess)
	print("Distance = " + str(distance))
