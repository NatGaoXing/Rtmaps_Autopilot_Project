import numpy as np

#http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/
#https://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_une_droite
#la taille du produit vectoriel est le produit des 2 normes, donc en divisant l'un, 
def getDistancePoint2Line(pointDroite1,pointDroite2, pointToProcess):
	""" il faut envoyer des points de taille 3 differants"""
	if (pointDroite1.shape[0] != 3) or (pointDroite1.shape[0] != 3) or (pointDroite1.shape[0] != 3):
		print ("point shape 0 must be 3")
		return None

	if np.array_equal( pointDroite1,pointDroite2):
		print ("une droite est definie par deux points differants")
		return None
		
	p2lessp1 = pointDroite2-pointDroite1
	p1less0 = pointDroite1-pointToProcess
	
	numerator= np.cross(p2lessp1,p1less0)
	normeUp = np.linalg.norm(numerator)
	normeDown = np.linalg.norm(p2lessp1)
	if(normeDown):
		return normeUp/normeDown
	else:
		return 0.0;#le point est confondue


if __name__ == "__main__":
	pointDroite1=np.array([0.0,1.0,2.0])
	pointDroite2=np.array([8.0,1.0,2.0])
	pointToProcess=np.array([9.0,1.0,15.0])

	distance = getDistancePoint2Line(pointDroite1,pointDroite2, pointToProcess)
	print ("distance ="+str(distance))
