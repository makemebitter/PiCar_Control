from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
 
def decode(im) : 
	# Find barcodes and QR codes
	decodedObjects = pyzbar.decode(im)
 
	# Print results
	for obj in decodedObjects:
	print('Type : ', obj.type)
	print('Data : ', obj.data,'\n')
	 
	return decodedObjects
 
 
# Display barcode and QR code location  
def display(im, decodedObjects):
 
	# Loop over all decoded objects
	for decodedObject in decodedObjects: 
	points = decodedObject.location
	print (points)
	# If the points do not form a quad, find convex hull
	if len(points) > 4 : 
		hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
		hull = list(map(tuple, np.squeeze(hull)))
	else : 
		hull = points;
	 
	# Number of points in the convex hull
	n = len(hull)
 
	# Draw the convext hull
	for j in range(0,n):
		cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
	font = cv2.FONT_HERSHEY_SIMPLEX
	for point in points:
		cv2.putText(im,point,(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)
	# Display results 
	cv2.imshow("Results", im);
	cv2.waitKey(1);
 
	 
# Main 
if __name__ == '__main__':
	cap = cv2.VideoCapture(0)

	while True:
	_, frame = cap.read()
	# Read image 
	decodedObjects = decode(frame)
	display(frame, decodedObjects)