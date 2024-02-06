import cv2
import numpy as np
from matplotlib import pyplot as plt 
def canny(image):
	gray = cv2.cvtColor(lane_image, cv2.COLOR_RGB2GRAY)
	gaussian_blur = cv2.GaussianBlur(gray, (5,5), 0)
	canny = cv2.Canny(gaussian_blur, 130, 200) 
	return canny
	
def region_of_interest(image):
	height = image.shape[0]
	polygons = np.array([[(0, height),(600,250),(200,100)]]) #cropping desired area
	#it is not shown here but I used matplotlib to decide region of interest.
	mask = np.zeros_like(image)
	cv2.fillPoly(mask, polygons, 255) #taking desired region
	masked_image = cv2.bitwise_and(image, mask)
	return masked_image

def display_lines(image, lines):
	line_image = np.zeros_like(image)
	if lines is not None:
		for line in lines:
			x1, y1, x2, y2 = line.reshape(4)
			cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
	return line_image
	
image = cv2.imread('test_image.jpeg')
lane_image = np.copy(image)
mcanny = canny(lane_image)
cropped_image = region_of_interest(mcanny)

# HOUGH TRANSFORM
lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=150, maxLineGap=10) # (np.pi/180 equals 1 degree)
# the third value is threshold of how many intersections required to decide whether something is a line and 100 is a good number.
# fifth and sixth values are optional, fourt one adjusts the minimum pixels required to make a line and fifth one adjust the max gap between two line
line_image = display_lines(lane_image, lines)
cv2.imshow('result', line_image)
cv2.waitKey(0)
