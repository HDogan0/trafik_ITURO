import cv2
import numpy as np

image = cv2.imread('test_image.jpeg')

#CONVERTING IMAGE TO GRAYSCALE (step 1)

lane_image = np.copy(image)	#with numpy image converted to a set of pixels which has some color identity arrays

gray = cv2.cvtColor(lane_image, cv2.COLOR_RGB2GRAY) # with opencv again, we convert all the pixels to tones of black

#ADDING GAUSSIAN BLUR TO THE IMAGE in order to reducing the noise (step 2)
#		!!gaussian blur makes images smoother!!

gaussian_blur = cv2.GaussianBlur(gray, (5,5), 0) # (5,5) means 5 by 5 kernel which gaussian blur gets applied to makes average of pixels of that grid more similar to each other. (5,5) is good for most cases.
	
#ADDING 'CANNY' METHOD TO IDENTIFY EDGES (step 3)	
# we are gonna use derivative to detect the rate of change of image brightness.
# a large change means a sharp change(edge), a small change means a smooth change.

canny = cv2.Canny(gaussian_blur, 130, 230) # low threshold and high threshold.

#FINDIN REGION OF INTEREST (step 4)

#as now on, I continued with defining a canny method and writing my all process into that function, so I changed to another file.

cv2.imshow('result', canny)

cv2.waitKey(0)
