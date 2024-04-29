import numpy as np
import cv2
from picamera2 import Picamera2
import matplotlib.pyplot as plt

def hist(img):
    # Grab only the bottom two-thirds of the image
    # Lane lines are likely to be mostly vertical nearest to the car
    h = img.shape[0] 
    bottom_half = img[h//2:, :]

    # Sum values across image pixels vertically
    return np.sum(bottom_half, axis=0)

def threshold(img, color):
    img = cv2.blur(img, (3, 3))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask = cv2.inRange(hsv, np.array([110,50,165]), np.array([125,255,255]))
    elif color == "yellow":
        mask = cv2.inRange(hsv, np.array([10,50,165]), np.array([30,255,255]))
    return mask

def correct_perspective(img):
    size = (img.shape[1], img.shape[0])
    pts = np.float32([[220,375],[460,375],
                      [105,470],[577,470]])
    dst = np.float32([[105,150],[577,150],
                      [105,470],[577,470]])
    mat = cv2.getPerspectiveTransform(pts, dst)
    tr_img = cv2.warpPerspective(img, mat, size)
    return tr_img
def window_mask(width, height, img_ref, center,level):
    output = np.zeros_like(img_ref)
    output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0]-level*height),max(0,int(center-width/2)):min(int(center+width/2),img_ref.shape[1])] = 1
    return output

def find_window_centroids(image, window_width, window_height, margin):
    
    window_centroids = [] # Store the (left,right) window centroid positions per level
    window = np.ones(window_width) # Create our window template that we will use for convolutions
    
    # First find the two starting positions for the left and right lane by using np.sum to get the vertical image slice
    # and then np.convolve the vertical image slice with the window template 
    
    # Sum quarter bottom of image to get slice, could use a different ratio
    l_sum = np.sum(image[int(3*image.shape[0]/4):,:int(image.shape[1]/2)], axis=0)
    l_center = np.argmax(np.convolve(window,l_sum))-window_width/2
    r_sum = np.sum(image[int(3*image.shape[0]/4):,int(image.shape[1]/2):], axis=0)
    r_center = np.argmax(np.convolve(window,r_sum))-window_width/2+int(image.shape[1]/2)
    
    # Add what we found for the first layer
    window_centroids.append((l_center,r_center))
    
    # Go through each layer looking for max pixel locations
    for level in range(1,(int)(image.shape[0]/window_height)):
        # convolve the window into the vertical slice of the image
        image_layer = np.sum(image[int(image.shape[0]-(level+1)*window_height):int(image.shape[0]-level*window_height),:], axis=0)
        conv_signal = np.convolve(window, image_layer)
        # Find the best left centroid by using past left center as a reference
        # Use window_width/2 as offset because convolution signal reference is at right side of window, not center of window
        offset = window_width/2
        l_min_index = int(max(l_center+offset-margin,0))
        l_max_index = int(min(l_center+offset+margin,image.shape[1]))
        l_center = np.argmax(conv_signal[l_min_index:l_max_index])+l_min_index-offset
        # Find the best right centroid by using past right center as a reference
        r_min_index = int(max(r_center+offset-margin,0))
        r_max_index = int(min(r_center+offset+margin,image.shape[1]))
        r_center = np.argmax(conv_signal[r_min_index:r_max_index])+r_min_index-offset
        # Add what we found for that layer
        window_centroids.append((l_center,r_center))

    return window_centroids
def find_lane_pixels2(binary_warped, show_windows=True):
    
    '''Using the convolution method'''
    
    # Take a histogram of the bottom half of the image
    histogram = hist(binary_warped)
    
    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 9
    # Set the width of the windows +/- margin
    margin = 80
    
    window_height = int(binary_warped.shape[0]//nwindows)
    window_width = 80   # used for convolution method
        
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    window_centroids = find_window_centroids(binary_warped, window_width, window_height, margin)
    
    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        
        ### Find the four below boundaries of the window ###
        win_xleft_low = int(window_centroids[window][0]) - window_width
        win_xleft_high = int(window_centroids[window][0]) + window_width
        win_xright_low = int(window_centroids[window][1]) - window_width
        win_xright_high = int(window_centroids[window][1]) + window_width

        # Draw the windows on the visualization image
        if show_windows:
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2) 
        
        ### Identify the nonzero pixels in x and y within the window ###
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    if len(left_lane_inds) > 0:
        left_lane_inds = np.concatenate(left_lane_inds)
    if len(right_lane_inds) > 0:
        right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img

def fit_polynomial_show_windows(binary_warped):
    # Find our lane pixels first
    leftx, lefty, rightx, righty, out_img = find_lane_pixels2(binary_warped, False)

    ### Fit a second order polynomial to each using `np.polyfit` ###
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    try:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        left_fitx = 1*ploty**2 + 1*ploty
        right_fitx = 1*ploty**2 + 1*ploty

    ## Visualization ##
    # Colors in the left and right lane regions
    out_img[lefty, leftx] = [255, 0, 0]
    out_img[righty, rightx] = [0, 0, 255]

    # Plots the left and right polynomials on the lane lines
    plt.plot(left_fitx, ploty, color='yellow')
    plt.plot(right_fitx, ploty, color='yellow')

    return out_img, left_fit, right_fit

def curvature_and_offset(left_fit, right_fit):
    ym_per_pix = 13.3/100 # cm per pixel in y dimension
    xm_per_pix = 13.3/308 # cm per pixel in x dimension
    
    y_eval = 485   # bottom of image
    
    # Calculation of R_curve (radius of curvature)
    left_curverad  = ((1 + (2*(left_fit[0]/xm_per_pix)*y_eval*ym_per_pix + left_fit[1])**2)**1.5) / np.absolute(2*(left_fit[0]/xm_per_pix))
    right_curverad = ((1 + (2*(right_fit[0]/xm_per_pix)*y_eval*ym_per_pix + right_fit[1])**2)**1.5) / np.absolute(2*(right_fit[0]/xm_per_pix))
    
    # find offset
    camera_center = 324
    
    left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
    right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
    lane_center = (left_x + right_x) // 2
    
    offset = (lane_center - camera_center) * xm_per_pix
    ave_curvature = (left_curverad + right_curverad) / 2
    
    return ave_curvature, offset
cam = Picamera2()
cam_config = cam.create_video_configuration(main={"format" :"RGB888", "size": (648, 486)}, raw=cam.sensor_modes[1])
cam.configure(cam_config)
cam.start()
while True :
    img = cam.capture_array()
    cv2.imshow("img",img)
    binary_img = threshold(img,"yellow")    
    birds_eye = correct_perspective(binary_img)
    cv2.imshow("mask",birds_eye)
    try :
        out_img, left_fit, right_fit = fit_polynomial_show_windows(birds_eye)
    except TypeError :
        continue
    cv2.imshow("img",out_img)
    curvature, offset = curvature_and_offset(left_fit, right_fit)
    print('Lane Curvature:{:.2f}cm, Offset:{:.2f}cm'.format(curvature, offset))
    if cv2.waitKey(1) == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()



