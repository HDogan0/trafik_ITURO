def arrow(image):
    image = cv2.resize(image, (int(image.shape[1] * 0.5),int(image.shape[0] * 0.5)))
    hell_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    mask_kernel = np.ones((2, 2))

    lower_red = np.array([120,60,60])
    upper_red = np.array([150,255,255])
    mask = cv2.inRange(hell_hsv, lower_red, upper_red)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, mask_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, mask_kernel)
    normal_gray = cv2.cvtColor(cv2.cvtColor(image,cv2.COLOR_HSV2BGR),cv2.COLOR_BGR2GRAY)
    unmasked_gray = cv2.bitwise_and(255 - normal_gray, 255 - normal_gray, mask= cv2.bitwise_not(mask))
    if np.sum(mask)<100:
        return 0 
# Use Hough Circle Transform to detect circles
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=20, minRadius=0)
# If circles are found, draw them
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            #if cv2.inRange(hell_hsv[y,x], lower_red, upper_red) :
            #    return 4
            test = np.zeros(image.shape[:2],dtype=np.uint8)
            cv2.circle(test, (x, y), r, 1, -1)
            unmasked_gray = cv2.bitwise_and(unmasked_gray , unmasked_gray, mask=test)
            area = unmasked_gray[int(y-r):int(y+r),int(x-r):int(x+r)]
            #plt.imshow( cv2.cvtColor(area, cv2.COLOR_GRAY2BGR))
    try :
        maske = cv2.threshold(area, 170, 255, cv2.THRESH_BINARY)
        
        
    except UnboundLocalError :
        return 4
    area = cv2.bitwise_and(area,area, mask = maske[1])
    #weight_dict={"x":np.sum(area[:, :area.shape[0]//2]),"y":np.sum(area[:, area.shape[0]//2:]),"z":np.sum(area[:area.shape[1]//2,:]),"t": np.sum(area[area.shape[1]//2:,:])}
    weight_list = [np.sum(area[:, :area.shape[0]//2]),np.sum(area[:, area.shape[0]//2:]),np.sum(area[:area.shape[1]//2,:]), np.sum(area[area.shape[1]//2:,:])]
    #plt.imshow(area)
    #print(weight_list)
    #x = np.sum(area[:, :area.shape[0]//2])
    #y = np.sum(area[:, area.shape[0]//2:])

    # if x > y:
    #     return 1
    # elif  x < y :
    #     return 2
    # z = np.sum(area[:area.shape[1]//2,:])
    # t = np.sum(area[area.shape[1]//2:,:])
    # if z > t:
    #     return 3
    # return 0
    max_weight = weight_list.index(max(weight_list))
    print(weight_list)
    if max_weight==0 :
        return 1
    elif max_weight==1 :
        return 2
    elif max_weight==2 :
        return 3
    else :
        return 0
