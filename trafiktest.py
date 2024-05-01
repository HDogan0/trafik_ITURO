import numpy as np
import cv2

def correct_perspective(img):
    pts=np.float32([[100,300],[500,300],
                    [10,480],[590,480]])
    dst=np.float32([[10,200],[590,200],
                    [10,480],[590,480]])
    mat=cv2.getPerspectiveTransform(pts,dst)
    tr_img=cv2.warpPerspective(img,mat,(350,150))
    return tr_img
    
def treshold(img,color,) :
    img=cv2.blur(img,(3,3))
    kernel = np.ones((3, 3), np.uint8)
    hsv=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    if color=="red":
        mask=cv2.inRange(hsv,np.array([110,50,165]),np.array([125,255,255]))
    elif color=="yellow" :
        mask=cv2.inRange(hsv,np.array([10,50,165]),np.array([30,255,255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1) 
    return mask
def find_lanes(mask):
    #find contours
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    #check number of contours
    if len(contours)==2:
        angs=[]
        pts=[]
        halfwidths=[]
        #record angles, centers, widths for every contour
        for cnt in contours :           
            blackbox=cv2.minAreaRect(cnt)  
            (x,y),(w,h),ang = blackbox   
            if w < h :
                ang = (ang+90)
                halfwidths.append(h/2)
            else:
                halfwidths.append(w/2)    
            pts.append((x,y))
            angs.append(ang)
        #find target line
        target1=(((pts[0][0]+pts[1][0])/2,(pts[0][1]+pts[1][1])/2))
        target0=((target1[0]-sum(halfwidths)/2*np.cos(sum(angs)/2*np.pi/180-np.pi),target1[1]+sum(halfwidths)/2*np.sin(sum(angs)/2*np.pi/180)))
        #find errors
        #ang_error=sum(ang)/2-90
        #dist_error=((target0[0]-np.shape(mask)[0]/2)**2+(target0[1]-np.shape(mask)[1])**2)**0.5
        return target1,target0
    elif len(contours)==1 :
        pass
    elif len(contours)>2 :
        pass
    else :
        pass
video = cv2.VideoCapture(0) 
while(True): 
    ret, img = video.read() 
  
    if ret == True:   
        img=correct_perspective(img)
        mask=treshold(img,"yellow")
        # Display the frame 
        cv2.imshow('mask',mask) 
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        if len(contours) !=2 :
            continue
        z=np.zeros(np.shape(mask))
        angs=[]
        pts=[]
        halfwidths=[]
        for cnt in contours :
            blackbox=cv2.minAreaRect(cnt) 
            (x,y),(w,h),ang = blackbox
            
            if w < h :
        
                ang+=90
        #print("!",ang)
                halfwidths.append(h/2)
            else:
                halfwidths.append(w/2)
    
            pts.append((x,y))
            angs.append(ang)
    #print(blackbox)
            box = cv2.boxPoints(blackbox)
    #print(box)
            box = np.int64(box)
            cv2.drawContours(z, [box], -1, (255),1)
#print(pts,halfwidths,angs)
        target1=np.int64(((pts[0][0]+pts[1][0])/2,(pts[0][1]+pts[1][1])/2))
        target0=np.int64((target1[0]-sum(halfwidths)/2*np.cos(sum(angs)/2*np.pi/180-np.pi),target1[1]+sum(halfwidths)/2*np.sin(sum(angs)/2*np.pi/180)))
        cv2.line(z,target0,target1,(255),3)
        # Press S on keyboard  
        # to stop the process 
        cv2.imshow("line",z)
        if cv2.waitKey(1) & 0xFF == ord('s'): 
            break
  
    # Break the loop 
    else: 
        break
