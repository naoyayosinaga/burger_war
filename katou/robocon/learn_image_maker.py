import numpy as np
import cv2

def map_init():

    map = np.zeros((250,250),np.uint8)
    map = cv2.rectangle(map,(5,5),(244,244),color=255,thickness=1)
    map = cv2.drawMarker(map,(125,125),255,markerType=cv2.MARKER_DIAMOND,markerSize=35,thickness=1)

    #center
    pts = np.array([[100,125],[125,100],[150,125],[125,150]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],255)    

     #block1
    pts = np.array([[125,36],[139,50],[125,64],[111,50]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],255)
   
     #block2
    pts = np.array([[200,111],[214,125],[200,139],[186,125]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],255)

     #block3
    pts = np.array([[125,186],[139,200],[125,214],[111,200]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],255)

    #block4
    pts = np.array([[50,111],[64,125],[50,139],[36,125]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],255)

    
    return map


 

def learn_image_maker(r1x,r1y,r1_theta,r2x,r2y,m11,m12,m21,m22,m31,m32,m41,m42,mc1,mc2,mc3,mc4,rm1,rm2,rm3,erm1,erm2,erm3):#r1 =robot r2 = enemy
#def learn_image_maker():
    map = map_init()
    
    #point state
    if m11 == 1:
        map = cv2.line(map,(125,36),(111,50),170,thickness=2)
    elif m11 == -1:
        map = cv2.line(map,(125,36),(111,50),85,thickness=2)
    
    if m12 == 1:
        map = cv2.line(map,(139,50),(125,64),170,thickness=2)
    elif m12 == -1:
        map = cv2.line(map,(139,50),(125,64),85,thickness=2)

    if m21 == 1:
        map = cv2.line(map,(200,111),(186,125),170,thickness=2)
    elif m21 == -1:
        map = cv2.line(map,(200,111),(186,125),85,thickness=2)
    if m22 == 1:
        map = cv2.line(map,(214,125),(200,139),170,thickness=2)
    elif m22 == -1:
        map = cv2.line(map,(214,125),(200,139),85,thickness=2)


    if m31 == 1:
        map = cv2.line(map,(125,186),(111,200),170,thickness=2)
    elif m31 == -1:
        map = cv2.line(map,(125,186),(111,200),85,thickness=2)
    if m32 == 1:
        map = cv2.line(map,(139,200),(125,214),170,thickness=2)
    elif m32 == -1:
        map = cv2.line(map,(139,200),(125,214),85,thickness=2)

    if m41 == 1:
        map = cv2.line(map,(50,111),(36,125),170,thickness=2)
    elif m41 == -1:
        map = cv2.line(map,(50,111),(36,125),85,thickness=2)
    if m42 == 1:
        map = cv2.line(map,(64,125),(50,139),170,thickness=2)
    elif m42 == -1:
        map = cv2.line(map,(64,125),(50,139),85,thickness=2)

    if mc1 == 1:
        map = cv2.line(map,(125,100),(100,125),170,thickness=2)
    elif mc1 == -1:
        map = cv2.line(map,(125,100),(100,125),85,thickness=2)
    if mc2 == 1:
        map = cv2.line(map,(125,100),(150,125),170,thickness=2)
    elif mc2 == -1:
        map = cv2.line(map,(125,100),(150,125),85,thickness=2)
    if mc3 == 1:
        map = cv2.line(map,(150,125),(125,150),170,thickness=2)
    elif mc3 == -1:
        map = cv2.line(map,(150,125),(125,150),85,thickness=2)
    if mc4 == 1:
        map = cv2.line(map,(125,150),(100,125),170,thickness=2)
    elif mc4 == -1:
        map = cv2.line(map,(125,150),(100,125),85,thickness=2)
    

    #robot_state
     #robot1
    pts = np.array([[r1x,r1y-13],[r1x+13,r1y],[r1x,r1y+13],[r1x-13,r1y]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],170)
     #robot rotation
    if r1_theta == 1:
        map = cv2.drawMarker(map,(r1x-7,r1y-7), 255, markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
        #robot marker
        if rm1 == 1:
            map = cv2.line(map,(r1x,r1y-13),(r1x+13,r1y),85,thickness=2)
        if rm2 == 1:
            map = cv2.line(map,(r1x+13,r1y),(r1x,r1y+13),85,thickness=2)
        if rm3 == 1:
            map = cv2.line(map,(r1x,r1y+13),(r1x-13,r1y),85,thickness=2)
            
    elif r1_theta ==2:
        map = cv2.drawMarker(map,(r1x+7,r1y-7), 255, markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
        #robot marker
        if rm1 == 1:
            map = cv2.line(map,(r1x+13,r1y),(r1x,r1y+13),85,thickness=2)
        if rm2 == 1:
            map = cv2.line(map,(r1x,r1y+13),(r1x-13,r1y),85,thickness=2)
        if rm3 == 1:
            map = cv2.line(map,(r1x,r1y-13),(r1x-13,r1y),85,thickness=2)

    elif r1_theta ==3:
        map = cv2.drawMarker(map,(r1x+7,r1y+7), 255, markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
        #robot marker
        if rm1 == 1:
            map = cv2.line(map,(r1x,r1y+13),(r1x-13,r1y),85,thickness=2)
        if rm2 == 1:
            map = cv2.line(map,(r1x,r1y-13),(r1x-13,r1y),85,thickness=2)
        if rm3 == 1:
            map = cv2.line(map,(r1x+13,r1y),(r1x,r1y-13),85,thickness=2)
            
    elif r1_theta ==4:
        map = cv2.drawMarker(map,(r1x-7,r1y+7), 255, markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
        #robot marker
        if rm1 == 1:
            map = cv2.line(map,(r1x,r1y-13),(r1x-13,r1y),85,thickness=2)
        if rm2 == 1:
            map = cv2.line(map,(r1x+13,r1y),(r1x,r1y-13),85,thickness=2)
        if rm3 == 1:
            map = cv2.line(map,(r1x,r1y+13),(r1x+13,r1y),85,thickness=2)

    #robot2
    pts = np.array([[r2x,r2y-13],[r2x+13,r2y],[r2x,r2y+13],[r2x-13,r2y]], np.int32)
    pts = pts.reshape((-1,1,2))
    map = cv2.fillPoly(map,[pts],85)
    map = cv2.drawMarker(map,(r2x+7,r2y+7), 255, markerType=cv2.MARKER_TILTED_CROSS,thickness=1)
    #robot marker
    if erm1 == 1:
        map = cv2.line(map,(r2x,r2y+13),(r2x-13,r2y),170,thickness=2)
    if erm2 == 1:
        map = cv2.line(map,(r2x,r2y-13),(r2x-13,r2y),170,thickness=2)
    if erm3 == 1:
        map = cv2.line(map,(r2x+13,r2y),(r2x,r2y-13),170,thickness=2)



    
    #cv2.imwrite('test.png',map)
    return map

def learn_image_init():

    map = learn_image_maker(226,226,1,23,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)

    return map
    
if __name__ == '__main__':

    #learn_image_maker(229,20,1,23,23,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,1,1,1,1,1)
    map =  learn_image_init()

    center_xylist = np.array([[125,50],[200,125],[125,200],[50,125],[125,125]])

    for i in range(4):

        map = cv2.circle(map, (center_xylist[i][0], center_xylist[i][1]), 70,255)
        map = cv2.circle(map, (center_xylist[i][0], center_xylist[i][1]), 40,255)

        map = cv2.circle(map, (center_xylist[i][0], center_xylist[i][1]), 50,255)

        map = cv2.circle(map, (center_xylist[i][0], center_xylist[i][1]), 60,255)
    #map = cv2.circle(map, (center_xylist[4][0], center_xylist[4][1]), 30,255)

    #map = cv2.circle(map, (center_xylist[4][0], center_xylist[4][1]), 35,255)
    
    map = cv2.circle(map, (center_xylist[4][0], center_xylist[4][1]), 49,255)

    cv2.imwrite('test.png',map)
