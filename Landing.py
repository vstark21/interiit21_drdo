import os
import cv2
import math
import numpy as np
from cv2 import aruco

optimal_length = 100 # Need To Be Adjust
Square = [(pos[0] - optimal_length,pos[1]),
            (pos[0] + optimal_length,pos[1]),
            (pos[0] - optimal_length,pos[1] + optimal_length),
            (pos[0] + optimal_length,pos[1] + optimal_length)]
    
Visited = []
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

def Aruco(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    corners, ids, _ = aruco.detectMarkers(thresh, aruco_dict, parameters=aruco.DetectorParameters_create())
    
    Centres = []
    for i, corner in enumerate(corners):
        x = int((corner[0][0][0] + corner[0][2][0]) / 2)
        y = int((corner[0][0][1] + corner[0][2][1]) / 2)
        
        if ids[i][0] == 0:
            return [(x,y)], True
        
        Centres.append((x,y))
    
    return Centres, False

def White_Points(img):
    mask = cv2.inRange(img, np.array([100,100,100]), np.array([255,255,255]))
    img = cv2.bitwise_and(img, img, mask = mask)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray,(5,5),0)
    _, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CROSS, kernel, iterations = 5)

    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    Centres = []
    for contour in contours:
        if cv2.contourArea(contour) > 150:
            _, _, w, h = cv2.boundingRect(contour)
            if w/h > 0.5:
                M = cv2.moments(contour)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                Centres.append((cx,cy))
                
    return Centres

def Distance(A,B):
    return ((B[0] - A[0])**2 + (B[1] - A[1])**2)**(1/2)

def World_Pos(pitch, pos, centre):
    del_x = centre[0] - 480
    del_y = centre[1] - 640
    Complex = complex(math.cos(pitch), math.sin(pitch)) * complex(del_x, del_y)
    k = 100 ## Need to be Adjusted
    x = pos[0] + Complex.real / (k * pos[2])
    y = pos[1] + Complex.imag / (k * pos[2])
    return x, y

def No_Point():
    pass

def Main(img, pos, pitch):
    Centres, Flag = Aruco(img)
    
    if Flag:
        cx, cy = World_Pos(pitch, pos, Centres[0])
        return (cx, cy, pos[2]), True
    
    for Centre in Centres:
        world_pos = World_Pos(pitch, pos, Centre)
        Flag = False
        for P in Visited:
            if Distance(P, world_pos) < 20:
                Flag = True
                break
        if not Flag:
            Visited.append(world_pos)
    
    Unvisited = []
    Centres = White_Points(img)
    
    for Centre in Centres:
        world_pos = World_Pos(pitch, pos, Centre)
        Flag = False
        for P in Visited:
            if Distance(P, world_pos) < 50:
                Flag = True
                break
        if not Flag:
            Unvisited.append([Distance(pos, world_pos), world_pos])
    
    if len(Unvisited):
        Unvisited = sorted(Unvisited)
        cx, cy = Unvisited[0][1]
        return (cx, cy, pos[2]), False
    
    return No_Point()