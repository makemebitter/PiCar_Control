import numpy as np
from math import *
import pyzbar.pyzbar as pyzbar
import cv2
from collections import defaultdict

length_ofedge=0.16

def read_and_distort(cap,mtx,dist):
    rat, frame = cap.read()
    
    if rat:
        height_ymax,width_xmax,channel=frame.shape
#         newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(width_xmax,height_ymax),1,(width_xmax,height_ymax))
        frame_distored = cv2.undistort(frame, mtx, dist, None)
    else:
        print 'read_and_distort: no image'
        return None
    return frame_distored,frame,height_ymax,width_xmax
def QR_scan_decode(img) : 
    # Find barcodes and QR codes
    
    decodedObjects = pyzbar.decode(img)
#     {name:locations}
    # Print results
    name_locations={int(obj.data):np.array(obj.location).astype(np.int32) for obj in decodedObjects if obj.type=='QRCODE'}
    return name_locations

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else :
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def sample_and_avg(cap,mtx,dist, n):
    total_dict=defaultdict(list)
    for _ in range(n):
        frame,frame_undistorted,height_ymax,width_xmax=read_and_distort(cap,mtx,dist)
        name_locations = QR_scan_decode(frame)
        for lmk,lc in name_locations.items():
            total_dict[lmk].append(lc)
    for lmk,lc in total_dict.items():
        total_dict[lmk]=np.average(np.array(lc),axis=0).astype(np.int32)
    return total_dict,frame
    
def draw_data(frame,pts,lc,pos,angle,dis,bearing):
    cv2.polylines(frame,[pts],True,(255,0,0), 3)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for h in lc:
        cv2.putText(frame,str(h),tuple(h), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame,'Pos of camera in lmk frame: '+str(pos[:,0]),(50,50), font, 0.5,(255,255,0),2,cv2.LINE_AA)
        cv2.putText(frame,'Z-Euler angle of object: '+str(angle),(50,100), font, 0.5,(255,255,0),2,cv2.LINE_AA)
        cv2.putText(frame,'Distance: '+str(dis),(50,150), font, 0.5,(255,255,0),2,cv2.LINE_AA)
        cv2.putText(frame,'Bearing wrt camera frame: '+str(bearing*180/pi)+'d',(50,200), font, 0.5,(255,255,0),2,cv2.LINE_AA)
    return frame
def QR_bearing_dis(cap,mtx,dist,mtx_fool,objp,sample_rate=50,draw=False):
    
    name_locations,frame=sample_and_avg(cap,mtx,dist, sample_rate)
#     if name_locations:
#         print name_locations
    observed=np.array(name_locations.keys())
    name_dis_bearing={}
    for lmk,lc in name_locations.items():
        center_line=303.790334392223
        qr_center=np.average(lc, axis=0)[0]
        x_dif=center_line-qr_center
        pts=lc.reshape(-1,1,2)
        ret,rvecs, tvecs = cv2.solvePnP(objp,lc.astype(np.float32), mtx_fool, np.array([]))
        Rt = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvecs,Rt)
        R = Rt.transpose()
        pos=-R.dot(tvecs)
        euler_angles=rotationMatrixToEulerAngles(R)
        angle=[euler_angle*180/pi for euler_angle in euler_angles][1]
        dis=np.linalg.norm(pos)
        bearing=atan2(x_dif/625,dis)
        name_dis_bearing[lmk]=np.vstack([dis,bearing])
        if draw:
            draw_data(frame,pts,lc,pos,angle,dis,bearing)
    return name_dis_bearing,frame
