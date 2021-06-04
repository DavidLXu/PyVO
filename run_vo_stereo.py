

# built-in
from enum import Enum
import time
import math

# 3rd-party
import cv2
from sophus import SE3,SO3 # sophuspy
import numpy as np
from scipy.spatial.transform import Rotation as R

# diy
from myconfig import Config
from mycamera import Camera
from myframe import Frame
from mymappoint import MapPoint
from mymap import Map
from myvisualodometry import VisualOdometry,VOState

from myplot import SVO_Plot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 视觉里程计的轨迹用matplotlib的动画模块来动态展现 (可能用不了这么多)
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


cv2.namedWindow("imgL", cv2.WINDOW_NORMAL)
cv2.namedWindow("imgR", cv2.WINDOW_NORMAL)
cv2.namedWindow("depth", cv2.WINDOW_NORMAL)

cv2.resizeWindow("imgL",850,300)
cv2.resizeWindow("imgR",850,300)
cv2.resizeWindow("depth",850,300)

cv2.moveWindow("imgL", 100, 0)
cv2.moveWindow("imgR", 100, 850)
cv2.moveWindow("depth", 100, 420)
#cv2.createTrackbar("num", "depth", 0, 10, lambda x: None)
#cv2.createTrackbar("blockSize", "depth", 5, 255, lambda x: None)



config = Config("config_kitti.yml")
dataset_dir = config.get("dataset_dir").string()
print("dataset path:",dataset_dir)

vo = VisualOdometry(config)
camera_kitti = Camera(config)

left_images = []
right_images = []
time_stamp = []

with open( dataset_dir+"/times.txt" ) as f:
    n = [line.strip('\n') for line in f.readlines()]    
    for line in n:
        time_stamp.append(float(line))

print("read total number of images:",len(time_stamp))
    

#fig = plt.figure()
#ax = fig.add_subplot(111, projection="3d")
#points = []


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi
# 可视化
Plotter = SVO_Plot(figSize=(8, 12))

euler_prev = 0


# 改成自动以时间命名，不然太容易覆盖掉了
file_trace =  open('trace_orb.txt','w')
time_all = time.time()

frame_num = len(time_stamp)-1
#for i in range(len(time_stamp)-1): #( int i=0 i<rgb_files.size() i++ )
for i in range(0,frame_num,2):
    '''
    调整的参数：
    myvisualodometry.py line 129: if m.distance < max(min_dis*self.match_ratio_,40): # 原为
    cv2.StereoBM_create(numDisparities=32, blockSize=15)
    myvisualodometry.py line 208: if ( d_norm > 1 ): # 5.0
    '''

    '''
    容易出现的问题，就是没有inliers: 原因是特征点找少了
    self.num_inliers_ = len(inliers) #inliers.rows
    TypeError: object of type 'NoneType' has no len()
    ''' 
    filename = "{:06d}.png".format(i)

    imgL = cv2.imread(dataset_dir + "/image_0/" + filename,0)
    imgR = cv2.imread(dataset_dir + "/image_1/" + filename,0)

    if ( imgL.data==None or imgR.data==None):
        break


    stereo = cv2.StereoBM_create(numDisparities=32, blockSize=15)
    disparity = stereo.compute(imgL,imgR)
    disparity = cv2.max(disparity,0)


    depth = 386.1448 / disparity
    _,depth = cv2.threshold(depth,100,255,cv2.THRESH_TOZERO_INV)
    depth = depth * 600
    depth = cv2.max(depth,0)

    depth = np.int16(depth)


    cv2.imshow("imgL",imgL) # 特征点可以画在左图上，有助于增强演示效果
    cv2.imshow("imgR",imgR)

    cv2.imshow("depth",depth)
    cv2.waitKey(1)

    print("================================")
    pFrame = Frame.createFrame()
    pFrame.camera_ = camera_kitti
    pFrame.color_ = imgL
    pFrame.depth_ = depth
    pFrame.time_stamp_ = time_stamp[i]

    vo.addFrame(pFrame)

    if(vo.state_ == VOState.LOST):
        break


    Tcw = pFrame.T_c_w_.inverse()
    trans = Tcw.translation()
    rot = R.from_matrix(Tcw.rotationMatrix()).as_quat()

    print("相机位置:",trans)
    print("相机角度:",rot) # 四元数

    #制作TUM数据集样式 timestamp tx ty tz qx qy qz qw
    file_trace.write(f"{time_stamp[i]} {trans[0]} {trans[1]} {trans[2]} {rot[0]} {rot[1]} {rot[2]} {rot[3]}\n")

    if i % 2 == 0:
        Plotter.draw_trajectory_3D(Tcw.rotationMatrix(),Tcw.translation(),Plotter.ax1,Plotter.ax2)
        Plotter.clear()
    

time_total = time.time()-time_all
print("extractKeyPoints time:\t",VisualOdometry.time_extract,"\tpercentage:",VisualOdometry.time_extract/time_total*100)
print("computeDescriptors time:",VisualOdometry.time_compute,"\tpercentage:",VisualOdometry.time_compute/time_total*100)
print("featureMatching time:\t",VisualOdometry.time_match,"\tpercentage:",VisualOdometry.time_match/time_total*100)
print("poseEstimationPnP time:\t",VisualOdometry.time_pnp,"\tpercentage:",VisualOdometry.time_pnp/time_total*100)

print("total time:",time_total)
print("total frames:",frame_num)



file_trace.close()

print("saved to trace.txt")


