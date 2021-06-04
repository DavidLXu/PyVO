'''
SIFT特征版本，改动了第47，48行 self.orb_= cv2.SIFT_create()
和featureMatching函数 第139，140行

'''
# built-in
from enum import Enum
import time

# 3rd-party
import cv2
from sophus import SE3,SO3 # sophuspy
#from mysophus import SE3,SO3

import numpy as np

# diy
from myconfig import Config
from mycamera import Camera
from myframe import Frame
from mymappoint import MapPoint
from mymap import Map

'''
Python 中 opencv keypoint 坐标获取
point_x = cv2.KeyPoint.pt[0]
point_y = cv2.KeyPoint.pt[1]
(x,y) = cv2.KeyPoint.pt
'''
class VOState(Enum):
    """视觉里程计的运行状态"""
    INITIALIZING = -1
    OK = 0
    LOST = 1

class VisualOdometry:
    time_extract = 0
    time_compute = 0
    time_match = 0
    time_pnp = 0

    def __init__(self,config):
        self.state_         = VOState.INITIALIZING  # current VO
        self.ref_           = None                  # Frame ptr in cpp, reference frame 
        self.curr_          = None                  # Frame ptr in cpp, current frame 
        self.map_           = Map()                 # new Map(), map with all frames and map points

        
        self.orb_               = cv2.ORB_create(nfeatures=2500)  # cv::Ptr<cv::ORB>          orb detector and computer 
        #self.orb_               = cv2.SIFT_create()
        self.pts_3d_ref_        = []                # vector<cv::Point3f>       3d points in reference frame, here in python: list of arrays
        self.keypoints_curr_    = []                # vector<cv::KeyPoint>      keypoints in current frame
        self.descriptors_curr_  = None              # Mat                       descriptor in current frame 
        self.descriptors_ref_   = None              # Mat                       descriptor in reference frame 
        self.feature_matches_   = []                # vector<cv::DMatch>      
        
        #self.T_c_r_estimated_   = SE3()     # SE3       the estimated pose of current frame       
        self.num_lost_          = 0         # int       number of inlier features in icp
        self.num_inliers_       = 0         # int       number of lost times

        # parameters 
        self.num_of_features_   = config.get("number_of_features").real()   # int       number of features
        self.scale_factor_      = config.get("scale_factor").real()         # double    scale in image pyramid
        self.level_pyramid_     = config.get("level_pyramid").real()        # int       number of pyramid levels
        self.match_ratio_       = config.get("match_ratio").real()          # float     ratio for selecting  good matches
        self.max_num_lost_      = config.get("max_num_lost").real()         # int       max number of continuous lost times
        self.min_inliers_       = config.get("min_inliers").real()          # int       minimum inliers
        
        self.key_frame_min_rot  = config.get("keyframe_rotation").real()    # double    minimal rotation of two key-frames
        self.key_frame_min_trans= config.get("keyframe_translation").real() # double    minimal translation of two key-frames 

        # equivalent C++
        # num_of_features_    = Config::get<int> ( "number_of_features" );
        # scale_factor_       = Config::get<double> ( "scale_factor" );
        # level_pyramid_      = Config::get<int> ( "level_pyramid" );
        # match_ratio_        = Config::get<float> ( "match_ratio" );
        # max_num_lost_       = Config::get<float> ( "max_num_lost" );
        # min_inliers_        = Config::get<int> ( "min_inliers" );
        # key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        # key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        # orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );

    def addFrame(self,frame): # bool:  Frame::Ptr     # add a new frame 
        if self.state_ == VOState.INITIALIZING:
            self.state_ = VOState.OK
            self.curr_ = self.ref_ = frame  # 第一帧当做reference frame
            self.map_.insertKeyFrame(frame) # 第一帧当做key frame
            # extract features from first frame 
            self.extractKeyPoints()
            self.computeDescriptors()
            # compute the 3d position of features in ref frame 
            self.setRef3DPoints()
            #break
        
        elif self.state_ == VOState.OK:
            self.curr_ = frame            # 更新当前帧
            self.extractKeyPoints()
            self.computeDescriptors()
            self.featureMatching()
            self.poseEstimationPnP()
            if ( self.checkEstimatedPose() == True ): # a good estimation
                self.curr_.T_c_w_ = self.T_c_r_estimated_ * self.ref_.T_c_w_  # T_c_w = T_c_r*T_r_w 
                self.ref_ = self.curr_
                self.setRef3DPoints() # 设为参考，为下一次matching做准本
                self.num_lost_ = 0
                if ( self.checkKeyFrame() == True ): # is a key-frame                
                    self.addKeyFrame()
                
            else: # bad estimation due to various reasons
                self.num_lost_+=1
                if ( self.num_lost_ > self.max_num_lost_ ):             
                    self.state_ = VOState.LOST
                
                return False
            #break
        
        elif self.state_ == VOState.LOST:
            print("vo has lost.")
            #break
        
        return True

    # inner operation 
    def extractKeyPoints(self):
        a = time.time()        
        self.keypoints_curr_ = self.orb_.detect(self.curr_.color_, None)
        VisualOdometry.time_extract += time.time() - a
        #print("1.",len(self.keypoints_curr_))

    def computeDescriptors(self): 
        b = time.time()
        self.keypoints_curr_, self.descriptors_curr_ = self.orb_.compute(self.curr_.color_, self.keypoints_curr_)
        VisualOdometry.time_compute += time.time() - b
        #print("2.",len(self.keypoints_curr_)) 两个数量相等
        #print(self.descriptors_curr_)

    def featureMatching(self):
        c = time.time()
        # match desp_ref and desp_curr, use OpenCV's brute force match 
        matches = [] # vector<cv::DMatch>
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
        #matcher = cv2.BFMatcher()#cv2.NORM_HAMMING,crossCheck=True) #建立匹配关系
        matches = matcher.match(self.descriptors_ref_,self.descriptors_curr_) #匹配描述子
        #matches = matcher.knnMatch(self.descriptors_ref_,self.descriptors_curr_, k=1)
        matches=sorted(matches,key=lambda x:x.distance) #据距离来排序，C++源码还挺麻烦
        min_dis = matches[0].distance # 最小的距离（应该是误差）

        self.feature_matches_.clear()
        for m in matches:
            if m.distance < max(min_dis*self.match_ratio_,75): #60 # 对于kitti 45 不错。调整后面这个数字，会影响的good matches的个数 ,高翔的版本是30
                self.feature_matches_.append(m)
        print("good matches:",len(self.feature_matches_))
        VisualOdometry.time_match += time.time() - c

        """
        # 更快的FLANN方法
        FLANN_INDEX_KDTREE = 0
        indexParams = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        searchParams = dict(checks=50)
        flann = cv2.FlannBasedMatcher(indexParams, searchParams)
        matches = flann.knnMatch(self.descriptors_ref_,self.descriptors_curr_, k=1)
        matches = [m[0] for m in matches]
        #print(matches)
        matches=sorted(matches,key=lambda x:x.distance) #据距离来排序，C++源码还挺麻烦
        min_dis = matches[0].distance # 最小的距离（应该是误差）

        self.feature_matches_.clear()
        for m in matches:
            if m.distance < max(min_dis*self.match_ratio_,60):  # 对于kitti 45 不错。调整后面这个数字，会影响的good matches的个数 ,高翔的版本是30
                self.feature_matches_.append(m)
        print("good matches:",len(self.feature_matches_))
        """
        '''
        matchesMask = [[0, 0] for i in range(len(matches))]
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.7 * n.distance:
                matchesMask[i] = [1, 0]
        drawParams = dict(matchColor=(0, 255, 0),
                        singlePointColor=(255, 0, 0),
                        matchesMask=matchesMask,
                        flags=0
                        )
        resultImage = cv2.drawMatchesKnn(queryImage, kp1, trainingImage, kp2, matches, None, **drawParams)
        plt.xticks([]), plt.yticks([])
        plt.imshow(resultImage), plt.show()
        '''

    def setRef3DPoints(self):
        # select the features with depth measurements
        self.pts_3d_ref_        = []    # 重设参考点
        self.descriptors_ref_   = []    # Mat()
        print("设定参考坐标系")
        for i in range(len(self.keypoints_curr_)):
            d = self.ref_.findDepth(self.keypoints_curr_[i])
            if d > 0:
                p_cam = self.ref_.camera_.pixel2camera(self.keypoints_curr_[i].pt,d)
                self.pts_3d_ref_.append(p_cam)
                self.descriptors_ref_.append(self.descriptors_curr_[i]) # 为什么是32个数字

        self.pts_3d_ref_ = np.vstack(self.pts_3d_ref_)  # list of arrays -> array
        self.descriptors_ref_ = np.vstack(self.descriptors_ref_)

        print(f"- {len(self.pts_3d_ref_)} 3D reference points created.")
        print(f"- {len(self.descriptors_ref_)} reference descritptors created.")


    def poseEstimationPnP(self): 
        d = time.time()
        # construct the 3d 2d observations
        pts3d = []  # vector<cv::Point3f> 
        pts2d = []  # vector<cv::Point2f> 
        
        for m in self.feature_matches_:
        
            pts3d.append(self.pts_3d_ref_[m.queryIdx] )         # list of arrays of 3d points
            pts2d.append(self.keypoints_curr_[m.trainIdx].pt )  # list of tuples of 2d points

        # 转换成numpy格式    
        pts3d = np.vstack(pts3d)
        pts2d = np.vstack(pts2d)


        K = np.array([[self.ref_.camera_.fx_,0,self.ref_.camera_.cx_],
                      [0,self.ref_.camera_.fy_,self.ref_.camera_.cy_],
                      [0,0,1]])

        _, rvec, tvec, inliers = cv2.solvePnPRansac(objectPoints = pts3d.reshape(-1, 1, 3), 
                                     imagePoints = pts2d.reshape(-1, 1, 2),
                                     cameraMatrix = K,
                                     distCoeffs = np.array([]),
                                     ) #https://blog.csdn.net/SSG18829575503/article/details/89504261
        
        # inliers 可能是零，无法len()
        # self.num_inliers_ = len(inliers) #inliers.rows
        # print("PnP inliers:",self.num_inliers_)

        try:                             
            self.num_inliers_ = len(inliers) #inliers.rows
            print("PnP inliers:",self.num_inliers_)
        except:
            self.num_inliers_ = 0 #inliers.rows
            print("PnP inliers:",self.num_inliers_)
        #print(rvec)
        #print(tvec)
        
        self.T_c_r_estimated_ = SE3(SO3.exp(rvec).matrix(),tvec.reshape(3)) # 这段代码调了很长时间，终于成了

        VisualOdometry.time_pnp += time.time() - d

    

    def addKeyFrame(self):
        '''
        把当前帧设为关键帧，个人认为setKeyFrame更好一些，这里保持和C++版本的一致性
        '''
        print("adding a key-frame")
        self.map_.insertKeyFrame ( self.curr_ )

    def checkEstimatedPose(self):   # bool
        # check if the estimated pose is good
        if ( self.num_inliers_ < self.min_inliers_ ):
            print("reject because inlier is too small:",self.num_inliers_)
            return False

        # if the motion is too large, it is probably wrong
        d = self.T_c_r_estimated_.log()
        d_norm = np.linalg.norm(d)
        if ( d_norm > 1 ): # 在tum是5.0
            print("reject because motion is too large:",d_norm)
            return False
        
        return True

    def checkKeyFrame(self):        # bool
        d = self.T_c_r_estimated_.log()
        trans = d[:3]
        rot = d[3:]
        #print(trans)
        #print(rot)
        if ( np.linalg.norm(rot) > self.key_frame_min_rot or np.linalg.norm(trans) > self.key_frame_min_trans ):
            # 多于这些旋转和平移，就可以认为有较大的变化，就可以设为关键帧了
            return True


if __name__ == "__main__":

    # VOState.LOST 还没有测试
    

    config = Config("config.yml")
    vo = VisualOdometry(config)
    camera_tum = Camera(config=config)

    # 测试了三帧，都没有问题，照片来自rgbd_dataset_freiburg3_long_office_household的第1，3，5，60 张

    # 添加第一帧
    print("=====Adding 1st Frame=====")
    color_img = cv2.imread("images/1341847980.722988.png")
    depth_img = cv2.imread("images/1341847980.723020.png",-1) # 如果这里不加-1,通道数会有问题,findDepth的时候会出错
    frame1 = Frame.createFrame()           # 内部会调用构造函数
    frame1.camera_ = camera_tum
    frame1.color_ = color_img
    frame1.depth_ = depth_img
    
    vo.addFrame(frame1) # 测试 VOState.INITIALIZING

    # 添加第二帧
    print("=====Adding 2nd Frame=====")
    color_img2 = cv2.imread("images/1341847980.786856.png")
    depth_img2 = cv2.imread("images/1341847980.786879.png",-1)    
    frame2 = Frame.createFrame()
    frame2.camera_ = camera_tum
    frame2.color_ = color_img2
    frame2.depth_ = depth_img2
    vo.addFrame(frame2) # 测试 VOState.OK
    # vo.addKeyFrame() 转换当前帧为关键帧

    # 添加第三帧
    print("=====Adding 3rd Frame=====")
    color_img3 = cv2.imread("images/1341847980.854676.png")
    depth_img3 = cv2.imread("images/1341847980.854690.png",-1) 
    frame3 = Frame.createFrame()
    frame3.camera_ = camera_tum
    frame3.color_ = color_img3
    frame3.depth_ = depth_img3
    vo.addFrame(frame3) # 测试 VOState.OK


    # 添加第四帧
    print("=====Adding 4th Frame=====")
    color_img4 = cv2.imread("images/1341847982.698638.png")
    depth_img4 = cv2.imread("images/1341847982.798982.png",-1) 
    frame4 = Frame.createFrame()
    frame4.camera_ = camera_tum
    frame4.color_ = color_img4
    frame4.depth_ = depth_img4
    vo.addFrame(frame4) 
    # 因为是第60张，跟前面变化较大，经测试成为了关键帧；
    # 也尝试过时间靠后变化更大的，reject because inlier is too small: 20

    # 除了VOState.LOST 没测试，别的应该都没有问题，可以写run_vo了
