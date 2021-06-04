import cv2
from myconfig import Config
from mycamera import Camera
import numpy as np

# 关于李代数的库有两个: sophuspy 和 pysophus(需要git clone)
from sophus import SO3,SE3

class Frame:
    factory_id = 0
    def __init__(self,id=-1,time_stamp=-1,T_c_w=SE3(),camera=None,color=None,depth=None):
        self.id_ = id
        self.time_stamp_ = time_stamp
        self.T_c_w_ = T_c_w
        self.camera_ = camera
        # 这里的行列容易搞错
        self.color_ = color # RGB图 Mat color_.shape[0] 是有几行，color_.shape[1]是有几列 color_[y,x] 第y行第x列
        self.depth_ = depth # 深度图 Mat depth_.shape[0] 是有几行，depth_.shape[1]是有几列 color_[y,x] 第y行第x列
        '''
        print("Frame initialized")
        print(f"id = {self.id_}")
        print(f"time_stamp = {self.time_stamp_}")
        print(f"T_c_w = {self.T_c_w_}")
        print(f"camera = {self.camera_}")
        print(f"color = {self.color_}")
        print(f"depth = {self.depth_}")
        '''


    @classmethod
    def createFrame(self):
        #factory_id = 0 # 静态变量，在外面
        frame = Frame(Frame.factory_id)
        print(f"Frame with factory_id {Frame.factory_id} created.")
        Frame.factory_id+=1
        return frame

    
    def findDepth(self,kp): # const cv::KeyPoint&

        #x = round(kp.x)
        #y = round(kp.y)
        x = round(kp.pt[0])
        y = round(kp.pt[1])
        #print("x,y:",x,y)
        #print(self.depth_)
        d = self.depth_[y,x] # 这里的行列容易搞错   ushort d = depth_.ptr<ushort>(y)[x]; // prt<T>(列)[行]
        #print("d",d)
        if ( d!=0 ):
    
            return d / self.camera_.depth_scale_
    
        else:
            # check the nearby points 
            dx = [-1,0,1,0]
            dy = [0,-1,0,1]
            for i in range(4):
                d = self.depth_[y+dy[i],x+dx[i]]  # d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
                if ( d!=0 ):                
                    return d/self.camera_.depth_scale_
                
            
        # 如果 nearby points 也都是 0，就返回 -1
        return -1.0
    


    def getCamCenter(self):
        return self.T_c_w_.inverse().translation()


    def isInFrame(self,pt_world): # const Vector3d&
        p_cam = self.camera_.world2camera( pt_world, self.T_c_w_ ) # Vector3d 
        print(p_cam)
        if ( p_cam[2]<0 ):
             return False
        pixel = self.camera_.world2pixel( pt_world, self.T_c_w_ ) # Vector2d
        print(pixel)
        return (pixel[0]>0 and pixel[1]>0 and pixel[0]<self.color_.shape[0] and pixel[1]<self.color_.shape[1])


class KP:
    def __init__(self,x,y):
        self.pt = (x,y)
        #self.y = y


if __name__ == "__main__":




    Frame.createFrame()
    Frame.createFrame()
    Frame.createFrame()
    Frame.createFrame()

    config = Config("config.yml")
    camera = Camera(config)
    src_depth = cv2.imread('images/1311878198.898057.png',flags = -1)
    src_color = cv2.imread("images/1341847980.722988.png")
    #print(dir(src_color))
    #print(src_color.shape)
    frame1 = Frame(depth = src_depth,color = src_color,camera=camera)
    kp = KP(400,400)
    #print(frame1.findDepth(kp))

    point = np.array([0,2,3])#.reshape(3,1) 对行列还是一维 要求不大
    #print(point)

    print(frame1.isInFrame(point))

