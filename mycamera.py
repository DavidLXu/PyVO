import numpy as np
import cv2

from myconfig import Config

# 通过编写这段代码，可以很好地理解三种坐标系的互相转换过程

class Camera:
    def __init__(self,config): #(self,fx = -1,fy = -1 , cx = -1,cy = -1,depth_scale = -1 ):
        
            self.fx_ = config.get("camera.fx").real()
            self.fy_ = config.get("camera.fy").real()
            self.cx_ = config.get("camera.cx").real()
            self.cy_ = config.get("camera.cy").real()
            self.depth_scale_ = config.get("camera.depth_scale").real()

            # baseline * focal
            self.bf_ = config.get("camera.bf").real()


            print("相机初始化完成")
            print(f"- fx = {self.fx_}")
            print(f"- fy = {self.fy_}")
            print(f"- cx = {self.cx_}")
            print(f"- cy = {self.cy_}")
            print(f"- depth_scale = {self.depth_scale_}")


    def world2camera (self, p_w, T_c_w):
        '''
        世界坐标系到相机坐标系 3d-3d
        p_w:    const Vector3d&
        T_c_w:  const SE3&

        '''
        return T_c_w*p_w


    def camera2world ( self, p_c, T_c_w ):
        '''
        相机坐标系到世界坐标系 3d-3d
        p_c:    const Vector3d&
        T_c_w:  const SE3& 
        '''

        return np.linalg.inv(T_c_w)*p_c


    def camera2pixel ( self,p_c ):
        '''
        相机坐标系到像素平面 3d-2d
        p_c:    const Vector3d& 
        '''
        return np.array([
            self.fx_ * p_c[0] / p_c[2] + self.cx_,
            self.fy_ * p_c[1] / p_c[2] + self.cy_])


    def pixel2camera (self,  p_p,  depth ):
        '''
        像素平面到相机坐标系 2d-3d
        p_p:    const Vector2d& 
        depth:  double
        '''
        return np.array ([
            ( p_p[0]-self.cx_ ) *depth/self.fx_,
            ( p_p[1]-self.cy_ ) *depth/self.fy_,
            depth]
        )


    def world2pixel (self, p_w, T_c_w ):
        '''
        世界坐标系像素平面 3d-2d
        p_w:    const Vector3d& 
        T_c_w:  const SE3& 
        '''

        return self.camera2pixel ( self.world2camera ( p_w, T_c_w ) )


    def pixel2world (self, p_p, T_c_w, depth ):
        '''
        世界坐标系像素平面 3d-2d
        p_p:    const Vector2d& 
        T_c_w:  const SE3& 
        depth:  double
        '''

        return self.camera2world ( self.pixel2camera ( p_p, depth ), T_c_w )

if __name__ == "__main__":

    # TUM数据集的camera内参
    '''
    camera.fx: 517.3
    camera.fy: 516.5
    camera.cx: 325.1
    camera.cy: 249.7
    '''
    config = Config("config.yml")
    camera_tum = Camera(config)
    
    # point = [1,2,3] 
    point = np.array([1,2,3])
    # 不好用，需要是一维向量：point = np.array([1,2,3]).reshape(3,1)
    print(camera_tum.camera2pixel(point))
    print(camera_tum.pixel2camera(camera_tum.camera2pixel(point),3))

