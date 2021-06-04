#可视化部分，参考GitHub Stereo-Visual-Odometry-SFM，研究是如何绘图的。

#五个窗口 三个plot窗口 两个cv窗口

import cv2
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class SVO_Plot():

    def __init__(self, figSize):
        
        fig1 = plt.figure(figsize=figSize)
        plt.subplot(2,1,1)
        plt.axis('off')
        #plt.title('Schematic Representation of Camera in 3D')
        self.ax1 = fig1.add_subplot(211, projection='3d')
        
        
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_zlabel('Z')
        
        self.ax1.set_xlim3d(-300, 300)
        self.ax1.set_ylim3d(-300, 300)
        self.ax1.set_zlim3d(-500, 500)

        self.ax1.view_init()


        plt.subplot(2,1,2)
        plt.axis('off')
        #plt.title('Trajectory of Camera in 3D')
        self.ax2 = fig1.add_subplot(212, projection='3d')
        #fig2 = plt.figure(figsize=figSize)
        
        
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Z')
        
        # KITTI

        self.ax2.set_xlim3d(-15, 15)
        self.ax2.set_ylim3d(-15, 15)
        self.ax2.set_zlim3d(5, 35)



        self.ax2.view_init()

        #fig3 = plt.figure(figsize=figSize)
        #self.ax3 = fig3.add_subplot(111)
        #self.ax1.title('Schematic Representation of Camera in 3D')
        #fig1.canvas.set_window_title('Schematic Representation of Camera in 3D')
        #fig2.canvas.set_window_title('Trajectory of Camera in 3D')
        #fig3.canvas.set_window_title('Location RMSE with respect to frame number')

        #self.initialize_axes()

        # Initialise an empty drawing board for trajectory
        #self.blank_slate = np.zeros((600,600,3), dtype=np.uint8)

    def initialize_axes(self):
        pass
        

        

        #self.ax3.set_xlim(0,500)
        #self.ax3.set_ylim(0,5)

    def plot_camera_traectories(self, index,pred_location, pred_orientation, ground_pose):
        
        x, y, z = pred_location[0], pred_location[1], pred_location[2]
        offset_x, offset_y = 1,1
        draw_x, draw_y = int(x) + 290 - offset_x ,  500 - int(z) + offset_y
        true_x, true_y = int(ground_pose[0][-1]) + 290, 500 - int(ground_pose[2][-1])

        #self.draw_trajectory_2D(self.blank_slate, index, x, y, z, draw_x, draw_y, true_x, true_y)
        self.draw_trajectory_3D(pred_orientation, pred_location, self.ax1, self.ax2)

    def plot_frame(self, frame):

        cv2.imshow('Road facing camera', frame)
        cv2.waitKey(1)


    def draw_trajectory_3D(self, r_mat, t_vec, ax1, ax2):

        X = round(t_vec[0], 2)
        Y = round(t_vec[1], 2)
        Z = round(t_vec[2], 2)
        ax1.title.set_text('Schematic Representation of Camera in 3D\nX = {} m, Y = {} m, Z = {} m'.format(X, Y, Z))
        ax2.title.set_text('Trajectory of Camera in 3D\nX = {} m, Y = {} m, Z = {} m'.format(X, Y, Z))
        
        axes = np.zeros((3,6))
        axes[0,1], axes[1,3],axes[2,5] = 2,2,2
        t_vec = t_vec.reshape(-1,1)
        axes= r_mat @ (axes) + np.tile(t_vec,(1,6))

        ax1.plot3D(xs=axes[0,:2],ys=axes[1,:2],zs=axes[2,:2],c='r')
        ax1.plot3D(xs=axes[0,2:4],ys=axes[1,2:4],zs=axes[2,2:4],c='g')
        ax1.plot3D(xs=axes[0,4:],ys=axes[1,4:],zs=axes[2,4:],c='b')

        scale=50
        depth=100

        #generating 5 corners of camera polygon 
        pt1 = np.array([[0,0,0]]).T                 #camera centre
        pt2 = np.array([[scale,-scale,depth]]).T    #upper right 
        pt3 = np.array([[scale,scale,depth]]).T     #lower right 
        pt4 = np.array([[-scale,-scale,depth]]).T   #upper left
        pt5 = np.array([[-scale,scale,depth]]).T    #lower left
        pts = np.concatenate((pt1,pt2,pt3,pt4,pt5),axis=-1) 

        #Transforming to world-coordinate system
        pts = r_mat @ (pts) + np.tile(t_vec,(1,5))
        ax1.scatter3D(xs=pts[0,:],ys=pts[1,:],zs=pts[2,:],c='k')
        ax2.scatter3D(xs=pts[0,0],ys=pts[1,0],zs=pts[2,0],c='r', s=4)

        #Generating a list of vertices to be connected in polygon
        verts = [[pts[:,0],pts[:,1],pts[:,2]], [pts[:,0],pts[:,2],pts[:,-1]],
                [pts[:,0],pts[:,-1],pts[:,-2]],[pts[:,0],pts[:,-2],pts[:,1]]]
        
        #Generating a polygon now..
        ax1.add_collection3d(Poly3DCollection(verts, facecolors='grey',
                                            linewidths=1, edgecolors='k', alpha=.25))

        plt.pause(0.001)



    def clear(self):

        self.ax1.clear()
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_zlabel('Z')
        
        self.ax1.set_xlim3d(-300, 300)
        self.ax1.set_ylim3d(-300, 300)
        self.ax1.set_zlim3d(-500, 500)
        self.ax1.view_init()




# 显示方向和轨迹
def draw_trajectory_3D(self, r_mat, t_vec, ax1, ax2):

    X = round(t_vec[0], 2)
    Y = round(t_vec[1], 2)
    Z = round(t_vec[2], 2)
    ax1.title.set_text('X = {} m, Y = {} m, Z = {} m'.format(X, Y, Z))
    ax2.title.set_text('X = {} m, Y = {} m, Z = {} m'.format(X, Y, Z))
    
    axes = np.zeros((3,6))
    axes[0,1], axes[1,3],axes[2,5] = 2,2,2
    t_vec = t_vec.reshape(-1,1)
    axes= r_mat @ (axes) + np.tile(t_vec,(1,6))

    ax1.plot3D(xs=axes[0,:2],ys=axes[1,:2],zs=axes[2,:2],c='r')
    ax1.plot3D(xs=axes[0,2:4],ys=axes[1,2:4],zs=axes[2,2:4],c='g')
    ax1.plot3D(xs=axes[0,4:],ys=axes[1,4:],zs=axes[2,4:],c='b')

    scale=50
    depth=100

    #generating 5 corners of camera polygon 
    pt1 = np.array([[0,0,0]]).T                 #camera centre
    pt2 = np.array([[scale,-scale,depth]]).T    #upper right 
    pt3 = np.array([[scale,scale,depth]]).T     #lower right 
    pt4 = np.array([[-scale,-scale,depth]]).T   #upper left
    pt5 = np.array([[-scale,scale,depth]]).T    #lower left
    pts = np.concatenate((pt1,pt2,pt3,pt4,pt5),axis=-1) 

    #Transforming to world-coordinate system
    pts = r_mat @ (pts) + np.tile(t_vec,(1,5))
    ax1.scatter3D(xs=pts[0,:],ys=pts[1,:],zs=pts[2,:],c='k')
    ax2.scatter3D(xs=pts[0,0],ys=pts[1,0],zs=pts[2,0],c='r', s=4)

    #Generating a list of vertices to be connected in polygon
    verts = [[pts[:,0],pts[:,1],pts[:,2]], [pts[:,0],pts[:,2],pts[:,-1]],
            [pts[:,0],pts[:,-1],pts[:,-2]],[pts[:,0],pts[:,-2],pts[:,1]]]
    
    #Generating a polygon now..
    ax1.add_collection3d(Poly3DCollection(verts, facecolors='grey',
                                        linewidths=1, edgecolors='k', alpha=.25))

    plt.pause(0.01)


if __name__ == "__main__":
    Plotter = SVO_Plot(figSize=(6, 4))
    '''
    model = StereoVO(cameraMatrix, projectionMatrixL, projectionMatrixR, params)

    # Iterate over the frame and update the rotation and translation vector
    for index in range(num_frames):

        left_frame, right_frame, ground_pose = dataset[index]

        # Do model prediction
        pred_location, pred_orientation = model(left_frame, right_frame, index)

        # Calculate error
        rmse = rmse_error(pred_location, ground_pose[:, -1])

        # Plot camera trajectory, frame and error
        Plotter.plot_camera_traectories(index, pred_location, pred_orientation, ground_pose)
        Plotter.plot_errors(index, rmse)
        Plotter.plot_frame(left_frame)

        # Clear plot and prepare for next iterations
        Plotter.clear()

        if index==0:
            cv2.waitKey(0)
        else:
            cv2.waitKey(1)

    plt.show()
    '''