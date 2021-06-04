# 关键点和关键帧的地图

from myframe import Frame
from mymappoint import MapPoint

# Frame和MapPoint推荐使用类方法
# mappoint1 = MapPoint.createMapPoint() # 内部会调用构造函数
# mappoint2 = MapPoint.createMapPoint()
# frame1 = Frame.createFrame()           # 内部会调用构造函数

# Map推荐使用map对象方法
# map = Map()
# map.insertKeyFrame()
# map.insertMapPoint()

class Map:
    def __init__(self):

        self.map_points_ = {}    # unordered_map<unsigned long, MapPoint::Ptr >
        self.keyframes_ = {}     # unordered_map<unsigned long, Frame::Ptr >

    def print_map_points(self):
        print(self.map_points_)

    def print_keyframes(self):
        print(self.keyframes_)

    def insertKeyFrame(self,frame):
        print("Number of keyframes =",len(self.keyframes_))
        #if ( keyframes_.get(frame.id_) == None ):
        self.keyframes_[frame.id_] = frame
        #else:
            #keyframes_[ frame->id_ ] = frame;
        

    def insertMapPoint (self, map_point ): # MapPoint::Ptr 
        #if map_points_.get(map_point.id_) == map_points_.end():
        #    map_points_.insert( make_pair(map_point.id_, map_point) )
        #else:
            self.map_points_[map_point.id_] = map_point

if __name__ == "__main__":
    frame = []
    keyframes_ = {}
    for i in range(5):
        frame.append(Frame.createFrame())
        keyframes_[frame[i].id_] = frame[i]
    #frame2 = Frame.createFrame()
    print(frame)
    print(keyframes_)

    map = Map()
    map.insertKeyFrame(frame[0])
    map.insertKeyFrame(frame[1])
    map.insertKeyFrame(frame[2])
    map.print_map_points()

    mappoint = MapPoint()
    mappoint.createMapPoint()
    f = Frame()
    f.createFrame()