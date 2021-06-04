import numpy as np

# 推荐使用类方法createMapPonit而不实例化，当然也可以使用对象方法，factory_id是静态变量都会累加
# 使用方法：
'''
mappoint1 = MapPoint.createMapPoint() # 内部会调用构造函数
mappoint2 = MapPoint.createMapPoint()
'''
class MapPoint:
    factory_id = 0  #静态变量，在外面
    def __init__(self,
                 id = -1, # 和关键词重了
                 position = np.zeros([3,1]),
                 norm = np.zeros([3,1])):
        self.id_                = id        
        self.pos_               = position  # Vector3d
        self.norm_              = norm      # Vector3d
        self.descriptor_        = None
        self.observed_times_    = 0
        self.matched_times_     = None
        #print(self.id_)

    @classmethod
    def createMapPoint(self):
        #factory_id = 0 # 静态变量，在外面
        mp = MapPoint(MapPoint.factory_id)   #, np.zeros([3,1]), np.zeros([3,1]))
        print(f"MapPoint with factory_id {MapPoint.factory_id} created.")
        MapPoint.factory_id+=1
        
        return mp

if __name__ == "__main__":
    MapPoint.createMapPoint()
    MapPoint.createMapPoint()
    MapPoint.createMapPoint()

    mappoint = MapPoint()
    mappoint.createMapPoint()