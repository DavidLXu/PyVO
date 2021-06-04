import cv2

class Config:
    @classmethod
    def __init__(self,filename):
        self.file_ = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
        if (self.file_.isOpened() == False ):
            
            self.file_.release()
            raise IOError(f"{filename} does not exist.")


    #@classmethod
    # def setParameterFile(self,filename):
    #     self.file_ = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    #     if (self.file_.isOpened() == False ):
    #         print("parameter file ",filename," does not exist.")
    #         self.file_.release()
    #         return None

    @classmethod
    def get(self,nodename):
        # self.file_ = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ) 在setParameterFile设置过路径了
                 
        return self.file_.getNode(nodename)

    def __del__(self):  # 并没有创建对象，一致在用classmethod, 所以可能file_一直没release
        if self.file_.isOpened():
            self.file_.release()

if __name__ == "__main__":        
    #Config.setParameterFile("config.yml")
    config = Config("config.yml")
    print(config.get("realNode").real())