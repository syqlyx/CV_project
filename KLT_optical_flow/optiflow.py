#!usr/bin/env python
# -*- coding: utf-8 -*-
# @Time     : 2019/12/5
# @Author   : qing
import numpy as np
import cv2 as cv 

lk_params = dict( winSize  = (15, 15),   
                  maxLevel = 2,   
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))      
  
feature_params = dict( maxCorners = 500,   
                       qualityLevel = 0.3,  
                       minDistance = 7,  
                       blockSize = 7 )  

class OptiFlow:
    def __init__(self, video_path):#构造方法，初始化一些参数和视频路径  
        self.track_len = 10  
        self.detect_interval = 5  
        self.tracks = []  # vector<Point2f>
        self.cap = cv.VideoCapture(video_path)  #按照视频帧率获取图像
        self.frame_idx = 0  
        self.fps = self.cap.get(cv.CAP_PROP_FPS)
        self.frame_count = self.cap.get(cv.CAP_PROP_FRAME_COUNT)
        # print("帧率是{}帧/s".format(fps))
    def flow(self):
        for i in range(self.frame_count):
            ret, old_frame = self.cap.read()#是否成功获取；捕获到的图像
            if ret == False:
                print("读取图像失败")
                break
            self.old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
            if i == 0:
                first_gray = self.old_gray 
                mask = np.zeros_like(self.old_gray)#初始化和视频大小相同的图像  
                mask[:] = 255#将mask赋值255也就是算全部图像的角点  
                rawfeature = cv.goodFeaturesToTrack(first_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, mask = mask, blockSize=7)
                feature = cv.cornerSubPix(first_gray, rawfeature, (11, 11), (-1, -1), (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.01))
                if feature is not None:  
                    for x, y in np.float32(feature).reshape(-1, 2):  
                        self.tracks.append([(x, y)])#将检测到的角点放在待跟踪序列中 ,转成浮点数
            # old_feature = cv.goodFeaturesToTrack(old_gray, maxCorners=100, qualityLevel=0.3,minDistance=7,blockSize=7)

            new_frame = self.cap.read()
            new_gray = cv.cvtColor(new_frame, cv.COLOR_BGR2GRAY)

            if(len(self.tracks)>0):
                img1, img2 = self.old_gray, new_gray
                ## 确保跟踪准确
                p1, st, err = cv.calcOpticalFlowPyrLK(img1, img2, feature, None, **lk_params)
                p0, st1, err1 = cv.calcOpticalFlowPyrLK(img2, img1, p1, None, **lk_params)#回溯生成跟踪角点
                ## 判断正确跟踪点
                d = abs(np.float32(p0 - feature)).reshape(-1,2).max(1)#输出每行的最大值，1×n
                good = (d<1).reshape(-1,1)
                newtracks=[]
                for (x,y), flag in zip(np.float32(p1).reshape(-1,2), good):
                    if flag:
                        newtracks.append([(x,y)])
                
                self.tracks = newtracks
                # 根据跟踪计算运动
                move = self.tracks


##确认之前的基准，避免丢失

            self.old_gray = new_gray

def main():
    import sys  
    try: video_path = sys.argv[1]  
    except: video_path = "/media/vickylzy/文件共享盘/A3_grade_one/class/computer_vision/Assignment2/100208AA_out.mp4"
    OptiFlow(video_path)
    #
    #cap.release()


if __name__ == "__main__":
    main()