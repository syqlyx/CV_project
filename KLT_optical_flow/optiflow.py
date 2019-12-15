#!usr/bin/env python
# -*- coding: utf-8 -*-
# @Time     : 2019/12/5
# @Author   : qing
# idea:第一帧进行角点检测，之后每隔一段时间，重新进行一次特征点选择；及时的添加新的特征点，同时摒弃已经失去追踪的特征点；存储考虑时、空维度；每次对新的一帧的追踪要进行回溯，保证准确性（不匹配则代表追踪错误或者已经超出图像范围）。
# 使用Python练手
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
        # self.track_len = 10  
        self.N = 10  
        self.tracks = []  # vector<vector<Point2f>>
        self.cap = cv.VideoCapture(video_path)  #按照视频帧率获取图像
        self.frame_idx = 0  
        self.fps = self.cap.get(cv.CAP_PROP_FPS)
        self.frame_count = int(self.cap.get(7))#cv.CAP_PROP_FRAME_COUNT
        # print("帧率是{}帧/s".format(fps))
        print("视频是{}帧".format(self.frame_count))
    def flow(self):
        for i in range(self.frame_count):
            ret, new_frame = self.cap.read()#是否成功获取；捕获到的图像
            if ret == False:
                print("读取图像失败")
                break
            new_gray = cv.cvtColor(new_frame, cv.COLOR_BGR2GRAY)

            if(len(self.tracks)>0):
                img1, img2 = self.old_gray, new_gray
                feature = np.float32([vec[-1] for vec in self.tracks]).reshape(-1,1,2)#每个特征点追踪序列的最新值，也就是最近的图中的特征点坐标
                p1, st, err = cv.calcOpticalFlowPyrLK(img1, img2, feature, None, **lk_params)#新特征点的位置（-1,1,2）
                p0, st1, err1 = cv.calcOpticalFlowPyrLK(img2, img1, p1, None, **lk_params)#回溯生成跟踪角点
                ## 判断正确跟踪点
                d = abs(np.float32(p0 - feature)).reshape(-1,2).max(1)#输出每行的最大值，1×n
                good = (d<1).reshape(-1,1)#生成一列标志位
                newtracks=[]
                for im, (x,y), flag in zip(self.tracks, np.float32(p1).reshape(-1,2), good):
                    if flag:
                        im.append((x,y))#时间序列上进行扩展
                    newtracks.append(im)##滤除丢失的跟踪点
#对于存储过长序列的某个点，则可认为是随车运动的物体？？？
                self.tracks = newtracks
                print("第{}帧".format(i))

                # 根据跟踪计算特征点在像素上的运动
                # move = self.tracks
            
            if i%self.N == 0:#添加新的追踪点
                first_gray = new_gray 
                mask = np.zeros_like(first_gray) 
                mask[:] = 255 
                rawfeature = cv.goodFeaturesToTrack(first_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, mask = mask, blockSize=7)
                feature = cv.cornerSubPix(first_gray, rawfeature, (11, 11), (-1, -1), (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.01))
                if feature is not None:  
                    for x, y in np.float32(feature).reshape(-1, 2):  
                        self.tracks.append([(x, y)])#新添追踪点   ([()])  -1,1,2

                ##确认之前的基准，避免丢失
            self.old_gray = new_gray
            print("第{}帧".format(i))
            cv.imshow('lk_track', vis)  
            cv.waitKey(0)#跟在图片显示之后
        self.cap.release()
            

def main():
    import sys  
    try: video_path = sys.argv[1]  
    except: video_path = "/media/vickylzy/文件共享盘/A3_grade_one/class/computer_vision/Assignment2/100208AA_out.mp4"
    OptiFlow(video_path).flow()
    cv.destroyAllWindows()    


if __name__ == "__main__":
    main()
    
    