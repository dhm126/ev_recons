from cmath import nan
import numpy as np
from math import isnan
from cv_bridge import CvBridge ,CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image
class DepthimageTest:
    def __init__(self):
         self.Bridge = CvBridge()
         self.image_pub=rospy.Subscriber("DepthMap",Image,self.DepthImageCallback)
         #davis/left/depth_image_rect /inverse_Depth_Map
         print("open depthimage node ")
    def DepthImageCallback(self,data):
        try:
            depImage=self.Bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e :
            print(e)
        print("new image \n\n\n ")
        print(depImage.shape)
        bb=list()
        for i in range(0,int(depImage.shape[0])):#260
            for j in range(0,int(depImage.shape[1])):
                x =depImage[i,j]
                if isnan(x) or x<0.01:
                     continue
                else :
                    bb.append(x)
                    # print(x)
                # print(depImage[i,j])
        if(len(bb)!=0):
            mean = np.mean(bb)
            std = np.std(bb)
            max=np.amax(bb)
            min=np.amin(bb)
            print("mean= {}".format(mean))
            print("std= {}".format(std))
            print("depthrange=({},{})".format(min,max))

if __name__ =='__main__':
    try:
        rospy.init_node("depth_image_test")
        DepthimageTest()
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down depth callback node")