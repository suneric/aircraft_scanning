#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError

class rs_image:
    # create a image view with a frame size for the ROI
    def __init__(self):
        print("create image instance...")
        # camera information
        self.cameraInfoUpdate = False
        self.pixel = 0.003 # mm
        self.focal = np.array([1.93,1.93]) # mm
        self.principal = np.array([320,240]) # in pixel, could not be the center of the image
        self.img_size = np.array([640,480])

        self.bridge=CvBridge()
        # ros-realsense
        self.caminfo_sub = rospy.Subscriber('/rs435/color/camera_info', CameraInfo, self._caminfo_callback)
        self.depth_sub = rospy.Subscriber('/rs435/depth/image_raw', Image, self._depth_callback)
        self.color_sub = rospy.Subscriber('/rs435/color/image_raw', Image, self._color_callback)
        self.point_sub = rospy.Subscriber('/rs435/depth/points', PointCloud2, self._point_callback)

        # color image and depth image
        self.cv_color = None
        self.cv_depth = None
        self.points = None
        self.center_dist = None

        self.window = "view"

    def image_size(self):
        return self.img_size

    def center_distance(self):
        return self.center_dist

    def ready(self):
        return self.cameraInfoUpdate

    # depth in mm
    def depth_image(self):
        return self.cv_depth

    def color_image(self):
        return self.cv_color

    # focal in mm [x,y]
    def camera_focal(self):
        return self.focal

    # in pixel
    def camera_principal(self):
        return self.principal

    def pixel_size(self):
        return self.pixel

    # center of the image frame
    # return center of width and height
    def image_center(self):
        return self.img_size/2

    def point_cloud(self):
        data = pc2.read_points(self.points,skip_nans=True)#,field_names=('x','y','z','rgb'))
        # data is a generator
        return data

    def _point_callback(self,data):
        self.points = data

    # private functions
    def _caminfo_callback(self, data):
        if self.cameraInfoUpdate == False:
            self.img_size = np.array([data.width,data.height])
            self.focal = self.pixel*np.array([data.K[0],data.K[4]])
            self.principal = [data.K[2],data.K[5]]
            self.cameraInfoUpdate = True
            #print(data)

    def _depth_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                self.cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) #"16UC1"
            except CvBridgeError as e:
                print(e)

    def _color_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                self.cv_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
        if self.cameraInfoUpdate == True:
            self._create_view()

    # create a view
    def _create_view(self):
        img = self.cv_color
        center = self.image_center()
        x,y = center[0],center[1]
        dist = self.distance(x,y,3) # [column,row]
        if dist <= 1.5:
            self._draw_box(20,x,y,(0,0,255),1,dist) # color (b,g,r)
        else:
            self._draw_box(20,x,y,(0,255,0),1,dist) # color (b,g,r)

        self.center_dist = dist

        # show the image
        cv.namedWindow(self.window)#,cv.WINDOW_NORMAL) # create window with freedom of dimension
        cv.imshow(self.window, img)
        cv.waitKey(3)

    def _draw_box(self,size,cx,cy,clr,lw,dist):
        cv.rectangle(self.cv_color, (cx-size, cy-size), (cx+size, cy+size), clr, lw)
        if dist != None:
            strDist = "{:.6}".format(str(dist)) # cm
            font_scale = 0.3
            cv.putText(self.cv_color, strDist, (cx-size+4, cy), cv.FONT_HERSHEY_SIMPLEX, font_scale, clr, lw)

    # calculate mean distance in a small pixel frame around u,v
    # a non-zero mean value for the pixel with its neighboring pixels
    def distance(self,u,v,size=3):
        dist_list=[]
        for i in range(-size,size):
            for j in range(-size,size):
                value = self.cv_depth[v+j,u+i]
                if value != 0 and value != None:
                    dist_list.append(value)
        #print(dist_list)
        if not dist_list:
            return -1
        else:
            return np.mean(dist_list)

if __name__ == '__main__':
    rospy.init_node("uav_scanning_image", anonymous=True, log_level=rospy.INFO)
    camera = rs_image()
    rospy.spin()
