###Imports
import rclpy
from rclpy.node import Node

#msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

#convert images
from cv_bridge import CvBridge
import cv2


###
class LevelOne(Node):
    def __init__(self):
        super().__init__('level_one')
        
        #publish to cmd_vel to move robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel',1)
        
        #Subscribe to camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw',self.camera_callback,1)
        #used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
    def camera_callback(self,data):
        
        #Show image window
        cv2.namedWindow("Image Window",1)
        
        #Convert Rose Image to OpenCV image for processing
        current_frame = self.br.imgmsg_to_cv2(data,desired_encoding='bgr8')
        
        #convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        current_frame_hsv = current_frame_hsv[240:-1,:] #height of original window is 480, we only want to consider the bottom half, so from 240 onwards
        
        # Create mask for range of colours (HSV low values, HSV high values)
        current_frame_mask = cv2.inRange(current_frame_hsv,(0, 150, 150), (15, 255, 255)) #Range for red

        contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        # Sort by area (keep only the biggest one) Change reverse to false for the smallest Area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        contours[0] += (0,240)
        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (255, 255, 0), 20)

        self.tw=Twist() # twist message to publish
        if len(contours) > 0:
            # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            M = cv2.moments(contours[0]) # only select the largest controur
            if M['m00'] > 0:
                # find the centroid of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                # Draw a circle centered at centroid coordinates
                # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                cv2.circle(current_frame, (round(cx), round(cy)), 5, (0, 255, 0), -1)
                            
                # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                # if center of object is to the left of image center move left
                if cx < data.width / 3:
                    self.tw.angular.z=0.3
                # else if center of object is to the right of image center move right
                elif cx >= 2 * data.width / 3:
                    self.tw.angular.z=-0.3
                else: # center of object is in a 100 px range in the center of the image so dont turn
                    print("object in the center of image")
                    self.tw.angular.z=0.0
                    
        else:
            print("No Centroid Found")
            # turn until patch located
            self.tw.angular.z=0.3

        self.pub_cmd_vel.publish(self.tw)

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)
        
def main(args=None):
    print('Starting level_one.py.')
    cv2.startWindowThread()

    rclpy.init(args=args)

    colour_chaser = LevelOne()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()