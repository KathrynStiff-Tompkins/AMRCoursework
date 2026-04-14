###Imports
import rclpy
from rclpy.node import Node
import numpy as np

#msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#convert images
from cv_bridge import CvBridge
import cv2


#code
class RobotBlockPush(Node):
    def __init__(self):
        super().__init__('level_one')
        
        #publish to cmd_vel to move robot
        self.pub_cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        
        
        #Subscribers
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw',self.camera_callback,1)
        self.create_subscription(LaserScan, '/scan', self.laser_callback,1)
        
        #utils
        #used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.spin_check = 0
        
        #Focal length personally calculated for future use using formal (image_height*known_distance) / real_height
        self.focal_length_height = 42.1875
        self.camera_width = 640
        
        #flag to control robot function
        self.step = 'red_scan'
        
        #control ariables
        self.patch = None
        self.cubes = None
        self.distance = 0
        self.spin = 0
        self.wiggle_check = 0
        
        
        #timers
        self.timer = self.create_timer(0.1,self.control_loop)
    
    def calculate_distance(self,pix_height): #calaculate approixmate distance from object
        #https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
        #https://stackoverflow.com/questions/6714069/finding-distance-from-camera-to-object-of-known-size
        distance_to_object = (self.focal_length_height*0.8)/pix_height 
        return distance_to_object
        
    def laser_callback(self,data): #subscribes to the laser scan topic
        #gets all the ranges of the lasers, records the distance of the centre laser
       ranges = np.asarray(data.ranges)
       self.centre_dist = ranges[146]
       
    def camera_callback(self,data): #subscribes to the camera topic, majority of code here originates from codes provided in the workshops
        
        #Show image window
        cv2.namedWindow("Camera Vision",1)
        
        #Convert Rose Image to OpenCV image for processing
        current_frame = self.br.imgmsg_to_cv2(data,desired_encoding='bgr8')
        
        #convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        current_frame_hsv_floor = current_frame_hsv[240:-1,:] #height of original window is 480, we only want to consider the bottom half, so from 240 onwards
        
        # Create mask for range of colours (HSV low values, HSV high values)
        current_frame_mask_patch = cv2.inRange(current_frame_hsv_floor,(0, 150, 150), (15, 255, 255)) #Range for red patch
        current_frame_mask_cube = cv2.inRange(current_frame_hsv_floor,(100,150,100),(130,255,150)) #Range for blue cubes

        #Find the contours of both the patch and the cubes
        patch_contours, hierarchy = cv2.findContours(current_frame_mask_patch, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cube_contours, hierarchy = cv2.findContours(current_frame_mask_cube,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sort by area, Change reverse to false for the smallest Area
        patch_contours = sorted(patch_contours, key=cv2.contourArea, reverse=True)
        cube_contours = sorted(cube_contours, key=cv2.contourArea, reverse=True)
        
        #get bounding rectangles
        patch_list_x = list()
        patch_list_y = list()
        self.cubes = []
        distance_from_centre = []
        
        for i in range(len(cube_contours)): 
            cube_contours[i] += (0,240)
            rect = cv2.boundingRect(cube_contours[i])
            self.cubes.append(rect)
            distance_from_centre.append(abs((self.camera_width//2)-(rect[0]+(rect[2]//2))))
            
        if len(self.cubes) > 0:
            self.cubes = [self.cubes[i] for i in np.argsort(distance_from_centre)]
        
            
        for i in range(len(patch_contours)):
            patch_contours[i] += (0,240) #Makes sure the contour is in the right position, considering we only looked at the bottom half of the view.
            rect =  cv2.boundingRect(patch_contours[i])
            patch_list_x.append(rect[0])
            patch_list_x.append(rect[0]+rect[2])
            patch_list_y.append(rect[1])
            patch_list_y.append(rect[1]+rect[3])    
        
        
        if len(patch_contours) > 0:
            self.patch = [min(patch_list_x),min(patch_list_y),max(patch_list_x),max(patch_list_y)] #provides a bounding rectangle that fits all the separate pieces of patch to represent them as one
        else:
            self.patch = [0,0,1,1] #places the rectangle outside of the range that the cubes could be. Required to prevent errors when the patch is not in camera view        

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours_patch = cv2.drawContours(current_frame, patch_contours, 0, (255, 255, 0), 20)
        current_frame_contours_cube = cv2.drawContours(current_frame_contours_patch, cube_contours, -1, (0,255,255),20)

        # show the cv images
        current_frame_contours_cube = cv2.rectangle(current_frame_contours_cube, (self.patch[0],self.patch[1]), (self.patch[2],self.patch[3]),(255,0,0),20)
        current_frame_contours_small = cv2.resize(current_frame_contours_cube, (0,0), fx=0.4, fy=0.4) # reduce image size
        
        cv2.imshow("Camera Vision", current_frame_contours_small)
        cv2.waitKey(1)
        
    def face_centre(self, x_centre): #gets the robot to face centre point
        #variables
        speed = 0.2
        centre_buffer = 30
        mid_point = self.camera_width//2
        k = 0.02
        
        speed = speed * k * abs(mid_point - x_centre) #makes turning speed proportional to distance between the centre of the shape and the centre of the camera
        
        #turns towards the given point based
        if mid_point - centre_buffer < x_centre < mid_point + centre_buffer:
            return 0.0
        if x_centre > mid_point - centre_buffer:
            self.wiggle_check += 1            
            return -speed 
        elif x_centre < mid_point + centre_buffer:
            return speed
    
    def cube_scan(self): #checks the cubes in camera view to choose a valid one to push off patch
        turn = 0.75
        for cube in self.cubes: #loop through cubes in view
            #check if cube is in patch bounding box
            if (self.patch[0] <= cube[0] + 0.5*cube[2] <= self.patch[0]+self.patch[2]) and (self.patch[1] <= cube[1] + 0.75*cube[3] <= self.patch[1]+self.patch[3]) and cube[0] + 0.5*cube[2] < 0.9*self.camera_width:
                turn = self.face_centre(cube[0] + 0.5*cube[2]) #centre view on cube
                if turn == 0:
                    self.step = 'cube_approach' #proceed to next step
                    self.distance = min(min((self.centre_dist/0.1)-0.2,self.calculate_distance(cube[3])/0.1) + 1.5 ,6) #calculate distance from cube, chooses the minimum between answer gotten and 6 to prevent logic errors in the calculations
                break
            if (self.patch[1] <= cube[1] + 0.75*cube[3] <= self.patch[1]+self.patch[3]) and (self.patch[0] + self.patch[2] + 20 >= cube[0] or self.patch[0] - 20 <= cube[0]+cube[2]) and self.calculate_distance(self.patch[3])/0.1 < 6:
                return 0.0, -0.1 #moves the robot back if the cube's positioning makes it questionable on whether or not it is actually on the patch
        return turn, 0.0
        
    def control_loop(self): #controls the robot, dictating what to do based on the step it is on
        tw = Twist()
        
        
        if self.spin >= 120: #checks to see if the robot has done a full circle without moving. If so, finishes.
            print('complete')
            self.destroy_node()
            rclpy.shutdown()
            
        elif self.wiggle_check >= 5: #If the positioning of a cube makes the robot keep wiggling, move backwards to prevent getting stuck. This is for when a cube only looks like it's in the bounding rectangle at certain angles
            tw.linear.x = -0.5
            self.wiggle_check = 0
            
        elif self.step == 'red_scan': #rotate until the red patch is found in the centre
            #check if patch none to stop errors
            if self.patch != None:
                tw.angular.z = self.face_centre(self.patch[0] + self.patch[2]//2)
                if tw.angular.z == 0:
                    self.step = 'cube_scan'
            else:
                tw.angular.z = 0.5
            self.spin += tw.angular.z
            
        elif self.centre_dist < 0.3: #prevents robot from running into the wall. Restarts the loop
            tw.linear.x = -0.3
            self.step = 'red_scan'    
                
        elif self.step == 'cube_scan': #rotate to face a cube that is on the red patch
            turn, move = self.cube_scan()
            tw.angular.z = turn
            tw.linear.x = move
            self.spin += turn     
               
        elif self.step == 'cube_approach' or self.step == 'cube_push': #approach the cube or push cube off of patch
            #resets spin and wiggle checks when moving
            self.spin = 0
            self.wiggle_check = 0
            if self.distance > 0: #move robot 
                moving = min(0.1, self.distance) #choose smaller between the two. Do this to prevent robot shooting forward
                self.distance -= moving #reduce distance value
                tw.linear.x = moving #move robot
                
            elif self.distance <= 0 and self.step == 'cube_approach': #if cube has been reached 
                self.step = 'cube_push' #goes to next step in loop
                if len(self.patch) > 0: #if patch is in camera view, calculate distance
                    self.distance = min(min((self.centre_dist/0.1)-0.2,self.calculate_distance(self.patch[3])/0.1) + 1.5 , 6)
                    
                else: #If patch cannot be seen, i.e. if edge is too close to robot to be seen by camera
                    self.distance = 0.3 
                    
            elif self.distance <= 0 and self.step == 'cube_push': #restarts the loop to find next cube
                self.step = 'red_scan'
        
        self.pub_cmd_vel.publish(tw)
               
        
def main(args=None):
    print('Starting level_one.py.')
    cv2.startWindowThread()

    rclpy.init(args=args)

    block_push = RobotBlockPush()

    rclpy.spin(block_push)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    block_push.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()