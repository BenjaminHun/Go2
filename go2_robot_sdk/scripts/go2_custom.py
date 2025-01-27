import cv2
import numpy as np
from scripts.go2_func import gen_mov_command
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


def generate_custom_commands(robot_cmd_vel, robot_num, camera_image,self):
    
    custom_x = 0.0  # Custom linear velocity
    custom_z = 0.0  # Custom angular velocity
    # Example: Use the camera image to generate custom commands
    #self.get_logger().info(str(camera_image))
    if camera_image is None:
        self.get_logger().info("Camera image is None")
        return
    self.get_logger().info("Camera image is not None")
    
    rotation,throotle = detect_circle_and_calculate_distance(camera_image, 30, 70, 'blue')
    
    if rotation is None:
        self.get_logger().info("Circle not found")
        return
    else:
        self.get_logger().info("Distance:" + str(rotation))
    
        # Process the distance and generate commands
    rotation = rotation*-1  # Custom angular velocity
    if abs(rotation)>0.01 and abs(throotle)>0.01:
        robot_cmd_vel[robot_num] = gen_mov_command(throotle, 0.0, rotation)
    #robot_cmd_vel[robot_num] = gen_mov_command(0.0, 0.0, 0.2)

def detect_circle_and_calculate_distance(camera_image, min_radius, max_radius, color):
    try:
        cv_image = bridge.imgmsg_to_cv2(camera_image, desired_encoding='bgr8')
    except CvBridgeError as e:
        print(e)
        return 0.0,0.0

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the color range for the desired color
    if color == 'red':
        # True red in HSV is approximately (0, 255, 255)
        lower_red1 = np.array([0, 70, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == 'green':
        lower_color = np.array([40, 70, 50])
        upper_color = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
    elif color == 'blue':
        lower_color = np.array([100, 70, 50])
        upper_color = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
    else:
        print("Unsupported color")
        return 0.0,0.0

    # Apply the mask to the image
    masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Convert the masked image to grayscale
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=50,
        param2=30,
        minRadius=min_radius,
        maxRadius=max_radius
    )
    # Display the images in separate windows
    #cv2.imshow('Detected Circles', cv_image)
    #cv2.imshow('Masked Image', masked_image)
    #cv2.waitKey(1)  # Wait for 1 ms to allow window updates

 # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use the Canny edge detector to find edges
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour to a circle
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        if min_radius < radius < max_radius:
            # Draw the circle in the output image
            cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 4)
            # Optionally, draw the center of the circle
            cv2.circle(cv_image, (int(x), int(y)), 2, (0, 0, 255), 3)

            # Calculate the horizontal distance between the circle center and the image center
            image_center_x = cv_image.shape[1] // 2
            distance = x - image_center_x

            # Normalize the distance to a value between -0.1 and 0.1
            normalized_distance = distance / image_center_x * 0.4

            # Save the image with the detected circle
            cv_image_path = 'detected_circle_image.jpg'
            masked_image_path = 'masked_image.jpg'
            cv2.imwrite(cv_image_path, cv_image)
            cv2.imwrite(masked_image_path, masked_image)
            throotle = 0.0
            if radius<50:
                throotle = 0.2

            return normalized_distance,throotle


    # If no circle is found, return None or a default value
    return 0.0,0.0