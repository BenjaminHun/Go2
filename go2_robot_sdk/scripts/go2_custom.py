import cv2
import numpy as np
from go2_robot_sdk.scripts.go2_func import gen_mov_command
import logging

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def generate_custom_commands(robot_cmd_vel, robot_num, camera_image):
    custom_x = 0.0  # Custom linear velocity
    custom_z = 0.0  # Custom angular velocity
    # Example: Use the camera image to generate custom commands
    if camera_image is None:
        logger.info("Camera image is None")
        return
    
    distance = detect_circle_and_calculate_distance(camera_image, 10, 50, 'red')
    
    if distance is None:
        logger.info("Circle not found")
        return
    else:
        logger.info("Distance:", distance)
    
        # Process the distance and generate commands
    custom_z = distance  # Custom angular velocity
    robot_cmd_vel[robot_num] = gen_mov_command(custom_x, 0.0, custom_z)

def detect_circle_and_calculate_distance(camera_image, min_radius, max_radius, color):
    # Convert ROS Image message to OpenCV image
    np_arr = np.frombuffer(camera_image.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

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

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # Draw the circle in the output image (optional)
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)

            # Calculate the horizontal distance between the circle center and the image center
            image_center_x = cv_image.shape[1] // 2
            distance = x - image_center_x

            # Normalize the distance to a value between 0 and 0.1
            normalized_distance = distance / image_center_x * 0.1
            return normalized_distance

    # If no circle is found, return None or a default value
    return None