#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf
import os
import time

class MapImgNode:
    def __init__(self):
        self.listener = tf.TransformListener(cache_time=rospy.Duration(1.0))
        self.default_image_path = "/tmp/shared/map_stream.jpg"
        self.image_path = rospy.get_param("~image_path", self.default_image_path)
        if not os.path.exists(os.path.dirname(self.image_path)):
            rospy.logwarn("Image path %s does not exist" % self.image_path)
            self.image_path = self.default_image_path
        
        self.map_data = None
        rospy.Subscriber("/slamware_sdk_server/map", OccupancyGrid, self.map_callback)
        self.rate = rospy.Rate(1)

    def map_callback(self, msg):
        self.map_data = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.map_data:
                self.process_map()
            else:
                self.empty_map_image()
                rospy.logwarn("Map data not received yet")
            self.rate.sleep()
    
    def process_map(self):
        try:
            # Use the most recent common time to avoid extrapolation errors
            timestamp = self.listener.getLatestCommonTime("map", "base_link")
            listener.waitForTransform("map", "base_link", timestamp, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("map", "base_link", timestamp)
            self.save_map_image(trans, rot)
        except Exception as e:
            self.save_map_image()
            rospy.logwarn("TF Lookup failed with error: %s" % e)
    
    def empty_map_image(self):
        # Create a 10x10 black image
        black_image = np.zeros((100, 100, 3), dtype=np.uint8)
        
        # Save to the file path
        if not cv2.imwrite(self.image_path, black_image):
            rospy.logerr("Failed to save placeholder image to %s" % self.image_path)
        else:
            rospy.loginfo("No map. Placeholder image at %s" % self.image_path)

    def convert_map_to_image(self):
        # Convert map to numpy image
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        map_np = np.array(self.map_data.data).reshape((height, width))
        map_np = np.clip(map_np, 0, 100)  # Clamp values to [0, 100]
        map_img = np.uint8((100 - map_np) * 2.55)  # Convert to grayscale
        map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
        
        return map_img, width, height
    
    def draw_robot_on_map(self, map_img, trans, rot):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        
        # Transform robot position to map coordinates
        robot_x = int((trans[0] - origin.position.x) / resolution)
        robot_y = int((trans[1] - origin.position.y) / resolution)
        robot_y = height - robot_y  # flip y for image
        
        if not (0 <= robot_x < width and 0 <= robot_y < height):
            rospy.logwarn("Robot position is outside the map boundaries")
            return map_img
        
        # Draw robot position
        cv2.circle(map_img, (robot_x, robot_y), 5, (0, 0, 255), -1)
        
        # Draw heading
        quaternion = rot
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        heading_length = 15
        end_x = int(robot_x + heading_length * np.cos(yaw))
        end_y = int(robot_y - heading_length * np.sin(yaw))
        cv2.line(map_img, (robot_x, robot_y), (end_x, end_y), (255, 0, 0), 2)
        
        return map_img
    
    def save_map_image(self, trans=None, rot=None):
        map_img, width, height = self.convert_map_to_image()
        
        if trans is not None and rot is not None:
            map_img = self.draw_robot_on_map(map_img, trans, rot)
            
        # Save to file
        cv2.imwrite(self.image_path, map_img)


if __name__ == "__main__":
    rospy.init_node("map_image_publisher")
    node = MapImgNode()
    node.run()
