#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
from interactive_markers.menu_handler import MenuHandler


class WaypointInteractiveMarkersNode(Node):
    def __init__(self):
        super().__init__('waypoint_interactive_markers')
        
        # Parameters
        self.declare_parameter(
            'fixed_frame',
            'map',
            ParameterDescriptor(description='Frame ID for interactive markers')
        )
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        
        # State
        self.waypoints = {}  # {marker_name: InteractiveMarker}
        self.next_id = 0
        self.menu_handler = MenuHandler()
        
        # Publishers
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/interactive_waypoints',
            queue_size=10
        )
        self.command_pub = self.create_publisher(
            Empty,
            '/waypoint_server/command',
            queue_size=1
        )
        
        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'waypoint_interactive_markers')
        
        # Create menu handler
        self.menu_handler.insert('Start Following', callback=self.menu_callback)
        self.menu_handler.insert('Clear All Waypoints', callback=self.menu_callback)
        self.menu_handler.insert('Delete This Waypoint', callback=self.menu_callback)
        
        # Create the "add waypoint" marker
        self.create_add_waypoint_marker()
        self.server.applyChanges()
        
        self.get_logger().info('Waypoint Interactive Markers node started.')

    def create_add_waypoint_marker(self):
        """Create the initial marker for adding waypoints."""
        add_marker = InteractiveMarker()
        add_marker.header.frame_id = self.fixed_frame
        add_marker.header.stamp = self.get_clock().now().to_msg()
        add_marker.name = "add_waypoint_button"
        add_marker.description = "Click to add Waypoint"
        add_marker.pose.position.z = 0.0
        add_marker.scale = 1.0
        
        # Create button control
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        
        # Cube marker (green)
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        control.markers.append(marker)
        
        add_marker.controls.append(control)
        self.server.insert(add_marker, feedback_callback=self.button_callback)

    def button_callback(self, feedback):
        """Callback for the add waypoint button."""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.add_waypoint_at_pose(feedback.pose)

    def add_waypoint_at_pose(self, pose):
        """Create a new waypoint marker at the given pose."""
        marker_name = f"waypoint_{self.next_id}"
        self.next_id += 1
        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.fixed_frame
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = marker_name
        int_marker.description = f"Waypoint {self.next_id - 1}"
        int_marker.pose = pose
        int_marker.scale = 1.0
        
        # Sphere marker (red)
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.5
        sphere_marker.scale.y = 0.5
        sphere_marker.scale.z = 0.5
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8
        
        # Move/rotate control
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        control.always_visible = True
        control.markers.append(sphere_marker)
        int_marker.controls.append(control)
        
        # Menu control
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.always_visible = True
        int_marker.controls.append(menu_control)
        
        self.server.insert(int_marker, feedback_callback=self.waypoint_callback)
        self.menu_handler.apply(self.server, marker_name)
        self.waypoints[marker_name] = int_marker
        self.server.applyChanges()
        
        self.get_logger().info(f'Added {marker_name} at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})')

    def waypoint_callback(self, feedback):
        """Callback for waypoint markers."""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Update position
            self.get_logger().debug(f'Updated {feedback.marker_name} to ({feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f})')
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.handle_menu_select(feedback)

    def menu_callback(self, feedback):
        """Generic menu callback."""
        self.handle_menu_select(feedback)

    def handle_menu_select(self, feedback):
        """Handle menu selections."""
        menu_id = feedback.menu_entry_id
        marker_name = feedback.marker_name
        
        if menu_id == 1:  # Start Following
            self.send_waypoints_to_follower()
        elif menu_id == 2:  # Clear All Waypoints
            self.clear_all_waypoints()
        elif menu_id == 3:  # Delete This Waypoint
            self.delete_waypoint(marker_name)

    def send_waypoints_to_follower(self):
        """Publish all waypoints and send start command."""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to follow.')
            return
        
        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints to follower.')
        
        # Sort waypoints by ID for consistent ordering
        sorted_waypoints = sorted(self.waypoints.items(), key=lambda x: int(x[0].split('_')[1]))
        
        for marker_name, int_marker in sorted_waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = int_marker.header.frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = int_marker.pose
            
            self.waypoint_pub.publish(pose_stamped)
            self.get_logger().info(f'Published {marker_name}')
        
        # Send command to start following
        self.command_pub.publish(Empty())
        self.get_logger().info('Sent start command to waypoint follower.')

    def clear_all_waypoints(self):
        """Remove all waypoint markers."""
        for marker_name in list(self.waypoints.keys()):
            self.server.erase(marker_name)
        self.waypoints.clear()
        self.server.applyChanges()
        self.get_logger().info('Cleared all waypoints.')

    def delete_waypoint(self, marker_name):
        """Delete a specific waypoint marker."""
        if marker_name in self.waypoints:
            self.server.erase(marker_name)
            del self.waypoints[marker_name]
            self.server.applyChanges()
            self.get_logger().info(f'Deleted {marker_name}.')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointInteractiveMarkersNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
