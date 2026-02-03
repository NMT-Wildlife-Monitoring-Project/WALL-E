#python3

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, MenuEntry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class WaypointMarkerServer(Node):
    def __init__(self):
        super().__init__('waypoint_marker_server')
        self.server = InteractiveMarkerServer(self, "waypoints")
        self.path_publisher = self.create_publisher(Path, 'waypoints_to_follow', 10)
        self.waypoints = {}
        self.waypoint_count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Waypoint marker server started.")

    def timer_callback(self):
        """Periodically publishes the waypoints as a nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        sorted_waypoints = sorted(self.waypoints.items(), key=lambda item: int(item[0].split('_')[1]))
        for name, pose in sorted_waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        if path_msg.poses:
            self.path_publisher.publish(path_msg)
            self.get_logger().info(f"Published a path with {len(path_msg.poses)} waypoints.")

    def add_waypoint(self, pose):
        """Adds a new interactive marker (waypoint) to the server."""
        self.waypoint_count += 1
        name = f"waypoint_{self.waypoint_count}"
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = f"Waypoint {self.waypoint_count}"
        int_marker.pose = pose
        int_marker.scale = 0.5

        control = InteractiveMarkerControl()
        control.orientation.w, control.orientation.x, control.orientation.y, control.orientation.z = 1.0, 0.0, 1.0, 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x, sphere_marker.scale.y, sphere_marker.scale.z = 0.2, 0.2, 0.2
        sphere_marker.color.r, sphere_marker.color.a = 1.0, 1.0

        control.markers.append(sphere_marker)
        control.always_visible = True
        int_marker.controls.append(control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"

        menu_entry_remove = MenuEntry()
        menu_entry_remove.id = 1
        menu_entry_remove.title = "Remove Waypoint"
        menu_control.menu_entries.append(menu_entry_remove)

        int_marker.controls.append(menu_control)

        self.server.insert(int_marker, feedback_callback=self.feedback_callback)
        self.server.applyChanges()
        self.waypoints[name] = pose

    def feedback_callback(self, feedback):
        """Handles feedback from RViz (e.g., marker moved or menu item clicked)."""
        if feedback.event_type == InteractiveMarker.POSE_UPDATE:
            self.waypoints[feedback.marker_name] = feedback.pose
            self.get_logger().info(f"Waypoint '{feedback.marker_name}' moved to x: {feedback.pose.position.x}, y: {feedback.pose.position.y}")
        elif feedback.event_type == InteractiveMarker.MENU_SELECT and feedback.menu_entry_id == 1:
            self.server.erase(feedback.marker_name)
            self.server.applyChanges()
            self.waypoints.pop(feedback.marker_name)
            self.get_logger().info(f"Waypoint '{feedback.marker_name}' removed.")

    def create_add_waypoint_marker(self):
        """Creates a special marker to add new waypoints."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "add_waypoint_tool"
        int_marker.description = "Add New Waypoint"
        int_marker.pose.position.x, int_marker.pose.position.y = 0.0, 0.0
        int_marker.scale = 0.5

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "add_menu"

        menu_entry = MenuEntry()
        menu_entry.id = 1
        menu_entry.title = "Add Waypoint"
        menu_control.menu_entries.append(menu_entry)
        int_marker.controls.append(menu_control)

        self.server.insert(int_marker, self.add_waypoint_callback)
        self.server.applyChanges()

    def add_waypoint_callback(self, feedback):
        """Callback for the 'add waypoint' marker."""
        if feedback.event_type == InteractiveMarker.MENU_SELECT and feedback.menu_entry_id == 1:
            self.add_waypoint(feedback.pose)

def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WaypointMarkerServer()
    waypoint_server.create_add_waypoint_marker()
    rclpy.spin(waypoint_server)
    waypoint_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

