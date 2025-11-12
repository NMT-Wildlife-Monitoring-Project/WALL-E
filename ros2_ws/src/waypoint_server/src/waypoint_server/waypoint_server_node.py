#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty

class WaypointServer:
    def __init__(self):
        rospy.init_node('waypoint_server')
        self.server = InteractiveMarkerServer("waypoint_server")
        self.waypoints = []
        self.next_id = 0
        self.path_publisher = rospy.Publisher('/interactive_waypoints', PoseStamped, queue_size=10)
        self.state_machine_pub = rospy.Publisher('/waypoint_server/command', Empty, queue_size=1)

        self.create_add_waypoint_marker()
        self.server.applyChanges()
        rospy.spin()

    def process_feedback(self, feedback):
        p = feedback.pose.position
        
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name.startswith("waypoint_"):
                rospy.loginfo(f"Waypoint {feedback.marker_name} at: {p.x}, {p.y}, {p.z}")
                # Update the waypoint's pose
                marker = self.server.get(feedback.marker_name)
                marker.pose = feedback.pose
                self.server.insert(marker, self.process_feedback)
                self.server.applyChanges()
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1: # Start Following
                self.send_waypoints_to_follower()
            elif feedback.menu_entry_id == 2: # Delete Waypoint
                self.server.erase(feedback.marker_name)
                self.waypoints = [w for w in self.waypoints if w.name != feedback.marker_name]
                self.server.applyChanges()
        
    def add_waypoint_marker(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            new_id = self.next_id
            self.next_id += 1
            marker_name = f"waypoint_{new_id}"

            int_marker = InteractiveMarker()
            int_marker.header.frame_id = feedback.header.frame_id
            int_marker.name = marker_name
            int_marker.description = f"Waypoint {new_id}"
            int_marker.pose = feedback.pose

            # Create waypoint marker visuals (e.g., a colored sphere)
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.a = 1.0
            
            # Add a movable control
            control = InteractiveMarkerControl()
            control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
            control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
            control.always_visible = True
            control.markers.append(marker)
            int_marker.controls.append(control)

            # Add a menu control
            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.MENU
            menu_control.always_visible = True
            int_marker.controls.append(menu_control)

            self.server.insert(int_marker, self.process_feedback)

            # Add menu entries
            self.server.insert(int_marker)
            self.server.setMenuHandler(int_marker.name, self.create_waypoint_menu())
            self.server.applyChanges()

            self.waypoints.append(int_marker)

    def create_waypoint_menu(self):
        from interactive_markers.menu_handler import MenuHandler
        menu_handler = MenuHandler()
        menu_handler.insert("Start Following", callback=self.process_feedback)
        menu_handler.insert("Delete Waypoint", callback=self.process_feedback)
        return menu_handler

    def create_add_waypoint_marker(self):
        add_marker = InteractiveMarker()
        add_marker.header.frame_id = "map"  # Or your desired fixed frame
        add_marker.name = "add_waypoint_button"
        add_marker.description = "Click to add Waypoint"
        add_marker.scale = 1.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

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

        self.server.insert(add_marker, self.add_waypoint_marker)

    def send_waypoints_to_follower(self):
        if not self.waypoints:
            rospy.logwarn("No waypoints to follow.")
            return
        
        rospy.loginfo("Sending waypoints to follower node.")
        
        # Publish each waypoint to the follower node
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = waypoint.header.frame_id
            pose.pose = waypoint.pose
            self.path_publisher.publish(pose)
            rospy.sleep(0.5) # A small delay to ensure sequential publishing

        # Send a signal to the follower to start execution
        self.state_machine_pub.publish(Empty())

if __name__ == '__main__':
    try:
        WaypointServer()
    except rospy.ROSInterruptException:
        pass
