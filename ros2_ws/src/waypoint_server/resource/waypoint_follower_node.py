#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')
        self.waypoints = []
        self.current_waypoint_index = 0
        self.action_client = SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()

        rospy.Subscriber('/interactive_waypoints', PoseStamped, self.waypoint_callback)
        rospy.Subscriber('/waypoint_server/command', Empty, self.start_following_callback)

    def waypoint_callback(self, msg):
        rospy.loginfo("Received new waypoint.")
        self.waypoints.append(msg)

    def start_following_callback(self, msg):
        rospy.loginfo("Starting to follow waypoints.")
        self.current_waypoint_index = 0
        self.follow_next_waypoint()

    def follow_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("Finished all waypoints.")
            self.waypoints = [] # Clear waypoints after completion
            return

        waypoint_pose = self.waypoints[self.current_waypoint_index]
        goal = MoveBaseGoal()
        goal.target_pose = waypoint_pose

        rospy.loginfo(f"Sending goal to waypoint {self.current_waypoint_index}: {waypoint_pose.pose.position}")
        self.action_client.send_goal(goal, done_cb=self.done_callback)

    def done_callback(self, status, result):
        if status == 3:  # Succeeded
            rospy.loginfo(f"Successfully reached waypoint {self.current_waypoint_index}.")
            self.current_waypoint_index += 1
            self.follow_next_waypoint()
        else:
            rospy.logwarn(f"Failed to reach waypoint {self.current_waypoint_index}. Status: {status}")
            # Optional: handle failure (e.g., retry or skip)

if __name__ == '__main__':
    try:
        WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
