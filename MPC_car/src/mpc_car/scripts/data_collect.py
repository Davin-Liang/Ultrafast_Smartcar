#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import csv
import os
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class TrajectoryRecorder:
    def __init__(self, path_type, goal_mode):
        rospy.init_node('trajectory_recorder')

        self.tf_listener = tf.TransformListener()
        self.path_type = path_type  # 0: bspline, 1: hybrid
        self.goal_mode = goal_mode
        self.tracking_started = False
        self.tracking_stopped = False

        self.path_info = {
            "hybrid": {
                "topic": "/move_base/HybridAStarPlanner/plan",
                "path_file": "planned_path.csv"
            },
            "bspline": {
                "topic": "/move_base/HybridAStarPlanner/plan_bspline",
                "path_file": "bspline_path.csv"
            }
        }

        self.actual_file_map = {
            "hybrid": "actual_path.csv",
            "bspline": "actual_bspline_path.csv"
        }

        self.key = "hybrid" if path_type == 1 else "bspline"
        self.goal_filename = "goal_pose.csv"

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher(self.path_info[self.key]["topic"], Path, queue_size=1)

        if goal_mode != 2:
            for k in self.path_info:
                with open(self.path_info[k]["path_file"], "w", newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['x', 'y', 'z', 'yaw', 'path_length', 'point_count'])

        with open(self.actual_file_map[self.key], "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'yaw'])

        self.actual_file = open(self.actual_file_map[self.key], "a", newline='')
        self.actual_writer = csv.writer(self.actual_file)

        if goal_mode in [0, 1]:
            for k in self.path_info:
                rospy.Subscriber(self.path_info[k]["topic"], Path, self.path_callback, callback_args=k)

        if goal_mode == 1:
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        rospy.Subscriber("/tracking_status", Int32, self.tracking_status_callback)
        self.timer = rospy.Timer(rospy.Duration(0.04), self.timer_callback)

        if goal_mode in [0, 2]:
            rospy.sleep(1.0)
            if 0 == goal_mode:
                self.publish_previous_goal()
            if goal_mode == 2:
                self.publish_path_from_csv()

    def path_callback(self, msg, path_key):
        if self.goal_mode == 2:
            return

        rospy.loginfo(f"Received path from {path_key}. Recording...")
        length = 0.0
        last_x, last_y = None, None
        count = 0
        path_data = []

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            q = pose.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            if last_x is not None:
                length += math.hypot(x - last_x, y - last_y)
            last_x, last_y = x, y
            path_data.append((x, y, z, yaw))
            count += 1

        with open(self.path_info[path_key]["path_file"], "a", newline='') as f:
            writer = csv.writer(f)
            for x, y, z, yaw in path_data:
                writer.writerow([x, y, z, yaw, round(length, 3), count])

        rospy.loginfo(f"Path recorded: {path_key}, {count} points, total length {length:.2f} m")

    def goal_callback(self, msg):
        if not self.tracking_started and not self.tracking_stopped:
            rospy.loginfo("Received goal. Starting to record actual path.")
            with open(self.goal_filename, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z', 'yaw'])

                x = msg.pose.position.x
                y = msg.pose.position.y
                z = msg.pose.position.z
                q = msg.pose.orientation
                yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                writer.writerow([x, y, z, yaw])
            self.tracking_started = True

    def publish_previous_goal(self):
        if os.path.exists(self.goal_filename):
            with open(self.goal_filename, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row['z'])
                    yaw = float(row['yaw'])
                    break

            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            rospy.loginfo("Publishing previous goal.")
            self.goal_pub.publish(msg)
            self.tracking_started = True

    def publish_path_from_csv(self):
        if not os.path.exists(self.path_info[self.key]["path_file"]):
            rospy.logwarn("No path file found for replay.")
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        with open(self.path_info[self.key]["path_file"], "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.position.z = float(row['z'])

                yaw = float(row['yaw'])
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                path.poses.append(pose)

        rospy.sleep(1.0)
        self.path_pub.publish(path)
        rospy.loginfo("Published path from CSV.")
        self.tracking_started = True

    def tracking_status_callback(self, msg):
        if msg.data == 1 and self.tracking_started:
            rospy.loginfo("Tracking finished. Stopping actual path recording.")
            self.tracking_stopped = True
            self.actual_file.close()
            self.timer.shutdown()

    def timer_callback(self, event):
        if (self.goal_mode != 1 or self.tracking_started) and not self.tracking_stopped:
            try:
                (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                x, y, z = trans
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                self.actual_writer.writerow([x, y, z, yaw])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def shutdown(self):
        rospy.loginfo("Shutting down node.")
        if not self.actual_file.closed:
            self.actual_file.close()

if __name__ == "__main__":
    print("Select path type:")
    print("1 - Hybrid A*")
    print("0 - B-spline")
    path_type = int(input("Enter: "))

    print("\nSelect goal mode:")
    print("1 - Record new goal from topic")
    print("0 - Reuse last recorded goal and publish")
    print("2 - Replay last path only (no new goal published)")
    goal_mode = int(input("Enter: "))

    recorder = TrajectoryRecorder(path_type, goal_mode)
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()
