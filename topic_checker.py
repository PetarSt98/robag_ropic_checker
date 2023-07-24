#!/usr/bin/env python

import rosbag
import Tkinter as tk
import sys

bag = rosbag.Bag(sys.argv[1])

# Define the topic groups
sbg_topics = ['/sbg/ekf_euler', '/sbg/ekf_nav', '/sbg/ekf_quat', '/sbg/gps_pos', '/sbg/gps_raw', '/sbg/gps_vel', '/sbg/imu_data', '/sbg/status', '/sbg/utc_time']
imu_topics = ['/imu/data', '/imu/nav_sat_fix', '/imu/pos_ecef', '/imu/temp', '/imu/utc_ref', '/imu/velocity']
camera0_topics = ['/camera', '/camera_array/cam0/camera_info', '/camera_array/cam0/image_raw']
camera1_topics = ['/camera_array/cam1/camera_info', '/camera_array/cam1/image_raw']
camera2_topics = ['/camera_array/cam2/camera_info', '/camera_array/cam2/image_raw']
steer_angle_topic = ['/steer_angle']
wheelspeed_b_topic = ['/wheelspeed_b']
wheelspeed_f_topic = ['/wheelspeed_f']
velodyne_topics = ['/velodyne_nodelet_manager_driver/parameter_descriptions', '/velodyne_nodelet_manager_driver/parameter_updates', '/velodyne_packets']
points_raw_topic = ['/points_raw']

def check_topics(topics):
    return all(topic in bag.get_type_and_topic_info()[1].keys() for topic in topics)

def create_label(root, text, is_on):
    label_color = "green" if is_on else "red"
    label_text = "{}: {}".format(text, "ON" if is_on else "OFF")
    tk.Label(root, text=label_text, fg=label_color).pack()

root = tk.Tk()

create_label(root, "SBG topics", check_topics(sbg_topics))
create_label(root, "IMU topics", check_topics(imu_topics))
create_label(root, "Camera 0 topics", check_topics(camera0_topics))
create_label(root, "Camera 1 topics", check_topics(camera1_topics))
create_label(root, "Camera 2 topics", check_topics(camera2_topics))
create_label(root, "Steer Angle topic", check_topics(steer_angle_topic))
create_label(root, "Wheelspeed B topic", check_topics(wheelspeed_b_topic))
create_label(root, "Wheelspeed F topic", check_topics(wheelspeed_f_topic))
create_label(root, "Velodyne topics", check_topics(velodyne_topics))
create_label(root, "Points Raw topic", check_topics(points_raw_topic))

root.mainloop()

