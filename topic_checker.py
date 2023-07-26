#!/usr/bin/env python

import rospy
import Tkinter as tk

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

# Timeout to wait for a message in seconds
timeout = 2.0

def check_topic(topic):
    try:
        rospy.wait_for_message(topic, rospy.AnyMsg, timeout)
        return True
    except rospy.ROSException:
        return False

def create_label(root, text, is_on):
    label_color = "green" if is_on else "red"
    label_text = "{}: {}".format(text, "ON" if is_on else "OFF")
    tk.Label(root, text=label_text, fg=label_color).pack()

def check_topics_and_create_labels(root, topic_group_name, topics):
    is_group_on = all(check_topic(topic) for topic in topics)
    create_label(root, topic_group_name, is_group_on)

def main():
    rospy.init_node('topic_checker')
    root = tk.Tk()

    check_topics_and_create_labels(root, "SBG topics", sbg_topics)
    check_topics_and_create_labels(root, "IMU topics", imu_topics)
    check_topics_and_create_labels(root, "Camera 0 topics", camera0_topics)
    check_topics_and_create_labels(root, "Camera 1 topics", camera1_topics)
    check_topics_and_create_labels(root, "Camera 2 topics", camera2_topics)
    check_topics_and_create_labels(root, "Steer Angle topic", steer_angle_topic)
    check_topics_and_create_labels(root, "Wheelspeed B topic", wheelspeed_b_topic)
    check_topics_and_create_labels(root, "Wheelspeed F topic", wheelspeed_f_topic)
    check_topics_and_create_labels(root, "Velodyne topics", velodyne_topics)
    check_topics_and_create_labels(root, "Points Raw topic", points_raw_topic)

    root.mainloop()

if __name__ == "__main__":
    main()
