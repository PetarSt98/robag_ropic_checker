#!/usr/bin/env python

import rospy
import Tkinter as tk

topic_groups = {
    "sbg_topics": ['/sbg/ekf_euler', '/sbg/ekf_nav', '/sbg/ekf_quat', '/sbg/gps_pos', '/sbg/gps_raw', '/sbg/gps_vel', '/sbg/imu_data', '/sbg/status', '/sbg/utc_time'],
    "imu_topics": ['/imu/data', '/imu/nav_sat_fix', '/imu/pos_ecef', '/imu/temp', '/imu/utc_ref', '/imu/velocity'],
    "camera0_topics": ['/camera', '/camera_array/cam0/camera_info', '/camera_array/cam0/image_raw'],
    "camera1_topics": ['/camera_array/cam1/camera_info', '/camera_array/cam1/image_raw'],
    "camera2_topics": ['/camera_array/cam2/camera_info', '/camera_array/cam2/image_raw'],
    "steer_angle_topic": ['/steer_angle'],
    "wheelspeed_b_topic": ['/wheelspeed_b'],
    "wheelspeed_f_topic": ['/wheelspeed_f'],
    "velodyne_topics": ['/velodyne_nodelet_manager_driver/parameter_descriptions', '/velodyne_nodelet_manager_driver/parameter_updates', '/velodyne_packets'],
    "points_raw_topic": ['/points_raw']
}
# Timeout to wait for a message in seconds
timeout = 1.0


def check_topic_exists(topic):
    topic_list = rospy.get_published_topics()
    for t, _ in topic_list:
        if t == topic:
            return True
    return False


def check_topic_message(topic):
    try:
        rospy.wait_for_message(topic, rospy.AnyMsg, timeout)
        return True
    except rospy.ROSException:
        return False


def create_label(root, row, text, column):
    label_text = text
    status_text = "OFF"
    label = tk.Label(root, text=label_text, font=("Helvetica", 10))
    status = tk.Label(root, text=status_text, font=("Helvetica", 10), fg="red")
    label.grid(row=row, column=column*2, padx=10, pady=10)  # note the padding
    status.grid(row=row, column=column*2+1, padx=10, pady=10)  # note the padding
    return status  # return the status label so we can update it later


def check_topics_and_create_labels(root):
    labels = []  # a list to store the labels
    row = 0
    for group_name, topics in topic_groups.items():
        label_online = create_label(root, row, group_name + " online", 0)
        label_message = create_label(root, row, group_name + " receiving messages", 1)
        labels.append((label_online, label_message, topics))  # store the labels and topics
        row += 1
    return labels


def update_labels(root, labels):
    for label_online, label_message, topics in labels:
        is_group_online = all(check_topic_exists(topic) for topic in topics)
        is_group_on = all(check_topic_message(topic) for topic in topics)

        label_online.config(
            text="{}".format("ON" if is_group_online else "OFF"),
            fg="green" if is_group_online else "red")
        label_message.config(
            text="{}".format("ON" if is_group_on else "OFF"),
            fg="green" if is_group_on else "red")

    # Schedule the next update
    root.after(1000, update_labels, root, labels)  # update every 2 seconds (2000 milliseconds)


def main():
    rospy.init_node('topic_checker')

    root = tk.Tk()
    root.title("Topic Checker")

    labels = check_topics_and_create_labels(root)

    update_labels(root, labels)  # initial update

    root.mainloop()


if __name__ == "__main__":
    main()
