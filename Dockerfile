FROM ros:melodic

WORKDIR /app

COPY . .

RUN apt-get update && apt-get install -y \
    python-pip \
    ros-melodic-rospy \
    python-tk \
    x11-xserver-utils \
&& rm -rf /var/lib/apt/lists/*

CMD python topic_checker.py
