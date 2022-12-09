FROM ros:noetic-perception-focal

ENV ROS_PYTHON_VERSION=3

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3 python3-dev python3-pip python3-setuptools \
    ros-noetic-diagnostic-updater \
    iproute2 \
    can-utils \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/xnergy_ws
COPY . src/xnergy_charger_rcu

RUN rosdep install --os=ubuntu:focal --from-paths src --ignore-src --rosdistro noetic --skip-keys=sbcl -y
RUN pip3 install -r src/xnergy_charger_rcu/requirements.txt

RUN /ros_entrypoint.sh catkin_make  \
    && sed -i '$isource "/home/xnergy_ws/devel/setup.bash"' /ros_entrypoint.sh