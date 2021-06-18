FROM ros:melodic-perception-bionic



ENV ROS_PYTHON_VERSION=3

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3 python3-dev python3-pip python3-setuptools && \
    pip3 install -U wheel rosdep rosinstall_generator wstool rosinstall

# ROS dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-melodic-diagnostic-updater \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/xnergy_ws
COPY . src/xnergy_charger_rcu

RUN rosdep install --os=ubuntu:bionic --from-paths src --ignore-src --rosdistro melodic --skip-keys=sbcl -y
RUN pip3 install -r src/xnergy_charger_rcu/requirements.txt

RUN /ros_entrypoint.sh catkin_make  \
    && sed -i '$isource "/home/xnergy_ws/devel/setup.bash"' /ros_entrypoint.sh