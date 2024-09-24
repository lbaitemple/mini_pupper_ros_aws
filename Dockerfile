# Set main arguments.
ARG ROS_DISTRO=humble
ARG LOCAL_WS_DIR=workspace

# ==== ROS Build Stages ====

# ==== Base ROS Build Image ====
#FROM ros:${ROS_DISTRO}-ros-base AS build-base
FROM --platform=linux/arm64 ros:${ROS_DISTRO}-ros-base AS build-base
LABEL component="com.example.ros2.mini_pupper_v2"
LABEL build_step="ROSDemoNodes_Build"

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update --fix-missing && apt-get install python3-pip  ffmpeg portaudio19-dev -y
RUN apt-get update --fix-missing && apt-get install ros-$ROS_DISTRO-example-interfaces \
ros-$ROS_DISTRO-xacro  ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-v4l2-camera \
ros-$ROS_DISTRO-image-transport-plugins  -y
RUN python3 -m pip install awsiotsdk pydub pyaudio pydub sounddevice


# ==== Package 1: ROS Demos Talker/Listener ==== 
FROM build-base AS ros-demos-package
LABEL component="com.example.ros2.mini_pupper_v2"
LABEL build_step="DemoNodesROSPackage_Build"

# Clone the demos_ros_cpp package from within the ROS Demos monorepo.
RUN mkdir -p /ws/src/mini_pupper_ros
WORKDIR /ws/src

RUN git clone  -b ros2-dev-music  https://github.com/lbaitemple/mini_pupper_ros_aws  /ws/src/mini_pupper_ros  

RUN vcs import < mini_pupper_ros/.minipupper.repos --recursive && \
touch champ/champ/champ_gazebo/AMENT_IGNORE && \
touch champ/champ/champ_navigation/AMENT_IGNORE  && \
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE && \
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE 



RUN . /opt/ros/$ROS_DISTRO/setup.sh && cd .. && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} \
     --skip-keys=joint_state_publisher_gui --skip-keys=rviz2 --skip-keys=gazebo_plugins \
     --skip-keys=velodyne_gazebo_plugins -y && \
    colcon build --build-base workspace/build --install-base /opt/ros_demos

# ==== Package 2: Greengrass Bridge Node ==== 
FROM build-base AS greengrass-bridge-package
LABEL component="com.example.ros2.mini_pupper_v2"
LABEL build_step="GreengrassBridgeROSPackage_Build"
ARG LOCAL_WS_DIR

COPY ${LOCAL_WS_DIR}/src /ws/src
WORKDIR /ws

# Cache the colcon build directory.
RUN --mount=type=cache,target=${LOCAL_WS_DIR}/build:/ws/build \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
     --install-base /opt/greengrass_bridge

# ==== ROS Runtime Image (with the two packages) ====
FROM build-base AS runtime-image
LABEL component="com.example.ros2.demo"

# Clone the GitHub repository into the container
RUN git clone https://github.com/mangdangroboticsclub/mini_pupper_2_bsp /tmp/mini_pupper_2_bsp

# Set the working directory to the Python package directory
WORKDIR /tmp/mini_pupper_2_bsp/Python_Module
RUN apt update 
RUN apt install python3-pip python3-dev python-is-python3 -y 
# Install Python dependencies from requirements.txt if available
RUN pip install -r requirements.txt

# Install your Python package
RUN python setup.py install


COPY --from=ros-demos-package /opt/ros_demos /opt/ros_demos
COPY --from=greengrass-bridge-package /opt/greengrass_bridge /opt/greengrass_bridge

RUN mkdir -p /root/.ros/camera_info/
COPY mmal_service_16.1.yaml /root/.ros/camera_info
COPY scripts/robot-entrypoint.sh  /robot-entrypoint.sh
RUN chmod +x /robot-entrypoint.sh
ENTRYPOINT ["/robot-entrypoint.sh"]

