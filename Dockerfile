FROM ros:dashing

# Install build dependencies
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt update -qq && \
    apt install -y \
      curl \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      python-rosdep \
      python3-vcstool && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install Gradle
RUN apt update -qq && \
    apt install -y gradle && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Colcon gradle support
RUN python3 -m pip install -U git+https://github.com/colcon/colcon-gradle
RUN python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle

# Install Java (or just rely on rosdep key 'java'?)
RUN apt update -qq && \
    apt install -y default-jdk && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Alternatively, Oracle's JDK
# RUN add-apt-repository ppa:linuxuprising/java && \
#     apt update -qq && \
#     apt install -y oracle-java11-installer && \
#     apt clean && \
#     rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Fetch code and install dependencies with rosdep
RUN mkdir -p ~/ros2_java_ws/src && \
    cd ~/ros2_java_ws && \
    curl -skL https://raw.githubusercontent.com/ros2-java/ros2_java/dashing/ros2_java_desktop.repos -o rcljava.repos && \
    vcs import src < rcljava.repos && \
    apt update -qq && \
    rosdep update && \
    RTI_NC_LICENSE_ACCEPTED=yes rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "ament_tools" && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Build (rcljava and examples)
RUN cd ~/ros2_java_ws && \
    . /opt/ros/dashing/setup.sh && \
    colcon build --packages-up-to rcljava rcljava_examples
