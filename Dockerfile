FROM ubuntu:bionic

# Install ROS 2 dependencies
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt update -qq && \
    apt install -y \
      curl \
      gnupg2 \
      locales \
      lsb-release \
      software-properties-common && \
    curl http://repo.ros2.org/repos.key | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update -qq && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    apt install -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-lark-parser \
      python3-lxml \
      python3-numpy \
      python3-pip \
      python-rosdep \
      python3-vcstool \
      wget && \
    python3 -m pip install -U \
      argcomplete \
      flake8 \
      flake8-blind-except \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest \
      pytest-cov \
      pytest-runner \
      setuptools && \
    apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install Gradle
RUN apt update -qq && \
    apt install -y gradle && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Colcon gradle support
RUN python3 -m pip install -U git+https://github.com/colcon/colcon-gradle@jacob/classpath_hook
RUN python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle@jacob/ament_prefix_path_hook

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
    curl -skL https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos -o ros2.repos && \
    curl -skL https://gist.githubusercontent.com/jacobperron/c21b5fd9a9661e5d03cb444d0565254b/raw/bbd0b600ad1c15a9c0883fa39d18df37e90d5e92/rcljava.repos -o rcljava.repos && \
    vcs import src < ros2.repos && \
    vcs import src < rcljava.repos && \
    apt update -qq && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers rosidl_typesupport_java" && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Build (rcljava and examples)
# RUN cd ~/ros2_java_ws && \
#     colcon build --packages-up-to rcljava rcljava_examples
