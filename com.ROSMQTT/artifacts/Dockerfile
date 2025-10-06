# ---- stage 1: get a single self-contained docker CLI binary ----
FROM docker:25.0-cli AS dockercli

# ---- stage 2: final ROS listener image ----
FROM ros:melodic-ros-base

RUN apt-get update && apt-get install -y lsb-release sudo && apt-get clean all && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install -y curl wget
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install basic packages
RUN apt-get update && apt-get install -y \
    python-pip python3-pip \
    ros-melodic-rospy-message-converter \
    && apt-get clean

# Add ROS repository

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install Python 3.10 from source (for AWS SDK)
RUN apt-get update && apt-get install -y \
    wget build-essential checkinstall libreadline-gplv2-dev \
    libncursesw5-dev libssl-dev libsqlite3-dev tk-dev \
    libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev && \
    cd /usr/src && \
    wget https://www.python.org/ftp/python/3.10.13/Python-3.10.13.tgz && \
    tar xzf Python-3.10.13.tgz && \
    cd Python-3.10.13 && \
    ./configure --enable-optimizations && \
    make altinstall && \
    apt-get clean

# Install AWS packages for Python 3.10
RUN /usr/local/bin/python3.10 -m pip install --no-cache-dir awsiotsdk awscrt

# Copy the static docker client from stage 1
COPY --from=dockercli /usr/local/bin/docker /usr/local/bin/docker

# RUN apt install -y ros-melodic-uuv-simulator
RUN /usr/local/bin/python3.10 -m pip install --no-cache-dir awsiotsdk

# Install custom messages
COPY bluerov_src /msgs/bluerov

# Before the catkin_make, add this to see dependencies
RUN bash -c "source /opt/ros/melodic/setup.bash && \
    cd /msgs/bluerov && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make"

# Copy application files
COPY ros-listener.py /ros-listener.py      
COPY aws-publisher.py /aws-publisher.py  
COPY entrypoint.sh /entrypoint.sh

# Copy AWS certificates
# COPY ben-dell-xps-Policy /ben-dell-xps-Policy
# COPY ben-dell-xps.cert.pem /ben-dell-xps.cert.pem
# COPY ben-dell-xps.private.key /ben-dell-xps.private.key
# COPY ben-dell-xps.public.key /ben-dell-xps.public.key
# COPY root-CA.crt /root-CA.crt

# COPY /opt/ros/ws /ws

# Set permissions
RUN chmod +x /entrypoint.sh /ros-listener.py /aws-publisher.py

ENTRYPOINT ["/entrypoint.sh"]