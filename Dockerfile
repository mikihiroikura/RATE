FROM osrf/ros:melodic-desktop-full
RUN apt update && apt install -y git wget
ENV DISPLAY=host.docker.internal:0.0
## python-catkin-tools
#### Add User ID and Group ID
ARG UNAME=rate
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

ARG CODE_DIR=/home/${UNAME}

#### ROS setup ####
RUN mkdir -p ${CODE_DIR}/catkin_ws/src && \
    echo "source /opt/ros/melodic/setup.bash" >> ${CODE_DIR}/.bashrc

#### Install dependencies ####
RUN apt-get update && apt-get install -y python-catkin-tools
# build libcaer library
RUN mkdir ${CODE_DIR}/src && cd ${CODE_DIR}/src && \
    git clone https://gitlab.com/inivation/dv/libcaer.git && cd libcaer && \
    apt-get install -y build-essential cmake pkg-config libusb-1.0-0-dev && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr . && \
    make && make install
# build ceres
RUN apt-get install -y libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev && \
    cd ${CODE_DIR}/src && wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && \
    tar zxf ceres-solver-2.1.0.tar.gz && \
    mkdir -p ${CODE_DIR}/src/ceres-solver-2.1.0/build && cd ${CODE_DIR}/src/ceres-solver-2.1.0/build && \
    cmake .. && make && make install

#### Copy source code ####
COPY ./ ${CODE_DIR}/catkin_ws/src

RUN chown -R $UNAME:$UNAME ${CODE_DIR}/catkin_ws

USER $UNAME
WORKDIR ${CODE_DIR}