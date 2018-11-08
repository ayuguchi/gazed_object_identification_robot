FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# install system default utils and ros-kinetic
RUN set -x \
    && apt update \
    && : "install system default utils"\
    && apt install -y\
        git \
        wget \
        apt-utils\
        build-essential \
        llvm \
        clang-3.7 \
        libc++-dev \
        libc++abi-dev \
        cmake \
        libopenblas-dev \
        liblapack-dev \
        libgtk2.0-dev \
        v4l-utils \
        pkg-config \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        python-dev \
        python-numpy \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libjasper-dev \
        libdc1394-22-dev \
        checkinstall \
        libbz2-dev \
        libc++-dev \
        libboost-all-dev \
    && : "install ros-kinetic"\
    && apt-get install -y --no-install-recommends \
        dirmngr \
        gnupg2 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116\
    && echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list\
    && apt update\
    && apt install -y ros-kinetic-desktop-full\
    && rosdep init\
    && rosdep update\
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc\
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc\
    && apt install -y \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
    && : "install depends for this app" \
    && apt install -y \
        python-pip \
        python-sympy \
        python-scipy \
        libbullet-dev\
        ros-kinetic-amcl \
        ros-kinetic-move-base\
        libsdl1.2-dev \
        ros-kinetic-fake-localization \
        ros-kinetic-move-slow-and-clear \
        ros-kinetic-robot-pose-ekf \
        libnetpbm10-dev \
        ros-kinetic-map-server \
        ros-kinetic-global-planner \
        ros-kinetic-dwa-local-planner \
        ros-kinetic-carrot-planner \
        ros-kinetic-csm \
        ros-kinetic-depthimage-to-laserscan \
        ros-kinetic-joy \
        ros-kinetic-laser-proc \
        ros-kinetic-urg-c\
        python-catkin-tools\
        ros-kinetic-openni*

# install librealsense
RUN set -x && \
    apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE && \
    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u && \
    apt update && \
    apt install -y \
        librealsense2-dkms \
        librealsense2=2.16.3-0~realsense0.116 \
        librealsense2-utils=2.16.3-0~realsense0.116 \
        librealsense2-dev=2.16.3-0~realsense0.116 \
        librealsense2-dbg=2.16.3-0~realsense0.116

#install opencv3
WORKDIR /root/
ADD ./opencv /root/opencv
RUN set -x\
    && cd opencv\
    && mkdir build\
    && cd build\
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -DWITH_LAPACK=OFF ..\
    && make -j$(nproc)\
    && make install

# install dlib
WORKDIR /root/
ADD ./dlib /root/dlib
RUN set -x\
    && cd dlib\
    && mkdir build\
    && cd build\
    && cmake ..\
    && cmake --build . --config Release\
    && make -j$(nproc)\
    && make install\
    && ldconfig

#install openface
WORKDIR /root/
ADD ./OpenFace /root/OpenFace
RUN set -x\
    && cd OpenFace\
    && mkdir build\
    && cd build\
    && cmake -D CMAKE_BUILD_TYPE=RELEASE ..\
    && make -j$(nproc)\
    && make install

#setup ros command
WORKDIR /root/
RUN set -x \
    echo "alias cw='cd ~/catkin_ws'" >> ~/.bashrc\
    && echo "alias cs='cd ~/catkin_ws/src'" >> ~/.bashrc\
    && echo "alias cm='cd ~/catkin_ws && catkin build'" >> ~/.bashrc

WORKDIR /root/catkin_ws
ADD ./utils /root/utils
ENTRYPOINT [ "/root/utils/entrypoint.sh" ]
CMD [ "bash" ]
