FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

RUN apt update
# System normal libraries
RUN apt install -y git wget apt-utils

# install opencv
# RUN apt install -y libopencv-dev 

# install ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116\
    && echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list\
    && apt update\
    && apt install -y ros-kinetic-desktop-full\
    && rosdep init\
    && rosdep update\
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc\
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc\
    && apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
WORKDIR /root

# install opencv-python
RUN apt install -y python-numpy
# python-opencv

# install naoqi
ADD ./naoqi /root/naoqi
RUN apt install -y ros-kinetic-pepper-.*\
    && echo "export PYTHONPATH=/root/naoqi/pynaoqi-python2.7-2.4.3.28-linux64:\$PYTHONPATH" >> ~/.bashrc

# install environmets for model_free_appliance_manipulation
RUN apt install -y python-pip python-sympy libbullet-dev\
    && pip install rdflib dill\
    && apt install -y ros-kinetic-amcl ros-kinetic-move-base\
    libsdl1.2-dev ros-kinetic-fake-localization\
    ros-kinetic-move-slow-and-clear ros-kinetic-robot-pose-ekf\
    libnetpbm10-dev ros-kinetic-map-server ros-kinetic-global-planner\
    ros-kinetic-dwa-local-planner ros-kinetic-carrot-planner
RUN apt install -y python-catkin-tools

WORKDIR /root/catkin_ws
ADD ./utils /root/utils
ENTRYPOINT [ "/root/utils/entrypoint.sh" ]
CMD [ "bash" ]

#install openni2
RUN apt-get install -y ros-kinetic-openni*

RUN apt-get update\
    && apt-get install -y build-essential\
    && apt-get install -y llvm\
    && apt-get install -y clang-3.7 libc++-dev libc++abi-dev\
    && apt-get install -y cmake\
    && apt-get install -y libopenblas-dev liblapack-dev\
    && apt-get install -y git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev\
    && apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev checkinstall
   
#install opencv3
WORKDIR /root/
ADD ./opencv-3.2.0 /root/opencv-3.2.0
RUN cd opencv-3.2.0\
    && mkdir build\
    && cd build\
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -DWITH_LAPACK=OFF ..\
    && make -j9\
    && make install

        
WORKDIR /root/
ADD ./dlib-19.6 /root/dlib-19.6
RUN cd dlib-19.6\
    && mkdir build\
    && cd build\
    && cmake ..\
    && cmake --build . --config Release\
    && make install\
    && ldconfig

#install boost 
WORKDIR /root/
RUN apt-get install -y libbz2-dev\
    && apt-get install -y clang libc++-dev\
    && apt-get install -y libboost-all-dev

#install openface
WORKDIR /root/
ADD ./OpenFace /root/OpenFace
RUN cd OpenFace\
    && mkdir build\
    && cd build\
    && cmake -D CMAKE_BUILD_TYPE=RELEASE ..\
    && make -j9\
    && make install

#install darknet
WORKDIR /root/
ADD ./darknet /root/darknet
RUN cd darknet\
    && make -j9

#setup ros command
WORKDIR /root/
RUN echo "alias cw='cd ~/catkin_ws'" >> ~/.bashrc\
    && echo "alias cs='cd ~/catkin_ws/src'" >> ~/.bashrc\
    && echo "alias cm='cd ~/catkin_ws && catkin build'" >> ~/.bashrc

#setup urg_node,scantools
WORKDIR /root/
RUN apt-get install -y v4l-utils\
    && apt-get install -y ros-kinetic-csm\
    && apt-get install -y ros-kinetic-joy\
    && apt-get install -y ros-kinetic-depthimage-to-laserscan\
    && apt-get install -y ros-kinetic-laser-proc\
    && apt-get install -y ros-kinetic-urg-c

#setup python scipy
WORKDIR /root/
RUN  pip install --upgrade pip\
     && pip install scipy\
     && pip install --upgrade matplotlib

#install lsusb
WORKDIR /root/
RUN  apt-get install -y usbutils

#install Bluetooth
WORKDIR /root/
RUN  apt-get install -y apt-file\
     && apt-file update\
     && apt-file search add-apt-repository\
     && apt-get install -y software-properties-common\
     && apt-get install -y bluetooth\
     && apt-get install -y bluez

#install sixad
WORKDIR /root/
RUN  apt-get install -y pyqt4-dev-tools git\
     && apt-get install -y libusb-dev libjack-dev libbluetooth-dev pyqt4-dev-tools\
     && add-apt-repository "deb http://ppa.launchpad.net/falk-t-j/qtsixa/ubuntu vivid main"\
     && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 736E4F0B\
     && apt update\
     && apt install -y sixad

#install xtion calibration file
WORKDIR /root/
RUN cd .ros\
    && mkdir -p camera_info

WORKDIR /root/
ADD ./calibration /root/calibration
RUN cd calibration
