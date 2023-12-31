FROM ubuntu:22.04

WORKDIR /app

# Build Essentials 
RUN apt-get update && apt-get install -y git build-essential cmake  \
    && rm -rf /var/lib/apt/lists/*

# Cmake Dependencies 
RUN apt-get update && apt-get install -y libeigen3-dev libspdlog-dev libsuitesparse-dev
RUN apt-get update && apt-get install -y qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5



# Clone g2o and Build it 
RUN git clone -q --depth 1 --branch 20230806_git https://github.com/RainerKuemmerle/g2o \
    && cd g2o \
    && mkdir build && cd build && cmake .. && make && make install 

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH


# Clone Opencv4.5 with contrib and build it 
# see: https://docs.opencv.org/4.x/db/d05/tutorial_config_reference.html for more options
#RUN apt-get update && apt-get install -y wget unzip 

#RUN git clone -q --depth 1 --branch 4.5.0 https://github.com/opencv/opencv \
#    && git clone -q --depth 1 --branch 4.5.0 https://github.com/opencv/opencv_contrib \
#    && mkdir opencv-build && cd opencv-build && cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv && make 

# OpenCV 4.5 only 
#RUN git clone -q --depth 1 --branch 4.5.0 https://github.com/opencv/opencv \
#    && mkdir opencv-build && cd opencv-build && cmake ../opencv && make 

#-----------------------------
# How to build docker image  #
#-----------------------------
# docker build -t joke-image .
# docker commit <CONTAINER_ID> joke-image:taag
