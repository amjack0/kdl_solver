FROM ubuntu:latest
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y && \ 
    apt-get install python3.5 && \
    apt-get install -y build-essential git cmake autoconf libtool pkg-config && \
    apt-get install libeigen3-dev && \   
    apt-get install -y python-kdl-parser  && \ 
    apt-get install ros-noetic-orocos-toolchain  
EXPOSE 9003

