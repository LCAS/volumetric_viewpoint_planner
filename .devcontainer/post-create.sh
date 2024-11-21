#!/bin/bash


function add_config_if_not_exist {
    if ! grep -F -q "$1" $HOME/.bashrc; then
        echo "$1" >> $HOME/.bashrc
    fi
}

add_config_if_not_exist "export ROS_DOMAIN_ID=9"

add_config_if_not_exist "source /opt/ros/humble/setup.bash"
LOCAL_SETUP_FILE=`pwd`/install/setup.bash
add_config_if_not_exist "if [ -r $LOCAL_SETUP_FILE ]; then source $LOCAL_SETUP_FILE; fi"

source $HOME/.bashrc

source /opt/ros/humble/setup.bash
colcon build --symlink-install --continue-on-error

sudo apt remove -y "*libfranka*"

sudo git clone --recursive https://github.com/frankaemika/libfranka.git /libfranka
cd /libfranka
sudo git checkout 0.8.0
sudo git submodule update
sudo sed -i '6i#include <string>' /libfranka/include/franka/control_tools.h
sudo sed -i '6i#include <stdexcept>' /libfranka/src/control_types.cpp
sudo mkdir -p /libfranka/build
cd /libfranka/build
sudo cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
sudo cmake --build . 
cd /libfranka/build
sudo cpack -G DEB
sudo dpkg -i libfranka*.deb
