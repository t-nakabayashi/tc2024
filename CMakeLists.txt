cmake_minimum_required(VERSION 2.8.3)
project(tc2024)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS ypspur_ros tf)

## Declare a catkin package
catkin_package(
    DEPENDS ypspur_ros tf
)
