cmake_minimum_required(VERSION 3.0.2)
project(uav_follower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  tf
)




## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES uav_follower
  #  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy tf
  #  DEPENDS system_lib
  )
  
  ###########
  ## Build ##
  ###########
  
  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    )
    
    
    catkin_install_python(PROGRAMS
      src/uav_follower_node.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )