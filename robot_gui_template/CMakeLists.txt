cmake_minimum_required(VERSION 3.0.2)
project(robot_gui_template)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  message_generation
)




find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Widgets Network Core PrintSupport REQUIRED)


#add_message_files(
#  FILES
#  pid_gain_msgs.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui_template
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime

)

file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
    )

file(GLOB HDRS
    "include/${PROJECT_NAME}/*.h"
    )

file(GLOB SRCS
    "src/*.cpp"
    )

file(GLOB QCP
    "qcustomplot/*.h"
    "qcustomplot/*.cpp"
    )

qt5_wrap_ui(UI_HDRS ${UI})



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${HDRS} ${UI_HDRS} ${SRCS} ${QCP})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Qt5::Widgets Qt5::Network Qt5::Core  Qt5::PrintSupport ${QT_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PRIVATE "include/${PROJECT_NAME}/" qcustomplot)
