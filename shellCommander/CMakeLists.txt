cmake_minimum_required(VERSION 3.0.2)
project(shellCommander)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
)

find_package(OpenCV)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_maze_solver_oop
  src/maze_solver_oop_main.cpp
  src/CameraSensor.cpp
  src/LaserScanner.cpp
  src/RobotChassis.cpp
  src/MazeSolverBot.cpp
)

set_target_properties(${PROJECT_NAME}_maze_solver_oop PROPERTIES OUTPUT_NAME maze_solver_oop PREFIX "")
target_link_libraries(${PROJECT_NAME}_maze_solver_oop
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

add_executable(rviz_click_to_2d src/rviz_click_to_2d.cpp)
target_link_libraries(rviz_click_to_2d ${catkin_LIBRARIES})
