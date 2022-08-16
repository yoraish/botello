cmake_minimum_required(VERSION 2.8.3)
project(botello_aruco_gridboard)
set (CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  geometry_msgs
  message_filters
  image_transport
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  tf
)



#find_package(Boost REQUIRED
#  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


catkin_package(
   CATKIN_DEPENDS
   cv_bridge
   geometry_msgs
   message_filters
   image_transport
   roscpp
   sensor_msgs
   std_msgs
)

add_executable(botello_aruco_gridboard_node
  src/botello_aruco_gridboard_node.cpp
)

target_link_libraries(botello_aruco_gridboard_node
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

# add_executable(create_board
#    src/create_board.cpp
# )
#
# target_link_libraries(create_board
#    ${OpenCV_LIBS}
# )
#
