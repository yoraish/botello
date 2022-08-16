cmake_minimum_required(VERSION 2.8.3)
project(botello_maps)
set (CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
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
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


catkin_package(
   CATKIN_DEPENDS
   geometry_msgs
   message_filters
   roscpp
   sensor_msgs
   std_msgs
)

# add_executable(botello_maps_node
#   src/botello_maps_node.cpp
# )

# target_link_libraries(botello_maps_node
#     ${resource_retriever_LIBRARIES}
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
# )
