cmake_minimum_required(VERSION 3.0.0)
project(map4_point_type)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(PCL  REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
  LIBRARIES map4_point_type
)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(map4_point_type
  include/map4_point_type/map4_point_type.h
  # include/map4_point_type/type_detector.hpp
  src/type_detector.cpp
)
target_link_libraries(map4_point_type ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_library(map4_point_type INTERFACE)
# target_include_directories(map4_point_type INTERFACE .)
