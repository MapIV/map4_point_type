cmake_minimum_required(VERSION 3.5)
project(map4_point_type)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(map4_point_type
  src/map4_point_type.cpp
  src/map4_point_type_double.cpp
  src/type_detector.cpp
)
target_link_libraries(map4_point_type ${PCL_LIBRARIES})

install(TARGETS map4_point_type
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(PCL)
ament_export_include_directories(include)
ament_export_libraries(map4_point_type)

ament_package()