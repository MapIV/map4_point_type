cmake_minimum_required(VERSION 3.8)
project(map4_point_type)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_library(map4_point_type SHARED
  src/map4_point_type.cpp
  src/map4_point_type_double.cpp
  src/type_detector.cpp
)
target_include_directories(map4_point_type PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(map4_point_type PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(map4_point_type ${PCL_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS map4_point_type
  EXPORT export_map4_point_type
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_map4_point_type HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL)
ament_package()
