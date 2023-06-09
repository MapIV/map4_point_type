cmake_minimum_required(VERSION 3.8)
project(map4_point_type)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(map4_point_type SHARED
  src/map4_point_type.cpp
  src/map4_point_type_double.cpp
  src/type_detector.cpp
)
target_link_libraries(map4_point_type ${PCL_LIBRARIES})

ament_auto_add_executable(${PROJECT_NAME}_test src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test map4_point_type)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/
        DESTINATION include)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}_test
        EXPORT ${PROJECT_NAME}_test
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_auto_package()
