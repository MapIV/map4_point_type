cmake_minimum_required(VERSION 3.0.0)
project(map4_point_type)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(map4_point_type
			src/map4_point_type.cpp
			src/map4_point_type_double.cpp
			src/type_detector.cpp
			)
target_link_libraries(map4_point_type ${PCL_LIBRARIES})

# Install the library target
install(TARGETS map4_point_type
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib)

# Install the header files
install(DIRECTORY include/ DESTINATION include)

# Uninstall
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
	IMMEDIATE @ONLY)

add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
