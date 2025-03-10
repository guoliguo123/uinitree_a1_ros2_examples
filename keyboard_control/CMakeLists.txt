cmake_minimum_required(VERSION 3.5)
project(keyboard_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(a1_msgs REQUIRED)
set(dependencies
		rclcpp
		geometry_msgs
		)

include_directories(
		include
		$ENV{UNITREE_LEGGED_SDK_PATH}/include
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
set(LIB_DIR /usr/lib/x86_64-linux-gnu)
add_library(libboost_system SHARED IMPORTED)
set_target_properties(libboost_system PROPERTIES IMPORTED_LOCATION ${LIB_DIR}/libboost_system.so.1.65.1)
add_executable(keyboard 
               src/msg_pub.cpp
               src/keyboard.cpp 
			   src/main.cpp)
ament_target_dependencies(keyboard
	                      ${dependencies}
			              a1_msgs)

target_link_libraries(keyboard 
		boost_thread
		libboost_system)
install(TARGETS
        keyboard
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()

