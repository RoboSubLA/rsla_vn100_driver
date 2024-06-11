cmake_minimum_required(VERSION 3.8)
project(rsla_vn100_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rsla_interfaces REQUIRED)

add_subdirectory(include/vncpplib/cpp libvncxx)
include_directories(include/vncpplib/cpp/include)

add_executable(vn100 src/vn100.cpp)

target_link_libraries(vn100 PRIVATE libvncxx)
if (UNIX OR APPLE)
	target_link_libraries(vn100 LINK_PUBLIC pthread)
else()
	target_link_libraries(vn100 LINK_PUBLIC Advapi32 SetupAPI)
endif()

target_include_directories(vn100 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(vn100 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(vn100 PUBLIC rclcpp std_msgs rsla_interfaces)

install(TARGETS vn100
  DESTINATION lib/${PROJECT_NAME})

ament_package()
