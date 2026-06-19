cmake_minimum_required(VERSION 3.8)
project(ati_axia80_m20_ethercat_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

find_path(ECRT_INCLUDE_DIR ecrt.h
  PATHS /usr/include /usr/local/include /opt/etherlab/include)
find_library(ECRT_LIBRARY ethercat
  PATHS /usr/lib /usr/local/lib /opt/etherlab/lib)

if(NOT ECRT_INCLUDE_DIR OR NOT ECRT_LIBRARY)
  message(FATAL_ERROR "IgH EtherCAT master development files were not found. Install libethercat/ecrt headers and library.")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  std_srvs
)

add_library(${PROJECT_NAME} SHARED
  src/axia80_ethercat_config.cpp
  src/axia80_ethercat_driver.cpp
  src/axia80_m20_ethercat_sensor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ECRT_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} ${ECRT_LIBRARY})
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ros2_control/ati_axia80_m20_ethercat_sensor.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config examples launch ros2_control urdf DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
