cmake_minimum_required(VERSION 3.5)
project(atama)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(atama_interfaces REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(ninshiki_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shisen_interfaces REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/head/head.cpp"
  "src/${PROJECT_NAME}/receiver/node/receiver_node.cpp"
  "src/${PROJECT_NAME}/sender/node/sender_node.cpp"
  "src/${PROJECT_NAME}/node/atama_node.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  atama_interfaces
  kansei_interfaces
  keisan
  ninshiki_interfaces
  OpenCV
  rclcpp
  rclcpp_action
  shisen_interfaces
  tachimawari
  tachimawari_interfaces)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(atama_main "src/atama_main.cpp")
target_include_directories(atama_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(atama_main ${PROJECT_NAME})

install(TARGETS atama_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  atama_interfaces
  kansei_interfaces
  keisan
  ninshiki_interfaces
  OpenCV
  rclcpp
  rclcpp_action
  shisen_interfaces
  tachimawari
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
