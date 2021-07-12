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
find_package(aruku REQUIRED)
find_package(framework_common REQUIRED)
find_package(kansei REQUIRED)
find_package(ninshiki_opencv REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/head.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  aruku
  framework_common
  kansei
  ninshiki_opencv)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(main "src/head_main.cpp")
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS main
  DESTINATION "lib/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  aruku
  framework_common
  kansei
  ninshiki_opencv)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
