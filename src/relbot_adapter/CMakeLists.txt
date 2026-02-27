cmake_minimum_required(VERSION 3.8)
project(relbot_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find mandatory dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(relbot_msgs REQUIRED)

# Optional dependency
find_package(xrf2_msgs QUIET)

if(xrf2_msgs_FOUND)
  message(STATUS "xrf2_msgs found, enabling related features")
  add_definitions(-DHAS_XRF2_MSGS)
  set(HAS_XRF2_MSGS TRUE)
else()
  message(STATUS "xrf2_msgs not found, building without xrf2 support.")
  set(HAS_XRF2_MSGS FALSE)
endif()

include_directories(
  include
)

add_executable(relbot_adapter src/relbot_adapter.cpp)

# Add only the available dependencies
set(DEPENDENCIES rclcpp example_interfaces geometry_msgs relbot_msgs)
if(HAS_XRF2_MSGS)
  list(APPEND DEPENDENCIES xrf2_msgs)
endif()

ament_target_dependencies(relbot_adapter ${DEPENDENCIES})

target_include_directories(relbot_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(relbot_adapter PUBLIC c_std_99 cxx_std_17)

# Optional: If you're using custom message targets for linking (likely not needed unless you're doing advanced linking)
# Remove the line below if you get errors from unknown *_TARGETS variables
# target_link_libraries(${PROJECT_NAME} ${example_interfaces_TARGETS} ${std_msgs_TARGETS} ${relbot_msgs_TARGETS})

install(TARGETS relbot_adapter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
