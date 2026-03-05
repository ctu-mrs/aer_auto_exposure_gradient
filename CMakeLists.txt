cmake_minimum_required(VERSION 3.8)
project(aer_auto_exposure_gradient)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_FLAGS "-std=c++23 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(plotter_ros2 QUIET)
if(plotter_ros2_FOUND)
  message(STATUS "plotter_ros2 found — building with plotter support")
  list(APPEND DEPENDENCIES plotter_ros2)
else()
  message(STATUS "plotter_ros2 not found — building without plotter support")
endif()

include_directories(
  include
)

# include_directories(
#  include
#   ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME}_component SHARED src/auto_exp.cpp)
target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_component ${DEPENDENCIES})
if(plotter_ros2_FOUND)
  target_compile_definitions(${PROJECT_NAME}_component PRIVATE WITH_PLOTTER)
endif()
# install(TARGETS ${PROJECT_NAME}_component DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(
    ${PROJECT_NAME}_component
    PLUGIN "exp_node::ExpNode"
    EXECUTABLE ${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME}_component)
install(TARGETS ${PROJECT_NAME}_component
        EXPORT export_${PROJECT_NAME}_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_package()
