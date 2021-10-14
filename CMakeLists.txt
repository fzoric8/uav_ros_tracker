cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure uav_ros_lib
                                        dynamic_reconfigure mavros_msgs uav_ros_msgs tf2_ros tf2)

generate_dynamic_reconfigure_options(cfg/MPCTrackerParameters.cfg)

catkin_package(INCLUDE_DIRS include LIBRARIES CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_library(mpc_cvxgen src/cvxgen/ldl.c src/cvxgen/matrix_support.c
                       src/cvxgen/solver.c src/cvxgen/util.c)

add_library(mpc_wrapper src/cvx_wrapper.cpp)
target_link_libraries(mpc_wrapper mpc_cvxgen ${catkin_LIBRARIES})

add_executable(mpc_tracker src/mpc_tracker.cpp nodes/mpc_tracker_node.cpp)
target_link_libraries(mpc_tracker mpc_wrapper)
add_dependencies(mpc_tracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(waypoint_manager_nodelet src/waypoint_publisher.cpp
                                     nodes/waypoint_manager_nodelet.cpp)
add_dependencies(waypoint_manager_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_manager_nodelet ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
