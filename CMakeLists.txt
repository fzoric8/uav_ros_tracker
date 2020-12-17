cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure uav_ros_lib)

catkin_package(INCLUDE_DIRS include LIBRARIES CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_library(mpc_cvxgen src/cvxgen/ldl.c src/cvxgen/matrix_support.c src/cvxgen/solver.c
                       src/cvxgen/util.c)

add_library(mpc_wrapper src/cvx_wrapper.cpp)
target_link_libraries(mpc_wrapper mpc_cvxgen ${catkin_LIBRARIES})

add_executable(mpc_test nodes/mpc_test)
target_link_libraries(mpc_test mpc_wrapper)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})