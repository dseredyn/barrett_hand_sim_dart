cmake_minimum_required(VERSION 2.8)
project(barrett_hand_sim_dart)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs tf eigen_conversions cmake_modules planer_utils
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(DART REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS}
    ${DART_LIBRARY_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${DART_INCLUDE_DIRS})

add_executable(barrett_hand_sim_dart_test src/dart_test.cpp)

target_link_libraries(barrett_hand_sim_dart_test ${catkin_LIBRARIES} ${OpenRAVE_LIBRARIES} ${DART_LIBRARIES})

catkin_package(
    DEPENDS eigen
)

