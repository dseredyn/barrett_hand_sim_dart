cmake_minimum_required(VERSION 2.8)
project(barrett_hand_sim_dart)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros rtt_ros rtt_rosclock std_msgs tf eigen_conversions cmake_modules planer_utils collision_convex_model
  )

#find_package(Boost COMPONENTS math REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(DART REQUIRED)
find_package(DARTCore REQUIRED)
find_package(Assimp REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS}
    ${DART_LIBRARY_DIRS}
    ${DARTCore_LIBRARY_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${DART_INCLUDE_DIRS})

add_executable(barrett_hand_sim_dart_test src/dart_test.cpp)
add_executable(barrett_hand_sim_dart_grasp src/dart_grasp.cpp)
add_executable(barrett_hand_sim_dart_model src/dart_model.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)

target_link_libraries(barrett_hand_sim_dart_test ${catkin_LIBRARIES} ${DART_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_grasp ${catkin_LIBRARIES} ${DART_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_model ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES})

catkin_package(
    DEPENDS eigen
)

