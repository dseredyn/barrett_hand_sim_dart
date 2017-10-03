cmake_minimum_required(VERSION 2.8)
project(barrett_hand_sim_dart)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs tf eigen_conversions cmake_modules planer_utils kin_dyn_model collision_convex_model
  )

find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(Eigen3 REQUIRED)
find_package(DART REQUIRED)
find_package(DARTCore REQUIRED)

# Assimp
find_package(assimp QUIET)
if(assimp_FOUND)
  message(STATUS "Looking for Assimp - found")
else()
  message(SEND_ERROR "Looking for Assimp - NOT found, please install libassimp-dev")
endif()

link_directories(#${orocos_kdl_LIBRARY_DIRS}
    ${DART_LIBRARY_DIRS}
    ${DARTCore_LIBRARY_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${DARTCore_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(barrett_hand_sim_dart_test src/dart_test.cpp)
add_executable(barrett_hand_sim_dart_grasp src/dart_grasp.cpp src/gripper_controller.cpp src/grasp_specification.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)
add_executable(barrett_hand_sim_dart_experiments src/dart_experiments.cpp src/gripper_controller.cpp src/grasp_specification.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)
add_executable(barrett_hand_sim_dart_generate_object_model src/generate_object_model.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)
add_executable(barrett_hand_sim_dart_generate_query_density src/generate_query_density.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)
add_executable(barrett_hand_sim_dart_select_grasp src/select_grasp.cpp src/mesh_sampling.cpp src/models.cpp src/visual_debug.cpp)

target_link_libraries(barrett_hand_sim_dart_test ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_grasp ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_experiments ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_generate_object_model ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_generate_query_density ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(barrett_hand_sim_dart_select_grasp ${catkin_LIBRARIES} ${DART_LIBRARIES} ${DARTCore_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})

#target_link_libraries(barrett_hand_sim_dart_test ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})
#target_link_libraries(barrett_hand_sim_dart_grasp ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})
#target_link_libraries(barrett_hand_sim_dart_experiments ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})
#target_link_libraries(barrett_hand_sim_dart_generate_object_model ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})
#target_link_libraries(barrett_hand_sim_dart_generate_query_density ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})
#target_link_libraries(barrett_hand_sim_dart_select_grasp ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES})


