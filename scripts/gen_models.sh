#!/bin/bash

barrett_hand_defs_dir=$(rospack find barrett_hand_defs)

rosrun xacro xacro -o ${barrett_hand_defs_dir}/robots/barrett_hand.urdf ${barrett_hand_defs_dir}/robots/barrett_hand.urdf.xml

rosrun xacro xacro -o src/barrett_hand_sim_dart/scenes/hook.urdf src/barrett_hand_sim_dart/scenes/hook.urdf.xacro
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_grasp barrett_hand_sim_dart /scenes/hook.urdf grasp_models/hook.xml

rosrun xacro xacro -o src/barrett_hand_sim_dart/scenes/pinch.urdf src/barrett_hand_sim_dart/scenes/pinch.urdf.xacro
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_grasp barrett_hand_sim_dart /scenes/pinch.urdf grasp_models/pinch.xml

#rosrun xacro xacro -o src/barrett_hand_sim_dart/scenes/cyl.urdf src/barrett_hand_sim_dart/scenes/cyl.urdf.xacro
#rosrun barrett_hand_sim_dart barrett_hand_sim_dart_grasp barrett_hand_sim_dart /scenes/cyl.urdf grasp_models/cyl.xml

rosrun xacro xacro -o src/barrett_hand_sim_dart/scenes/force.urdf src/barrett_hand_sim_dart/scenes/force.urdf.xacro
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_grasp barrett_hand_sim_dart /scenes/force.urdf grasp_models/force.xml

rosrun xacro xacro -o src/barrett_hand_sim_dart/scenes/sph.urdf src/barrett_hand_sim_dart/scenes/sph.urdf.xacro
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_grasp barrett_hand_sim_dart /scenes/sph.urdf grasp_models/sph.xml

