#!/bin/bash

rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/jar.urdf grasp_models/om.jar.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/jar2.urdf grasp_models/om.jar2.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/cube.urdf grasp_models/om.cube.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/cylinder4.urdf grasp_models/om.cylinder4.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/kettle15d.urdf grasp_models/om.kettle15d.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/mug.urdf grasp_models/om.mug.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/container_1d.urdf grasp_models/om.container_1d.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model barrett_hand_sim_dart /objects/container_2d.urdf grasp_models/om.container_2d.xml

