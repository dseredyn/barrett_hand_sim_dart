#!/bin/bash

#grasp_models/om.jar.xml
#grasp_models/om.jar2.xml
#grasp_models/om.cube.xml
#grasp_models/om.cylinder4.xml
#grasp_models/om.kettle15d.xml
#grasp_models/om.mug.xml
#grasp_models/om.container_1d.xml
#grasp_models/om.container_2d.xml

# pinch grasps
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar.xml grasp_models/pinch.xml grasp_models/qd.jar.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar2.xml grasp_models/pinch.xml grasp_models/qd.jar2.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cube.xml grasp_models/pinch.xml grasp_models/qd.cube.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cylinder4.xml grasp_models/pinch.xml grasp_models/qd.cylinder4.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.kettle15d.xml grasp_models/pinch.xml grasp_models/qd.kettle15d.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.mug.xml grasp_models/pinch.xml grasp_models/qd.mug.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_1d.xml grasp_models/pinch.xml grasp_models/qd.container_1d.pinch.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_2d.xml grasp_models/pinch.xml grasp_models/qd.container_2d.pinch.xml

# force grasps
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar.xml grasp_models/force.xml grasp_models/qd.jar.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar2.xml grasp_models/force.xml grasp_models/qd.jar2.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cube.xml grasp_models/force.xml grasp_models/qd.cube.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cylinder4.xml grasp_models/force.xml grasp_models/qd.cylinder4.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.kettle15d.xml grasp_models/force.xml grasp_models/qd.kettle15d.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.mug.xml grasp_models/force.xml grasp_models/qd.mug.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_1d.xml grasp_models/force.xml grasp_models/qd.container_1d.force.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_2d.xml grasp_models/force.xml grasp_models/qd.container_2d.force.xml

# cyl grasps
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar.xml grasp_models/cyl.xml grasp_models/qd.jar.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar2.xml grasp_models/cyl.xml grasp_models/qd.jar2.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cube.xml grasp_models/cyl.xml grasp_models/qd.cube.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cylinder4.xml grasp_models/cyl.xml grasp_models/qd.cylinder4.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.kettle15d.xml grasp_models/cyl.xml grasp_models/qd.kettle15d.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.mug.xml grasp_models/cyl.xml grasp_models/qd.mug.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_1d.xml grasp_models/cyl.xml grasp_models/qd.container_1d.cyl.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_2d.xml grasp_models/cyl.xml grasp_models/qd.container_2d.cyl.xml

# sph grasps
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar.xml grasp_models/sph.xml grasp_models/qd.jar.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.jar2.xml grasp_models/sph.xml grasp_models/qd.jar2.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cube.xml grasp_models/sph.xml grasp_models/qd.cube.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.cylinder4.xml grasp_models/sph.xml grasp_models/qd.cylinder4.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.kettle15d.xml grasp_models/sph.xml grasp_models/qd.kettle15d.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.mug.xml grasp_models/sph.xml grasp_models/qd.mug.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_1d.xml grasp_models/sph.xml grasp_models/qd.container_1d.sph.xml
rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density grasp_models/om.container_2d.xml grasp_models/sph.xml grasp_models/qd.container_2d.sph.xml

