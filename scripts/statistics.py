#!/usr/bin/env python

# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import math
import numpy as np
from operator import itemgetter

if __name__ == '__main__':

    success_map = {}

    for runid in range(1, 11):
        path = "run_" + str(runid).zfill(2) + "/"
        for testid in range(0, 32):
            filename = "test_result_" + str(testid).zfill(3) + ".txt"
            with open(path + filename, 'r') as f:
                line = f.readline()
                params = line.split()
                scene = params[0]
                grasp = params[1]
                sol_gen = int(params[3])
                sol_red = int(params[5])
                val_gr = int(params[7])
                succ_list = []
                succ_count = 0
                idx = 9
                while idx < len(params):
                    if params[idx] == "1":
                        succ_list.append(True)
                        succ_count += 1
                    elif params[idx] == "0":
                        succ_list.append(False)
                    else:
                        print "ERROR"
                    idx += 4
                print str(succ_count) + " / " + str(val_gr)

                if not (scene, grasp) in success_map:
                    success_map[(scene, grasp)] = (0, 0, 0)

                succ_tuple = success_map[(scene, grasp)]
                success_map[(scene, grasp)] = (succ_tuple[0]+sol_red, succ_tuple[1]+val_gr, succ_tuple[2]+succ_count)

    obj_seq = [
    '/scenes/test_container_1.urdf',
    '/scenes/test_container_2.urdf',
    '/scenes/test_cuboid.urdf',
    '/scenes/test_cylinder4_lying.urdf',
    '/scenes/test_cylinder4_standing.urdf',
    '/scenes/test_jar.urdf',
    '/scenes/test_kettle.urdf',
    '/scenes/test_mug.urdf',
    ]

    grasp_seq = [
    'grasp_models/pinch.xml',
    'grasp_models/sph.xml',
    'grasp_models/force.xml',
    'grasp_models/hook.xml',
    ]

    print "scene / grasp sequence:"
    for obj_name in obj_seq:
        for gr_name in grasp_seq:
            print (obj_name, gr_name)

    print "reduced_solutions / valid_grasps / successfull_grasps sequence:"
    for obj_name in obj_seq:
        for gr_name in grasp_seq:
            print success_map[(obj_name, gr_name)]

    print " % valid_grasps sequence:"
    for obj_name in obj_seq:
        line = obj_name
        for gr_name in grasp_seq:
            line += " & " + "{0:.1f}".format(float(success_map[(obj_name, gr_name)][1]) / success_map[(obj_name, gr_name)][0] * 100.0)
        line += " \\\\ \\hline"
        print line

    print " % successfull_grasps sequence:"
    for obj_name in obj_seq:
        line = obj_name
        for gr_name in grasp_seq:
            if success_map[(obj_name, gr_name)][1] == 0:
                line += " & " + "0 / 0"
            else:
                line += " & " + "{0:.1f}".format(float(success_map[(obj_name, gr_name)][2]) / success_map[(obj_name, gr_name)][1] * 100.0)
        line += " \\\\ \\hline"
        print line

    exit(0)

    succ_vec = []
    for key in success_map:
        print key, success_map[key]
        succ_vec.append( (key, success_map[key]) )

    print "sorted:"
    succ_vec = sorted(succ_vec, key=itemgetter(0,0))


    for elem in succ_vec:
        print elem

# /scenes/test_container_1.urdf	grasp_models/sph.xml	solutions:	381	reduced_sol:	38	valid_grasps:	6	success:	1	score:	4.04321e-05	success:	1	score:	3.034e-05	success:	1	score:	1.69282e-05	success:	1	score:	9.36973e-06	success:	1	score:	6.00807e-06	success:	1	score:	5.9228e-06	

