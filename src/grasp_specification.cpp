// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <kdl/frames.hpp>

#include "planer_utils/utilities.h"
#include <tinyxml.h>

#include "grasp_specification.h"

    double GraspSpecification::getInitPosition(const std::string &joint_name) const {
        std::map<std::string, double >::const_iterator it = q_init_map_.find(joint_name);
        if (it == q_init_map_.end()) {
            return 0.0;
        }
        return it->second;
    }

    double GraspSpecification::getGoalPosition(const std::string &joint_name) const {
        std::map<std::string, double >::const_iterator it = q_goal_map_.find(joint_name);
        if (it == q_goal_map_.end()) {
            return 0.0;
        }
        return it->second;
    }

    boost::shared_ptr<GraspSpecification > GraspSpecification::readFromUrdf(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<GraspSpecification > u_gs(new GraspSpecification);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: GraspSpecification::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<GraspSpecification >(NULL);
	    }

    	TiXmlElement *elementR = doc.FirstChildElement("robot");
	    if (!elementR)
	    {
		    std::cout << "ERROR: GraspSpecification::readFromXml: " << "Could not find the 'robot' element in the xml file" << std::endl;
		    return boost::shared_ptr<GraspSpecification >(NULL);
	    }

        int grasp_specs_count = 0;
	    for (TiXmlElement* elementGS = elementR->FirstChildElement("grasp_specification"); elementGS; elementGS = elementGS->NextSiblingElement("grasp_specification")) {
            grasp_specs_count++;
        }
        if (grasp_specs_count != 1) {
		    std::cout << "ERROR: GraspSpecification::readFromXml: wrong number of grasp_specification elements: " << grasp_specs_count << std::endl;
    		return boost::shared_ptr<GraspSpecification >(NULL);
        }

        TiXmlElement* elementGS = elementR->FirstChildElement("grasp_specification");

	    for (TiXmlElement* elementI = elementGS->FirstChildElement("init_state"); elementI; elementI = elementI->NextSiblingElement("init_state")) {
        	const char *str = elementI->Attribute("joint");
            if (!str) {
		        std::cout << "ERROR: GraspSpecification::readFromXml: init_state joint" << std::endl;
        		return boost::shared_ptr<GraspSpecification >(NULL);
            }
            std::string joint_name( str );

        	str = elementI->Attribute("position");
            if (!str) {
		        std::cout << "ERROR: GraspSpecification::readFromXml: init_state position" << std::endl;
        		return boost::shared_ptr<GraspSpecification >(NULL);
            }
            u_gs->q_init_map_[joint_name] = string2double( str );
	    }

	    for (TiXmlElement* elementG = elementGS->FirstChildElement("goal_state"); elementG; elementG = elementG->NextSiblingElement("goal_state")) {
        	const char *str = elementG->Attribute("joint");
            if (!str) {
		        std::cout << "ERROR: GraspSpecification::readFromXml: init_state joint" << std::endl;
        		return boost::shared_ptr<GraspSpecification >(NULL);
            }
            std::string joint_name( str );

        	str = elementG->Attribute("position");
            if (!str) {
		        std::cout << "ERROR: GraspSpecification::readFromXml: init_state position" << std::endl;
        		return boost::shared_ptr<GraspSpecification >(NULL);
            }
            u_gs->q_goal_map_[joint_name] = string2double( str );
	    }

        return boost::shared_ptr<GraspSpecification >(new GraspSpecification(*u_gs.get()));
    }

