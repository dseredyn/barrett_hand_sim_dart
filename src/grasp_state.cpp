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
//#include <time.h>
//#include "Eigen/Dense"
#include <kdl/frames.hpp>

#include "planer_utils/utilities.h"
#include <tinyxml.h>

#include "grasp_state.h"

    boost::shared_ptr<GraspState > GraspState::readFromXml(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<GraspState > u_gs(new GraspState);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: GraspState::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<GraspState >(NULL);
	    }

    	TiXmlElement *elementGS = doc.FirstChildElement("GraspState");
	    if (!elementGS)
	    {
		    std::cout << "ERROR: GraspState::readFromXml: " << "Could not find the 'GraspState' element in the xml file" << std::endl;
		    return boost::shared_ptr<GraspState >(NULL);
	    }

    	// Get model parameters
    	const char *str = elementGS->Attribute("TEO");
    	if (!str)
    	{
		    std::cout << "ERROR: GraspState::readFromXml: TEO" << std::endl;
    		return boost::shared_ptr<GraspState >(NULL);
    	}
        u_gs->T_E_O_ = string2frameKdl(str);

        str = elementGS->Attribute("path_urdf");
    	if (!str)
    	{
		    std::cout << "ERROR: GraspState::readFromXml: path_urdf" << std::endl;
    		return boost::shared_ptr<GraspState >(NULL);
    	}
        u_gs->path_urdf_ = str;

        str = elementGS->Attribute("package_name");
    	if (!str)
    	{
		    std::cout << "ERROR: GraspState::readFromXml: package_name" << std::endl;
    		return boost::shared_ptr<GraspState >(NULL);
    	}
        u_gs->package_name_ = str;

	    for (TiXmlElement* elementJ = elementGS->FirstChildElement("Joint"); elementJ; elementJ = elementJ->NextSiblingElement("Joint")) {
        	str = elementJ->Attribute("name");
            if (!str) {
		        std::cout << "ERROR: GraspState::readFromXml: Joint name" << std::endl;
        		return boost::shared_ptr<GraspState >(NULL);
            }
            std::string joint_name( str );

        	str = elementJ->Attribute("value");
            if (!str) {
		        std::cout << "ERROR: GraspState::readFromXml: Joint value" << std::endl;
        		return boost::shared_ptr<GraspState >(NULL);
            }
            u_gs->q_map_[joint_name] = string2double( str );
	    }

        return boost::shared_ptr<GraspState >(new GraspState(*u_gs.get()));
    }

    void GraspState::writeToXml(const std::string &filename) const {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementGS = new TiXmlElement( "GraspState" );
        elementGS->SetAttribute("TEO", frameKdl2string(T_E_O_));
        elementGS->SetAttribute("path_urdf", path_urdf_);
        elementGS->SetAttribute("package_name", package_name_);
        for (std::map<std::string, double >::const_iterator it = q_map_.begin(); it != q_map_.end(); it++) {
    	    TiXmlElement * elementJ = new TiXmlElement( "Joint" );
            elementJ->SetAttribute("name", it->first);
            elementJ->SetAttribute("value", double2string(it->second));
    	    elementGS->LinkEndChild( elementJ );
        }
	    doc.LinkEndChild( elementGS );
	    doc.SaveFile( filename );
    }

