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
#include <time.h>

#include <iostream>
#include <fstream>

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/marker_publisher.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/features/principal_curvatures.h>

#include <tinyxml.h>

#include "models.h"

static const double PI(3.141592653589793);

    CollisionModel::CollisionModel()
    {
    }

    const std::string &CollisionModel::getRandomLinkNameCol() const {
        int idx = rand() % col_link_names_.size();
        return col_link_names_[idx];
    }

    const std::vector<std::string >& CollisionModel::getLinkNamesCol() const {
        return col_link_names_;
    }

    const std::vector<CollisionModel::Feature >& CollisionModel::getLinkFeatures(const std::string &link_name) const {
        std::map<std::string, LinkCollisionModel >::const_iterator it = link_models_map_.find(link_name);
        if (it == link_models_map_.end()) {
            std::cout << "ERROR: CollisionModel::getLinkFeatures( " << link_name << " ): given link has no collision data" << std::endl;
            return empty_f_vec_;
        }
        return it->second.features_;
    }

    bool CollisionModel::getT_L_C(const std::string &link_name, KDL::Frame &T_L_C) const {
        std::map<std::string, LinkCollisionModel >::const_iterator it = link_models_map_.find(link_name);
        if (it == link_models_map_.end()) {
            return false;
        }
        T_L_C = it->second.T_L_C_;
        return true;
    }

    void CollisionModel::addLinkContacts(double dist_range, const std::string &link_name, const pcl::PointCloud<pcl::PointNormal>::Ptr &res,
                        const KDL::Frame &T_W_L, const std::vector<ObjectModel::Feature > &ob_features,
                        const KDL::Frame &T_W_O) {
        const double lambda = - std::log(0.01) / (dist_range * dist_range);
        std::list<std::pair<int, double> > link_pt;

        for (int poidx = 0; poidx < ob_features.size(); poidx++) {
            double min_dist = dist_range + 1.0;
            for (int pidx = 0; pidx < res->points.size(); pidx++) {
                KDL::Vector p1(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z);
                const KDL::Vector &p2( ob_features[poidx].T_O_F_.p );
                double dist = (T_W_L * p1 - T_W_O * p2).Norm();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            if (min_dist < dist_range) {
                link_pt.push_back( std::make_pair(poidx, min_dist) );
            }
        }
        if ( link_pt.size() > 0 ) {
            link_models_map_[link_name].features_.resize( link_pt.size() );
            std::vector<KDL::Frame > T_L_F_vec( link_pt.size() );
            col_link_names_.push_back(link_name);
            int fidx = 0;
            KDL::Vector col_pt;
            double sum_weight = 0.0;
            for (std::list<std::pair<int, double> >::const_iterator it = link_pt.begin(); it != link_pt.end(); it++, fidx++) {
                int poidx = it->first;
                link_models_map_[link_name].features_[fidx].pc1 = ob_features[poidx].pc1_;
                link_models_map_[link_name].features_[fidx].pc2 = ob_features[poidx].pc2_;
                KDL::Frame T_W_F = T_W_O * ob_features[poidx].T_O_F_;
                T_L_F_vec[fidx] = T_W_L.Inverse() * T_W_F;
                double dist = it->second;
                link_models_map_[link_name].features_[fidx].dist = dist;
                double weight = std::exp(-lambda * dist * dist);
                link_models_map_[link_name].features_[fidx].weight = weight;
                col_pt = col_pt + weight * T_L_F_vec[fidx].p;
                sum_weight += weight;
            }
            link_models_map_[link_name].T_L_C_ = KDL::Frame(col_pt / sum_weight);
            for (int fidx = 0; fidx < link_models_map_[link_name].features_.size(); fidx++) {
                link_models_map_[link_name].features_[fidx].T_C_F = link_models_map_[link_name].T_L_C_.Inverse() * T_L_F_vec[fidx];
            }
        }
    }

    void CollisionModel::setSamplerParameters(double sigma_p, double sigma_q, double sigma_r) {
        sigma_p_ = sigma_p;
        sigma_q_ = sigma_q;
        sigma_r_ = sigma_r;
        Cp_ = misesFisherKernelConstant(sigma_q_, 4);

        p_dist_max_ = 10000000.0;
        double f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_p_);
        for (double x = 0; x < 100.0; x += 0.0001) {
            double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_p_);
            if (f < f_max * 0.1) {
                p_dist_max_ = x;
                std::cout << "f_max " << f_max << "   f_min " << f << "   p_dist_max_ " << p_dist_max_ << std::endl;
                break;
            }
        }

        r_dist_max_ = 10000000.0;
        f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_r_);
        for (double x = 0; x < 100.0; x += 0.0001) {
            double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_r_);
            if (f < f_max * 0.1) {
                r_dist_max_ = x;
                std::cout << "f_max " << f_max << "   f_min " << f << "   r_dist_max_ " << r_dist_max_ << std::endl;
                break;
            }
        }
    }

    double CollisionModel::getMarginalDensityForR(const std::string &link_name, const Eigen::Vector2d &r) const {
        std::map<std::string, LinkCollisionModel >::const_iterator it = link_models_map_.find( link_name );
        if (it == link_models_map_.end()) {
            return 0.0;
        }
        const std::vector<Feature > &features = it->second.features_;

        double sum = 0.0;
        for (int pidx = 0; pidx < features.size(); pidx++) {
            if (std::fabs(r(0) - features[pidx].pc1) > r_dist_max_ || std::fabs(r(1) - features[pidx].pc2) > r_dist_max_ || features[pidx].weight < 0.0000001) {
                continue;
            }
            sum += features[pidx].weight * biVariateIsotropicGaussianKernel(r, Eigen::Vector2d(features[pidx].pc1, features[pidx].pc2), sigma_r_);
        }
        return sum;
    }
/*
    bool CollisionModel::sampleForR(int seed, const std::string &link_name, const Eigen::Vector2d &r, Eigen::Vector3d &p, Eigen::Vector4d &q) const {
        std::map<std::string, LinkCollisionModel >::const_iterator it = link_models_map_.find( link_name );
        if (it == link_models_map_.end()) {
            return false;
        }
        const std::vector<Feature > &features = it->second.features_;
        std::mt19937 gen_(seed);

        const int n_points = features.size();
        std::vector<double > weights(n_points, 0.0);

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(r(0) - features[pidx].pc1) > r_dist_max_ || std::fabs(r(1) - features[pidx].pc2) > r_dist_max_ || features[pidx].weight < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] = features[pidx].weight * biVariateIsotropicGaussianKernel(r, Eigen::Vector2d(features[pidx].pc1, features[pidx].pc2), sigma_r_);
            }
            sum += weights[pidx];
        }

        double rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = features[pidx].T_C_F.p.x();
                break;
            }
        }

        std::normal_distribution<> d = std::normal_distribution<>(result_x, sigma_p_);
        result_x = d(gen_);

        // sample the y coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_x - features[pidx].T_C_F.p.x()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, features[pidx].T_C_F.p.x(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = features[pidx].T_C_F.p.y();
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);

        // sample the z coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_y - features[pidx].T_C_F.p.y()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, features[pidx].T_C_F.p.y(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = features[pidx].T_C_F.p.z();
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        // sample the orientation
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_z - features[pidx].T_C_F.p.z()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, features[pidx].T_C_F.p.z(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        Eigen::Vector4d mean_q;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                features[pidx].T_C_F.M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                
                break;
            }
        }

        double pdf_mean = misesFisherKernel(mean_q, mean_q, sigma_q_, Cp_);

        // TODO
        result_q = mean_q;
        int iterations = 1;
//        int iterations = vonMisesFisherSample(mean_q, pdf_mean, sigma_q_, Cp_, result_q);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        p(0) = result_x;
        p(1) = result_y;
        p(2) = result_z;
        q = result_q;

        return true;
    }
*/

    bool CollisionModel::sampleForR(int seed, const std::string &link_name, const Eigen::Vector2d &r, Eigen::Vector3d &p, Eigen::Vector4d &q) const {
        std::map<std::string, LinkCollisionModel >::const_iterator it = link_models_map_.find( link_name );
        if (it == link_models_map_.end()) {
            return false;
        }
        const std::vector<Feature > &features = it->second.features_;
        std::mt19937 gen_(seed);

        const int n_points = features.size();
        std::vector<double > weights(n_points, 0.0);

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(r(0) - features[pidx].pc1) > r_dist_max_ || std::fabs(r(1) - features[pidx].pc2) > r_dist_max_ || features[pidx].weight < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] = features[pidx].weight * biVariateIsotropicGaussianKernel(r, Eigen::Vector2d(features[pidx].pc1, features[pidx].pc2), sigma_r_);
            }
            sum += weights[pidx];
        }

        Feature random_kernel;
        double rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                random_kernel = features[pidx];
                break;
            }
        }

        Eigen::Vector4d mean_q;
        result_x = random_kernel.T_C_F.p.x();
        result_y = random_kernel.T_C_F.p.y();
        result_z = random_kernel.T_C_F.p.z();
        random_kernel.T_C_F.M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                

        std::normal_distribution<> d = std::normal_distribution<>(result_x, sigma_p_);
        result_x = d(gen_);
        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);
        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        double pdf_mean = misesFisherKernel(mean_q, mean_q, sigma_q_, Cp_);
        // TODO
        result_q = mean_q;
        int iterations = 1;
//        int iterations = vonMisesFisherSample(mean_q, pdf_mean, sigma_q_, Cp_, result_q);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        p(0) = result_x;
        p(1) = result_y;
        p(2) = result_z;
        q = result_q;

        return true;
    }

    CollisionModel::Feature::Feature() {
        pc1 = 0.0;
        pc2 = 0.0;
        dist = 0.0;
        weight = 0.0;
    }

    std::ostream& operator<< (std::ostream& stream, const CollisionModel::Feature& f) {
        stream << f.T_C_F << " " <<  f.pc1 << " " <<  f.pc2 << " " << f.dist << " " << f.weight;
        return stream;
    }

    std::istream& operator>> (std::istream& stream, CollisionModel::Feature& f) {
        stream >> f.T_C_F >> f.pc1 >> f.pc2 >> f.dist >> f.weight;
        return stream;
    }

    boost::shared_ptr<CollisionModel > CollisionModel::readFromXml(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<CollisionModel > u_cm(new CollisionModel);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: CollisionModel::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<CollisionModel >(NULL);
	    }

    	TiXmlElement *elementCM = doc.FirstChildElement("CollisionModel");
	    if (!elementCM) {
            TiXmlElement *elementCHM = doc.FirstChildElement("CollisionAndHandModel");
            elementCM = elementCHM->FirstChildElement("CollisionModel");
        }

        if (!elementCM) {
		    std::cout << "ERROR: CollisionModel::readFromXml: " << "Could not find the 'CollisionModel' element in the xml file" << std::endl;
		    return boost::shared_ptr<CollisionModel >(NULL);
	    }

    	// Get model parameters
        double sigma_p, sigma_q, sigma_r;
    	const char *str = elementCM->Attribute("sigma_p");
    	if (!str)
    	{
		    std::cout << "ERROR: CollisionModel::readFromXml: sigma_p" << std::endl;
    		return boost::shared_ptr<CollisionModel >(NULL);
    	}
        sigma_p = string2double(str);

    	str = elementCM->Attribute("sigma_q");
    	if (!str)
    	{
		    std::cout << "ERROR: CollisionModel::readFromXml: sigma_q" << std::endl;
    		return boost::shared_ptr<CollisionModel >(NULL);
    	}
        sigma_q = string2double(str);

    	str = elementCM->Attribute("sigma_r");
    	if (!str)
    	{
		    std::cout << "ERROR: CollisionModel::readFromXml: sigma_r" << std::endl;
    		return boost::shared_ptr<CollisionModel >(NULL);
    	}
        sigma_r = string2double(str);

        u_cm->setSamplerParameters(sigma_p, sigma_q, sigma_r);

	    for (TiXmlElement* elementLCM = elementCM->FirstChildElement("LinkCollisionModel"); elementLCM; elementLCM = elementLCM->NextSiblingElement("LinkCollisionModel")) {
        	str = elementLCM->Attribute("name");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkCollisionModel name" << std::endl;
        		return boost::shared_ptr<CollisionModel >(NULL);
            }
            std::string link_name( str );

        	str = elementLCM->Attribute("TLC");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkCollisionModel TLC" << std::endl;
        		return boost::shared_ptr<CollisionModel >(NULL);
            }
            u_cm->link_models_map_[link_name].T_L_C_ = string2frameKdl(str);

            int features_count = 0;
    	    for (TiXmlElement* elementF = elementLCM->FirstChildElement("Feature"); elementF; elementF = elementF->NextSiblingElement("Feature"), features_count++) {
            }
            u_cm->link_models_map_[link_name].features_.resize(features_count);

            features_count = 0;
    	    for (TiXmlElement* elementF = elementLCM->FirstChildElement("Feature"); elementF; elementF = elementF->NextSiblingElement("Feature"), features_count++) {
            	str = elementF->Attribute("TCF");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: Feature TCF" << std::endl;
            		return boost::shared_ptr<CollisionModel >(NULL);
                }
                u_cm->link_models_map_[link_name].features_[features_count].T_C_F = string2frameKdl(str);

            	str = elementF->Attribute("pc1");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: Feature pc1" << std::endl;
            		return boost::shared_ptr<CollisionModel >(NULL);
                }
                u_cm->link_models_map_[link_name].features_[features_count].pc1 = string2double(str);

            	str = elementF->Attribute("pc2");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: Feature pc2" << std::endl;
            		return boost::shared_ptr<CollisionModel >(NULL);
                }
                u_cm->link_models_map_[link_name].features_[features_count].pc2 = string2double(str);

            	str = elementF->Attribute("dist");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: Feature dist" << std::endl;
            		return boost::shared_ptr<CollisionModel >(NULL);
                }
                u_cm->link_models_map_[link_name].features_[features_count].dist = string2double(str);

            	str = elementF->Attribute("weight");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: Feature weight" << std::endl;
            		return boost::shared_ptr<CollisionModel >(NULL);
                }
                u_cm->link_models_map_[link_name].features_[features_count].weight = string2double(str);
            }
	    }

        u_cm->col_link_names_.resize(u_cm->link_models_map_.size());
        int idx = 0;
        for (std::map<std::string, CollisionModel::LinkCollisionModel >::const_iterator it = u_cm->link_models_map_.begin(); it != u_cm->link_models_map_.end(); it++, idx++) {
            u_cm->col_link_names_[idx] = it->first;
        }

        return boost::shared_ptr<CollisionModel >(new CollisionModel(*u_cm.get()));
    }

    void CollisionModel::writeToXml(const std::string &filename) const {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementCM = makeXmlNode();
	    doc.LinkEndChild( elementCM );
	    doc.SaveFile( filename );
    }

    TiXmlElement* CollisionModel::makeXmlNode() const {
	    TiXmlElement * elementCM = new TiXmlElement( "CollisionModel" );
        elementCM->SetAttribute("sigma_p", double2string(sigma_p_));
        elementCM->SetAttribute("sigma_q", double2string(sigma_q_));
        elementCM->SetAttribute("sigma_r", double2string(sigma_r_));

        for (std::map<std::string, CollisionModel::LinkCollisionModel >::const_iterator it1 = link_models_map_.begin(); it1 != link_models_map_.end(); it1++) {
    	    TiXmlElement * elementLCM = new TiXmlElement( "LinkCollisionModel" );
            elementLCM->SetAttribute("name", it1->first);
            elementLCM->SetAttribute("TLC", frameKdl2string(it1->second.T_L_C_));
            for (std::vector<CollisionModel::Feature >::const_iterator it2 = it1->second.features_.begin(); it2 != it1->second.features_.end(); it2++) {
        	    TiXmlElement * elementF = new TiXmlElement( "Feature" );
                elementF->SetAttribute("TCF", frameKdl2string(it2->T_C_F));
                elementF->SetAttribute("pc1", double2string(it2->pc1));
                elementF->SetAttribute("pc2", double2string(it2->pc2));
                elementF->SetAttribute("dist", double2string(it2->dist));
                elementF->SetAttribute("weight", double2string(it2->weight));
        	    elementLCM->LinkEndChild( elementF );
            }
    	    elementCM->LinkEndChild( elementLCM );
        }
        return elementCM;
    }

/****************************************************************************************************************************************/

    void QueryDensity::setSamplerParameters(double sigma_p, double sigma_q) {
        sigma_p_ = sigma_p;
        sigma_q_ = sigma_q;
        Cp_ = misesFisherKernelConstant(sigma_q_, 4);

        p_dist_max_ = 10000000.0;
        double f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_p_);
        for (double x = 0; x < 100.0; x += 0.0001) {
            double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_p_);
            if (f < f_max * 0.1) {
                p_dist_max_ = x;
                std::cout << "f_max " << f_max << "   f_min " << f << "   p_dist_max_ " << p_dist_max_ << std::endl;
                break;
            }
        }
    }

    void QueryDensity::addQueryDensity(const std::string &link_name, const std::vector<QueryDensityElement > &qd_vec) {
        qd_map_[link_name].vec_ = qd_vec;
    }

    bool QueryDensity::sampleQueryDensity(int seed, const std::string &link_name, Eigen::Vector3d &p, Eigen::Vector4d &q) const {
        std::map<std::string, LinkQueryDensity >::const_iterator it = qd_map_.find( link_name );
        if (it == qd_map_.end()) {
            return false;
        }
        const std::vector<QueryDensityElement > &qd = it->second.vec_;
        std::mt19937 gen_(seed);

        const int n_points = qd.size();
        std::vector<double > weights(n_points, 0.0);

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] = qd[pidx].weight_;
            sum += weights[pidx];
        }

        double rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = qd[pidx].p_(0);
                break;
            }
        }

        std::normal_distribution<> d = std::normal_distribution<>(result_x, sigma_p_);
        result_x = d(gen_);

        // sample the y coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_x - qd[pidx].p_(0)) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, qd[pidx].p_(0), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = qd[pidx].p_(1);
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);

        // sample the z coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_y - qd[pidx].p_(1)) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, qd[pidx].p_(1), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = qd[pidx].p_(2);
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        // sample the orientation
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_z - qd[pidx].p_(2)) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, qd[pidx].p_(2), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        Eigen::Vector4d mean_q;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                mean_q = qd[pidx].q_;
                break;
            }
        }

        double pdf_mean = misesFisherKernel(mean_q, mean_q, sigma_q_, Cp_);
        int iterations = vonMisesFisherSample(mean_q, pdf_mean, sigma_q_, Cp_, result_q);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        p(0) = result_x;
        p(1) = result_y;
        p(2) = result_z;
        q = result_q;

        return true;
    }

    bool QueryDensity::sampleQueryDensity(int seed, const std::string &link_name, KDL::Frame &T_O_L) const {
        Eigen::Vector3d p;
        Eigen::Vector4d q;
        bool result = sampleQueryDensity(seed, link_name, p, q);
        T_O_L = KDL::Frame(KDL::Rotation::Quaternion(q(0), q(1), q(2), q(3)), KDL::Vector(p(0), p(1), p(2)));
        return result;
    }

    double QueryDensity::getQueryDensity(const std::string &link_name, const Eigen::Vector3d &p, const Eigen::Vector4d &q) const {
        std::map<std::string, LinkQueryDensity >::const_iterator it = qd_map_.find( link_name );
        if (it == qd_map_.end()) {
            return 0.0;
        }
        const std::vector<QueryDensityElement > &qd = it->second.vec_;

        const int n_points = qd.size();

        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            double Q = qd[pidx].weight_;
            if (Q < 0.0000001) {
                continue;
            }
            else {
                Q *= triVariateIsotropicGaussianKernel(p, qd[pidx].p_, sigma_p_);
                if (Q < 0.0000001) {
                    continue;
                }
                else {
                    Q *= misesFisherKernel(q, qd[pidx].q_, sigma_q_, Cp_);
                    sum += Q;
                }
            }
        }

        return sum;
    }

    double QueryDensity::getQueryDensity(const std::string &link_name, const KDL::Frame &T_O_L) const {
        double qx, qy, qz, qw;
        T_O_L.M.GetQuaternion(qx, qy, qz, qw);
        return getQueryDensity(link_name, Eigen::Vector3d(T_O_L.p.x(), T_O_L.p.y(), T_O_L.p.z()), Eigen::Vector4d(qx, qy, qz, qw));
    }

    boost::shared_ptr<QueryDensity > QueryDensity::readFromXml(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<QueryDensity > u_qd(new QueryDensity);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: QueryDensity::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<QueryDensity >(NULL);
	    }

    	TiXmlElement *elementQD = doc.FirstChildElement("QueryDensity");
	    if (!elementQD)
	    {
		    std::cout << "ERROR: QueryDensity::readFromXml: " << "Could not find the 'QueryDensity' element in the xml file" << std::endl;
		    return boost::shared_ptr<QueryDensity >(NULL);
	    }

    	// Get model parameters
        double sigma_p, sigma_q, sigma_r;
    	const char *str = elementQD->Attribute("sigma_p");
    	if (!str)
    	{
		    std::cout << "ERROR: QueryDensity::readFromXml: sigma_p" << std::endl;
    		return boost::shared_ptr<QueryDensity >(NULL);
    	}
        sigma_p = string2double(str);

    	str = elementQD->Attribute("sigma_q");
    	if (!str)
    	{
		    std::cout << "ERROR: QueryDensity::readFromXml: sigma_q" << std::endl;
    		return boost::shared_ptr<QueryDensity >(NULL);
    	}
        sigma_q = string2double(str);

        u_qd->setSamplerParameters(sigma_p, sigma_q);

	    for (TiXmlElement* elementLQD = elementQD->FirstChildElement("LinkQueryDensity"); elementLQD; elementLQD = elementLQD->NextSiblingElement("LinkQueryDensity")) {
        	str = elementLQD->Attribute("name");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkQueryDensity name" << std::endl;
        		return boost::shared_ptr<QueryDensity >(NULL);
            }
            std::string link_name( str );

            int features_count = 0;
    	    for (TiXmlElement* elementQDE = elementLQD->FirstChildElement("QueryDensityElement"); elementQDE; elementQDE = elementQDE->NextSiblingElement("QueryDensityElement"), features_count++) {
            }
            u_qd->qd_map_[link_name].vec_.resize(features_count);

            features_count = 0;
    	    for (TiXmlElement* elementQDE = elementLQD->FirstChildElement("QueryDensityElement"); elementQDE; elementQDE = elementQDE->NextSiblingElement("QueryDensityElement"), features_count++) {
            	str = elementQDE->Attribute("p");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: QueryDensityElement p" << std::endl;
            		return boost::shared_ptr<QueryDensity >(NULL);
                }
                std::istringstream strs_p(str);
                strs_p >> u_qd->qd_map_[link_name].vec_[features_count].p_(0) >> u_qd->qd_map_[link_name].vec_[features_count].p_(1) >>
                    u_qd->qd_map_[link_name].vec_[features_count].p_(2);

            	str = elementQDE->Attribute("q");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: QueryDensityElement q" << std::endl;
            		return boost::shared_ptr<QueryDensity >(NULL);
                }
                std::istringstream strs_q(str);
                strs_q >> u_qd->qd_map_[link_name].vec_[features_count].q_(0) >> u_qd->qd_map_[link_name].vec_[features_count].q_(1) >>
                    u_qd->qd_map_[link_name].vec_[features_count].q_(2) >> u_qd->qd_map_[link_name].vec_[features_count].q_(3);

            	str = elementQDE->Attribute("weight");
                if (!str) {
		            std::cout << "ERROR: CollisionModel::readFromXml: QueryDensityElement weight" << std::endl;
            		return boost::shared_ptr<QueryDensity >(NULL);
                }
                u_qd->qd_map_[link_name].vec_[features_count].weight_ = string2double(str);
            }
	    }

        return boost::shared_ptr<QueryDensity >(new QueryDensity(*u_qd.get()));
    }

    void QueryDensity::writeToXml(const std::string &filename) const {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementQD = makeXmlNode();
	    doc.LinkEndChild( elementQD );
	    doc.SaveFile( filename );
    }

    TiXmlElement* QueryDensity::makeXmlNode() const {
	    TiXmlElement * elementQD = new TiXmlElement( "QueryDensity" );
        elementQD->SetAttribute("sigma_p", double2string(sigma_p_));
        elementQD->SetAttribute("sigma_q", double2string(sigma_q_));
        for (std::map<std::string, QueryDensity::LinkQueryDensity >::const_iterator it1 = qd_map_.begin(); it1 != qd_map_.end(); it1++) {
    	    TiXmlElement * elementLQD = new TiXmlElement( "LinkQueryDensity" );
            elementLQD->SetAttribute("name", it1->first);
            for (std::vector<QueryDensity::QueryDensityElement >::const_iterator it2 = it1->second.vec_.begin(); it2 != it1->second.vec_.end(); it2++) {
        	    TiXmlElement * elementQDE = new TiXmlElement( "QueryDensityElement" );
                elementQDE->SetAttribute("p", double2string(it2->p_(0)) + " " + double2string(it2->p_(1)) + " " + double2string(it2->p_(2)));
                elementQDE->SetAttribute("q", double2string(it2->q_(0)) + " " + double2string(it2->q_(1)) + " " + double2string(it2->q_(2)) + " " + double2string(it2->q_(3)));
                elementQDE->SetAttribute("weight", double2string(it2->weight_));
        	    elementLQD->LinkEndChild( elementQDE );
            }
    	    elementQD->LinkEndChild( elementLQD );
        }
        return elementQD;
    }


    bool QueryDensity::QueryDensityElement::operator== (const QueryDensity::QueryDensityElement &qd) const {
        return (p_ - qd.p_).norm() < 0.00001 && (q_ - qd.q_).norm() < 0.00001 && std::fabs(weight_ - qd.weight_) < 0.00001;
    }

    bool QueryDensity::operator== (const QueryDensity &qd) const {
        if (sigma_p_ != qd.sigma_p_ || sigma_q_ != qd.sigma_q_ || p_dist_max_ != qd.p_dist_max_ || Cp_ != qd.Cp_) {
            return false;
        }

        if (qd_map_.size() != qd.qd_map_.size()) {
            return false;
        }

        for (std::map<std::string, QueryDensity::LinkQueryDensity >::const_iterator it1 = qd_map_.begin(); it1 != qd_map_.end(); it1++) {
            std::map<std::string, QueryDensity::LinkQueryDensity >::const_iterator it2 = qd.qd_map_.find(it1->first);
            if (it2 == qd.qd_map_.end()) {
                return false;
            }
            if (it1->second.vec_.size() != it2->second.vec_.size()) {
                return false;
            }
            for (int idx = 0; idx < it1->second.vec_.size(); idx++) {
                if (!(it1->second.vec_[idx] == it2->second.vec_[idx])) {
                    return false;
                }
            }
        }

        return true;
    }

/******************************************************************************************************************************/

    ObjectModel::ObjectModel()
    {
    }

    void ObjectModel::randomizeSurface() {
        for (int idx = 0; idx < fv_.size(); idx++) {
            double pc1 = fv_[idx].pc1_;
            double pc2 = fv_[idx].pc2_;
            if (pc1 > 1.1 * pc2) {
                // e.g. pc1=1, pc2=0
                // edge
                if ((rand() % 2) == 0) {
                    fv_[idx].T_O_F_ = fv_[idx].T_O_F_ * KDL::Frame(KDL::Rotation::RotZ(PI));
                }
            }
            else {
                fv_[idx].T_O_F_ = fv_[idx].T_O_F_ * KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)));
            }
        }
    }

    void ObjectModel::setSamplerParameters(double sigma_p, double sigma_q, double sigma_r) {
        sigma_p_ = sigma_p;
        sigma_q_ = sigma_q;
        sigma_r_ = sigma_r;
        Cp_ = misesFisherKernelConstant(sigma_q_, 4);

        p_dist_max_ = 10000000.0;
        double f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_p_);
        for (double x = 0; x < 100.0; x += 0.0001) {
            double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_p_);
            if (f < f_max * 0.1) {
                p_dist_max_ = x;
                std::cout << "f_max " << f_max << "   f_min " << f << "   p_dist_max_ " << p_dist_max_ << std::endl;
                break;
            }
        }

        r_dist_max_ = 10000000.0;
        f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_r_);
        for (double x = 0; x < 100.0; x += 0.0001) {
            double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_r_);
            if (f < f_max * 0.1) {
                r_dist_max_ = x;
                std::cout << "f_max " << f_max << "   f_min " << f << "   r_dist_max_ " << r_dist_max_ << std::endl;
                break;
            }
        }
    }
/*
    int ObjectModel::sample(int seed, Eigen::Vector3d &p, Eigen::Vector4d &q, Eigen::Vector2d &r) const {
        const int n_points = fv_.size();
        std::vector<double > weights(n_points, 1.0 / static_cast<double >(n_points));

        std::mt19937 gen_(seed);

        double sum = 1.0;

        // sample the x coordinate
        double rr = randomUniform(0.0, sum);
        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;
        double result_pc1, result_pc2;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = fv_[pidx].T_O_F_.p.x();
                break;
            }
        }

        std::normal_distribution<> d(result_x, sigma_p_);
        result_x = d(gen_);

        // sample the y coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_x - fv_[pidx].T_O_F_.p.x()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, fv_[pidx].T_O_F_.p.x(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = fv_[pidx].T_O_F_.p.y();
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);

        // sample the z coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_y - fv_[pidx].T_O_F_.p.y()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, fv_[pidx].T_O_F_.p.y(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = fv_[pidx].T_O_F_.p.z();
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        // sample the orientation
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_z - fv_[pidx].T_O_F_.p.z()) > p_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, fv_[pidx].T_O_F_.p.z(), sigma_p_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        Eigen::Vector4d mean_q;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                fv_[pidx].T_O_F_.M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                
                break;
            }
        }

        double pdf_mean = misesFisherKernel(mean_q, mean_q, sigma_q_, Cp_);
        int iterations = vonMisesFisherSample(mean_q, pdf_mean, sigma_q_, Cp_, result_q);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        // sample the pc1 feature
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                Eigen::Vector4d q;
                fv_[pidx].T_O_F_.M.GetQuaternion(q(0), q(1), q(2), q(3));
                weights[pidx] *= misesFisherKernel(result_q, q, sigma_q_, Cp_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_pc1 = fv_[pidx].pc1_;
                break;
            }
        }

        d = std::normal_distribution<>(result_pc1, sigma_r_);
        result_pc1 = d(gen_);

        // sample the pc2 feature
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            if (std::fabs(result_pc1 - fv_[pidx].pc1_) > r_dist_max_ || weights[pidx] < 0.0000001) {
                weights[pidx] = 0.0;
            }
            else {
                weights[pidx] *= uniVariateIsotropicGaussianKernel(result_pc1, fv_[pidx].pc1_, sigma_r_);
            }
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_pc2 = fv_[pidx].pc2_;
                break;
            }
        }

        d = std::normal_distribution<>(result_pc2, sigma_r_);
        result_pc2 = d(gen_);

        p(0) = result_x;
        p(1) = result_y;
        p(2) = result_z;
        q = result_q;
        r(0) = result_pc1;
        r(1) = result_pc2;

        return iterations;
    }
*/

    int ObjectModel::sample(int seed, Eigen::Vector3d &p, Eigen::Vector4d &q, Eigen::Vector2d &r) const {
        const int n_points = fv_.size();
        std::mt19937 gen_(seed);

        Feature rand_kernel = fv_[rand() % n_points];

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q, mean_q;
        double result_pc1, result_pc2;

        result_x = rand_kernel.T_O_F_.p.x();
        result_y = rand_kernel.T_O_F_.p.y();
        result_z = rand_kernel.T_O_F_.p.z();
        rand_kernel.T_O_F_.M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                
        result_pc1 = rand_kernel.pc1_;
        result_pc2 = rand_kernel.pc2_;

        std::normal_distribution<> d(result_x, sigma_p_);
        result_x = d(gen_);
        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);
        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);
        double pdf_mean = misesFisherKernel(mean_q, mean_q, sigma_q_, Cp_);
        // TODO
        result_q = mean_q;
        int iterations = 1;
//        int iterations = vonMisesFisherSample(mean_q, pdf_mean, sigma_q_, Cp_, result_q);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }
        d = std::normal_distribution<>(result_pc1, sigma_r_);
        result_pc1 = d(gen_);
        d = std::normal_distribution<>(result_pc2, sigma_r_);
        result_pc2 = d(gen_);

        p(0) = result_x;
        p(1) = result_y;
        p(2) = result_z;
        q = result_q;
        r(0) = result_pc1;
        r(1) = result_pc2;

        return iterations;
    }

ObjectModel::Feature::Feature() {
}

ObjectModel::Feature::Feature(const KDL::Frame &T_O_F, double pc1, double pc2) {
    T_O_F_ = T_O_F;
    pc1_ = pc1;
    pc2_ = pc2;
}

void ObjectModel::addPointFeature(const KDL::Frame &T_O_F, double pc1, double pc2) {
    Feature f(T_O_F, pc1, pc2);
    fv_.push_back(f);
}

const std::vector<ObjectModel::Feature >& ObjectModel::getPointFeatures() const {
    return fv_;
}

int ObjectModel::getFeaturesCount() const {
    return fv_.size();
}

boost::shared_ptr<ObjectModel > ObjectModel::readFromXml(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<ObjectModel > u_om(new ObjectModel);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: ObjectModel::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<ObjectModel >(NULL);
	    }

    	TiXmlElement *elementOM = doc.FirstChildElement("ObjectModel");
	    if (!elementOM)
	    {
		    std::cout << "ERROR: ObjectModel::readFromXml: " << "Could not find the 'ObjectModel' element in the xml file" << std::endl;
		    return boost::shared_ptr<ObjectModel >(NULL);
	    }

    	// Get model parameters
        double sigma_p, sigma_q, sigma_r;
    	const char *str = elementOM->Attribute("sigma_p");
    	if (!str)
    	{
		    std::cout << "ERROR: ObjectModel::readFromXml: sigma_p" << std::endl;
    		return boost::shared_ptr<ObjectModel >(NULL);
    	}
        sigma_p = string2double(str);

    	str = elementOM->Attribute("sigma_q");
    	if (!str)
    	{
		    std::cout << "ERROR: ObjectModel::readFromXml: sigma_q" << std::endl;
    		return boost::shared_ptr<ObjectModel >(NULL);
    	}
        sigma_q = string2double(str);

    	str = elementOM->Attribute("sigma_r");
    	if (!str)
    	{
		    std::cout << "ERROR: ObjectModel::readFromXml: sigma_r" << std::endl;
    		return boost::shared_ptr<ObjectModel >(NULL);
    	}
        sigma_r = string2double(str);

        u_om->setSamplerParameters(sigma_p, sigma_q, sigma_r);

    	str = elementOM->Attribute("path_urdf");
    	if (!str)
    	{
		    std::cout << "ERROR: ObjectModel::readFromXml: path_urdf" << std::endl;
    		return boost::shared_ptr<ObjectModel >(NULL);
    	}
        u_om->path_urdf_ = str;

    	str = elementOM->Attribute("package_name");
    	if (!str)
    	{
		    std::cout << "ERROR: ObjectModel::readFromXml: package_name" << std::endl;
    		return boost::shared_ptr<ObjectModel >(NULL);
    	}
        u_om->package_name_ = str;

        int features_count = 0;
        for (TiXmlElement* elementF = elementOM->FirstChildElement("Feature"); elementF; elementF = elementF->NextSiblingElement("Feature"), features_count++) {
        }
        u_om->fv_.resize(features_count);

        features_count = 0;
        for (TiXmlElement* elementF = elementOM->FirstChildElement("Feature"); elementF; elementF = elementF->NextSiblingElement("Feature"), features_count++) {
            str = elementF->Attribute("TOF");
            if (!str) {
                std::cout << "ERROR: ObjectModel::readFromXml: Feature TOF" << std::endl;
                return boost::shared_ptr<ObjectModel >(NULL);
            }
            u_om->fv_[features_count].T_O_F_ = string2frameKdl(str);

            str = elementF->Attribute("pc1");
            if (!str) {
                std::cout << "ERROR: ObjectModel::readFromXml: Feature pc1" << std::endl;
                return boost::shared_ptr<ObjectModel >(NULL);
            }
            u_om->fv_[features_count].pc1_ = string2double(str);

            str = elementF->Attribute("pc2");
            if (!str) {
                std::cout << "ERROR: ObjectModel::readFromXml: Feature pc2" << std::endl;
                return boost::shared_ptr<ObjectModel >(NULL);
            }
            u_om->fv_[features_count].pc2_ = string2double(str);
	    }

        return boost::shared_ptr<ObjectModel >(new ObjectModel(*u_om.get()));
}

const std::vector<ObjectModel::Feature >& ObjectModel::getFeaturesData() const {
    return fv_;
}

void ObjectModel::writeToXml(const std::string &filename) const {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementOM = makeXmlNode();
	    doc.LinkEndChild( elementOM );
	    doc.SaveFile( filename );
}

TiXmlElement* ObjectModel::makeXmlNode() const {
	    TiXmlElement * elementOM = new TiXmlElement( "ObjectModel" );
        elementOM->SetAttribute("sigma_p", double2string(sigma_p_));
        elementOM->SetAttribute("sigma_q", double2string(sigma_q_));
        elementOM->SetAttribute("sigma_r", double2string(sigma_r_));
        elementOM->SetAttribute("path_urdf", path_urdf_);
        elementOM->SetAttribute("package_name", package_name_);
        for (std::vector<Feature >::const_iterator it = fv_.begin(); it != fv_.end(); it++) {
    	    TiXmlElement * elementF = new TiXmlElement( "Feature" );
            elementF->SetAttribute("TOF", frameKdl2string(it->T_O_F_));
            elementF->SetAttribute("pc1", double2string(it->pc1_));
            elementF->SetAttribute("pc2", double2string(it->pc2_));
    	    elementOM->LinkEndChild( elementF );
        }
        return elementOM;
}

/******************************************************************************************************************/

HandConfigurationModel::HandConfigurationModel() {
}

void HandConfigurationModel::generateModel(const std::map<std::string, double> &q_map_before, const std::map<std::string, double> &q_map_grasp, double beta, int n_samples, double sigma_c) {
    int idx = 0;
    Eigen::VectorXd qb(q_map_before.size()), qg(q_map_before.size());

//    qb_.resize( q_map_before.size() );
//    qg_.resize( q_map_before.size() );
    joint_names_.resize( q_map_before.size() );
    for (std::map<std::string, double>::const_iterator it = q_map_before.begin(); it != q_map_before.end(); it++, idx++) {
        qb(idx) = it->second;
        joint_names_[idx] = it->first;

        std::map<std::string, double>::const_iterator it_g = q_map_grasp.find(it->first);
        if (it_g == q_map_grasp.end()) {
            std::cout << "ERROR: HandConfigurationModel::generateModel: joint name " << it->first << " not in q_map_grasp" << std::endl;
        }
        qg(idx) = it_g->second;
    }

    samples_.resize(n_samples, Eigen::VectorXd(q_map_before.size()));
    for (int i = 0; i < n_samples; i++) {
        double f = static_cast<double >(i) / static_cast<double >(n_samples-1);
        double gamma = (-beta) * (1.0 - f) + beta * f;
        samples_[i] = (1.0 - gamma) * qg + gamma * qb;
    }

    sigma_c_ = sigma_c;
}

void HandConfigurationModel::sample(int seed, std::map<std::string, double> &q_ret) const {
    const int n_samples = samples_.size();
    std::vector<double > weights(n_samples, 1.0 / static_cast<double >(n_samples));
    std::mt19937 gen_(seed);

    Eigen::VectorXd result(joint_names_.size());
    for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
        double sum = 0.0;
        for (int pidx = 0; pidx < n_samples; pidx++) {
            sum += weights[pidx];
        }

        double rr = randomUniform(0.0, sum);
        double result_i;
        for (int pidx = 0; pidx < n_samples; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_i = samples_[pidx][qidx];
                break;
            }
        }

        std::normal_distribution<> d(result_i, sigma_c_);
        result_i = d(gen_);

        result(qidx) = result_i;

        for (int pidx = 0; pidx < n_samples; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_i, samples_[pidx][qidx], sigma_c_);
        }
    }


    for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
        q_ret[joint_names_[qidx]] = result(qidx);
    }
}

double HandConfigurationModel::getDensity(const std::map<std::string, double>& q_map) const {
    const int n_samples = samples_.size();
    std::vector<double > weights(n_samples, 1.0);

    Eigen::VectorXd q(joint_names_.size());
    for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
        std::map<std::string, double>::const_iterator it = q_map.find(joint_names_[qidx]);
        if (it == q_map.end()) {
            return 0.0;
        }
        q(qidx) = it->second;
    }

    for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
        for (int pidx = 0; pidx < n_samples; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(q(qidx), samples_[pidx][qidx], sigma_c_);
        }
    }

    double sum = 0.0;
    for (int pidx = 0; pidx < n_samples; pidx++) {
        sum += weights[pidx];
    }

    return sum;
}

    boost::shared_ptr<HandConfigurationModel > HandConfigurationModel::readFromXml(const std::string &filename) {
        TiXmlDocument doc( filename );
        doc.LoadFile();
        std::unique_ptr<HandConfigurationModel > u_hm(new HandConfigurationModel);

	    if (doc.Error())
	    {
		    std::cout << "ERROR: HandConfigurationModel::readFromXml: " << doc.ErrorDesc() << std::endl;
		    doc.ClearError();
		    return boost::shared_ptr<HandConfigurationModel >(NULL);
	    }

    	TiXmlElement *elementHCM = doc.FirstChildElement("HandConfigurationModel");
	    if (!elementHCM) {
            TiXmlElement *elementCHM = doc.FirstChildElement("CollisionAndHandModel");
            elementHCM = elementCHM->FirstChildElement("HandConfigurationModel");
        }

	    if (!elementHCM)
	    {
		    std::cout << "ERROR: HandConfigurationModel::readFromXml: " << "Could not find the 'HandConfigurationModel' element in the xml file" << std::endl;
		    return boost::shared_ptr<HandConfigurationModel >(NULL);
	    }

    	// Get model parameters
    	const char *str = elementHCM->Attribute("sigma_c");
    	if (!str)
    	{
		    std::cout << "ERROR: HandConfigurationModel::readFromXml: sigma_c" << std::endl;
    		return boost::shared_ptr<HandConfigurationModel >(NULL);
    	}
        u_hm->sigma_c_ = string2double(str);

        int joints_count = 0;
	    for (TiXmlElement* elementJ = elementHCM->FirstChildElement("Joint"); elementJ; elementJ = elementJ->NextSiblingElement("Joint"), joints_count++) {
        }
        u_hm->joint_names_.resize(joints_count);

	    for (TiXmlElement* elementJ = elementHCM->FirstChildElement("Joint"); elementJ; elementJ = elementJ->NextSiblingElement("Joint")) {
        	str = elementJ->Attribute("name");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkQueryDensity name" << std::endl;
        		return boost::shared_ptr<HandConfigurationModel >(NULL);
            }
            std::string joint_name( str );

        	str = elementJ->Attribute("idx");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkQueryDensity name" << std::endl;
        		return boost::shared_ptr<HandConfigurationModel >(NULL);
            }
            int qidx = string2int( str );
            u_hm->joint_names_[qidx] = joint_name;
	    }

        int samples_count = 0;
	    for (TiXmlElement* elementS = elementHCM->FirstChildElement("Sample"); elementS; elementS = elementS->NextSiblingElement("Sample"), samples_count++) {
        }
        u_hm->samples_.resize(samples_count);

        samples_count = 0;
	    for (TiXmlElement* elementS = elementHCM->FirstChildElement("Sample"); elementS; elementS = elementS->NextSiblingElement("Sample"), samples_count++) {
        	str = elementS->Attribute("value");
            if (!str) {
		        std::cout << "ERROR: CollisionModel::readFromXml: LinkQueryDensity name" << std::endl;
        		return boost::shared_ptr<HandConfigurationModel >(NULL);
            }
            u_hm->samples_[samples_count].resize(joints_count);
            std::istringstream strs(str);
            for (int qidx = 0; qidx < joints_count; qidx++) {
                double v;
                strs >> v;
                u_hm->samples_[samples_count][qidx] = v;
            }
        }

        return boost::shared_ptr<HandConfigurationModel >(new HandConfigurationModel(*u_hm.get()));
    }

    void HandConfigurationModel::writeToXml(const std::string &filename) const {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementHCM = makeXmlNode();
	    doc.LinkEndChild( elementHCM );
	    doc.SaveFile( filename );
    }

    TiXmlElement* HandConfigurationModel::makeXmlNode() const {
	    TiXmlElement * elementHCM = new TiXmlElement( "HandConfigurationModel" );
        elementHCM->SetAttribute("sigma_c", double2string(sigma_c_));
        for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
    	    TiXmlElement * elementJ = new TiXmlElement( "Joint" );
            elementJ->SetAttribute("idx", qidx);
            elementJ->SetAttribute("name", joint_names_[qidx]);
    	    elementHCM->LinkEndChild( elementJ );
        }

        for (std::vector<Eigen::VectorXd >::const_iterator it = samples_.begin(); it != samples_.end(); it++) {
    	    TiXmlElement * elementS = new TiXmlElement( "Sample" );
            std::ostringstream strs;
            for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
                strs << (*it)(qidx) << " ";
            }
            elementS->SetAttribute("value", strs.str());
    	    elementHCM->LinkEndChild( elementS );
        }
        return elementHCM;
    }

void writeToXml(const std::string &filename, const boost::shared_ptr<CollisionModel > &cm, const boost::shared_ptr<HandConfigurationModel > &hcm) {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementCM = cm->makeXmlNode();
	    doc.LinkEndChild( elementCM );
	    TiXmlElement * elementHCM = hcm->makeXmlNode();
	    doc.LinkEndChild( elementHCM );
	    doc.SaveFile( filename );
}

void writeToXml(const std::string &filename, const boost::shared_ptr<QueryDensity > &qd, const boost::shared_ptr<CollisionModel > &cm,
                const boost::shared_ptr<HandConfigurationModel > &hcm) {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementQD = qd->makeXmlNode();
	    doc.LinkEndChild( elementQD );
	    TiXmlElement * elementCM = cm->makeXmlNode();
	    doc.LinkEndChild( elementCM );
	    TiXmlElement * elementHCM = hcm->makeXmlNode();
	    doc.LinkEndChild( elementHCM );
	    doc.SaveFile( filename );
}

void writeToXml(const std::string &filename, const boost::shared_ptr<QueryDensity > &qd, const boost::shared_ptr<CollisionModel > &cm,
                const boost::shared_ptr<HandConfigurationModel > &hcm, const boost::shared_ptr<ObjectModel > &om) {
        TiXmlDocument doc;
	    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	    doc.LinkEndChild( decl );
	    TiXmlElement * elementQD = qd->makeXmlNode();
	    doc.LinkEndChild( elementQD );
	    TiXmlElement * elementCM = cm->makeXmlNode();
	    doc.LinkEndChild( elementCM );
	    TiXmlElement * elementHCM = hcm->makeXmlNode();
	    doc.LinkEndChild( elementHCM );
	    TiXmlElement * elementOM = om->makeXmlNode();
	    doc.LinkEndChild( elementOM );
	    doc.SaveFile( filename );
}

