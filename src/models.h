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

#ifndef MODELS_H__
#define MODELS_H__

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <iostream>

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

class ObjectModel {
public:
    class Feature {
    public:
        Feature(const KDL::Frame &T_O_F, double pc1, double pc2);

        KDL::Frame T_O_F_;
        double pc1_, pc2_;
    };

protected:
    std::vector<Feature > fv_;
    double p_dist_max_, r_dist_max_;
    double sigma_p_, sigma_q_, sigma_r_, Cp_;

public:

    ObjectModel();

    void randomizeSurface();

    void setSamplerParameters(double sigma_p, double sigma_q, double sigma_r);

    void sample(int seed, Eigen::Vector3d &p, Eigen::Vector4d &q, Eigen::Vector2d &r) const;

    void addPointFeature(const KDL::Frame &T_O_F, double pc1, double pc2);

    const std::vector<Feature >& getPointFeatures() const;
};

class QueryDensity {
protected:
    double sigma_p_, sigma_q_;
    double p_dist_max_, Cp_;

public:
    class QueryDensityElement {
    public:
        Eigen::Vector3d p_;
        Eigen::Vector4d q_;
        double weight_;
        bool operator== (const QueryDensityElement &qd) const;
    };

    class LinkQueryDensity {
    public:
        std::vector<QueryDensityElement > vec_;
    };

    std::map<std::string, LinkQueryDensity > qd_map_;

    void setSamplerParameters(double sigma_p, double sigma_q);

    void addQueryDensity(const std::string &link_name, const std::vector<QueryDensityElement > &qd_vec);
    bool sampleQueryDensity(int seed, const std::string &link_name, Eigen::Vector3d &p, Eigen::Vector4d &q) const;
    bool sampleQueryDensity(int seed, const std::string &link_name, KDL::Frame &T_O_L) const;
    double getQueryDensity(const std::string &link_name, const Eigen::Vector3d &p, const Eigen::Vector4d &q) const;
    double getQueryDensity(const std::string &link_name, const KDL::Frame &T_O_L) const;

    static boost::shared_ptr<QueryDensity > readFromXml(const std::string &filename);
    void writeToXml(const std::string &filename) const;

    bool operator== (const QueryDensity &qd) const;
};

class CollisionModel {
public:
    class Feature {
    public:
        Feature();

        KDL::Frame T_C_F;
        double pc1, pc2;
        double dist;
        double weight;

        friend std::ostream& operator<< (std::ostream& stream, const Feature& f);
        friend std::istream& operator>> (std::istream& stream, Feature& f);
    };

protected:

    class LinkCollisionModel {
    public:
        std::vector<Feature > features_;
        KDL::Frame T_L_C_;
    };

    double sigma_p_, sigma_q_, sigma_r_;
    double p_dist_max_, r_dist_max_, Cp_;
    std::map<std::string, LinkCollisionModel > link_models_map_;
    std::vector<Feature > empty_f_vec_;
    std::vector<std::string > col_link_names_;

public:

    static boost::shared_ptr<CollisionModel > readFromXml(const std::string &filename);
    void writeToXml(const std::string &filename) const;

    CollisionModel();

    const std::string& getRandomLinkNameCol() const;
    const std::vector<std::string >& getLinkNamesCol() const;
    const std::vector<Feature >& getLinkFeatures(const std::string &link_name) const;

    bool getT_L_C(const std::string &link_name, KDL::Frame &T_L_C) const;

    void addLinkContacts(double dist_range, const std::string &link_name, const pcl::PointCloud<pcl::PointNormal>::Ptr &res,
                        const KDL::Frame &T_W_L, const std::vector<ObjectModel::Feature > &ob_features,
                        const KDL::Frame &T_W_O);

    void setSamplerParameters(double sigma_p, double sigma_q, double sigma_r);

    double getMarginalDensityForR(const std::string &link_name, const Eigen::Vector2d &r) const;
    bool sampleForR(int seed, const std::string &link_name, const Eigen::Vector2d &r, Eigen::Vector3d &p, Eigen::Vector4d &q) const;

    void saveToFile(const std::string &filename) const;
};

class HandConfigurationModel {
protected:
    std::vector<std::string > joint_names_;
    std::vector<Eigen::VectorXd > samples_;
    double sigma_c_;
public:
    HandConfigurationModel();
    void generateModel(const std::map<std::string, double> &q_map_before, const std::map<std::string, double> &q_map_grasp, double beta, int n_samples, double sigma_c);
    void sample(int seed, std::map<std::string, double> &q_ret_) const;
    double getDensity(const std::map<std::string, double>& q_map) const;

    static boost::shared_ptr<HandConfigurationModel > readFromXml(const std::string &filename);
    void writeToXml(const std::string &filename) const;
};

#endif  // MODELS_H__

