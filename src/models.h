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

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/marker_publisher.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/features/principal_curvatures.h>

class Feature {
public:
    KDL::Frame T_L_F;
    double pc1, pc2;
    double dist;
    double weight;
};

class CollisionModel {
public:
    class QueryDensityElement {
    public:
        Eigen::Vector3d p_;
        Eigen::Vector4d q_;
        double weight_;
    };
    std::map<std::string, std::vector<Feature > > link_features_map;
    std::map<std::string, std::vector<Feature > > link_features_map_edge;
    std::map<std::string, std::vector<Feature > > link_features_map_sym;

    std::map<std::string, double> joint_q_map_;
    std::map<std::string, KDL::Frame > frames_map_;
    std::map<std::pair<std::string, std::string>, std::list<KDL::Frame> > frames_rel_map_;
    std::vector<std::string > col_link_names;
    std::map<std::string, std::vector<QueryDensityElement > > qd_map_;

    std::random_device rd_;
    std::mt19937 gen_;

    double sigma_p_, sigma_q_, sigma_r_, Cp_;

    CollisionModel();

    const std::string& getRandomLinkNameCol() const;

    const std::vector<std::string >& getLinkNamesCol() const;

    void buildFeatureMaps();

    void getTransform(const std::string &link1_name, const std::string &link2_name, KDL::Frame &T_L1_L2) const;

    void addLinkContacts(double dist_range, const std::string &link_name, const pcl::PointCloud<pcl::PointNormal>::Ptr &res,
                        const KDL::Frame &T_W_S,
                        const pcl::PointCloud<pcl::PointNormal>::Ptr &ob_res, const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &ob_principalCurvatures,
                        const KDL::Frame &T_W_O, const boost::shared_ptr<std::vector<KDL::Frame > > &feature_frames);

    void addQueryDensity(const std::string &link_name, const std::vector<QueryDensityElement > &qd_vec);

    void setSamplerParameters(double sigma_p, double sigma_q, double sigma_r);

    double getMarginalDensityForR(const std::string &link_name, const Eigen::Vector2d &r);
    bool sampleForR(const std::string &link_name, const Eigen::Vector2d &r, Eigen::Vector3d &p, Eigen::Vector4d &q);

    bool sampleQueryDensity(const std::string &link_name, Eigen::Vector3d &p, Eigen::Vector4d &q);
    bool sampleQueryDensity(const std::string &link_name, KDL::Frame &T_O_L);
    double getQueryDensity(const std::string &link_name, const Eigen::Vector3d &p, const Eigen::Vector4d &q) const;
    double getQueryDensity(const std::string &link_name, const KDL::Frame &T_O_L) const;
};

class ObjectModel {
public:
    pcl::PointCloud<pcl::PointNormal>::Ptr res;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures;
    boost::shared_ptr<pcl::VoxelGrid<pcl::PointNormal> > grid_;
    boost::shared_ptr<std::vector<KDL::Frame > > features_;
    double sigma_p_, sigma_q_, sigma_r_, Cp_;
    std::random_device rd_;
    std::mt19937 gen_;

    ObjectModel();

    void randomizeSurface();

    void setSamplerParameters(double sigma_p, double sigma_q, double sigma_r);

    void sample(Eigen::Vector3d &p, Eigen::Vector4d &q, Eigen::Vector2d &r);
};

class HandConfigurationModel {
protected:
    Eigen::VectorXd qb_, qg_;
    std::vector<std::string > joint_names_;
    std::map<std::string, double> q_ret_;
    std::vector<double > samples_;
    std::random_device rd_;
    std::mt19937 gen_;
    double sigma_c_;
public:
    HandConfigurationModel();
    void generateModel(const std::map<std::string, double> &q_map_before, const std::map<std::string, double> &q_map_grasp, double beta, int n_samples, double sigma_c);
    const std::map<std::string, double>& sample();
};

