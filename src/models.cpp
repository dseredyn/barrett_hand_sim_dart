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

#include "models.h"

static const double PI(3.141592653589793);

    CollisionModel::CollisionModel() :
        gen_(rd_())
    {
    }

    const std::string &CollisionModel::getRandomLinkNameCol() const {
        int idx = rand() % col_link_names.size();
        return col_link_names[idx];
    }

    const std::vector<std::string >& CollisionModel::getLinkNamesCol() const {
        return col_link_names;
    }

    void CollisionModel::buildFeatureMaps() {
        for (std::map<std::string, std::vector<Feature > >::const_iterator it = link_features_map.begin(); it != link_features_map.end(); it++) {
            const std::string &link_name = it->first;
            const std::vector<Feature > &fvec = it->second;
            link_features_map_edge.insert( std::make_pair(link_name, std::vector<Feature>()) );
            link_features_map_sym.insert( std::make_pair(link_name, std::vector<Feature>()) );
            for (std::vector<Feature >::const_iterator fit = fvec.begin(); fit != fvec.end(); fit++) {
                const Feature &feature = (*fit);
                if (feature.pc1 > 1.1 * feature.pc2) {
                    // edge
                    link_features_map_edge[it->first].push_back(feature);
                }
                else {
                    link_features_map_sym[it->first].push_back(feature);
                }
            }
        }
    }

    bool CollisionModel::getRandomFeature(std::string &link_name, Feature &feature) const {
        int features_count = 0;
        for (std::map<std::string, std::vector<Feature > >::const_iterator it = link_features_map.begin(); it != link_features_map.end(); it++) {
            features_count += it->second.size();
        }
        int feature_idx = rand() % features_count;
        for (std::map<std::string, std::vector<Feature > >::const_iterator it = link_features_map.begin(); it != link_features_map.end(); it++) {
            if (feature_idx < it->second.size()) {
                feature = it->second[feature_idx];
                link_name = it->first;
                return true;
            }
            else {
                feature_idx -= it->second.size();
            }
        }
        std::cout << "ERROR: getRandomFeature" << std::endl;
        return false;
    }

    void CollisionModel::getTransform(const std::string &link1_name, const std::string &link2_name, KDL::Frame &T_L1_L2) const {
        std::map<std::string, KDL::Frame >::const_iterator it1( frames_map_.find(link1_name) );
        std::map<std::string, KDL::Frame >::const_iterator it2( frames_map_.find(link2_name) );
        const KDL::Frame &T_W_L1 = it1->second;
        const KDL::Frame &T_W_L2 = it2->second;
        T_L1_L2 = T_W_L1.Inverse() * T_W_L2;
    }

    void CollisionModel::addLinkContacts(double dist_range, const std::string &link_name, const pcl::PointCloud<pcl::PointNormal>::Ptr &res,
                        const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &principalCurvatures, const KDL::Frame &T_W_S,
                        const pcl::PointCloud<pcl::PointNormal>::Ptr &ob_res, const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &ob_principalCurvatures,
                        const KDL::Frame &T_W_O, const boost::shared_ptr<std::vector<KDL::Frame > > &feature_frames) {

        const double lambda = 40.0;
        std::list<std::pair<int, double> > link_pt;

        for (int poidx = 0; poidx < ob_res->points.size(); poidx++) {
            double min_dist = dist_range + 1.0;
            for (int pidx = 0; pidx < res->points.size(); pidx++) {
                KDL::Vector p1(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z), p2(ob_res->points[poidx].x, ob_res->points[poidx].y, ob_res->points[poidx].z);
                double dist = (T_W_S * p1 - T_W_O * p2).Norm();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            if (min_dist < dist_range) {
                link_pt.push_back( std::make_pair(poidx, min_dist) );
            }
        }
        if ( link_pt.size() > 0 ) {
            link_features_map[link_name].resize( link_pt.size() );
            col_link_names.push_back(link_name);
            int fidx = 0;
            for (std::list<std::pair<int, double> >::const_iterator it = link_pt.begin(); it != link_pt.end(); it++, fidx++) {
                int poidx = it->first;
                link_features_map[link_name][fidx].pc1 = ob_principalCurvatures->points[poidx].pc1;
                link_features_map[link_name][fidx].pc2 = ob_principalCurvatures->points[poidx].pc2;
                KDL::Frame T_W_F = T_W_O * (*feature_frames)[poidx];
                link_features_map[link_name][fidx].T_L_F = T_W_S.Inverse() * T_W_F;
                double dist = it->second;
                link_features_map[link_name][fidx].dist = dist;
                link_features_map[link_name][fidx].weight = std::exp(-lambda * dist * dist);
            }
        }
    }

    void CollisionModel::addQueryDensity(const std::string &link_name, const std::vector<QueryDensityElement > &qd_vec) {
        qd_map_[link_name] = qd_vec;
    }

    void CollisionModel::setSamplerParameters(double sigma_p, double sigma_q, double sigma_r) {
        sigma_p_ = sigma_p;
        sigma_q_ = sigma_q;
        sigma_r_ = sigma_r;
        Cp_ = misesFisherKernelConstant(sigma_q_, 4);
    }

    double CollisionModel::getMarginalDensityForR(const std::string &link_name, const Eigen::Vector2d &r) {
        std::map<std::string, std::vector<Feature > >::const_iterator it = link_features_map.find( link_name );
        if (it == link_features_map.end()) {
            return 0.0;
        }
        const std::vector<Feature > &features = it->second;

        double sum = 0.0;
        for (int pidx = 0; pidx < features.size(); pidx++) {
            sum += features[pidx].weight * biVariateIsotropicGaussianKernel(r, Eigen::Vector2d(features[pidx].pc1, features[pidx].pc2), sigma_r_);
        }
        return sum;
    }

    bool CollisionModel::sampleForR(const std::string &link_name, const Eigen::Vector2d &r, Eigen::Vector3d &p, Eigen::Vector4d &q) {
        std::map<std::string, std::vector<Feature > >::const_iterator it = link_features_map.find( link_name );
        if (it == link_features_map.end()) {
            return false;
        }
        const std::vector<Feature > &features = it->second;

        const int n_points = features.size();
        std::vector<double > weights(n_points, 0.0);

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] = features[pidx].weight * biVariateIsotropicGaussianKernel(r, Eigen::Vector2d(features[pidx].pc1, features[pidx].pc2), sigma_r_);
            sum = weights[pidx];
        }

        double rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = features[pidx].T_L_F.p.x();
                break;
            }
        }

        std::normal_distribution<> d = std::normal_distribution<>(result_x, sigma_p_);
        result_x = d(gen_);

        // sample the y coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, features[pidx].T_L_F.p.x(), sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = features[pidx].T_L_F.p.y();
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);

        // sample the z coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, features[pidx].T_L_F.p.y(), sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = features[pidx].T_L_F.p.z();
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        // sample the orientation
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, features[pidx].T_L_F.p.z(), sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        Eigen::Vector4d mean_q;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                features[pidx].T_L_F.M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                
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

    bool CollisionModel::sampleQueryDensity(const std::string &link_name, Eigen::Vector3d &p, Eigen::Vector4d &q) {
        std::map<std::string, std::vector<QueryDensityElement > >::const_iterator it = qd_map_.find( link_name );
        if (it == qd_map_.end()) {
            return false;
        }
        const std::vector<QueryDensityElement > &qd = it->second;

        const int n_points = qd.size();
        std::vector<double > weights(n_points, 0.0);

        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] = qd[pidx].weight_;//1.0 / static_cast<double>(n_points);
            sum = weights[pidx];
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
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, qd[pidx].p_(0), sigma_p_);
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
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, qd[pidx].p_(1), sigma_p_);
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
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, qd[pidx].p_(2), sigma_p_);
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

    bool CollisionModel::sampleQueryDensity(const std::string &link_name, KDL::Frame &T_O_L) {
        Eigen::Vector3d p;
        Eigen::Vector4d q;
        bool result = sampleQueryDensity(link_name, p, q);
        T_O_L = KDL::Frame(KDL::Rotation::Quaternion(q(0), q(1), q(2), q(3)), KDL::Vector(p(0), p(1), p(2)));
        return result;
    }

    double CollisionModel::getQueryDensity(const std::string &link_name, const Eigen::Vector3d &p, const Eigen::Vector4d &q) const {
        std::map<std::string, std::vector<QueryDensityElement > >::const_iterator it = qd_map_.find( link_name );
        if (it == qd_map_.end()) {
            return 0.0;
        }
        const std::vector<QueryDensityElement > &qd = it->second;

        const int n_points = qd.size();

        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            sum += qd[pidx].weight_ *
                triVariateIsotropicGaussianKernel(p, qd[pidx].p_, sigma_p_) *
//                uniVariateIsotropicGaussianKernel(p(1), qd[pidx].p_(1), sigma_p_) *
//                uniVariateIsotropicGaussianKernel(p(2), qd[pidx].p_(1), sigma_p_) *
                misesFisherKernel(qd[pidx].q_, qd[pidx].q_, sigma_q_, Cp_);
        }

        return sum;
    }

    double CollisionModel::getQueryDensity(const std::string &link_name, const KDL::Frame &T_O_L) const {
        double qx, qy, qz, qw;
        T_O_L.M.GetQuaternion(qx, qy, qz, qw);
        return getQueryDensity(link_name, Eigen::Vector3d(T_O_L.p.x(), T_O_L.p.y(), T_O_L.p.z()), Eigen::Vector4d(qx, qy, qz, qw));
    }

/******************************************************************************************************************************/

    ObjectModel::ObjectModel() :
        gen_(rd_())
    {
    }

    void ObjectModel::randomizeSurface() {
        for (int idx = 0; idx < res->points.size(); idx++) {
            double pc1 = principalCurvatures->points[idx].pc1;
            double pc2 = principalCurvatures->points[idx].pc2;
            if (pc1 > 1.1 * pc2) {
                // e.g. pc1=1, pc2=0
                // edge
                if ((rand() % 2) == 0) {
                    (*features_)[idx] = (*features_)[idx] * KDL::Frame(KDL::Rotation::RotZ(PI));
                }
            }
            else {
                (*features_)[idx] = (*features_)[idx] * KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)));
            }
        }
    }

    int ObjectModel::getRandomIndex(double pc1, double pc2, double tolerance1, double tolerance2) const {
        std::vector<int > indices(principalCurvatures->points.size());
        int indices_count = 0;
        int idx = 0;
        for (pcl::PointCloud<pcl::PrincipalCurvatures>::const_iterator it = principalCurvatures->begin(); it != principalCurvatures->end(); it++, idx++) {
            if (std::fabs(it->pc1-pc1) < tolerance1 && std::fabs(it->pc2-pc2) < tolerance2) {
                indices[indices_count] = idx;
                indices_count++;
            }
        }
        if (indices_count == 0) {
            return -1;
        }
        return indices[(rand() % indices_count)];
    }

    bool ObjectModel::findFeature(double pc1, double pc2, double tolerance1, double tolerance2, const KDL::Frame &f, double radius, double tolerance4) const {
        pcl::PointNormal p;
        p.x = f.p.x();
        p.y = f.p.y();
        p.z = f.p.z();

        Eigen::Vector3f size = grid_->getLeafSize();
        Eigen::Vector3i min = grid_->getMinBoxCoordinates();
        Eigen::Vector3i max = grid_->getMaxBoxCoordinates();
        Eigen::Vector3i centre = grid_->getGridCoordinates(f.p.x(), f.p.y(), f.p.z());
        for (int i = 0; i < 3; i++) {
            int size_i = std::ceil(radius / size(i) );
            min(i) = std::max(centre(i)-size_i, min(i));
            max(i) = std::min(centre(i)+size_i, max(i));
        }

        for (int ix = min(0); ix <= max(0); ix++) {
            for (int iy = min(1); iy <= max(1); iy++) {
                for (int iz = min(2); iz <= max(2); iz++) {
                    int idx = grid_->getCentroidIndexAt(Eigen::Vector3i(ix, iy, iz));
                    if (std::fabs(principalCurvatures->points[idx].pc1-pc1) < tolerance1 && std::fabs(principalCurvatures->points[idx].pc2-pc2) < tolerance2) {
                        const KDL::Rotation &iM = (*features_)[idx].M;
                        double angle = 0.0;
                        if (pc1 > 1.1 * pc2) {
                            // e.g. pc1=1, pc2=0
                            // edge
                            KDL::Vector dM1 = KDL::diff(f.M, iM, 1.0);
                            KDL::Vector dM2 = KDL::diff(f.M * KDL::Rotation::RotZ(PI), iM, 1.0);
                            angle = std::min(dM1.Norm(), dM2.Norm());
                        }
                        else {
                            angle = getAngle(iM * KDL::Vector(0,0,1), f.M * KDL::Vector(0,0,1));
                        }
                        if (angle < tolerance4) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    void ObjectModel::setSamplerParameters(double sigma_p, double sigma_q, double sigma_r) {
        sigma_p_ = sigma_p;
        sigma_q_ = sigma_q;
        sigma_r_ = sigma_r;
        Cp_ = misesFisherKernelConstant(sigma_q_, 4);
    }

    void ObjectModel::sample(Eigen::Vector3d &p, Eigen::Vector4d &q, Eigen::Vector2d &r) {
        const int n_points = res->points.size();
        std::vector<double > weights(n_points, 0.0);

        // sample the x coordinate
        double sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] = 1.0 / static_cast<double >(n_points);
            sum += weights[pidx];
        }

        double rr = randomUniform(0.0, sum);
        double result_x, result_y, result_z;
        Eigen::Vector4d result_q;
        double result_pc1, result_pc2;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = res->points[pidx].x;
                break;
            }
        }

        std::normal_distribution<> d(result_x, sigma_p_);
        result_x = d(gen_);

        // sample the y coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_x, res->points[pidx].x, sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = res->points[pidx].y;
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p_);
        result_y = d(gen_);

        // sample the z coordinate
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_y, res->points[pidx].y, sigma_p_);
            //weights[pidx] = biVariateIsotropicGaussianKernel(Eigen::Vector2d(result_x, result_y), Eigen::Vector2d(res->points[pidx].x, res->points[pidx].y), sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = res->points[pidx].z;
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p_);
        result_z = d(gen_);

        // sample the orientation
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_z, res->points[pidx].z, sigma_p_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        Eigen::Vector4d mean_q;
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                (*features_)[pidx].M.GetQuaternion(mean_q(0), mean_q(1), mean_q(2), mean_q(3));                
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
            Eigen::Vector4d q;
            (*features_)[pidx].M.GetQuaternion(q(0), q(1), q(2), q(3));
            weights[pidx] *= misesFisherKernel(result_q, q, sigma_q_, Cp_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_pc1 = principalCurvatures->points[pidx].pc1;
                break;
            }
        }

        d = std::normal_distribution<>(result_pc1, sigma_r_);
        result_pc1 = d(gen_);

        // sample the pc2 feature
        sum = 0.0;
        for (int pidx = 0; pidx < n_points; pidx++) {
            weights[pidx] *= uniVariateIsotropicGaussianKernel(result_pc1, principalCurvatures->points[pidx].pc1, sigma_r_);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < n_points; pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_pc2 = principalCurvatures->points[pidx].pc2;
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
    }

/******************************************************************************************************************/

HandConfigurationModel::HandConfigurationModel() :
    gen_(rd_())
{
}

void HandConfigurationModel::generateModel(const std::map<std::string, double> &q_map_before, const std::map<std::string, double> &q_map_grasp, double beta, int n_samples, double sigma_c) {
    int idx = 0;
    qb_.resize( q_map_before.size() );
    qg_.resize( q_map_before.size() );
    joint_names_.resize( q_map_before.size() );
    for (std::map<std::string, double>::const_iterator it = q_map_before.begin(); it != q_map_before.end(); it++, idx++) {
        qb_(idx) = it->second;
        joint_names_[idx] = it->first;

        std::map<std::string, double>::const_iterator it_g = q_map_grasp.find(it->first);
        if (it_g == q_map_grasp.end()) {
            std::cout << "ERROR: HandConfigurationModel::generateModel: joint name " << it->first << " not in q_map_grasp" << std::endl;
        }
        qg_(idx) = it_g->second;
    }

    samples_.resize(n_samples);
    for (int i = 0; i < n_samples; i++) {
        double f = static_cast<double >(i) / static_cast<double >(n_samples-1);
        samples_[i] = (-beta) * (1.0 - f) + beta * f;
    }

    sigma_c_ = sigma_c;
}

const std::map<std::string, double>& HandConfigurationModel::sample() {
    const int n_samples = samples_.size();
    std::vector<double > weights(n_samples, 0.0);

    double sum = 0.0;
    for (int pidx = 0; pidx < n_samples; pidx++) {
        weights[pidx] = 1.0 / static_cast<double >(n_samples);
        sum += weights[pidx];
    }

    double rr = randomUniform(0.0, sum);
    double result_gamma;
    for (int pidx = 0; pidx < n_samples; pidx++) {
        rr -= weights[pidx];
        if (rr <= 0.0) {
            result_gamma = samples_[pidx];
            break;
        }
    }

    std::normal_distribution<> d(result_gamma, sigma_c_);
    result_gamma = d(gen_);

    for (int qidx = 0; qidx < joint_names_.size(); qidx++) {
        q_ret_[joint_names_[qidx]] = (1.0 - result_gamma) * qg_(qidx) + result_gamma * qb_(qidx);
    }
    return q_ret_;
}

