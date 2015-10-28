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

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <fstream>

#include <thread>

#include <boost/filesystem.hpp>

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/marker_publisher.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"

#include <dart/dart.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/features/principal_curvatures.h>

#include "mesh_sampling.h"
#include "models.h"
#include "visual_debug.h"

const double PI(3.141592653589793);

void generateQueryDensity(int seed, const std::string &link_name, const boost::shared_ptr<CollisionModel > &cm, const ObjectModel &om,
                            std::vector<QueryDensity::QueryDensityElement > &qd_vec) {
        double sum_weights = 0.0;
        int ori_iterations = 0;
        std::mt19937 gen(seed);

        for (int i = 0; i < qd_vec.size(); i++) {
            Eigen::Vector3d p, p2;
            Eigen::Vector4d q, q2;
            Eigen::Vector2d r;
            ori_iterations += om.sample(gen(), p, q, r);

            if (!cm->sampleForR(gen(), link_name, r, p2, q2)) {
                std::cout << "ERROR: cm->sampleForR" << std::endl;
            }
            double weight = cm->getMarginalDensityForR(link_name, r);
            KDL::Frame T_O_F( KDL::Frame(KDL::Rotation::Quaternion(q(0), q(1), q(2), q(3)), KDL::Vector(p(0), p(1), p(2))) );
            KDL::Frame T_C_F( KDL::Frame(KDL::Rotation::Quaternion(q2(0), q2(1), q2(2), q2(3)), KDL::Vector(p2(0), p2(1), p2(2))) );
            KDL::Frame T_L_C;
            if (!cm->getT_L_C(link_name, T_L_C)) {
                std::cout << "ERROR: generateQueryDensity: " << link_name << std::endl;
            }
            KDL::Frame T_O_C = T_O_F * T_C_F.Inverse();

            qd_vec[i].p_ = Eigen::Vector3d(T_O_C.p.x(), T_O_C.p.y(), T_O_C.p.z());
            if (qd_vec[i].p_(2) != qd_vec[i].p_(2)) {
                std::cout << "ERROR: generateQueryDensity: qd_vec[i]: " << qd_vec[i].p_.transpose() << std::endl;
                std::cout << "T_O_C " << T_O_C << std::endl;
                std::cout << "T_O_F " << T_O_F << std::endl;
                std::cout << "T_C_F " << T_C_F << std::endl;
                return;
            }
            double qx, qy, qz, qw;
            T_O_C.M.GetQuaternion(qx, qy, qz, qw);
            qd_vec[i].q_ = Eigen::Vector4d(qx, qy, qz, qw);
            qd_vec[i].weight_ = weight;
            sum_weights += qd_vec[i].weight_;
        }
        // normalize the weights
        for (int i = 0; i < qd_vec.size(); i++) {
            qd_vec[i].weight_ /= sum_weights;
        }

        std::cout << "generateQueryDensity: " << link_name << "  total_it: " << qd_vec.size() << "   ori_it: " << ori_iterations << "   om.features: " << om.getFeaturesCount() << std::endl;
}

int main(int argc, char** argv) {
/*
    Eigen::Vector4d mean, x;
    KDL::Rotation rot;
    rot.GetQuaternion(mean(0), mean(1), mean(2), mean(3));
    double pdf_mean;
    double sigma = 10.0/180.0*PI;
    double Cp = misesFisherKernelConstant(sigma, 4);
    pdf_mean = misesFisherKernel(mean, mean, sigma, Cp);
    misesFisherKernel(x, mean, sigma, Cp);

    std::cout << "pdf_mean: " << pdf_mean << std::endl;
    for (double angle = 0.0; angle < 180.0/180 * PI; angle += 5.0/180.0*PI) {
        KDL::Rotation::RotX(angle).GetQuaternion(x(0), x(1), x(2), x(3));
        std::cout << (angle / PI * 180.0) << "   " << misesFisherKernel(x, mean, sigma, Cp) << std::endl;
    }
    return 0;
*/
    if (argc != 4) {
        std::cout << "usage:" << std::endl;
        std::cout << argv[0] << " object_model collision_model output_file" << std::endl;
        return 0;
    }
    srand ( time(NULL) );

    const double sigma_p = 0.01;//05;
    const double sigma_q = 15.0/180.0*PI;//100.0;

    int m_id = 101;

    // generate object model
    boost::shared_ptr<ObjectModel > om = ObjectModel::readFromXml(argv[1]);

    std::cout << "om.getPointFeatures().size(): " << om->getPointFeatures().size() << std::endl;

    // generate collision model
    std::map<std::string, std::list<std::pair<int, double> > > link_pt_map;
    boost::shared_ptr<CollisionModel > cm = CollisionModel::readFromXml(argv[2]);

    boost::shared_ptr<HandConfigurationModel > hm = HandConfigurationModel::readFromXml(argv[2]);

//    m_id = visualiseAllFeatures(markers_pub, m_id, om->getPointFeatures(), ob_name);
//    m_id = visualiseRejectionSamplingVonMisesFisher3(markers_pub, m_id);
//    m_id = visualiseRejectionSamplingVonMisesFisher4(markers_pub, m_id);

    std::random_device rd;
    std::mt19937 gen(rd());

    boost::shared_ptr<QueryDensity > qd(new QueryDensity);

//    om->setSamplerParameters(sigma_p, sigma_q, sigma_r);
    qd->setSamplerParameters(sigma_p, sigma_q);

    // generate query density using multiple threads
    {
        const int qd_sample_count = om->getPointFeatures().size()*10;//50000;
        const int num_threads = cm->getLinkNamesCol().size();
        std::unique_ptr<std::thread[] > t(new std::thread[num_threads]);
        std::unique_ptr<std::vector<QueryDensity::QueryDensityElement >[] > qd_vec(new std::vector<QueryDensity::QueryDensityElement >[num_threads]);

        for (int thread_id = 0; thread_id < num_threads; thread_id++) {
            qd_vec[thread_id].resize(qd_sample_count);
            std::vector<QueryDensity::QueryDensityElement > aaa;
            t[thread_id] = std::thread(generateQueryDensity, gen(), std::cref(cm->getLinkNamesCol()[thread_id]), std::cref(cm), std::cref((*om)), std::ref(qd_vec[thread_id]));
        }

        for (int thread_id = 0; thread_id < num_threads; ++thread_id) {
            t[thread_id].join();
            qd->addQueryDensity(cm->getLinkNamesCol()[thread_id], qd_vec[thread_id]);
        }
    }

    writeToXml(argv[3], qd, cm, hm, om);

    return 0;
}

