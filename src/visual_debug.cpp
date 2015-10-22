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

#include "visual_debug.h"

/*#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/marker_publisher.h"
*/
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"

//#include "models.h"

static const double PI(3.141592653589793);

int visualiseContactRegion(MarkerPublisher &markers_pub, int m_id, const std::vector<CollisionModel::Feature > &f_vec, const KDL::Frame &T_W_L) {
        for (int idx = 0; idx < f_vec.size(); idx++) {
            KDL::Frame T_W_F = T_W_L * f_vec[idx].T_L_F;
            KDL::Vector v1 = T_W_F * KDL::Vector();
            KDL::Vector v2 = T_W_F * KDL::Vector(0, 0, 0.01);
            KDL::Vector v3 = T_W_F * KDL::Vector(0.01, 0, 0);
            //m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, "world");
            //m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, "world");
            double color = f_vec[idx].weight;
            m_id = markers_pub.addSinglePointMarkerCube(m_id, v1, color, color, color, 1, 0.001, 0.001, 0.001, "world");
        }
        markers_pub.addEraseMarkers(m_id, m_id+300);
        markers_pub.publish();
        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }
        return m_id;
}

int visualiseAllFeatures(MarkerPublisher &markers_pub, int m_id, const std::vector<ObjectModel::Feature > &f_vec, const std::string &frame_id) {
    // visuzlise all features on the object
    for (int idx = 0; idx < f_vec.size(); idx++) {
        const KDL::Frame &T_O_F = f_vec[idx].T_O_F_;
        KDL::Vector v1 = T_O_F * KDL::Vector();
        KDL::Vector v2 = T_O_F * KDL::Vector(0, 0, 0.01);
        KDL::Vector v3 = T_O_F * KDL::Vector(0.01, 0, 0);
        m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, frame_id);
        m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, frame_id);
    }
    markers_pub.addEraseMarkers(m_id, m_id+300);
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    return m_id;
}

int visualiseRejectionSamplingVonMisesFisher3(MarkerPublisher &markers_pub, int m_id) {
    // TEST: rejection sampling from von Mises-Fisher distribution for 3-D sphere
    Eigen::Vector3d mean;
    randomUnitSphere(mean);
    m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(mean(0)*0.1, mean(1)*0.1, mean(2)*0.1 + 1.0), 1, 0, 0, 1, 0.001, 0.001, 0.001, "world");

    double Cp = misesFisherKernelConstant(50.0, 3);
    double pdf_mean = misesFisherKernel(mean, mean, 50.0, Cp);

    int samples = 0;
    for (int i = 0; i < 1000; i++) {
        Eigen::Vector3d x;
        int iterations = vonMisesFisherSample(mean, pdf_mean, 50.0, Cp, x);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(x(0)*0.1, x(1)*0.1, x(2)*0.1 + 1.0), 0, 0, 1, 1, 0.001, 0.001, 0.001, "world");
        samples += iterations;
    }
    std::cout << "samples: " << samples << std::endl;

    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    return m_id;
}

int visualiseRejectionSamplingVonMisesFisher4(MarkerPublisher &markers_pub, int m_id) {
    // TEST: rejection sampling from von Mises-Fisher distribution for 4-D sphere
    Eigen::Vector4d mean;
    randomUnitQuaternion(mean);
    KDL::Frame fr = KDL::Frame(KDL::Rotation::Quaternion(mean(0), mean(1), mean(2), mean(3)));
    KDL::Vector of(0,0,1);
    m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0.15, 0, 0) + of, 1, 0, 0, 1, 0.002, "world");
    m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0, 0.15, 0) + of, 0, 1, 0, 1, 0.002, "world");
    m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0, 0, 0.15) + of, 0, 0, 1, 1, 0.002, "world");

    double sigma = 100.0;
    double Cp = misesFisherKernelConstant(sigma, 4);
    double pdf_mean = misesFisherKernel(mean, mean, sigma, Cp);

    int samples = 0;
    for (int i = 0; i < 1000; i++) {
        Eigen::Vector4d x;
        int iterations = vonMisesFisherSample(mean, pdf_mean, sigma, Cp, x);
        if (iterations < 0) {
            std::cout << "ERROR: vonMisesFisherSample" << std::endl;
        }

        KDL::Frame fr = KDL::Frame(KDL::Rotation::Quaternion(x(0), x(1), x(2), x(3)));
        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0.1, 0, 0) + of, 1, 0, 0, 1, 0.001, "world");
        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0, 0.1, 0) + of, 0, 1, 0, 1, 0.001, "world");
        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector() + of, fr * KDL::Vector(0, 0, 0.1) + of, 0, 0, 1, 1, 0.001, "world");
        samples += iterations;
    }
    std::cout << "samples: " << samples << std::endl;

    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    return 0;
}

int visualiseQueryDensityParticles(MarkerPublisher &markers_pub, int m_id, const std::vector<QueryDensity::QueryDensityElement > &qd_vec, const std::string &frame_id) {
    for (std::vector<QueryDensity::QueryDensityElement >::const_iterator it = qd_vec.begin(); it != qd_vec.end(); it++) {
        // visualisation
        KDL::Frame fr(KDL::Rotation::Quaternion(it->q_(0), it->q_(1), it->q_(2), it->q_(3)), KDL::Vector(it->p_(0), it->p_(1), it->p_(2)));
        double w = it->weight_ * 2000.0;
//        std::cout << it->weight_ << std::endl;
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0.01, 0, 0), 1, 0, 0, 1, 0.0002, ob_name);
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0, 0.01, 0), 0, 1, 0, 1, 0.0002, ob_name);
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0, 0, 0.01), 0, 0, 1, 1, 0.0002, ob_name);
//        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(p(0), p(1), p(2)), r(0)*4, 0, r(1)*4, 1, 0.001, 0.001, 0.001, ob_name);
        m_id = markers_pub.addSinglePointMarkerCube(m_id, fr * KDL::Vector(), w, w, w, 1, 0.001, 0.001, 0.001, frame_id);
//        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(result_x, result_y, result_z), sum/10000000000.0, 0, 0, 1, 0.001, 0.001, 0.001, ob_name);
//        std::cout << i << std::endl;
    }
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
/*
    for (int i = 0; i < 100; i++) {
        int idx = rand() % qd_vec.size();
        KDL::Frame T_O_L(KDL::Rotation::Quaternion(qd_vec[idx].q_(0), qd_vec[idx].q_(1), qd_vec[idx].q_(2), qd_vec[idx].q_(3)), KDL::Vector(qd_vec[idx].p_(0), qd_vec[idx].p_(1), qd_vec[idx].p_(2)));
        publishTransform(br, T_W_O * T_O_L, link_name_test, "world");
        std::cout << qd_vec[idx].weight_ << std::endl;
        ros::spinOnce();
        ros::Duration(2.0).sleep();
    }
*/
    return m_id;
}

int visualiseQueryDensityFunction(tf::TransformBroadcaster &br, MarkerPublisher &markers_pub, int m_id, const QueryDensity &qd, const std::string &link_name, const KDL::Frame &T_W_O, const std::string &frame_id) {
    // visualisation of query density
    KDL::Rotation rot(KDL::Rotation::RotZ(-(-90.0+30.0)/180.0*PI));
    double grid_size = 0.004;
    std::list<std::pair<KDL::Frame, double> > test_list;
    double max_cost = 0.0;
    for (double x = -0.12; x < 0.12; x += grid_size) {
        for (double y = -0.12; y < 0.12; y += grid_size) {
            KDL::Frame T_O_L = KDL::Frame(rot, KDL::Vector(x, y, 0.0));
            double cost = qd.getQueryDensity(link_name, T_O_L);
            test_list.push_back(std::make_pair(T_O_L, cost));
            if (cost > max_cost) {
                max_cost = cost;
            }
        }
    }

    std::cout << "max_cost " << max_cost << std::endl;

    for (std::list<std::pair<KDL::Frame, double> >::const_iterator it = test_list.begin(); it != test_list.end(); it++) {
        double color = it->second / max_cost;
        m_id = markers_pub.addSinglePointMarkerCube(m_id, it->first * KDL::Vector(), color, color, color, 1, grid_size, grid_size, grid_size, frame_id);
    }

    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        publishTransform(br, T_W_O * KDL::Frame(rot, KDL::Vector(0,0,-0.1)), link_name, "world");
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }

    return m_id;
}

