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
#include "grasp_state.h"
#include "visual_debug.h"

const double PI(3.141592653589793);

double getTransitionProbability(double cost1, double cost2, double temperature) {
    if (std::fabs(cost2) < 0.0000001) {
        return 0.0;
    }
    if (cost2 > cost1) {
        return 1.0;
    }
    
    return temperature / 100.0 * 0.05;
}

void getFK(const dart::dynamics::SkeletonPtr &bh, const std::map<std::string, double> &q_map, const std::string &link1_name, const std::string &link2_name, KDL::Frame &T_L1_L2) {
    std::map<std::string, double> saved_q_map;
    for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        saved_q_map[it->first] = j->getPosition( 0 );
        j->setPosition( 0, it->second );
    }

    KDL::Frame T_W_L1, T_W_L2;

    dart::dynamics::BodyNode *b1 = bh->getBodyNode(link1_name);
    const Eigen::Isometry3d &tf1 = b1->getTransform();
    EigenTfToKDL(tf1, T_W_L1);

    dart::dynamics::BodyNode *b2 = bh->getBodyNode(link2_name);
    const Eigen::Isometry3d &tf2 = b2->getTransform();
    EigenTfToKDL(tf2, T_W_L2);

    T_L1_L2 = T_W_L1.Inverse() * T_W_L2;

    for (std::map<std::string, double>::const_iterator it = saved_q_map.begin(); it != saved_q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "usage:" << std::endl;
        std::cout << "query_density" << std::endl;

        return 0;
    }

    srand ( time(NULL) );

    ros::init(argc, argv, "dart_test");

    ros::NodeHandle nh_;
        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

    ros::Rate loop_rate(400);

    boost::shared_ptr<QueryDensity > qd = QueryDensity::readFromXml(argv[1]);
    boost::shared_ptr<CollisionModel > cm = CollisionModel::readFromXml(argv[1]);
    boost::shared_ptr<HandConfigurationModel > hm = HandConfigurationModel::readFromXml(argv[1]);
    boost::shared_ptr<ObjectModel > om = ObjectModel::readFromXml(argv[1]);

    ros::Publisher joint_state_pub_;
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    tf::TransformBroadcaster br;
    MarkerPublisher markers_pub(nh_);

    std::string package_path_barrett = ros::package::getPath("barrett_hand_defs");
    std::string package_path_sim = ros::package::getPath("barrett_hand_sim_dart");

    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("barrett_hand_defs", package_path_barrett);
    loader.addPackageDirectory("barrett_hand_sim_dart", package_path_sim);
    dart::dynamics::SkeletonPtr bh( loader.parseSkeleton(package_path_barrett + "/robots/barrett_hand.urdf") );
    bh->setName("BarrettHand");

    // Position its base in a reasonable way
    Eigen::Isometry3d tf;
    KDLToEigenTf(KDL::Frame( KDL::Rotation::RotX(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.22) ), tf);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);

//    dart::dynamics::SkeletonPtr domino( loader.parseSkeleton(package_path_sim + om->path_urdf_) );
//    KDLToEigenTf(KDL::Frame( KDL::Vector(0.0, 0.0, 0.0) ), tf);
//    domino->getJoint(0)->setTransformFromParentBodyNode(tf);

    dart::simulation::World* world = new dart::simulation::World();

    world->addSkeleton(bh);
//    world->addSkeleton(domino);

    Eigen::Vector3d grav(0,0,0);
    world->setGravity(grav);

    KDL::Frame T_W_O;

/*
    for (std::map<std::string, double>::const_iterator it = gs->q_map_.begin(); it != gs->q_map_.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
    }

    tf = bh->getBodyNode("right_HandPalmLink")->getTransform();
    KDL::Frame T_W_E;
    EigenTfToKDL(tf, T_W_E);
    KDL::Frame T_W_O = T_W_E * gs->T_E_O_;

    KDLToEigenTf(T_W_O, tf);
    domino->getJoint(0)->setTransformFromParentBodyNode(tf);

    // visualisation
    for (int i = 0; i < 100; i++) {
        for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = bh->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
            publishTransform(br, T_W_L, b->getName(), "world");
        }

        int m_id = 0;
        for (int skidx = 0; skidx < world->getNumSkeletons(); skidx++) {
            dart::dynamics::SkeletonPtr sk = world->getSkeleton(skidx);
            if (sk->getName() == bh->getName()) {
                continue;
            }

            for (int bidx = 0; bidx < sk->getNumBodyNodes(); bidx++) {
                dart::dynamics::BodyNode *b = sk->getBodyNode(bidx);
                const Eigen::Isometry3d &tf = b->getTransform();
                KDL::Frame T_W_L;
                EigenTfToKDL(tf, T_W_L);
                publishTransform(br, T_W_L, b->getName(), "world");
                for (int cidx = 0; cidx < b->getNumCollisionShapes(); cidx++) {
                    dart::dynamics::ConstShapePtr sh = b->getCollisionShape(cidx);
                    if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                        std::shared_ptr<const dart::dynamics::MeshShape > msh = std::static_pointer_cast<const dart::dynamics::MeshShape >(sh);
                        m_id = markers_pub.addMeshMarker(m_id, KDL::Vector(), 0, 1, 0, 1, 1, 1, 1, std::string("file://") + msh->getMeshPath(), b->getName());
                    }
                }
            }
        }
        markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
*/

    const double sigma_p = 0.005;
    const double sigma_q = 100.0;
    const double sigma_r = 0.05;

//    {
//        KDL::Frame T_L_C;
//        cm->getT_L_C("right_HandFingerTwoKnuckleThreeLink", T_L_C);
//        m_id = visualiseContactRegion( markers_pub, m_id, cm->getLinkFeatures("right_HandFingerTwoKnuckleThreeLink"), T_L_C, T_W_E * frames_map["right_HandPalmLink"].Inverse() * frames_map["right_HandFingerTwoKnuckleThreeLink"] );
//    }
//    m_id = visualiseAllFeatures(markers_pub, m_id, om.getPointFeatures(), ob_name);
//    m_id = visualiseRejectionSamplingVonMisesFisher3(markers_pub, m_id);
//    m_id = visualiseRejectionSamplingVonMisesFisher4(markers_pub, m_id);


    std::random_device rd;
    std::mt19937 gen(rd());

/*
    boost::shared_ptr<QueryDensity > pqd = QueryDensity::readFromXml("qd.xml");
    if (*pqd.get() == *qd.get()) {
        std::cout << "the same" << std::endl;
    }
    else {
        std::cout << "different" << std::endl;
    }
//    pqd->writeToXml("qd2.xml");

    return 0;
*/
//    m_id = visualiseQueryDensityParticles(markers_pub, m_id, qd->qd_map_["right_HandFingerTwoKnuckleThreeLink"].vec_, ob_name);


    double cost_max = 0.0;
    int good_grasps_count = 0;
    KDL::Frame T_W_E_best;
    std::map<std::string, double> q_best;
    for (int  i = 0; i < 4000; i++) {
        const std::string &link_name = cm->getRandomLinkNameCol();
        KDL::Frame T_O_L1;
        KDL::Frame T_O_C1;
        qd->sampleQueryDensity(gen(), link_name, T_O_C1);
        KDL::Frame T_L1_C;
        cm->getT_L_C(link_name, T_L1_C);
        T_O_L1 = T_O_C1 * T_L1_C.Inverse();
        std::map<std::string, double> q_sample;
        hm->sample(gen(), q_sample);

        double cost = qd->getQueryDensity(link_name, T_O_L1 * T_L1_C);
        for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
            if (cost < 0.0000001) {
                cost = 0.0;
                break;
            }
            if (link_name == (*lit)) {
                continue;
            }

            KDL::Frame T_L2_C;
            cm->getT_L_C((*lit), T_L2_C);
            KDL::Frame T_L1_L2;
            getFK(bh, q_sample, link_name, (*lit), T_L1_L2);
            KDL::Frame T_O_L2( T_O_L1 * T_L1_L2 );
            cost *= qd->getQueryDensity((*lit), T_O_L2 * T_L2_C);
        }

        KDL::Frame T_L1_E;
        getFK(bh, q_sample, link_name, "right_HandPalmLink", T_L1_E);

        if (cost > cost_max) {
            cost_max = cost;
            T_W_E_best = T_W_O * T_O_L1 * T_L1_E;
            q_best = q_sample;
        }

        if (cost > 0.0000001) {
            good_grasps_count++;
        }

        if ((i % 1000) == 0) {
            std::cout << i << "   " << cost << std::endl;
        }
    }

    std::cout << "best: " << cost_max << "   good_grasps_count " << good_grasps_count << std::endl;

    const std::vector<ObjectModel::Feature > &om_f_vec = om->getFeaturesData();
    std::cout << "om_f_vec: " << om_f_vec.size() << std::endl;

    double temperature = 100.0;

    // visualisation

    // publish grasped object model
    int m_id = 0;
    publishTransform(br, T_W_O, "graspable", "world");
    for (int i = 0; i < om_f_vec.size(); i++) {
        if (rand() % 4 == 0) {
            m_id = markers_pub.addSinglePointMarkerCube(m_id, om_f_vec[i].T_O_F_.p, 1, 1, 1, 1, 0.001, 0.001, 0.001, "graspable");
        }
    }

/*
    {
        KDL::Frame T_L_C;
        cm->getT_L_C("right_HandFingerTwoKnuckleThreeLink", T_L_C);
        m_id = visualiseQueryDensityFunction(br, markers_pub, m_id, *qd.get(), "right_HandFingerTwoKnuckleThreeLink", T_L_C, T_W_O, "graspable");
    }

        markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
        ros::Duration(1.0).sleep();
    return 0;
//*/
    for (int i = 0; i < 100000; i++) {

        // Position its base in a reasonable way
        KDLToEigenTf(T_W_E_best, tf);
        bh->getJoint(0)->setTransformFromParentBodyNode(tf);
        for (std::map<std::string, double>::const_iterator it = q_best.begin(); it != q_best.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint( it-> first );
            j->setPosition( 0, it->second );
        }

        // publish transforms of the gripper
        for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = bh->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
            publishTransform(br, T_W_L, b->getName(), "world");
        }

        markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
        ros::Duration(0.02).sleep();
        if (!ros::ok()) {
            break;
        }

        Eigen::Vector4d qq, qq_res;
        T_W_E_best.M.GetQuaternion(qq(0), qq(1), qq(2), qq(3));

        Eigen::Vector3d axis;
        randomUnitSphere(axis);
        std::normal_distribution<> d = std::normal_distribution<>(0, 2.0/180.0*PI);
        KDL::Rotation rot( T_W_E_best.M * KDL::Rotation::Rot(KDL::Vector(axis(0), axis(1), axis(2)), d(gen)) );
        d = std::normal_distribution<>(0, 0.002);
        KDL::Frame T_E_G(KDL::Vector(0,0,0.15));
        KDL::Frame T_W_G_best = T_W_E_best * T_E_G;
        KDL::Frame T_W_G_new( rot, T_W_G_best.p + KDL::Vector(d(gen), d(gen), d(gen)));
        KDL::Frame T_W_E_new = T_W_G_new * T_E_G.Inverse();

        d = std::normal_distribution<>(0, 2.0/180.0*PI);
        std::map<std::string, double> q_new( q_best );
        double angleDiffF1 = d(gen);
        double angleDiffF2 = d(gen);
        double angleDiffF3 = d(gen);
        q_new["right_HandFingerOneKnuckleTwoJoint"] -= angleDiffF1;
        q_new["right_HandFingerTwoKnuckleTwoJoint"] -= angleDiffF2;
        q_new["right_HandFingerThreeKnuckleTwoJoint"] -= angleDiffF3;
        q_new["right_HandFingerOneKnuckleThreeJoint"] -= angleDiffF1*0.333333;
        q_new["right_HandFingerTwoKnuckleThreeJoint"] -= angleDiffF2*0.333333;
        q_new["right_HandFingerThreeKnuckleThreeJoint"] -= angleDiffF3*0.333333;
        for (std::map<std::string, double>::iterator it = q_new.begin(); it != q_new.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint( it-> first );
            it->second = std::max( j->getPositionLowerLimit( 0 ), it->second );
            it->second = std::min( j->getPositionUpperLimit( 0 ), it->second );
        }

        double cost = hm->getDensity(q_new);
        for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
            if (cost < 0.0000001) {
                cost = 0.0;
                break;
            }
            KDL::Frame T_E_L;
            getFK(bh, q_new, "right_HandPalmLink", (*lit), T_E_L);
            KDL::Frame T_O_L( T_W_O.Inverse() * T_W_E_new * T_E_L );
            KDL::Frame T_L_C;
            cm->getT_L_C((*lit), T_L_C);
            cost *= qd->getQueryDensity((*lit), T_O_L * T_L_C);
        }
        double trPr = getTransitionProbability(cost_max, cost, temperature);
        std::cout << "temp: " << temperature << "   cost: " << cost_max << "   cost_new: " << cost << "   trPr: " << trPr << std::endl;

        if (randomUniform(0.0, 1.0) < trPr) {
            T_W_E_best = T_W_E_new;
            cost_max = cost;
            q_best = q_new;
        }
        temperature -= 0.02;

    }

    return 0;
}

