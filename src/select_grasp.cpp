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
#include <algorithm>

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
#include "visual_debug.h"

#include <kin_model/kin_model.h>

const double PI(3.141592653589793);

double getTransitionProbability(double cost1, double cost2, double temperature) {
    if (std::fabs(cost2) < 0.0000001) {
        return 0.0;
    }
    if (cost2 > cost1) {
        return 1.0;
    }
    
    return temperature / 100.0 * 0.01;
}

void getFK(const dart::dynamics::SkeletonPtr &bh, const std::map<std::string, double> &q_map, const std::string &link1_name, const std::string &link2_name, KDL::Frame &T_L1_L2) {
//    std::map<std::string, double> saved_q_map;
    Eigen::Isometry3d tf;
    KDLToEigenTf(KDL::Frame(), tf);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);
    for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
//        saved_q_map[it->first] = j->getPosition( 0 );
        j->setPosition( 0, it->second );
        if (it->second != it->second) {
            std::cout << "ERROR: getFK" << std::endl;
            return;
        }
    }

    KDL::Frame T_W_L1, T_W_L2;

    dart::dynamics::BodyNode *b1 = bh->getBodyNode(link1_name);
    Eigen::Isometry3d tf1 = b1->getTransform();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (std::fabs(tf1(i,j)) > 3.0) {
                std::cout << "std::fabs(tf1(i,j)) > 3.0: " << tf1(i,j) << std::endl;
                for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
                    std::cout << "   " << it->first << " " << it->second << std::endl;
                }
            }
            if (tf1(i,j) != tf1(i,j)) {
                std::cout << "tf1 != tf1 " << i << " " << j << std::endl;
            }
        }
    }
    EigenTfToKDL(tf1, T_W_L1);

    dart::dynamics::BodyNode *b2 = bh->getBodyNode(link2_name);
    Eigen::Isometry3d tf2 = b2->getTransform();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (std::fabs(tf2(i,j)) > 3.0) {
                std::cout << "std::fabs(tf2(i,j)) > 3.0: " << tf1(i,j) << std::endl;
            }
            if (tf2(i,j) != tf2(i,j)) {
                std::cout << "tf2 != tf2 " << i << " " << j << std::endl;
            }
        }
    }
    EigenTfToKDL(tf2, T_W_L2);

    T_L1_L2 = T_W_L1.Inverse() * T_W_L2;
    if (T_W_L1 != T_W_L1) {
        std::cout << "ERROR: getFK T_W_L1 " << link1_name << " " << link2_name << std::endl;
    }
    if (T_W_L2 != T_W_L2) {
        std::cout << "ERROR: getFK T_W_L2 " << link1_name << " " << link2_name << std::endl;
    }
    if (T_L1_L2 != T_L1_L2) {
        std::cout << "ERROR: getFK T_L1_L2 " << link1_name << " " << link2_name << " " << T_W_L1 << " " << T_W_L2 << std::endl;
        *((int*)0) = 0;
    }
/*
    for (std::map<std::string, double>::const_iterator it = saved_q_map.begin(); it != saved_q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
    }
*/
}

class GraspSolution {
public:
    KDL::Frame T_W_E_;
    std::map<std::string, double> q_map_;
    double score_;

    double evaluate(dart::dynamics::SkeletonPtr &bh, const KDL::Frame &T_W_O, const boost::shared_ptr<HandConfigurationModel > &hm, const boost::shared_ptr<QueryDensity > &qd, const boost::shared_ptr<CollisionModel > &cm) {
            if (T_W_E_ != T_W_E_) {
                score_ = 0.0;
                return score_;
            }
            Eigen::Isometry3d tf;
            KDLToEigenTf(T_W_E_, tf);
            bh->getJoint(0)->setTransformFromParentBodyNode(tf);
            for (std::map<std::string, double>::const_iterator it = q_map_.begin(); it != q_map_.end(); it++) {
                dart::dynamics::Joint *j = bh->getJoint( it-> first );
                if (it->second != it->second) {
                    std::cout << "ERROR: evaluate" << std::endl;
                    return 0.0;
                }
                j->setPosition( 0, it->second );
            }

            double score = hm->getDensity(q_map_);
            for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
                KDL::Frame T_E_L;
                getFK(bh, q_map_, "right_HandPalmLink", (*lit), T_E_L);
                KDL::Frame T_O_L( T_W_O.Inverse() * T_W_E_ * T_E_L );
                KDL::Frame T_L_C;
                cm->getT_L_C((*lit), T_L_C);
                score *= qd->getQueryDensity((*lit), T_O_L * T_L_C);
            }
            if (score != score) {// || score < 0.00000000001) {
                score = 0.0;
            }
            score_ = score;
            return score_;
    }

    bool operator< (const GraspSolution &g) const {
        return score_ < g.score_;
    }
};


//    boost::shared_ptr<QueryDensity > qd = QueryDensity::readFromXml(argv[1]);
//    boost::shared_ptr<CollisionModel > cm = CollisionModel::readFromXml(argv[1]);
//    boost::shared_ptr<HandConfigurationModel > hm = HandConfigurationModel::readFromXml(argv[1]);
//    boost::shared_ptr<ObjectModel > 

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


    std::vector<std::string > joint_names;
    joint_names.push_back("right_HandFingerOneKnuckleOneJoint");
    joint_names.push_back("right_HandFingerOneKnuckleTwoJoint");
    joint_names.push_back("right_HandFingerOneKnuckleThreeJoint");
    joint_names.push_back("right_HandFingerTwoKnuckleOneJoint");
    joint_names.push_back("right_HandFingerTwoKnuckleTwoJoint");
    joint_names.push_back("right_HandFingerTwoKnuckleThreeJoint");
    joint_names.push_back("right_HandFingerThreeKnuckleTwoJoint");
    joint_names.push_back("right_HandFingerThreeKnuckleThreeJoint");
    KinematicModel km(robot_description_str, joint_names);

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

    int m_id = 0;
    ros::Duration(1.0).sleep();

    const std::vector<ObjectModel::Feature > &om_f_vec = om->getFeaturesData();
    std::cout << "om_f_vec: " << om_f_vec.size() << std::endl;

    // visualisation

    // publish grasped object model
    publishTransform(br, T_W_O, "graspable", "world");
    for (int i = 0; i < om_f_vec.size(); i++) {
        if (rand() % 4 == 0) {
            m_id = markers_pub.addSinglePointMarkerCube(m_id, om_f_vec[i].T_O_F_.p, 1, 1, 1, 1, 0.001, 0.001, 0.001, "graspable");
        }
    }

    markers_pub.publish();
    ros::spinOnce();

/*
    publishTransform(br, T_W_O, "graspable", "world");

    m_id = visualiseQueryDensityParticles(markers_pub, m_id, qd->qd_map_["right_HandFingerTwoKnuckleThreeLink"].vec_, "graspable");
            markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
        ros::Duration(1.0).sleep();

    return 0;
*/

//    double cost_max = 0.0;
//    int good_grasps_count = 0;
//    KDL::Frame T_W_E_best;
//    std::map<std::string, double> q_best;
    int n_solutions = 400;
    std::vector<GraspSolution > solutions;
    for (int  i = 0; i < n_solutions; i++) {
        const std::string &link_name = cm->getRandomLinkNameCol();
        KDL::Frame T_O_L1;
        KDL::Frame T_O_C1;
        qd->sampleQueryDensity(gen(), link_name, T_O_C1);
        KDL::Frame T_L1_C;
        cm->getT_L_C(link_name, T_L1_C);
        T_O_L1 = T_O_C1 * T_L1_C.Inverse();

        publishTransform(br, T_W_O * T_O_L1, link_name, "world");

        std::map<std::string, double> q_sample;
        hm->sample(gen(), q_sample);

        // apply joint limits
        for (std::map<std::string, double>::iterator it = q_sample.begin(); it != q_sample.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint( it-> first );
            it->second = std::max( j->getPositionLowerLimit( 0 ), it->second );
            it->second = std::min( j->getPositionUpperLimit( 0 ), it->second );
        }

        KDL::Frame T_L1_E;
        getFK(bh, q_sample, link_name, "right_HandPalmLink", T_L1_E);

        GraspSolution sol;
        sol.T_W_E_ = T_W_O * T_O_L1 * T_L1_E;
        sol.q_map_ = q_sample;
        sol.evaluate(bh, T_W_O, hm, qd, cm);
        if (sol.score_ != 0.0) {
            solutions.push_back(sol);
            std::cout << "found solution: " << sol.score_ << std::endl;
        }

        if ((i % 500) == 0) {
            std::cout << i << "  solutions: " << solutions.size() << std::endl;
        }

    }

//    std::cout << "best: " << cost_max << "   good_grasps_count " << good_grasps_count << std::endl;

    std::cout << "solutions found: " << solutions.size() << std::endl;

    std::sort(solutions.begin(), solutions.end());
    std::reverse(solutions.begin(), solutions.end());

    solutions.resize(solutions.size() / 10);

    std::cout << "reduced solutions to: " << solutions.size() << std::endl;

    std::vector<GraspSolution > solutions_best(solutions);

    double temperature = 100.0;


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
    int steps = 40;
    for (int i = 0; i < steps; i++) {
        int good_solutions = 0;
        // simulated annealing step
        for (int sidx = 0; sidx < solutions.size(); sidx++) {
            if (solutions[sidx].score_ == 0.0) {
                continue;
            }
            good_solutions++;

            GraspSolution new_solution;

            Eigen::Vector3d axis;
            randomUnitSphere(axis);
            std::normal_distribution<> d_angle = std::normal_distribution<>(0, 2.0/180.0*PI);
            std::normal_distribution<> d_pos = std::normal_distribution<>(0, 0.002);
            KDL::Frame T_E_G(KDL::Vector(0,0,0.15));
            KDL::Frame T_W_G = solutions[sidx].T_W_E_ * T_E_G;
            KDL::Frame T_G_Gnew(KDL::Rotation::Rot(KDL::Vector(axis(0), axis(1), axis(2)), d_angle(gen)), KDL::Vector(d_pos(gen), d_pos(gen), d_pos(gen)));
            KDL::Frame T_W_G_new = T_W_G * T_G_Gnew;
            new_solution.T_W_E_ = T_W_G_new * T_E_G.Inverse();

            std::normal_distribution<> d_conf = std::normal_distribution<>(0, 2.0/180.0*PI);
            std::map<std::string, double> q_new( solutions[sidx].q_map_ );
            double angleDiffF1 = d_conf(gen);
            double angleDiffF2 = d_conf(gen);
            double angleDiffF3 = d_conf(gen);
            q_new["right_HandFingerOneKnuckleTwoJoint"] -= angleDiffF1;
            q_new["right_HandFingerTwoKnuckleTwoJoint"] -= angleDiffF2;
            q_new["right_HandFingerThreeKnuckleTwoJoint"] -= angleDiffF3;
            q_new["right_HandFingerOneKnuckleThreeJoint"] -= angleDiffF1*0.333333;
            q_new["right_HandFingerTwoKnuckleThreeJoint"] -= angleDiffF2*0.333333;
            q_new["right_HandFingerThreeKnuckleThreeJoint"] -= angleDiffF3*0.333333;
            // apply joint limits
            for (std::map<std::string, double>::iterator it = q_new.begin(); it != q_new.end(); it++) {
                dart::dynamics::Joint *j = bh->getJoint( it-> first );
                it->second = std::max( j->getPositionLowerLimit( 0 ), it->second );
                it->second = std::min( j->getPositionUpperLimit( 0 ), it->second );
            }
            new_solution.q_map_ = q_new;
            new_solution.evaluate(bh, T_W_O, hm, qd, cm);

            if (new_solution.score_ == 0.0) {
                continue;
            }
            double trPr = getTransitionProbability(solutions[sidx].score_, new_solution.score_, temperature);
            if (randomUniform(0.0, 1.0) < trPr) {
                solutions[sidx] = new_solution;
                if (solutions_best[sidx] < solutions[sidx]) {
                    solutions_best[sidx] = solutions[sidx];
                }
            }
        }

//        std::sort(solutions.begin(), solutions.end());
//        std::reverse(solutions.begin(), solutions.end());
//"   " << solutions[0].score_ << "   " << solutions[1].score_ << "   " << solutions[2].score_ << "   " << solutions[3].score_ << std::endl;

        temperature = 100.0 * static_cast<double >(steps - 1 - i) / static_cast<double >(steps - 1);
        std::cout << "good_solutions: " << good_solutions << std::endl;
    }


/*
        // evaluate all solutions
        for (int sidx = 0; sidx < n_solutions; sidx++) {
            if (solutions[sidx].score_ > -0.1) {
                continue;
            }
            double cost = hm->getDensity(solutions[sidx].q_map_);
            for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
                KDL::Frame T_E_L;
                getFK(bh, solutions[sidx].q_map_, "right_HandPalmLink", (*lit), T_E_L);
                KDL::Frame T_O_L( T_W_O.Inverse() * solutions[sidx].T_W_E_ * T_E_L );
                KDL::Frame T_L_C;
                cm->getT_L_C((*lit), T_L_C);
                cost *= qd->getQueryDensity((*lit), T_O_L * T_L_C);
            }
            if (cost != cost) {
                cost = 0.0;
            }
            solutions[sidx].score_ = cost;
        }
*/
    solutions = solutions_best;
    std::sort(solutions.begin(), solutions.end());
    std::reverse(solutions.begin(), solutions.end());
//    std::cout << solutions[0].score_ << "   " << solutions[1].score_ << "   " << solutions[2].score_ << "   " << solutions[3].score_ << std::endl;

    for (int sidx = 0; sidx < solutions.size(); sidx++) {
        KDLToEigenTf(solutions[sidx].T_W_E_, tf);
        bh->getJoint(0)->setTransformFromParentBodyNode(tf);
        for (std::map<std::string, double>::const_iterator it = solutions[sidx].q_map_.begin(); it != solutions[sidx].q_map_.end(); it++) {
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

            std::cout << "total cost " << solutions[sidx].score_ << std::endl;
            double cost = hm->getDensity(solutions[sidx].q_map_);
            std::cout << "   cost (hm) " << cost << std::endl;
            for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
                KDL::Frame T_E_L;
                getFK(bh, solutions[sidx].q_map_, "right_HandPalmLink", (*lit), T_E_L);
                KDL::Frame T_O_L( T_W_O.Inverse() * solutions[sidx].T_W_E_ * T_E_L );
                KDL::Frame T_L_C;
                cm->getT_L_C((*lit), T_L_C);
                double cost_link = qd->getQueryDensity((*lit), T_O_L * T_L_C);
                cost *= cost_link;
                std::cout << "   cost " << (*lit) << "   " << cost_link << std::endl;
            }

        markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
        ros::Duration(0.02).sleep();
        if (!ros::ok()) {
            break;
        }
        getchar();
    }
    return 0;
}

