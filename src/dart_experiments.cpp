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
#include <cstdlib>
#include <fstream>

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

#include "grasp_specification.h"
#include "models.h"
#include "mesh_sampling.h"
#include "visual_debug.h"

#include "gripper_controller.h"

bool checkCollision(dart::simulation::World* world, const dart::dynamics::SkeletonPtr &bh, const std::map<std::string, double> &q_map, const KDL::Frame &T_W_E) {
    Eigen::Isometry3d tf;
    KDLToEigenTf(T_W_E, tf);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);
    for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
        if (it->second != it->second) {
            std::cout << "ERROR: checkCollision" << std::endl;
            return false;
        }
    }

    dart::collision::CollisionDetector* detector =
        world->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    bool collision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i = 0; i < collisionCount; ++i)
    {
      const dart::collision::Contact& contact = detector->getContact(i);
      if(contact.bodyNode1.lock()->getSkeleton()->getName() == bh->getName()
         || contact.bodyNode2.lock()->getSkeleton()->getName() == bh->getName())
      {
        collision = true;
        break;
      }
    }
    return collision;
}

int addScene(MarkerPublisher &markers_pub, tf::TransformBroadcaster &br, int m_id, dart::simulation::World* world, dart::dynamics::SkeletonPtr &bh) {
    // visualisation
    for (int i = 0; i < 100; i++) {
        for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = bh->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
            publishTransform(br, T_W_L, b->getName(), "world");
        }

        for (int skidx = 0; skidx < world->getNumSkeletons(); skidx++) {
            dart::dynamics::SkeletonPtr sk = world->getSkeleton(skidx);
            if (sk->getName() == bh->getName()) {
                continue;
            }

            for (int bidx = 0; bidx < sk->getNumBodyNodes(); bidx++) {
                dart::dynamics::BodyNode *b = sk->getBodyNode(bidx);
                const Eigen::Isometry3d &tf = b->getTransform();
                Eigen::Vector3d color(0, 0.8, 0);
                if (b->getName() == "graspable") {
                    color = Eigen::Vector3d(0.8, 0, 0);
                }
                else {
//                    continue;
                }
                KDL::Frame T_W_L;
                EigenTfToKDL(tf, T_W_L);
                publishTransform(br, T_W_L, b->getName(), "world");
                for (int cidx = 0; cidx < b->getNumCollisionShapes(); cidx++) {
                    dart::dynamics::ConstShapePtr sh = b->getCollisionShape(cidx);
                    if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                        std::shared_ptr<const dart::dynamics::MeshShape > msh = std::static_pointer_cast<const dart::dynamics::MeshShape >(sh);
                        m_id = markers_pub.addMeshMarker(m_id, KDL::Vector(), color(0), color(1), color(2), 1, 1, 1, 1, msh->getMeshUri(), b->getName());
                    }
                }
            }
        }
    }
    return m_id;
}

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
    Eigen::Isometry3d tf;
    KDLToEigenTf(KDL::Frame(), tf);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);
    for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
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
            if (score != score) {
                score = 0.0;
            }
            score_ = score;
            return score_;
    }

    bool operator< (const GraspSolution &g) const {
        return score_ < g.score_;
    }
};

int main(int argc, char** argv) {
    const double PI(3.141592653589793);

    if (argc != 5) {
        std::cout << "usage:" << std::endl;
        std::cout << "package_scene path_scene grasp_model_file output_file" << std::endl;
        return 0;
    }

    std::string package_name( argv[1] );
    std::string scene_urdf( argv[2] );
    std::string grasp_model_name(argv[3]);
    std::ofstream outf(argv[4]);

    outf << scene_urdf << "\t" << grasp_model_name << "\t";

    // generate the object model
    std::cout << "generating the Object Model..." << std::endl;
    {
        std::string om_cmd(std::string("rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_object_model ") + package_name + std::string(" ") + scene_urdf + std::string(" /tmp/om.xml"));
        int exit_code = std::system(om_cmd.c_str());
        if (exit_code != 0) {
            std::cout << "ERROR: Object Model generator returned exit code " << exit_code << std::endl;
        }
    }
    boost::shared_ptr<ObjectModel > om = ObjectModel::readFromXml("/tmp/om.xml");
    std::cout << "Object Model features: " << om->getFeaturesCount() << std::endl;

    // generate the query density
    std::cout << "generating the Query Density..." << std::endl;
    {
        std::string om_cmd(std::string("rosrun barrett_hand_sim_dart barrett_hand_sim_dart_generate_query_density /tmp/om.xml ") + grasp_model_name + std::string(" /tmp/qd.xml"));
        int exit_code = std::system(om_cmd.c_str());
        if (exit_code != 0) {
            std::cout << "ERROR: Query Density generator returned exit code " << exit_code << std::endl;
        }
    }
    boost::shared_ptr<QueryDensity > qd = QueryDensity::readFromXml("/tmp/qd.xml");
    boost::shared_ptr<CollisionModel > cm = CollisionModel::readFromXml("/tmp/qd.xml");
    boost::shared_ptr<HandConfigurationModel > hm = HandConfigurationModel::readFromXml("/tmp/qd.xml");

    const std::string ob_name( "graspable" );

    ros::init(argc, argv, "dart_test");
    ros::NodeHandle nh_;

    std::string package_path = ros::package::getPath(package_name);

    ros::Rate loop_rate(400);

    tf::TransformBroadcaster br;
    MarkerPublisher markers_pub(nh_);

    ros::Duration(0.5).sleep();
    markers_pub.addEraseMarkers(0, 10000);
    markers_pub.publish();
    ros::spinOnce();

    std::string package_path_barrett = ros::package::getPath("barrett_hand_defs");
    std::string package_path_sim = ros::package::getPath("barrett_hand_sim_dart");

    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("barrett_hand_defs", package_path_barrett);
    loader.addPackageDirectory("barrett_hand_sim_dart", package_path_sim);

    dart::dynamics::SkeletonPtr scene( loader.parseSkeleton(package_path + scene_urdf) );
    scene->enableSelfCollision(true);

    dart::dynamics::SkeletonPtr bh( loader.parseSkeleton(package_path_barrett + "/robots/barrett_hand.urdf") );
    Eigen::Isometry3d tf;
    tf = scene->getBodyNode("gripper_mount_link")->getRelativeTransform();
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);

    dart::simulation::World* world = new dart::simulation::World();

    world->addSkeleton(scene);
    world->addSkeleton(bh);

    Eigen::Vector3d grav(0,0,-1);
    world->setGravity(grav);
    scene->getBodyNode(ob_name)->setFrictionCoeff(0.1);

    addScene(markers_pub, br, 0, world, bh);
    markers_pub.publish();
    ros::spinOnce();

    // let the object fall
    int counter = 0;
    while (ros::ok()) {
            world->step(false);

            if (counter % 10 == 0) {
                addScene(markers_pub, br, 0, world, bh);
                markers_pub.publish();
                ros::spinOnce();
            }

            counter++;
            if (counter > 2000) {
                break;
            }
    }

    tf = scene->getBodyNode(ob_name)->getRelativeTransform();
    KDL::Frame T_W_O;
    EigenTfToKDL(tf, T_W_O);

    Eigen::VectorXd graspable_object_pose(6);
    for (int didx = 0; didx < 6; didx++) {
        graspable_object_pose(didx) = scene->getJoint("map_graspable_joint")->getDof(didx)->getPosition();
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    int m_id = 0;
    ros::Duration(1.0).sleep();

    const std::vector<ObjectModel::Feature > &om_f_vec = om->getFeaturesData();
    std::cout << "om_f_vec: " << om_f_vec.size() << std::endl;

    // visualisation

    // publish grasped object model
    publishTransform(br, T_W_O, ob_name, "world");

    m_id = addScene(markers_pub, br, m_id, world, bh);
//    m_id = visualiseQueryDensityFunction(br, markers_pub, m_id, *(qd.get()), "right_HandFingerOneKnuckleThreeLink", KDL::Frame(), T_W_O, ob_name);

//    m_id = visualiseAllFeatures(markers_pub, m_id, om->getFeaturesData(), ob_name);
    markers_pub.publish();
    ros::spinOnce();
    ros::Duration(1.0).sleep();

//    return 0;
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
        std::cout << "sol.score_ " << sol.score_ << std::endl;
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
    outf << "solutions:\t" << solutions.size() << "\t";

    std::sort(solutions.begin(), solutions.end());
    std::reverse(solutions.begin(), solutions.end());

    solutions.resize(solutions.size() / 10);

    std::cout << "reduced solutions to: " << solutions.size() << std::endl;
    outf << "reduced_sol:\t" << solutions.size() << "\t";

    std::vector<GraspSolution > solutions_best(solutions);

    double temperature = 100.0;

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

        temperature = 100.0 * static_cast<double >(steps - 1 - i) / static_cast<double >(steps - 1);
        std::cout << "good_solutions: " << good_solutions << std::endl;
    }

    solutions = solutions_best;
    std::sort(solutions.begin(), solutions.end());
    std::reverse(solutions.begin(), solutions.end());
//    std::cout << solutions[0].score_ << "   " << solutions[1].score_ << "   " << solutions[2].score_ << "   " << solutions[3].score_ << std::endl;

    addScene(markers_pub, br, 0, world, bh);
    markers_pub.publish();
    ros::spinOnce();

    std::list<GraspSpecification > gspec_list;

    for (int sidx = 0; sidx < solutions.size(); sidx++) {
        std::map<std::string, double> q_map(solutions[sidx].q_map_);
        KDL::Frame T_W_E(solutions[sidx].T_W_E_);
        double min_dist = -1.0;
        std::map<std::string, double> min_q_map(solutions[sidx].q_map_);
        KDL::Frame min_T_W_E(solutions[sidx].T_W_E_);
        for (int tryi = 0; tryi < 250; tryi++) {
            if (!checkCollision(world, bh, q_map, T_W_E)) {
                double dist = (T_W_E.p - solutions[sidx].T_W_E_.p).Norm();
                if (min_dist < 0.0 || min_dist > dist) {
                    min_dist = dist;
                    min_q_map = q_map;
                    min_T_W_E = T_W_E;
                }
            }

            std::normal_distribution<> d_conf = std::normal_distribution<>(0, 5.0/180.0*PI);
            q_map = solutions[sidx].q_map_;
            double angleDiffF1 = std::fabs(d_conf(gen)) + 5.0/180.0*PI;
            double angleDiffF2 = std::fabs(d_conf(gen)) + 5.0/180.0*PI;
            double angleDiffF3 = std::fabs(d_conf(gen)) + 5.0/180.0*PI;
            q_map["right_HandFingerOneKnuckleTwoJoint"] -= angleDiffF1;
            q_map["right_HandFingerTwoKnuckleTwoJoint"] -= angleDiffF2;
            q_map["right_HandFingerThreeKnuckleTwoJoint"] -= angleDiffF3;
            q_map["right_HandFingerOneKnuckleThreeJoint"] -= angleDiffF1*0.333333;
            q_map["right_HandFingerTwoKnuckleThreeJoint"] -= angleDiffF2*0.333333;
            q_map["right_HandFingerThreeKnuckleThreeJoint"] -= angleDiffF3*0.333333;
            // apply joint limits
            for (std::map<std::string, double>::iterator it = q_map.begin(); it != q_map.end(); it++) {
                dart::dynamics::Joint *j = bh->getJoint( it-> first );
                it->second = std::max( j->getPositionLowerLimit( 0 ), it->second );
                it->second = std::min( j->getPositionUpperLimit( 0 ), it->second );
            }
            std::normal_distribution<> d_pos = std::normal_distribution<>(0, 0.01);
            T_W_E = KDL::Frame(KDL::Vector(d_pos(gen), d_pos(gen), d_pos(gen))) * solutions[sidx].T_W_E_;
        }

        if (min_dist < 0.0) {
            std::cout << "could not find collision-less pose" << std::endl;
            q_map = solutions[sidx].q_map_;
            T_W_E = solutions[sidx].T_W_E_;
        }
        else {
            std::cout << "found pose with distance: " << min_dist << std::endl;
            q_map = min_q_map;
            T_W_E = min_T_W_E;
            GraspSpecification gspec;
            gspec.solution_idx_ = sidx;
            gspec.T_W_E_ = T_W_E;
            gspec.q_init_map_ = q_map;
            gspec.q_goal_map_ = q_map;
            gspec.q_goal_map_["right_HandFingerOneKnuckleTwoJoint"] += 30.0/180.0*PI;
            gspec.q_goal_map_["right_HandFingerTwoKnuckleTwoJoint"] += 30.0/180.0*PI;
            gspec.q_goal_map_["right_HandFingerThreeKnuckleTwoJoint"] += 30.0/180.0*PI;
            gspec_list.push_back(gspec);
        }

        KDLToEigenTf(T_W_E, tf);
        bh->getJoint(0)->setTransformFromParentBodyNode(tf);
        for (std::map<std::string, double>::const_iterator it = q_map.begin(); it != q_map.end(); it++) {
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
/*
            std::cout << "total cost " << solutions[sidx].score_ << std::endl;
            double cost = hm->getDensity(q_map);
            std::cout << "   cost (hm) " << cost << std::endl;
            for (std::vector<std::string >::const_iterator lit = cm->getLinkNamesCol().begin(); lit != cm->getLinkNamesCol().end(); lit++) {
                KDL::Frame T_E_L;
                getFK(bh, q_map, "right_HandPalmLink", (*lit), T_E_L);
                KDL::Frame T_O_L( T_W_O.Inverse() * T_W_E * T_E_L );
                KDL::Frame T_L_C;
                cm->getT_L_C((*lit), T_L_C);
                double cost_link = qd->getQueryDensity((*lit), T_O_L * T_L_C);
                cost *= cost_link;
                std::cout << "   cost " << (*lit) << "   " << cost_link << std::endl;
            }
*/
        markers_pub.publish();
        ros::spinOnce();
        loop_rate.sleep();
        ros::Duration(0.02).sleep();
        if (!ros::ok()) {
            break;
        }
    }

    outf << "valid_grasps:\t" << gspec_list.size() << "\t";

    scene->enableSelfCollision(true);

    for (std::list<GraspSpecification >::const_iterator gspec_it = gspec_list.begin(); gspec_it != gspec_list.end(); gspec_it++) {
        const GraspSpecification &gspec = (*gspec_it);

        {
                dart::dynamics::Joint::Properties prop = bh->getJoint(0)->getJointProperties();
                dart::dynamics::WeldJoint::Properties prop_weld;
                bh->getRootBodyNode()->changeParentJointType<dart::dynamics::WeldJoint >(prop_weld);
        }

        // set the transform of the gripper and the object
        Eigen::Isometry3d tf;
        KDLToEigenTf(gspec.T_W_E_, tf);
        bh->getJoint(0)->setTransformFromParentBodyNode(tf);

        KDLToEigenTf(T_W_O, tf);
        for (int didx = 0; didx < 6; didx++) {
            scene->getJoint("map_graspable_joint")->getDof(didx)->setPosition(graspable_object_pose(didx));
            scene->getJoint("map_graspable_joint")->getDof(didx)->setVelocity(0);
        }

        // set-up the gripper controller
        GripperController gc;
        double Kc = 200.0;      // 400.0
        double u_max = 30.0;    // 50.0
        double KcDivTi = Kc / 1.0;
        gc.addJoint("right_HandFingerOneKnuckleOneJoint", Kc, KcDivTi, 0.0, 0.001, u_max, true, false);
        gc.addJoint("right_HandFingerOneKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, u_max, false, true);
        gc.addJoint("right_HandFingerTwoKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, u_max, false, true);
        gc.addJoint("right_HandFingerThreeKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, u_max, false, true);
        gc.addJointMimic("right_HandFingerTwoKnuckleOneJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, u_max, true, "right_HandFingerOneKnuckleOneJoint", 1.0, 0.0);
        gc.addJointMimic("right_HandFingerOneKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, u_max, false, "right_HandFingerOneKnuckleTwoJoint", 0.333333, 0.0);
        gc.addJointMimic("right_HandFingerTwoKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, u_max, false, "right_HandFingerTwoKnuckleTwoJoint", 0.333333, 0.0);
        gc.addJointMimic("right_HandFingerThreeKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, u_max, false, "right_HandFingerThreeKnuckleTwoJoint", 0.333333, 0.0);
        gc.setGoalPosition("right_HandFingerOneKnuckleOneJoint", gspec.getGoalPosition("right_HandFingerOneKnuckleOneJoint"));
        gc.setGoalPosition("right_HandFingerOneKnuckleTwoJoint", gspec.getGoalPosition("right_HandFingerOneKnuckleTwoJoint"));
        gc.setGoalPosition("right_HandFingerTwoKnuckleTwoJoint", gspec.getGoalPosition("right_HandFingerTwoKnuckleTwoJoint"));
        gc.setGoalPosition("right_HandFingerThreeKnuckleTwoJoint", gspec.getGoalPosition("right_HandFingerThreeKnuckleTwoJoint"));
        std::map<std::string, double> joint_q_map;
        joint_q_map["right_HandFingerOneKnuckleOneJoint"] = gspec.getInitPosition("right_HandFingerOneKnuckleOneJoint");
        joint_q_map["right_HandFingerTwoKnuckleOneJoint"] = gspec.getInitPosition("right_HandFingerOneKnuckleOneJoint");
        joint_q_map["right_HandFingerOneKnuckleTwoJoint"] = gspec.getInitPosition("right_HandFingerOneKnuckleTwoJoint");
        joint_q_map["right_HandFingerOneKnuckleThreeJoint"] = 0.333333 * gspec.getInitPosition("right_HandFingerOneKnuckleTwoJoint");
        joint_q_map["right_HandFingerTwoKnuckleTwoJoint"] = gspec.getInitPosition("right_HandFingerTwoKnuckleTwoJoint");
        joint_q_map["right_HandFingerTwoKnuckleThreeJoint"] = 0.333333 * gspec.getInitPosition("right_HandFingerTwoKnuckleTwoJoint");
        joint_q_map["right_HandFingerThreeKnuckleTwoJoint"] = gspec.getInitPosition("right_HandFingerThreeKnuckleTwoJoint");
        joint_q_map["right_HandFingerThreeKnuckleThreeJoint"] = 0.333333 * gspec.getInitPosition("right_HandFingerThreeKnuckleTwoJoint");
        for (std::vector<std::string >::const_iterator it = gc.getJointNames().begin(); it != gc.getJointNames().end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint((*it));
            j->setActuatorType(dart::dynamics::Joint::FORCE);
         	j->setPositionLimited(true);
            j->setPosition(0, joint_q_map[(*it)]);
        }

        int counter = 0;
        while (ros::ok()) {
            world->step(false);

            for (std::map<std::string, double>::iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
                dart::dynamics::Joint *j = bh->getJoint(it->first);
                it->second = j->getPosition(0);
            }

            gc.controlStep(joint_q_map);

            // Compute the joint forces needed to compensate for Coriolis forces and
            // gravity
            const Eigen::VectorXd& Cg = bh->getCoriolisAndGravityForces();

            for (std::map<std::string, double>::iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
                dart::dynamics::Joint *j = bh->getJoint(it->first);
                int qidx = j->getIndexInSkeleton(0);
                double u = gc.getControl(it->first);
                double dq = j->getVelocity(0);
                if (!gc.isBackdrivable(it->first)) {
                    j->setPositionLowerLimit(0, std::max(j->getPositionLowerLimit(0), it->second-0.01));
                }

                if (gc.isStopped(it->first)) {
                    j->setPositionLowerLimit(0, std::max(j->getPositionLowerLimit(0), it->second-0.01));
                    j->setPositionUpperLimit(0, std::min(j->getPositionUpperLimit(0), it->second+0.01));
                }
                j->setForce(0, 0.02*(u-dq) + Cg(qidx));
            }
            if (counter % 10 == 0) {
                addScene(markers_pub, br, 0, world, bh);
                markers_pub.publish();
                ros::spinOnce();
            }

            counter++;
            if (counter < 3000) {
            }
            else if (counter == 3000) {
                dart::dynamics::Joint::Properties prop = bh->getJoint(0)->getJointProperties();
                dart::dynamics::FreeJoint::Properties prop_free;
                prop_free.mName = "bh_free";
                prop_free.mT_ParentBodyToJoint = prop.mT_ParentBodyToJoint;
                prop_free.mT_ChildBodyToJoint = prop.mT_ChildBodyToJoint;
                prop_free.mIsPositionLimited = false;
                prop_free.mActuatorType = dart::dynamics::Joint::VELOCITY;
                bh->getRootBodyNode()->changeParentJointType<dart::dynamics::FreeJoint >(prop_free);
                bh->getDof("bh_free_pos_x")->setVelocity(0.0);
                bh->getDof("bh_free_pos_y")->setVelocity(0.0);
                bh->getDof("bh_free_pos_z")->setVelocity(0.0);
            }
            else if (counter < 5000) {
                KDL::Vector vel_W(0, 0, 0.1);
                KDL::Frame T_E_W(gspec.T_W_E_.Inverse());
                KDL::Vector vel_E(KDL::Frame(T_E_W.M) * vel_W);

                bh->getDof("bh_free_pos_x")->setVelocity(vel_E.x());
                bh->getDof("bh_free_pos_y")->setVelocity(vel_E.y());
                bh->getDof("bh_free_pos_z")->setVelocity(vel_E.z());
            }
            else {
                bh->getDof("bh_free_pos_x")->setVelocity(0.0);
                bh->getDof("bh_free_pos_y")->setVelocity(0.0);
                bh->getDof("bh_free_pos_z")->setVelocity(0.0);
                break;
            }
        }
        tf = scene->getBodyNode(ob_name)->getRelativeTransform();
        KDL::Frame T_W_O_after;
        EigenTfToKDL(tf, T_W_O_after);

        bool success(false);
        if (T_W_O_after.p.z() > T_W_O.p.z() + 0.1) {
            success = true;
        }
        outf << "success:\t" << (success?1:0) << "\tscore:\t" << solutions[gspec.solution_idx_].score_ << "\t";
        std::cout << "success: " << (success?1:0) << "   score: " << solutions[gspec.solution_idx_].score_ << std::endl;
    }
}

