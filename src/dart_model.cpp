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

void generateQueryDensity(int seed, const std::string &link_name, const CollisionModel &cm, const ObjectModel &om,
                            std::vector<CollisionModel::QueryDensityElement > &qd_vec) {
        double sum_weights = 0.0;
        for (int i = 0; i < qd_vec.size(); i++) {
            Eigen::Vector3d p, p2;
            Eigen::Vector4d q, q2;
            Eigen::Vector2d r;
            om.sample(seed, p, q, r);

            if (!cm.sampleForR(seed, link_name, r, p2, q2)) {
                std::cout << "ERROR: cm.sampleForR" << std::endl;
            }
            double weight = cm.getMarginalDensityForR(link_name, r);
            KDL::Frame T_O_F( KDL::Frame(KDL::Rotation::Quaternion(q(0), q(1), q(2), q(3)), KDL::Vector(p(0), p(1), p(2))) );
            KDL::Frame T_L_F( KDL::Frame(KDL::Rotation::Quaternion(q2(0), q2(1), q2(2), q2(3)), KDL::Vector(p2(0), p2(1), p2(2))) );
            KDL::Frame T_O_L = T_O_F * T_L_F.Inverse();

            qd_vec[i].p_ = Eigen::Vector3d(T_O_L.p.x(), T_O_L.p.y(), T_O_L.p.z());
            double qx, qy, qz, qw;
            T_O_L.M.GetQuaternion(qx, qy, qz, qw);
            qd_vec[i].q_ = Eigen::Vector4d(qx, qy, qz, qw);
            qd_vec[i].weight_ = weight;
            sum_weights += qd_vec[i].weight_;
        }
        // normalize the weights
        for (int i = 0; i < qd_vec.size(); i++) {
            qd_vec[i].weight_ /= sum_weights;
        }
}

int main(int argc, char** argv) {

    srand ( time(NULL) );

    // read grasp state from the input
    double px, py, pz, qx, qy, qz, qw;
    std::cin >> px >> py >> pz >> qx >> qy >> qz >> qw;

    KDL::Frame T_E_O( KDL::Rotation::Quaternion(qx, qy, qz, qw), KDL::Vector(px, py, pz) );
    T_E_O.M.GetQuaternion(qx, qy, qz, qw);
    std::cout << T_E_O.p.x() << " " << T_E_O.p.y() << " " << T_E_O.p.z() << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;

    std::map<std::string, double> joint_q_map;
    for (int i = 0; i < 8; i++) {
        std::string joint_name;
        double q;
        std::cin >> joint_name >> q;
        joint_q_map[joint_name] = q;
        std::cout << joint_name << " " << q << std::endl;
    }

    std::string object_urdf;
    std::cin >> object_urdf;

    std::cout << object_urdf << std::endl;

    ros::init(argc, argv, "dart_test");

    ros::NodeHandle nh_;
        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

    ros::Rate loop_rate(400);

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

    dart::dynamics::SkeletonPtr domino( loader.parseSkeleton(package_path_sim + object_urdf) );
    KDLToEigenTf(KDL::Frame( KDL::Vector(0.0, 0.0, 0.07) ), tf);
    domino->getJoint(0)->setTransformFromParentBodyNode(tf);

    dart::simulation::World* world = new dart::simulation::World();

    world->addSkeleton(bh);
    world->addSkeleton(domino);

    Eigen::Vector3d grav(0,0,0);
    world->setGravity(grav);

    for (std::map<std::string, double>::const_iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
    }

    tf = bh->getBodyNode("right_HandPalmLink")->getTransform();
    KDL::Frame T_W_E;
    EigenTfToKDL(tf, T_W_E);
    KDL::Frame T_W_O = T_W_E * T_E_O;

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

    // calculate point clouds for all links and for the grasped object
    std::map<std::string, pcl::PointCloud<pcl::PointNormal>::Ptr > point_clouds_map;
    std::map<std::string, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr > point_pc_clouds_map;
    std::map<std::string, KDL::Frame > frames_map;
    std::map<std::string, boost::shared_ptr<std::vector<KDL::Frame > > > features_map;
    std::map<std::string, boost::shared_ptr<pcl::VoxelGrid<pcl::PointNormal> > > grids_map;
    for (int skidx = 0; skidx < world->getNumSkeletons(); skidx++) {
        dart::dynamics::SkeletonPtr sk = world->getSkeleton(skidx);

        for (int bidx = 0; bidx < sk->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = sk->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            const std::string &body_name = b->getName();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
            std::cout << body_name << "   " << b->getNumCollisionShapes() << std::endl;
            for (int cidx = 0; cidx < b->getNumCollisionShapes(); cidx++) {
                dart::dynamics::ConstShapePtr sh = b->getCollisionShape(cidx);
                if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                    std::shared_ptr<const dart::dynamics::MeshShape > msh = std::static_pointer_cast<const dart::dynamics::MeshShape >(sh);
                    std::cout << "mesh path: " << msh->getMeshPath() << std::endl;
                    const Eigen::Isometry3d &tf = sh->getLocalTransform();
                    KDL::Frame T_L_S;
                    EigenTfToKDL(tf, T_L_S);
                    KDL::Frame T_S_L = T_L_S.Inverse();

                    const aiScene *sc = msh->getMesh();
                    if (sc->mNumMeshes != 1) {
                        std::cout << "ERROR: sc->mNumMeshes = " << sc->mNumMeshes << std::endl;
                    }
                    int midx = 0;
//                    std::cout << "v: " << sc->mMeshes[midx]->mNumVertices << "   f: " << sc->mMeshes[midx]->mNumFaces << std::endl;
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
                    uniform_sampling(sc->mMeshes[midx], 100000, *cloud_1);
                    for (int pidx = 0; pidx < cloud_1->points.size(); pidx++) {
                        KDL::Vector pt_L = T_L_S * KDL::Vector(cloud_1->points[pidx].x, cloud_1->points[pidx].y, cloud_1->points[pidx].z);
                        cloud_1->points[pidx].x = pt_L.x();
                        cloud_1->points[pidx].y = pt_L.y();
                        cloud_1->points[pidx].z = pt_L.z();
                    }
                    // Voxelgrid
                    boost::shared_ptr<pcl::VoxelGrid<pcl::PointNormal> > grid_(new pcl::VoxelGrid<pcl::PointNormal>);
                    pcl::PointCloud<pcl::PointNormal>::Ptr res(new pcl::PointCloud<pcl::PointNormal>);
                    grid_->setDownsampleAllData(true);
                    grid_->setSaveLeafLayout(true);
                    grid_->setInputCloud(cloud_1);
                    grid_->setLeafSize(0.004, 0.004, 0.004);
                    grid_->filter (*res);
                    point_clouds_map[body_name] = res;
                    frames_map[body_name] = T_W_L;// * T_L_S;
                    grids_map[body_name] = grid_;

                    std::cout << "res->points.size(): " << res->points.size() << std::endl;

                    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);

                    // Setup the principal curvatures computation
                    pcl::PrincipalCurvaturesEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

                    // Provide the original point cloud (without normals)
                    principalCurvaturesEstimation.setInputCloud (res);

                    // Provide the point cloud with normals
                    principalCurvaturesEstimation.setInputNormals(res);

                    // Use the same KdTree from the normal estimation
                    principalCurvaturesEstimation.setSearchMethod (tree);
                    principalCurvaturesEstimation.setRadiusSearch(0.006);

                    // Actually compute the principal curvatures
                    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
                    principalCurvaturesEstimation.compute (*principalCurvatures);
                    point_pc_clouds_map[body_name] = principalCurvatures;

                    features_map[body_name].reset( new std::vector<KDL::Frame >(res->points.size()) );
                    for (int pidx = 0; pidx < res->points.size(); pidx++) {
                        KDL::Vector nx, ny, nz(res->points[pidx].normal[0], res->points[pidx].normal[1], res->points[pidx].normal[2]);
                        if ( std::fabs( principalCurvatures->points[pidx].pc1 - principalCurvatures->points[pidx].pc2 ) > 0.001) {
                            nx = KDL::Vector(principalCurvatures->points[pidx].principal_curvature[0], principalCurvatures->points[pidx].principal_curvature[1], principalCurvatures->points[pidx].principal_curvature[2]);
                        }
                        else {
                            if (std::fabs(nz.z()) < 0.7) {
                                nx = KDL::Vector(0, 0, 1);
                            }
                            else {
                                nx = KDL::Vector(1, 0, 0);
                            }
                        }
                        ny = nz * nx;
                        nx = ny * nz;
                        nx.Normalize();
                        ny.Normalize();
                        nz.Normalize();
                        (*features_map[body_name])[pidx] = KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z) );
                    }
                }
            }
        }
    }

    int m_id = 101;

    // generate object model
    const std::string &ob_name = domino->getRootBodyNode()->getName();
    ObjectModel om;

    for (int pidx = 0; pidx < point_clouds_map[ob_name]->points.size(); pidx++) {
        if (point_pc_clouds_map[ob_name]->points[pidx].pc1 > 1.1 * point_pc_clouds_map[ob_name]->points[pidx].pc2) {
            // e.g. pc1=1, pc2=0
            // edge
            om.addPointFeature((*features_map[ob_name])[pidx] * KDL::Frame(KDL::Rotation::RotZ(PI)), point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
            om.addPointFeature((*features_map[ob_name])[pidx], point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
        }
        else {
            for (double angle = 0.0; angle < 359.0/180.0*PI; angle += 45.0/180.0*PI) {
                om.addPointFeature((*features_map[ob_name])[pidx] * KDL::Frame(KDL::Rotation::RotZ(angle)), point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
            }
        }
    }

    std::cout << "om.getPointFeatures().size(): " << om.getPointFeatures().size() << std::endl;
//    om.res = point_clouds_map[ob_name];
//    om.principalCurvatures = point_pc_clouds_map[ob_name];
//    om.features_ = features_map[ob_name];
    T_W_O = frames_map[ob_name];

    // generate collision model
    std::map<std::string, std::list<std::pair<int, double> > > link_pt_map;
    CollisionModel cm;
    cm.joint_q_map_ = joint_q_map;
    cm.frames_map_ = frames_map;

    std::list<std::string > gripper_link_names;
    for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
        const std::string &link_name = bh->getBodyNode(bidx)->getName();
        gripper_link_names.push_back(link_name);
    }

    double dist_range = 0.01;
    for (std::list<std::string >::const_iterator nit = gripper_link_names.begin(); nit != gripper_link_names.end(); nit++) {
        const std::string &link_name = (*nit);
        if (point_clouds_map.find( link_name ) == point_clouds_map.end()) {
            continue;
        }
        cm.addLinkContacts(dist_range, link_name, point_clouds_map[link_name], frames_map[link_name],
                            om.getPointFeatures(), T_W_O);
    }

    // generate hand configuration model
    HandConfigurationModel hm;
    std::map<std::string, double> joint_q_map_before = joint_q_map;

    double angleDiffKnuckleTwo = 15.0/180.0*PI;
    joint_q_map_before["right_HandFingerOneKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerTwoKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerThreeKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerOneKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;
    joint_q_map_before["right_HandFingerTwoKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;
    joint_q_map_before["right_HandFingerThreeKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;

    hm.generateModel(joint_q_map_before, joint_q_map, 1.0, 30, 0.05);

//    m_id = visualiseContactRegion( markers_pub, m_id, cm.link_features_map["right_HandFingerTwoKnuckleThreeLink"], T_W_E * cm.getTransform("right_HandPalmLink", "right_HandFingerTwoKnuckleThreeLink") );
//    m_id = visualiseAllFeatures(markers_pub, m_id, om.getPointFeatures(), ob_name);
//    m_id = visualiseRejectionSamplingVonMisesFisher3(markers_pub, m_id);
//    m_id = visualiseRejectionSamplingVonMisesFisher4(markers_pub, m_id);

    std::vector<double > weights(om.getPointFeatures().size());

    const double sigma_p = 0.01;
    const double sigma_q = 100.0;
    const double sigma_r = 0.05;

    std::random_device rd;
    std::mt19937 gen(rd());

    om.setSamplerParameters(sigma_p, sigma_q, sigma_r);
    cm.setSamplerParameters(sigma_p, sigma_q, sigma_r);

    // generate query density using multiple threads
    {
        const int qd_sample_count = 50000;
        const int num_threads = cm.getLinkNamesCol().size();
        std::unique_ptr<std::thread[] > t(new std::thread[num_threads]);
        std::unique_ptr<std::vector<CollisionModel::QueryDensityElement >[] > qd_vec(new std::vector<CollisionModel::QueryDensityElement >[num_threads]);

        for (int thread_id = 0; thread_id < num_threads; thread_id++) {
            qd_vec[thread_id].resize(qd_sample_count);
            std::vector<CollisionModel::QueryDensityElement > aaa;
            t[thread_id] = std::thread(generateQueryDensity, gen(), std::cref(cm.getLinkNamesCol()[thread_id]), std::cref(cm), std::cref(om), std::ref(qd_vec[thread_id]));
        }

        for (int thread_id = 0; thread_id < num_threads; ++thread_id) {
            t[thread_id].join();
            cm.addQueryDensity(cm.getLinkNamesCol()[thread_id], qd_vec[thread_id]);
        }
    }

//    m_id = visualiseQueryDensityParticles(markers_pub, m_id, cm.qd_map_["right_HandFingerTwoKnuckleThreeLink"], ob_name);
//    m_id = visualiseQueryDensityFunction(br, markers_pub, m_id, cm, "right_HandFingerTwoKnuckleThreeLink", T_W_O, ob_name);

    double cost_max = 0.0;
    KDL::Frame T_W_E_best;
    std::map<std::string, double> q_best;
    for (int  i = 0; i < 4000; i++) {
        const std::string &link_name = cm.getRandomLinkNameCol();
        KDL::Frame T_O_L1;
        cm.sampleQueryDensity(gen(), link_name, T_O_L1);
        const std::map<std::string, double>& q_sample = hm.sample();

        double cost = cm.getQueryDensity(link_name, T_O_L1);
        for (std::vector<std::string >::const_iterator lit = cm.getLinkNamesCol().begin(); lit != cm.getLinkNamesCol().end(); lit++) {
            if (cost < 0.0000001) {
                cost = 0.0;
                break;
            }
            if (link_name == (*lit)) {
                continue;
            }
            KDL::Frame T_L1_L2;
            getFK(bh, q_sample, link_name, (*lit), T_L1_L2);
            KDL::Frame T_O_L2( T_O_L1 * T_L1_L2 );
            cost *= cm.getQueryDensity((*lit), T_O_L2);
        }

        KDL::Frame T_L1_E;
        getFK(bh, q_sample, link_name, "right_HandPalmLink", T_L1_E);

        if (cost > cost_max) {
            cost_max = cost;
            T_W_E_best = T_W_O * T_O_L1 * T_L1_E;
            q_best = q_sample;
        }

        if ((i % 1000) == 0) {
            std::cout << i << "   " << cost << std::endl;
        }
    }

    std::cout << "best: " << cost_max << std::endl;

    double temperature = 100.0;

    // visualisation
    for (int i = 0; i < 100000; i++) {

        // Position its base in a reasonable way
        KDLToEigenTf(T_W_E_best, tf);
        bh->getJoint(0)->setTransformFromParentBodyNode(tf);
        for (std::map<std::string, double>::const_iterator it = q_best.begin(); it != q_best.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint( it-> first );
            j->setPosition( 0, it->second );
        }

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
        ros::Duration(0.02).sleep();
        if (!ros::ok()) {
            break;
        }

        Eigen::Vector4d qq, qq_res;
        T_W_E_best.M.GetQuaternion(qq(0), qq(1), qq(2), qq(3));

        Eigen::Vector3d axis;
        randomUnitSphere(axis);
        std::normal_distribution<> d = std::normal_distribution<>(0, 0.5/180.0*PI);
        KDL::Rotation rot( T_W_E_best.M * KDL::Rotation::Rot(KDL::Vector(axis(0), axis(1), axis(2)), d(gen)) );
//        double sigma_q2 = 100.0;
//        double Cp = misesFisherKernelConstant(sigma_q2, 4);
//        double pdf_mean = misesFisherKernel(qq, qq, sigma_q2, Cp);
//        vonMisesFisherSample(qq, pdf_mean, sigma_q2, Cp, qq_res);
        d = std::normal_distribution<>(0, 0.002);
//        KDL::Frame T_W_E_new( KDL::Rotation::Quaternion(qq_res(0), qq_res(1), qq_res(2), qq_res(3)), T_W_E_best.p + KDL::Vector(d(gen), d(gen), d(gen)));
        KDL::Frame T_W_E_new( rot, T_W_E_best.p + KDL::Vector(d(gen), d(gen), d(gen)));

        d = std::normal_distribution<>(0, 2.0/180.0*PI);
        std::map<std::string, double> q_new( q_best );
//        const std::map<std::string, double>& q_new = hm.sample();
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

        double cost = 1.0;
        for (std::vector<std::string >::const_iterator lit = cm.getLinkNamesCol().begin(); lit != cm.getLinkNamesCol().end(); lit++) {
            if (cost < 0.0000001) {
                cost = 0.0;
                break;
            }
            KDL::Frame T_E_L;
            getFK(bh, q_new, "right_HandPalmLink", (*lit), T_E_L);
            KDL::Frame T_O_L( T_W_O.Inverse() * T_W_E_new * T_E_L );
            cost *= cm.getQueryDensity((*lit), T_O_L);
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

