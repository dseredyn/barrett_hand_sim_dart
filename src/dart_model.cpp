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

const double PI(3.141592653589793);

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
                    grid_->setLeafSize(0.003, 0.003, 0.003);
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
                    principalCurvaturesEstimation.setRadiusSearch(0.005);

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

/*
    // visualisation of the point clouds and the curvatures
    for (int skidx = 0; skidx < world->getNumSkeletons(); skidx++) {
        dart::dynamics::SkeletonPtr sk = world->getSkeleton(skidx);
        if (sk->getName() == bh->getName()) {
            continue;
        }
        for (int bidx = 0; bidx < sk->getNumBodyNodes(); bidx++) {
            const std::string &link_name = sk->getBodyNode(bidx)->getName();
            if (point_clouds_map.find( link_name ) == point_clouds_map.end()) {
                continue;
            }
            pcl::PointCloud<pcl::PointNormal>::Ptr res = point_clouds_map[link_name];
            pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures = point_pc_clouds_map[link_name];
            KDL::Frame T_W_S = frames_map[link_name];

            for (int pidx = 0; pidx < res->points.size(); pidx++) {
                KDL::Vector v1 = T_W_S * KDL::Vector(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z);
                KDL::Vector v2 = T_W_S * (*features_map[link_name])[pidx] * KDL::Vector(0, 0, 0.01);
                KDL::Vector v3 = T_W_S * (*features_map[link_name])[pidx] * KDL::Vector(0.01, 0, 0);
//                double f = principalCurvatures->points[pidx].pc1 * 4.0;
//                m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, "world");
//                m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, "world");
//                m_id = markers_pub.addSinglePointMarkerCube(m_id, v1, f, 1, f, 1, 0.001, 0.001, 0.001, "world");
                m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(principalCurvatures->points[pidx].pc1, principalCurvatures->points[pidx].pc2, 0), 0, 1, 0, 1, 0.001, 0.001, 0.001, "world");
            }
        }
    }
    markers_pub.publish();

    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
//*/

    // generate object model
    const std::string &ob_name = domino->getRootBodyNode()->getName();
    ObjectModel om;
    om.res = point_clouds_map[ob_name];
    om.principalCurvatures = point_pc_clouds_map[ob_name];
    om.grid_ = grids_map[ob_name];
    om.features_ = features_map[ob_name];
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
                            om.res, om.principalCurvatures, T_W_O, features_map[ob_name]);
    }

    cm.buildFeatureMaps();

    om.randomizeSurface();

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

/*
    // visuzlise all features on the object
    for (int idx = 0; idx < (*om.features_).size(); idx++) {
        const KDL::Frame &T_O_F = (*om.features_)[idx];
        KDL::Vector v1 = T_O_F * KDL::Vector();
        KDL::Vector v2 = T_O_F * KDL::Vector(0, 0, 0.01);
        KDL::Vector v3 = T_O_F * KDL::Vector(0.01, 0, 0);
        m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, ob_name);
        m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, ob_name);
    }
    markers_pub.addEraseMarkers(m_id, m_id+300);
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
//*
//*
    // visualise all features
    for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
        const std::string &link_name = bh->getBodyNode(bidx)->getName();
        if (cm.link_features_map.find( link_name ) == cm.link_features_map.end()) {
            continue;
        }
        KDL::Frame T_W_L = frames_map[link_name];
        for (int fidx = 0; fidx < cm.link_features_map[link_name].size(); fidx++) {
            KDL::Frame T_W_F = T_W_L * cm.link_features_map[link_name][fidx].T_L_F;
            KDL::Vector v1 = T_W_F * KDL::Vector();
            KDL::Vector v2 = T_W_F * KDL::Vector(0, 0, 0.01);
            KDL::Vector v3 = T_W_F * KDL::Vector(0.01, 0, 0);
            double f = cm.link_features_map[link_name][fidx].pc1 * 4.0;
            //double f = cm.link_features_map[link_name][fidx].dist;
            m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, "world");
            m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, "world");
            m_id = markers_pub.addSinglePointMarkerCube(m_id, v1, f, 1, f, 1, 0.001, 0.001, 0.001, "world");
        }
    }
    markers_pub.addEraseMarkers(m_id, m_id+300);
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
//*/

/*
    for (std::map<std::string, double>::const_iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        
        j->getPositionLowerLimit(0)
        j->setPosition( 0, it->second );
    }
*/

/*
    // TEST: finding features
    for (double x = -0.1; x < 0.1; x+=0.005) {
        for (double y = -0.1; y < 0.1; y+=0.005) {
            for (double z = -0.1; z < 0.1; z+=0.005) {
                KDL::Frame f(KDL::Rotation::RotX(45.0/180.0*PI) * KDL::Rotation::RotZ(180.0/180.0*PI), KDL::Vector(x, y, z));
                if (om.findFeature(0.3, 0.0, 0.1, 0.1, f, 0.005, 20.0/180.0*PI)) {
//                    std::cout << "found" << std::endl;
                    m_id = markers_pub.addSinglePointMarkerCube(m_id, f.p, 0, 0, 1, 1, 0.001, 0.001, 0.001, ob_name);
                }
            }
        }
    }
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }
*/

    std::random_device rd;
    std::mt19937 gen(rd());

/*
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
        loop_rate.sleep();
    }
    return 0;
//*/

/*
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
        loop_rate.sleep();
    }
    return 0;
//*/

    std::vector<double > weights(om.res->points.size());

    const double sigma_p = 0.01;
    const double sigma_q = 100.0;
    const double sigma_r = 0.01;
/*
    double f_max = uniVariateIsotropicGaussianKernel(0.0, 0.0, sigma_p);
    for (double x = 0; x < 10.0; x += 0.001) {
        double f = uniVariateIsotropicGaussianKernel(x, 0.0, sigma_p);
        if (f < f_max * 0.1) {
            std::cout << "f_max " << f_max << "   f_min " << f << "   x " << x << std::endl;
            break;
        }
    }

    return 0;
*/
    om.setSamplerParameters(sigma_p, sigma_q, sigma_r);
    cm.setSamplerParameters(sigma_p, sigma_q, sigma_r);

    const int qd_sample_count = 5000;
    for (std::vector<std::string >::const_iterator lit = cm.getLinkNamesCol().begin(); lit !=  cm.getLinkNamesCol().end(); lit++) {
        const std::string &link_name = (*lit);
        std::cout << "generating query density for: " << link_name << std::endl;
        std::vector<CollisionModel::QueryDensityElement > qd_vec(qd_sample_count);
        double sum_weights = 0.0;
        for (int i = 0; i < qd_sample_count; i++) {
            Eigen::Vector3d p, p2;
            Eigen::Vector4d q, q2;
            Eigen::Vector2d r;
            om.sample(p, q, r);

            if (!cm.sampleForR(link_name, r, p2, q2)) {
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
        for (int i = 0; i < qd_sample_count; i++) {
            qd_vec[i].weight_ /= sum_weights;
        }
        cm.addQueryDensity(link_name, qd_vec);
    }

/*
    // visualisation of query density
    const std::string link_name_test("right_HandFingerTwoKnuckleThreeLink");
    const std::vector<CollisionModel::QueryDensityElement > &qd_vec = cm.qd_map_[link_name_test];

    for (std::vector<CollisionModel::QueryDensityElement >::const_iterator it = qd_vec.begin(); it != qd_vec.end(); it++) {
        // visualisation
        KDL::Frame fr(KDL::Rotation::Quaternion(it->q_(0), it->q_(1), it->q_(2), it->q_(3)), KDL::Vector(it->p_(0), it->p_(1), it->p_(2)));//KDL::Frame(KDL::Rotation::Quaternion(q(0), q(1), q(2), q(3)), KDL::Vector(p(0), p(1), p(2)));

        double w = it->weight_ * 2000.0;
        std::cout << it->weight_ << std::endl;
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0.01, 0, 0), 1, 0, 0, 1, 0.0002, ob_name);
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0, 0.01, 0), 0, 1, 0, 1, 0.0002, ob_name);
//        m_id = markers_pub.addVectorMarker(m_id, fr * KDL::Vector(), fr * KDL::Vector(0, 0, 0.01), 0, 0, 1, 1, 0.0002, ob_name);
//        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(p(0), p(1), p(2)), r(0)*4, 0, r(1)*4, 1, 0.001, 0.001, 0.001, ob_name);
        m_id = markers_pub.addSinglePointMarkerCube(m_id, fr * KDL::Vector(), w, w, w, 1, 0.001, 0.001, 0.001, ob_name);
//        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(result_x, result_y, result_z), sum/10000000000.0, 0, 0, 1, 0.001, 0.001, 0.001, ob_name);
//        std::cout << i << std::endl;
    }
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    for (int i = 0; i < 100; i++) {
        int idx = rand() % qd_vec.size();
        KDL::Frame T_O_L(KDL::Rotation::Quaternion(qd_vec[idx].q_(0), qd_vec[idx].q_(1), qd_vec[idx].q_(2), qd_vec[idx].q_(3)), KDL::Vector(qd_vec[idx].p_(0), qd_vec[idx].p_(1), qd_vec[idx].p_(2)));
        publishTransform(br, T_W_O * T_O_L, link_name_test, "world");
        std::cout << qd_vec[idx].weight_ << std::endl;
        ros::spinOnce();
        ros::Duration(2.0).sleep();
    }

    return 0;
//*/

    double cost_max = 0.0;
    KDL::Frame T_W_E_best;
    std::map<std::string, double> q_best;
    for (int  i = 0; i < 40000; i++) {
        const std::string &link_name = cm.getRandomLinkNameCol();
        KDL::Frame T_O_L1;
        cm.sampleQueryDensity(link_name, T_O_L1);
        const std::map<std::string, double>& q_sample = hm.sample();

        double cost = cm.getQueryDensity(link_name, T_O_L1);

        for (std::vector<std::string >::const_iterator lit = cm.getLinkNamesCol().begin(); lit != cm.getLinkNamesCol().end(); lit++) {
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

    // Position its base in a reasonable way
    KDLToEigenTf(T_W_E_best, tf);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);

    for (std::map<std::string, double>::const_iterator it = q_best.begin(); it != q_best.end(); it++) {
        dart::dynamics::Joint *j = bh->getJoint( it-> first );
        j->setPosition( 0, it->second );
    }

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

    return 0;
//*
    // visualize the density pdf(p)
    for (double x = -0.07; x < 0.07; x += 0.002) {
        for (double y = -0.07; y < 0.07; y += 0.002) {
            double sum = 0.0;
            for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
                Eigen::Vector3d xx(x, y, 0.0);
                Eigen::Vector3d xj(om.res->points[pidx].x, om.res->points[pidx].y, om.res->points[pidx].z);
                sum += triVariateIsotropicGaussianKernel(xx, xj, 0.01);
            }
            sum /= om.res->points.size();
            m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(x, y, 0.0), 0, 0, 1, 1, 0.005*sum, 0.005*sum, 0.005*sum, ob_name);
        }
    }
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
//*/
}

