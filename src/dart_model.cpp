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

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/marker_publisher.h"

#include <dart/dart.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/features/principal_curvatures.h>

#include "mesh_sampling.h"

void EigenTfToKDL(const Eigen::Isometry3d &tf, KDL::Frame &kdlT) {
    kdlT = KDL::Frame( KDL::Rotation(tf(0,0),tf(0,1),tf(0,2), tf(1,0), tf(1,1), tf(1,2), tf(2,0), tf(2,1), tf(2,2)), KDL::Vector(tf(0,3), tf(1,3), tf(2,3)) );
}

void KDLToEigenTf(const KDL::Frame &kdlT, Eigen::Isometry3d &tf) {
    double qx, qy, qz, qw;
    kdlT.M.GetQuaternion(qx, qy, qz, qw);
    tf.fromPositionOrientationScale(Eigen::Vector3d(kdlT.p.x(), kdlT.p.y(), kdlT.p.z()), Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(1.0, 1.0, 1.0));
}

void publishTransform(tf::TransformBroadcaster &br, const KDL::Frame &T_B_F, const std::string &frame_id, const std::string &base_frame_id) {
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(T_B_F.p.x(), T_B_F.p.y(), T_B_F.p.z()) );
        tf::Quaternion q;
        double qx, qy, qz, qw;
        T_B_F.M.GetQuaternion(q[0], q[1], q[2], q[3]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, frame_id));
}

class Feature {
public:
    KDL::Frame T_L_F;
    double pc1, pc2;
    double dist;
};

class CollisionModel {
public:
    std::map<std::string, std::vector<Feature > > link_features_map;

};

int main(int argc, char** argv) {
    const double PI(3.141592653589793);

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
    std::map<std::string, std::vector<KDL::Frame > > features_map;
    for (int skidx = 0; skidx < world->getNumSkeletons(); skidx++) {
        dart::dynamics::SkeletonPtr sk = world->getSkeleton(skidx);

        for (int bidx = 0; bidx < sk->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = sk->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
            std::cout << b->getName() << "   " << b->getNumCollisionShapes() << std::endl;
            for (int cidx = 0; cidx < b->getNumCollisionShapes(); cidx++) {
                dart::dynamics::ConstShapePtr sh = b->getCollisionShape(cidx);
                if (sh->getShapeType() == dart::dynamics::Shape::MESH) {
                    std::shared_ptr<const dart::dynamics::MeshShape > msh = std::static_pointer_cast<const dart::dynamics::MeshShape >(sh);
                    const Eigen::Isometry3d &tf = sh->getLocalTransform();
                    KDL::Frame T_L_S;
                    EigenTfToKDL(tf, T_L_S);

                    const aiScene *sc = msh->getMesh();
                    for (int midx = 0; midx < sc->mNumMeshes; midx++) {
                        std::cout << "v: " << sc->mMeshes[midx]->mNumVertices << "   f: " << sc->mMeshes[midx]->mNumFaces << std::endl;
                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
                        uniform_sampling(sc->mMeshes[midx], 100000, *cloud_1);
                        // Voxelgrid
                        pcl::VoxelGrid<pcl::PointNormal> grid_;
                        pcl::PointCloud<pcl::PointNormal>::Ptr res(new pcl::PointCloud<pcl::PointNormal>);
                        grid_.setDownsampleAllData(true);
                        grid_.setSaveLeafLayout(true);
                        grid_.setInputCloud(cloud_1);
                        grid_.setLeafSize(0.003, 0.003, 0.003);
                        grid_.filter (*res);
                        point_clouds_map[b->getName()] = res;
                        frames_map[b->getName()] = T_W_L * T_L_S;

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
                        point_pc_clouds_map[b->getName()] = principalCurvatures;

                        features_map[b->getName()].resize( res->points.size() );

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
                            features_map[b->getName()][pidx] = KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z) );
                        }
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
                KDL::Vector v2 = T_W_S * features_map[link_name][pidx] * KDL::Vector(0, 0, 0.01);
                KDL::Vector v3 = T_W_S * features_map[link_name][pidx] * KDL::Vector(0.01, 0, 0);
//                KDL::Vector v2 = T_W_S * (KDL::Vector(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z) + 0.01 * KDL::Vector(res->points[pidx].normal[0], res->points[pidx].normal[1], res->points[pidx].normal[2]));
//                KDL::Vector v3 = T_W_S * (KDL::Vector(res->points[pidx].x, res->points[pidx].y, res->points[pidx].z) + 0.01 * KDL::Vector(principalCurvatures->points[pidx].principal_curvature[0], principalCurvatures->points[pidx].principal_curvature[1], principalCurvatures->points[pidx].principal_curvature[2]));
                double f = principalCurvatures->points[pidx].pc1 * 4.0;
                m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, "world");
                m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, "world");
                m_id = markers_pub.addSinglePointMarkerCube(m_id, v1, f, 1, f, 1, 0.001, 0.001, 0.001, "world");
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

    // generate collision model
    const std::string &ob_name = domino->getRootBodyNode()->getName();
    pcl::PointCloud<pcl::PointNormal>::Ptr ob_res = point_clouds_map[ob_name];
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr ob_principalCurvatures = point_pc_clouds_map[ob_name];
    T_W_O = frames_map[ob_name];

    std::map<std::string, std::list<std::pair<int, double> > > link_pt_map;
    CollisionModel cm;

    double dist_range = 0.01;
    for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
        const std::string &link_name = bh->getBodyNode(bidx)->getName();
        if (point_clouds_map.find( link_name ) == point_clouds_map.end()) {
            continue;
        }
        pcl::PointCloud<pcl::PointNormal>::Ptr res = point_clouds_map[link_name];
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures = point_pc_clouds_map[link_name];
        KDL::Frame T_W_S = frames_map[link_name];

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
                link_pt_map[link_name].push_back( std::make_pair(poidx, min_dist) );
            }
        }
        if ( link_pt_map[link_name].size() > 0 ) {
            cm.link_features_map[link_name].resize( link_pt_map[link_name].size() );
            int fidx = 0;
            for (std::list<std::pair<int, double> >::const_iterator it = link_pt_map[link_name].begin(); it != link_pt_map[link_name].end(); it++, fidx++) {
                int poidx = it->first;
                cm.link_features_map[link_name][fidx].pc1 = ob_principalCurvatures->points[poidx].pc1;
                cm.link_features_map[link_name][fidx].pc2 = ob_principalCurvatures->points[poidx].pc2;
                KDL::Frame T_W_F = T_W_O * features_map[ob_name][poidx];
                cm.link_features_map[link_name][fidx].T_L_F = T_W_S.Inverse() * T_W_F;
                cm.link_features_map[link_name][fidx].dist = it->second / dist_range;
            }
        }        
    }

//*
    // visualise the features
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
    markers_pub.publish();

    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }
//*/

    // get a random point on the surface of the object
    int rand_poidx = rand() % ob_res->points.size();

    return 0;
}

