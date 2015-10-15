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

const double PI(3.141592653589793);

double getAngle(const KDL::Vector &v1, const KDL::Vector &v2) {
    return std::atan2((v1*v2).Norm(), KDL::dot(v1,v2));
}

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
    std::map<std::string, std::vector<Feature > > link_features_map_edge;
    std::map<std::string, std::vector<Feature > > link_features_map_sym;

    std::map<std::string, double> joint_q_map_;
    std::map<std::string, KDL::Frame > frames_map_;
    std::map<std::pair<std::string, std::string>, std::list<KDL::Frame> > frames_rel_map_;

    void buildFeatureMaps() {
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

    bool getRandomFeatureForLink(const std::string &link_name, Feature &feature) const {
        std::map<std::string, std::vector<Feature > >::const_iterator it_edge = link_features_map_edge.find(link_name);
        std::map<std::string, std::vector<Feature > >::const_iterator it_sym = link_features_map_sym.find(link_name);
        if (it_edge == link_features_map_edge.end()) {
            std::cout << "ERROR: getRandomFeatureForLink: it_edge == link_features_map_edge.end()" << std::endl;
            return false;
        }
        if (it_sym == link_features_map_sym.end()) {
            std::cout << "ERROR: getRandomFeatureForLink: it_sym == link_features_map_sym.end()" << std::endl;
            return false;
        }

        int idx = rand() % (it_edge->second.size() * 2 + it_sym->second.size() * 36);
        if (idx < it_edge->second.size()*2) {
            idx = idx / 2;
            feature = it_edge->second[idx];
            if ((rand() % 2) == 0) {
                feature.T_L_F = feature.T_L_F * KDL::Frame( KDL::Rotation::RotZ(PI) );
            }
        }
        else {
            idx -= it_edge->second.size()*2;
            idx = idx / 36;
            feature = it_sym->second[idx];
            feature.T_L_F = feature.T_L_F * KDL::Frame( KDL::Rotation::RotZ(randomUniform(-PI, PI)) );
        }
        return true;
    }

    bool getRandomFeature(std::string &link_name, Feature &feature) const {
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

    void getTransform(const std::string &link1_name, const std::string &link2_name, KDL::Frame &T_L1_L2) const {
        std::map<std::string, KDL::Frame >::const_iterator it1( frames_map_.find(link1_name) );
        std::map<std::string, KDL::Frame >::const_iterator it2( frames_map_.find(link2_name) );
        const KDL::Frame &T_W_L1 = it1->second;
        const KDL::Frame &T_W_L2 = it2->second;
        T_L1_L2 = T_W_L1.Inverse() * T_W_L2;
    }

    void addLinkContacts(double dist_range, const std::string &link_name, const pcl::PointCloud<pcl::PointNormal>::Ptr &res,
                        const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &principalCurvatures, const KDL::Frame &T_W_S,
                        const pcl::PointCloud<pcl::PointNormal>::Ptr &ob_res, const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &ob_principalCurvatures,
                        const KDL::Frame &T_W_O, const boost::shared_ptr<std::vector<KDL::Frame > > &feature_frames) {

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
            int fidx = 0;
            for (std::list<std::pair<int, double> >::const_iterator it = link_pt.begin(); it != link_pt.end(); it++, fidx++) {
                int poidx = it->first;
                link_features_map[link_name][fidx].pc1 = ob_principalCurvatures->points[poidx].pc1;
                link_features_map[link_name][fidx].pc2 = ob_principalCurvatures->points[poidx].pc2;
                KDL::Frame T_W_F = T_W_O * (*feature_frames)[poidx];
                link_features_map[link_name][fidx].T_L_F = T_W_S.Inverse() * T_W_F;
                link_features_map[link_name][fidx].dist = it->second / dist_range;
            }
        }
    }

};

class ObjectModel {
public:
    pcl::PointCloud<pcl::PointNormal>::Ptr res;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures;
    boost::shared_ptr<pcl::VoxelGrid<pcl::PointNormal> > grid_;
    boost::shared_ptr<std::vector<KDL::Frame > > features_;

    void randomizeSurface() {
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

    int getRandomIndex(double pc1, double pc2, double tolerance1, double tolerance2) const {
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

    bool findFeature(double pc1, double pc2, double tolerance1, double tolerance2, const KDL::Frame &f, double radius, double tolerance4) const {
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
};

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

                    const aiScene *sc = msh->getMesh();
                    if (sc->mNumMeshes != 1) {
                        std::cout << "ERROR: sc->mNumMeshes = " << sc->mNumMeshes << std::endl;
                    }
                    int midx = 0;
//                    std::cout << "v: " << sc->mMeshes[midx]->mNumVertices << "   f: " << sc->mMeshes[midx]->mNumFaces << std::endl;
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
                    uniform_sampling(sc->mMeshes[midx], 100000, *cloud_1);
                    // Voxelgrid
                    boost::shared_ptr<pcl::VoxelGrid<pcl::PointNormal> > grid_(new pcl::VoxelGrid<pcl::PointNormal>);
                    pcl::PointCloud<pcl::PointNormal>::Ptr res(new pcl::PointCloud<pcl::PointNormal>);
                    grid_->setDownsampleAllData(true);
                    grid_->setSaveLeafLayout(true);
                    grid_->setInputCloud(cloud_1);
                    grid_->setLeafSize(0.003, 0.003, 0.003);
                    grid_->filter (*res);
                    point_clouds_map[body_name] = res;
                    frames_map[body_name] = T_W_L * T_L_S;
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
        cm.addLinkContacts(dist_range, link_name, point_clouds_map[link_name], point_pc_clouds_map[link_name], frames_map[link_name],
                            om.res, om.principalCurvatures, T_W_O, features_map[ob_name]);
    }

    cm.buildFeatureMaps();

    om.randomizeSurface();

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
    // visualise random features
    for (int i = 0; i < 100; i++) {
        std::string link_name("right_HandFingerOneKnuckleThreeLink");
        Feature feature;
        cm.getRandomFeatureForLink(link_name, feature);
        const KDL::Frame &T_W_L = frames_map[link_name];
            KDL::Frame T_W_F = T_W_L * feature.T_L_F;
            KDL::Vector v1 = T_W_F * KDL::Vector();
            KDL::Vector v2 = T_W_F * KDL::Vector(0, 0, 0.01);
            KDL::Vector v3 = T_W_F * KDL::Vector(0.01, 0, 0);
            double f = feature.pc1 * 4.0;
            //double f = feature.dist;
            m_id = markers_pub.addVectorMarker(m_id, v1, v2, 0, 0, 1, 1, 0.0005, "world");
            m_id = markers_pub.addVectorMarker(m_id, v1, v3, 1, 0, 0, 1, 0.0005, "world");
            m_id = markers_pub.addSinglePointMarkerCube(m_id, v1, f, 1, f, 1, 0.001, 0.001, 0.001, "world");
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

    std::vector<double > weights(om.res->points.size());

    const double sigma_p = 0.003;
    std::cout << "generating random points..." << std::endl;
    for (int i = 0; i < 10000; i++) {
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            weights[pidx] = 1.0 / static_cast<double >(om.res->points.size());
        }

        double rr = randomUniform(0.0, 1.0);
        double result_x, result_y, result_z;
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_x = om.res->points[pidx].x;
                break;
            }
        }

        
        std::normal_distribution<> d(result_x, sigma_p);
        result_x = d(gen);

        double sum = 0.0;
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            weights[pidx] = uniVariateIsotropicGaussianKernel(result_x, om.res->points[pidx].x, sigma_p);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_y = om.res->points[pidx].y;
                break;
            }
        }

        d = std::normal_distribution<>(result_y, sigma_p);
        result_y = d(gen);

        sum = 0.0;
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            weights[pidx] = biVariateIsotropicGaussianKernel(Eigen::Vector2d(result_x, result_y), Eigen::Vector2d(om.res->points[pidx].x, om.res->points[pidx].y), sigma_p);
            sum += weights[pidx];
        }

        rr = randomUniform(0.0, sum);
        for (int pidx = 0; pidx < om.res->points.size(); pidx++) {
            rr -= weights[pidx];
            if (rr <= 0.0) {
                result_z = om.res->points[pidx].z;
                break;
            }
        }

        d = std::normal_distribution<>(result_z, sigma_p);
        result_z = d(gen);

        m_id = markers_pub.addSinglePointMarkerCube(m_id, KDL::Vector(result_x, result_y, result_z), 0, 0, 1, 1, 0.001, 0.001, 0.001, ob_name);
    }
    markers_pub.publish();
    for (int i = 0; i < 100; i++) {
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
    for (int i = 0; i < 1000; i++) {


        std::string link_name("right_HandFingerOneKnuckleThreeLink");
        Feature feature;
        cm.getRandomFeatureForLink(link_name, feature);
        
    }



    std::string link1_name, link2_name;
    Feature feature1, feature2;
    cm.getRandomFeature(link1_name, feature1);
//    std::cout << "getRandomFeature: " << link1_name << " " << feature1.pc1 << " " << feature1.pc2 << " " << feature1.dist << " " << feature1.T_L_F.p.x() << " " << feature1.T_L_F.p.y() << " " << feature1.T_L_F.p.z() << std::endl;
    KDL::Frame T_F1_L1 = feature1.T_L_F.Inverse();

//    for (int try_idx = 0; try_idx < 10000; try_idx++) {
//    while (ros::ok()) {

    m_id = 101;
    int fidx1 = om.getRandomIndex(feature1.pc1, feature1.pc2, 0.05, 0.05);
    std::cout << "fidx1 " << fidx1 << std::endl;
    KDL::Frame T_O_Fo1 = (*om.features_)[fidx1];
    std::cout << T_O_Fo1.p.x() << " " << T_O_Fo1.p.y() << " " << T_O_Fo1.p.z() << std::endl;

    m_id = markers_pub.addSinglePointMarkerCube(m_id, T_O_Fo1.p, 1, 0, 0, 1, 0.001, 0.001, 0.001, ob_name);

    if (feature1.pc1 > 1.1 * feature1.pc2) {
        // edge
        if ((rand() % 2) == 0) {
            T_O_Fo1 = T_O_Fo1 * KDL::Frame( KDL::Rotation::RotZ(PI) );
        }
    }
    else {
        T_O_Fo1 = T_O_Fo1 * KDL::Frame( KDL::Rotation::RotZ(randomUniform(-PI, PI)) );
    }

    int features_found = 0;
    for (int i = 0; i < 100; i++) {
        cm.getRandomFeature(link2_name, feature2);
        KDL::Frame T_L1_L2;
        cm.getTransform(link1_name, link2_name, T_L1_L2);
        KDL::Frame T_F1_F2 = T_F1_L1 * T_L1_L2 * feature2.T_L_F;

        KDL::Frame T_O_Fo2 = T_O_Fo1 * T_F1_F2;
        if (om.findFeature(feature2.pc1, feature2.pc2, 0.05, 0.05, T_O_Fo2, 0.005, 20.0/180.0*PI)) {
            features_found++;
            m_id = markers_pub.addSinglePointMarkerCube(m_id, T_O_Fo2.p, 0, 0, 1, 1, 0.001, 0.001, 0.001, ob_name);
        }
    }
    std::cout << "features_found " << features_found << std::endl;

//    }

    markers_pub.addEraseMarkers(m_id, m_id+300);

    markers_pub.publish();

    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

//    getchar();
//    }   // while (ros::ok())

    return 0;
}

