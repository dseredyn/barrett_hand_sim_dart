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

#include "gripper_controller.h"

int main(int argc, char** argv) {
    const double PI(3.141592653589793);

    if (argc != 4) {
        std::cout << "usage:" << std::endl;
        std::cout << "package_scene path_scene output" << std::endl;
        return 0;
    }
    ros::init(argc, argv, "dart_test");

    ros::NodeHandle nh_;

    std::string package_name( argv[1] );
    std::string scene_urdf( argv[2] );

    ros::Rate loop_rate(400);

    ros::Publisher joint_state_pub_;
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    tf::TransformBroadcaster br;
    MarkerPublisher markers_pub(nh_);

    std::string package_path_barrett = ros::package::getPath("barrett_hand_defs");
    std::string package_path = ros::package::getPath(package_name);

    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("barrett_hand_defs", package_path_barrett);
    loader.addPackageDirectory("barrett_hand_sim_dart", package_path);

    boost::shared_ptr<GraspSpecification > gspec = GraspSpecification::readFromUrdf(package_path + scene_urdf);

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

    GripperController gc;

    double Kc = 400.0;
    double KcDivTi = Kc / 1.0;
    gc.addJoint("right_HandFingerOneKnuckleOneJoint", Kc, KcDivTi, 0.0, 0.001, 50.0, true, false);
    gc.addJoint("right_HandFingerOneKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, 50.0, false, true);
    gc.addJoint("right_HandFingerTwoKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, 50.0, false, true);
    gc.addJoint("right_HandFingerThreeKnuckleTwoJoint", Kc, KcDivTi, 0.0, 0.001, 50.0, false, true);

    gc.addJointMimic("right_HandFingerTwoKnuckleOneJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, 50.0, true, "right_HandFingerOneKnuckleOneJoint", 1.0, 0.0);
    gc.addJointMimic("right_HandFingerOneKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, 50.0, false, "right_HandFingerOneKnuckleTwoJoint", 0.333333, 0.0);
    gc.addJointMimic("right_HandFingerTwoKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, 50.0, false, "right_HandFingerTwoKnuckleTwoJoint", 0.333333, 0.0);
    gc.addJointMimic("right_HandFingerThreeKnuckleThreeJoint", 2.0*Kc, KcDivTi, 0.0, 0.001, 50.0, false, "right_HandFingerThreeKnuckleTwoJoint", 0.333333, 0.0);

    gc.setGoalPosition("right_HandFingerOneKnuckleOneJoint", gspec->getGoalPosition("right_HandFingerOneKnuckleOneJoint"));
    gc.setGoalPosition("right_HandFingerOneKnuckleTwoJoint", gspec->getGoalPosition("right_HandFingerOneKnuckleTwoJoint"));
    gc.setGoalPosition("right_HandFingerTwoKnuckleTwoJoint", gspec->getGoalPosition("right_HandFingerTwoKnuckleTwoJoint"));
    gc.setGoalPosition("right_HandFingerThreeKnuckleTwoJoint", gspec->getGoalPosition("right_HandFingerThreeKnuckleTwoJoint"));

    std::map<std::string, double> joint_q_map;
    joint_q_map["right_HandFingerOneKnuckleOneJoint"] = gspec->getInitPosition("right_HandFingerOneKnuckleOneJoint");
    joint_q_map["right_HandFingerTwoKnuckleOneJoint"] = gspec->getInitPosition("right_HandFingerOneKnuckleOneJoint");
    joint_q_map["right_HandFingerOneKnuckleTwoJoint"] = gspec->getInitPosition("right_HandFingerOneKnuckleTwoJoint");
    joint_q_map["right_HandFingerOneKnuckleThreeJoint"] = 0.333333 * gspec->getInitPosition("right_HandFingerOneKnuckleTwoJoint");
    joint_q_map["right_HandFingerTwoKnuckleTwoJoint"] = gspec->getInitPosition("right_HandFingerTwoKnuckleTwoJoint");
    joint_q_map["right_HandFingerTwoKnuckleThreeJoint"] = 0.333333 * gspec->getInitPosition("right_HandFingerTwoKnuckleTwoJoint");
    joint_q_map["right_HandFingerThreeKnuckleTwoJoint"] = gspec->getInitPosition("right_HandFingerThreeKnuckleTwoJoint");
    joint_q_map["right_HandFingerThreeKnuckleThreeJoint"] = 0.333333 * gspec->getInitPosition("right_HandFingerThreeKnuckleTwoJoint");

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
//                std::cout << it->first << " " << "stopped" << std::endl;
            }
            j->setForce(0, 0.02*(u-dq) + Cg(qidx));
        }

        for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = bh->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
//            std::cout << b->getName() << std::endl;
            publishTransform(br, T_W_L, b->getName(), "world");
        }

        int m_id = 0;
        for (int bidx = 0; bidx < scene->getNumBodyNodes(); bidx++) {
                dart::dynamics::BodyNode *b = scene->getBodyNode(bidx);

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

        markers_pub.publish();

        ros::spinOnce();
        loop_rate.sleep();

        counter++;
        if (counter < 3000) {
        }
        else if (counter == 3000) {
            dart::dynamics::Joint::Properties prop = bh->getJoint(0)->getJointProperties();
            dart::dynamics::FreeJoint::Properties prop_free;
            prop_free.mName = prop_free.mName;
            prop_free.mT_ParentBodyToJoint = prop.mT_ParentBodyToJoint;
            prop_free.mT_ChildBodyToJoint = prop.mT_ChildBodyToJoint;
            prop_free.mIsPositionLimited = false;
            prop_free.mActuatorType = dart::dynamics::Joint::VELOCITY;
            bh->getRootBodyNode()->changeParentJointType<dart::dynamics::FreeJoint >(prop_free);
        }
        else if (counter < 4000) {
            bh->getDof("Joint_pos_z")->setVelocity(-0.1);
        }
        else {
            break;
        }
    }

    //
    // generate models
    //

    const std::string ob_name( "graspable" );

    scene->getBodyNode(ob_name)->setFrictionCoeff(0.001);

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
            if (body_name.find("right_Hand") != 0 && body_name != ob_name) {
                continue;
            }
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
                    uniform_sampling(sc->mMeshes[midx], 1000000, *cloud_1);
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
                    frames_map[body_name] = T_W_L;
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
                    principalCurvaturesEstimation.setRadiusSearch(0.02);

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

    const double sigma_p = 0.01;//05;
    const double sigma_q = 10.0/180.0*PI;//100.0;
    const double sigma_r = 0.2;//05;
    double sigma_c = 5.0/180.0*PI;

    int m_id = 101;

    // generate object model
    boost::shared_ptr<ObjectModel > om(new ObjectModel);
    for (int pidx = 0; pidx < point_clouds_map[ob_name]->points.size(); pidx++) {
        if (point_pc_clouds_map[ob_name]->points[pidx].pc1 > 1.1 * point_pc_clouds_map[ob_name]->points[pidx].pc2) {
            // e.g. pc1=1, pc2=0
            // edge
            om->addPointFeature((*features_map[ob_name])[pidx] * KDL::Frame(KDL::Rotation::RotZ(PI)), point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
            om->addPointFeature((*features_map[ob_name])[pidx], point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
        }
        else {
            for (double angle = 0.0; angle < 359.0/180.0*PI; angle += 20.0/180.0*PI) {
                om->addPointFeature((*features_map[ob_name])[pidx] * KDL::Frame(KDL::Rotation::RotZ(angle)), point_pc_clouds_map[ob_name]->points[pidx].pc1, point_pc_clouds_map[ob_name]->points[pidx].pc2);
            }
        }
    }


    std::cout << "om.getPointFeatures().size(): " << om->getPointFeatures().size() << std::endl;
    KDL::Frame T_W_O = frames_map[ob_name];

    // generate collision model
    std::map<std::string, std::list<std::pair<int, double> > > link_pt_map;
    boost::shared_ptr<CollisionModel > cm(new CollisionModel);
    cm->setSamplerParameters(sigma_p, sigma_q, sigma_r);

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
        cm->addLinkContacts(dist_range, link_name, point_clouds_map[link_name], frames_map[link_name],
                            om->getPointFeatures(), T_W_O);
    }

    // generate hand configuration model
    boost::shared_ptr<HandConfigurationModel > hm(new HandConfigurationModel);
    std::map<std::string, double> joint_q_map_before( joint_q_map );

    double angleDiffKnuckleTwo = 15.0/180.0*PI;
    joint_q_map_before["right_HandFingerOneKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerTwoKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerThreeKnuckleTwoJoint"] -= angleDiffKnuckleTwo;
    joint_q_map_before["right_HandFingerOneKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;
    joint_q_map_before["right_HandFingerTwoKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;
    joint_q_map_before["right_HandFingerThreeKnuckleThreeJoint"] -= angleDiffKnuckleTwo*0.333333;

    hm->generateModel(joint_q_map_before, joint_q_map, 1.0, 10, sigma_c);

    writeToXml(argv[3], cm, hm);

    return 0;
}


