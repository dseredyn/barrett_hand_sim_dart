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
#include "ros/package.h"
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

void EigenTfToKDL(const Eigen::Isometry3d &tf, KDL::Frame &kdlT) {
    kdlT = KDL::Frame( KDL::Rotation(tf(0,0),tf(0,1),tf(0,2), tf(1,0), tf(1,1), tf(1,2), tf(2,0), tf(2,1), tf(2,2)), KDL::Vector(tf(0,3), tf(1,3), tf(2,3)) );
}

void publishJointState(ros::Publisher &joint_state_pub, const Eigen::VectorXd &q, const std::vector<std::string > &joint_names) {
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator it = joint_names.begin(); it != joint_names.end(); it++, q_idx++) {
            js.name.push_back(*it);
            js.position.push_back(q[q_idx]);
        }
        joint_state_pub.publish(js);
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

class GripperState {
//    class Puck {
//    public:
//    };

    class Joint {
    public:
        double q_des_;
        double e0_, e1_, e2_, u_;
        double Kc_;
        double KcTi_;
        double Td_;
        double Ts_;
        double u_max_;
        bool backdrivable_;
        bool breakable_;
        bool stopped_;
        std::vector<double > q_hist_;
        std::vector<bool > u_hist_;
        int q_hist_idx_;
    };

    class JointMimic : public Joint {
    public:
        std::string mimic_joint_name_;
        double mimic_factor_;
        double mimic_offset_;
    };

    std::map<std::string, Joint > joint_map_;
    std::map<std::string, JointMimic > joint_mimic_map_;
    std::vector<std::string > joint_names_vec_;

public:
    bool addJoint(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable, bool breakable) {
        if (joint_map_.find(joint_name) != joint_map_.end()) {
            std::cout << "ERROR: addJoint joint_map_.find(\"" << joint_name << "\") != joint_map_.end()" << std::endl;
            return false;
        }
        Joint j;
        j.Kc_ = Kc;
        j.KcTi_ = KcTi;
        j.Td_ = Td;
        j.Ts_ = Ts;
        j.u_max_ = u_max;
        j.backdrivable_ = backdrivable;
        j.breakable_ = breakable;
        j.stopped_ = false;
        j.q_hist_.resize(10, 0.0);
        j.q_hist_idx_ = 0;
        joint_map_.insert( std::make_pair(joint_name, j) );
        joint_names_vec_.push_back(joint_name);
        return true;
    }

    bool addJointMimic(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable, const std::string &mimic_joint, double mimic_factor, double mimic_offset) {
        if (joint_mimic_map_.find(joint_name) != joint_mimic_map_.end()) {
            std::cout << "ERROR: addJoint joint_mimic_map_.find(\"" << joint_name << "\") != joint_mimic_map_.end()" << std::endl;
            return false;
        }

        JointMimic j;
        j.Kc_ = Kc;
        j.KcTi_ = KcTi;
        j.Td_ = Td;
        j.Ts_ = Ts;
        j.u_max_ = u_max;
        j.mimic_joint_name_ = mimic_joint;
        j.mimic_factor_ = mimic_factor;
        j.mimic_offset_ = mimic_offset;
        j.backdrivable_ = backdrivable;
        j.breakable_ = false;
        j.stopped_ = false;
        j.q_hist_.resize(10, 0.0);
        j.q_hist_[0] = 100.0;
        j.q_hist_idx_ = 1;
        joint_mimic_map_.insert( std::make_pair(joint_name, j) );
        joint_names_vec_.push_back(joint_name);
        return true;
    }

    bool isBackdrivable(const std::string &joint_name) const {
        std::map<std::string, Joint >::const_iterator it = joint_map_.find(joint_name);
        if (it != joint_map_.end()) {
            return it->second.backdrivable_;
        }
        else {
            std::map<std::string, JointMimic >::const_iterator it2 = joint_mimic_map_.find(joint_name);
            if (it2 != joint_mimic_map_.end()) {
                return it2->second.backdrivable_;
            }
            else {
                std::cout << "ERROR: isBackdrivable it == joint_map_.end() " << joint_name << std::endl;
                return false;
            }
        }
        return false;
    }

    bool isStopped(const std::string &joint_name) const {
        std::map<std::string, Joint >::const_iterator it = joint_map_.find(joint_name);
        if (it != joint_map_.end()) {
            return it->second.stopped_;
        }
        else {
            std::map<std::string, JointMimic >::const_iterator it2 = joint_mimic_map_.find(joint_name);
            if (it2 != joint_mimic_map_.end()) {
                return it2->second.stopped_;
            }
            else {
                std::cout << "ERROR: isStopped it == joint_map_.end() " << joint_name << std::endl;
                return false;
            }
        }
        return false;
    }

    const std::vector<std::string >& getJointNames() const {
        return joint_names_vec_;
    }

    bool setGoalPosition(const std::string joint_name, double q_des) {
        std::map<std::string, Joint >::iterator it = joint_map_.find(joint_name);
        if (it == joint_map_.end()) {
            std::cout << "ERROR: setGoalPosition joint_map_.find(\"" << joint_name << "\") == joint_map_.end()" << std::endl;
            return false;
        }
        it->second.q_des_ = q_des;
        return true;
    }

    bool controlStep(const std::map<std::string, double> &q_map) {
        // update actuated joints
        for (std::map<std::string, Joint >::iterator it = joint_map_.begin(); it != joint_map_.end(); it++) {
            std::map<std::string, double>::const_iterator qit = q_map.find( it->first );
            if (qit == q_map.end()) {
                std::cout << "ERROR: controlStep qit == q_map.end() " << it->first << std::endl;
                return false;
            }
            double q = qit->second;
            Joint &j = it->second;
            j.e2_ = j.e1_;
            j.e1_ = j.e0_;
            j.e0_ = j.q_des_ - q;
            j.u_ = j.u_ + j.Kc_ * (j.e0_ - j.e1_) + j.KcTi_ * j.Ts_ * j.e0_ + j.Kc_ * j.Td_ / j.Ts_ * (j.e0_ - 2.0 * j.e1_ + j.e2_);
            if (j.u_ > j.u_max_) {
                j.u_ = j.u_max_;
                j.u_hist_[j.q_hist_idx_] = true;
            }
            else if (j.u_ < -j.u_max_) {
                j.u_ = -j.u_max_;
                j.u_hist_[j.q_hist_idx_] = true;
            }
            else {
                j.u_hist_[j.q_hist_idx_] = false;
            }
            j.q_hist_[j.q_hist_idx_] = q;
            j.q_hist_idx_ = (j.q_hist_idx_ + 1) % j.q_hist_.size();

            double mean = 0.0;
            for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                mean += j.q_hist_[hidx];
            }
            mean /= j.q_hist_.size();
            double variance = 0.0;
            for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                variance += (mean - j.q_hist_[hidx]) * (mean - j.q_hist_[hidx]);
            }
            if (std::sqrt(variance) < 0.001) {
                j.stopped_ = true;
            }
        }

        // update mimic joints
        for (std::map<std::string, JointMimic >::iterator it = joint_mimic_map_.begin(); it != joint_mimic_map_.end(); it++) {
            std::map<std::string, double>::const_iterator qit = q_map.find( it->first );
            if (qit == q_map.end()) {
                std::cout << "ERROR: controlStep qit == q_map.end() " << it->first << std::endl;
                return false;
            }
            double q = qit->second;
            JointMimic &j = it->second;
            std::map<std::string, double>::const_iterator qit_mim = q_map.find(j.mimic_joint_name_);
            Joint &j2 = joint_map_.find(j.mimic_joint_name_)->second;
            if (qit_mim == q_map.end()) {
                std::cout << "ERROR: controlStep qit_mim == q_map.end() " << j.mimic_joint_name_ << std::endl;
                return false;
            }
            j.q_des_ = qit_mim->second * j.mimic_factor_ + j.mimic_offset_;
            j.e2_ = j.e1_;
            j.e1_ = j.e0_;
            j.e0_ = j.q_des_ - q;
            j.u_ = j.u_ + j.Kc_ * (j.e0_ - j.e1_) + j.KcTi_ * j.Ts_ * j.e0_ + j.Kc_ * j.Td_ / j.Ts_ * (j.e0_ - 2.0 * j.e1_ + j.e2_);
            if (j.u_ > j.u_max_) {
                j.u_ = j.u_max_;
                j.u_hist_[j.q_hist_idx_] = true;
            }
            else if (j.u_ < -j.u_max_) {
                j.u_ = -j.u_max_;
                j.u_hist_[j.q_hist_idx_] = true;
            }
            else {
                j.u_hist_[j.q_hist_idx_] = false;
            }
            j.q_hist_[j.q_hist_idx_] = q;
            j.q_hist_idx_ = (j.q_hist_idx_ + 1) % j.q_hist_.size();

            if (j2.stopped_) {
                double mean = 0.0;
                for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                    mean += j.q_hist_[hidx];
                }
                mean /= j.q_hist_.size();
                double variance = 0.0;
                for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                    variance += (mean - j.q_hist_[hidx]) * (mean - j.q_hist_[hidx]);
                }
                if (std::sqrt(variance) < 0.001) {
                    j.stopped_ = true;
                }
            }
        }
        return true;
    }

    double getControl(const std::string &joint_name) const {
        std::map<std::string, Joint >::const_iterator it = joint_map_.find(joint_name);
        if (it != joint_map_.end()) {
            return it->second.u_;
        }
        else {
            std::map<std::string, JointMimic >::const_iterator it2 = joint_mimic_map_.find(joint_name);
            if (it2 != joint_mimic_map_.end()) {
                return it2->second.u_;
            }
            else {
                std::cout << "ERROR: getControl it == joint_map_.end() " << joint_name << std::endl;
                return 0.0;
            }
        }
        return 0.0;
    }
};

int main(int argc, char** argv) {
    const double PI(3.141592653589793);

    ros::init(argc, argv, "ode_test");

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

    // Load the Skeleton from a file
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("barrett_hand_defs", "/home/dseredyn/ws_velma/underlay/src/barrett_hand_robot/barrett_hand_defs");
    dart::dynamics::SkeletonPtr bh( loader.parseSkeleton("/home/dseredyn/ws_velma/underlay/barrett_hand.urdf") );
    bh->setName("BarrettHand");

    // Position its base in a reasonable way
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.3);
    bh->getJoint(0)->setTransformFromParentBodyNode(tf);

    // Create a Skeleton with the name "domino"
    dart::dynamics::SkeletonPtr domino = dart::dynamics::Skeleton::create("domino");

    // Create a body for the domino
    dart::dynamics::BodyNodePtr body =
      domino->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(NULL).second;
    body->setName("cube5");

    // Create a shape for the domino
    std::shared_ptr<dart::dynamics::BoxShape> box(
        new dart::dynamics::BoxShape(Eigen::Vector3d(0.07,
                                                    0.07,
                                                    0.07)));
    body->addVisualizationShape(box);
    body->addCollisionShape(box);

    // Set up inertia for the domino
    dart::dynamics::Inertia inertia;
    inertia.setMass(0.1);
    inertia.setMoment(box->computeInertia(0.1));
    body->setInertia(inertia);

    domino->getDof("Joint_pos_z")->setPosition(0.45);

    dart::simulation::World* world = new dart::simulation::World();

//    std::cout << "world time step: " << world->getTimeStep() << std::endl;
    world->addSkeleton(bh);
    world->addSkeleton(domino);

    Eigen::Vector3d grav(0,0,0);
    world->setGravity(grav);
//    std::cout << "world->getNumSimpleFrames: " << world->getNumSimpleFrames() << std::endl;

    GripperState gs;
//    gs.addJoint(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max_, const std::string &mimic_joint=std::string(), double mimic_factor=0.0, double mimic_offset=0.0);
    gs.addJoint("right_HandFingerOneKnuckleOneJoint", 100.0, 15.0, 0.0, 0.001, 5.0, true, false);
    gs.addJoint("right_HandFingerOneKnuckleTwoJoint", 100.0, 15.0, 0.0, 0.001, 5.0, false, true);
    gs.addJoint("right_HandFingerTwoKnuckleTwoJoint", 100.0, 15.0, 0.0, 0.001, 5.0, false, true);
    gs.addJoint("right_HandFingerThreeKnuckleTwoJoint", 100.0, 15.0, 0.0, 0.001, 5.0, false, true);

    gs.addJointMimic("right_HandFingerTwoKnuckleOneJoint", 200.0, 15.0, 0.0, 0.001, 5.0, true, "right_HandFingerOneKnuckleOneJoint", 1.0, 0.0);
    gs.addJointMimic("right_HandFingerOneKnuckleThreeJoint", 200.0, 15.0, 0.0, 0.001, 5.0, false, "right_HandFingerOneKnuckleTwoJoint", 0.333333, 0.0);
    gs.addJointMimic("right_HandFingerTwoKnuckleThreeJoint", 200.0, 15.0, 0.0, 0.001, 5.0, false, "right_HandFingerTwoKnuckleTwoJoint", 0.333333, 0.0);
    gs.addJointMimic("right_HandFingerThreeKnuckleThreeJoint", 200.0, 15.0, 0.0, 0.001, 5.0, false, "right_HandFingerThreeKnuckleTwoJoint", 0.333333, 0.0);

    gs.setGoalPosition("right_HandFingerOneKnuckleOneJoint", 0.6);
    gs.setGoalPosition("right_HandFingerOneKnuckleTwoJoint", 1.4);
    gs.setGoalPosition("right_HandFingerTwoKnuckleTwoJoint", 1.4);
    gs.setGoalPosition("right_HandFingerThreeKnuckleTwoJoint", 1.4);

    std::map<std::string, double> joint_q_map;
    for (std::vector<std::string >::const_iterator it = gs.getJointNames().begin(); it != gs.getJointNames().end(); it++) {
        joint_q_map.insert( std::make_pair((*it), 0.0) );

        dart::dynamics::Joint *j = bh->getJoint((*it));
        j->setActuatorType(dart::dynamics::Joint::FORCE);
     	j->setPositionLimited(true);
    }

    while (ros::ok()) {
        world->step(false);
//        dart::dynamics::SkeletonPtr bh = world->getSkeleton("BarrettHand");

        for (std::map<std::string, double>::iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint(it->first);
            it->second = j->getPosition(0);
        }

        gs.controlStep(joint_q_map);

        Eigen::VectorXd u(joint_q_map.size());
        Eigen::VectorXd dq(joint_q_map.size());
        for (std::map<std::string, double>::iterator it = joint_q_map.begin(); it != joint_q_map.end(); it++) {
            dart::dynamics::Joint *j = bh->getJoint(it->first);
            int qidx = j->getIndexInSkeleton(0);
            u(qidx) = gs.getControl(it->first);
            dq(qidx) = j->getVelocity(0);
            if (!gs.isBackdrivable(it->first)) {
                j->setPositionLowerLimit(0, std::max(j->getPositionLowerLimit(0), it->second-0.01));
            }

            if (gs.isStopped(it->first)) {
                j->setPositionLowerLimit(0, std::max(j->getPositionLowerLimit(0), it->second-0.01));
                j->setPositionUpperLimit(0, std::min(j->getPositionUpperLimit(0), it->second+0.01));
                std::cout << it->first << " " << "stopped" << std::endl;
            }
//            std::cout << it->first << " " << j->getActuatorType() << std::endl;
        }

        // Compute the joint forces needed to compensate for Coriolis forces and
        // gravity
        const Eigen::VectorXd& Cg = bh->getCoriolisAndGravityForces();
        // Compute the desired joint forces
        const Eigen::MatrixXd& M = bh->getMassMatrix();
        Eigen::VectorXd forces(joint_q_map.size());
        forces = M * (u - dq) + Cg;
        bh->setForces(forces);

        std::cout << u.transpose() << std::endl;

        for (int bidx = 0; bidx < bh->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = bh->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
//            std::cout << b->getName() << std::endl;
            publishTransform(br, T_W_L, b->getName(), "world");
        }

        for (int bidx = 0; bidx < domino->getNumBodyNodes(); bidx++) {
            dart::dynamics::BodyNode *b = domino->getBodyNode(bidx);
            const Eigen::Isometry3d &tf = b->getTransform();
            KDL::Frame T_W_L;
            EigenTfToKDL(tf, T_W_L);
//            std::cout << b->getName() << std::endl;
            publishTransform(br, T_W_L, b->getName(), "world");
        }
        
        markers_pub.addSinglePointMarkerCube(100, KDL::Vector(), 1, 0, 0, 1, 0.07, 0.07, 0.07, "cube5");
        markers_pub.publish();

        ros::spinOnce();
//        loop_rate.sleep();
    }

    return 0;
}


