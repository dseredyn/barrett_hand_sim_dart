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

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/utilities.h"

#include "gripper_controller.h"

    bool GripperController::addJoint(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable, bool breakable) {
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

    bool GripperController::addJointMimic(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable, const std::string &mimic_joint, double mimic_factor, double mimic_offset) {
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
        j.q_hist_.resize(100, 0.0);
        for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
            j.q_hist_[hidx] = hidx;
        }
        j.q_hist_[0] = 100.0;
        j.q_hist_idx_ = 1;
        joint_mimic_map_.insert( std::make_pair(joint_name, j) );
        joint_names_vec_.push_back(joint_name);
        return true;
    }

    bool GripperController::isBackdrivable(const std::string &joint_name) const {
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

    bool GripperController::isStopped(const std::string &joint_name) const {
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

    const std::vector<std::string >& GripperController::getJointNames() const {
        return joint_names_vec_;
    }

    bool GripperController::setGoalPosition(const std::string &joint_name, double q_des) {
        std::map<std::string, Joint >::iterator it = joint_map_.find(joint_name);
        if (it == joint_map_.end()) {
            std::cout << "ERROR: setGoalPosition joint_map_.find(\"" << joint_name << "\") == joint_map_.end()" << std::endl;
            return false;
        }
        it->second.q_des_ = q_des;
        return true;
    }

    bool GripperController::controlStep(const std::map<std::string, double> &q_map) {
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
//*
            double mean = 0.0;
            bool overload = true;
            for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                mean += j.q_hist_[hidx];
                if (!j.u_hist_[hidx]) {
                    overload = false;
                }
            }
            if (overload) {
                mean /= j.q_hist_.size();
                double variance = 0.0;
                for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                    variance += (mean - j.q_hist_[hidx]) * (mean - j.q_hist_[hidx]);
                }
                if (std::sqrt(variance) < 0.001) {
                    j.stopped_ = true;
                }
            }
//*/
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
//*
            if (j2.stopped_) {
                double mean = 0.0;
                bool overload = true;
                for (int hidx = 0; hidx < j.q_hist_.size(); hidx++) {
                    mean += j.q_hist_[hidx];
                    if (!j.u_hist_[hidx]) {
                        overload = false;
                    }
                }
                if (overload) {
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
//*/
        }
        return true;
    }

    double GripperController::getControl(const std::string &joint_name) const {
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

    Eigen::VectorXd GripperController::getControls() const {
        Eigen::VectorXd u(joint_map_.size() + joint_mimic_map_.size());
        int q_idx = 0;

        // update actuated joints
        for (std::map<std::string, Joint >::const_iterator it = joint_map_.begin(); it != joint_map_.end(); it++, q_idx++) {
            const Joint &j = it->second;
            u(q_idx) = j.u_;
        }

        // update mimic joints
        for (std::map<std::string, JointMimic >::const_iterator it = joint_mimic_map_.begin(); it != joint_mimic_map_.end(); it++, q_idx++) {
            const JointMimic &j = it->second;
            u(q_idx) = j.u_;
        }

        return u;
    }

