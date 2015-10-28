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

#ifndef GRIPPER_CONTROLLER_H__
#define GRIPPER_CONTROLLER_H__

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"

#include <kdl/frames.hpp>

#include "planer_utils/utilities.h"

class GripperController {
protected:
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
    bool addJoint(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable, bool breakable);
    bool addJointMimic(const std::string &joint_name, double Kc, double KcTi, double Td, double Ts, double u_max, bool backdrivable,
                        const std::string &mimic_joint, double mimic_factor, double mimic_offset);
    bool isBackdrivable(const std::string &joint_name) const;
    bool isStopped(const std::string &joint_name) const;
    const std::vector<std::string >& getJointNames() const;
    bool setGoalPosition(const std::string &joint_name, double q_des);
    bool controlStep(const std::map<std::string, double> &q_map);
    double getControl(const std::string &joint_name) const;
    Eigen::VectorXd getControls() const;
};

#endif  // GRIPPER_CONTROLLER_H__

