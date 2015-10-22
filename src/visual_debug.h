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

#ifndef VISUAL_DEBUG_H__
#define VISUAL_DEBUG_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#include "Eigen/Dense"

#include <kdl/frames.hpp>
#include "planer_utils/marker_publisher.h"
#include "models.h"

int visualiseContactRegion(MarkerPublisher &markers_pub, int m_id, const std::vector<CollisionModel::Feature > &f_vec, const KDL::Frame &T_L_C, const KDL::Frame &T_W_L);
int visualiseAllFeatures(MarkerPublisher &markers_pub, int m_id, const std::vector<ObjectModel::Feature > &f_vec, const std::string &frame_id);
int visualiseRejectionSamplingVonMisesFisher3(MarkerPublisher &markers_pub, int m_id);
int visualiseRejectionSamplingVonMisesFisher4(MarkerPublisher &markers_pub, int m_id);
int visualiseQueryDensityParticles(MarkerPublisher &markers_pub, int m_id, const std::vector<QueryDensity::QueryDensityElement > &qd_vec, const std::string &frame_id);
int visualiseQueryDensityFunction(tf::TransformBroadcaster &br, MarkerPublisher &markers_pub, int m_id, const QueryDensity &qd, const std::string &link_name, const KDL::Frame &T_L_C, const KDL::Frame &T_W_O, const std::string &frame_id);

#endif  // VISUAL_DEBUG_H__

