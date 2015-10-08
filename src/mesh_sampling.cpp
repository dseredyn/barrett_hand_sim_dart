/*
 * Software License Agreement (BSD License)
 *
 * This is the modification of the mesh_sampling.cpp file from PCL.
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "mesh_sampling.h"

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3,
                     Eigen::Vector4d& p)
{
  double r1 = uniform_deviate (rand ());
  double r2 = uniform_deviate (rand ());
  double r1sqr = sqrtf (r1);
  double OneMinR1Sqr = (1 - r1sqr);
  double OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void randPSurface (const aiMesh *mesh, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4d& p, Eigen::Vector4d& n) {
    double r = uniform_deviate(rand ()) * totalArea;

    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    int el = low - cumulativeAreas->begin();

    const unsigned int *ptIds = mesh->mFaces[el].mIndices;
    Eigen::Vector3d a(mesh->mVertices[ptIds[0]].x, mesh->mVertices[ptIds[0]].y, mesh->mVertices[ptIds[0]].z);
    Eigen::Vector3d b(mesh->mVertices[ptIds[1]].x, mesh->mVertices[ptIds[1]].y, mesh->mVertices[ptIds[1]].z);
    Eigen::Vector3d c(mesh->mVertices[ptIds[2]].x, mesh->mVertices[ptIds[2]].y, mesh->mVertices[ptIds[2]].z);
    randomPointTriangle(    a(0), a(1), a(2),
                            b(0), b(1), b(2),
                            c(0), c(1), c(2), p);
    Eigen::Vector3d n3 = (b-a).cross( (c-a) );
    n3.normalize();
    n(0) = n3(0);
    n(1) = n3(1);
    n(2) = n3(2);
    n(3) = 0;
}

double getTotalArea(const aiMesh *mesh, std::vector<double> *cumulativeAreas) {
    double totalArea = 0.0;

    if (cumulativeAreas != NULL) {
        cumulativeAreas->resize(mesh->mNumFaces, 0.0);
    }

    for (size_t i = 0; i < mesh->mNumFaces; i++) {
        const unsigned int *ptIds = mesh->mFaces[i].mIndices;
        if (mesh->mFaces[i].mNumIndices != 3) {
            std::cout << "ERROR: uniform_sampling: mesh->mFaces[i].mNumIndices " << mesh->mFaces[i].mNumIndices << std::endl;
        }

        const aiVector3D &a = mesh->mVertices[ptIds[0]];
        const aiVector3D &b = mesh->mVertices[ptIds[1]];
        const aiVector3D &c = mesh->mVertices[ptIds[2]];

        double ab = (b - a).Length();
        double bc = (c - b).Length();
        double ca = (a - c).Length();

        double area_sqr = (ab + bc + ca) * (bc + ca - ab) * (ca + ab - bc) * (ab + bc - ca);

        if (area_sqr > 0.0) {
            totalArea += 0.25 * std::sqrt( area_sqr );
        }
        if (totalArea != totalArea) {
            std::cout << "ERROR: uniform_sampling: " << i << "  ptIds: " << ptIds[0] << " " << ptIds[1] << " " << ptIds[2] << " totalArea " << totalArea << "  ab " << ab <<  "  bc " << bc <<  "  ca " << ca << std::endl;
            return 0.0;
        }
        if (cumulativeAreas != NULL) {
            (*cumulativeAreas)[i] = totalArea;
        }
    }
    return totalArea;
}

void uniform_sampling (const aiMesh *mesh, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out) {
    double totalArea = 0;
    std::vector<double> cumulativeAreas;
    totalArea = getTotalArea(mesh, &cumulativeAreas);

//    std::cout << "uniform_sampling totalArea: " << totalArea << std::endl;
    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;

    for (size_t i = 0; i < n_samples; i++) {
        Eigen::Vector4d p, n;
        randPSurface (mesh, &cumulativeAreas, totalArea, p, n);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}

void uniform_sampling (const aiMesh *mesh, size_t n_samples, pcl::PointCloud<pcl::PointNormal> & cloud_out) {
    double totalArea = 0;
    std::vector<double> cumulativeAreas;
    totalArea = getTotalArea(mesh, &cumulativeAreas);

//    std::cout << "uniform_sampling totalArea: " << totalArea << std::endl;
    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;

    for (size_t i = 0; i < n_samples; i++) {
        Eigen::Vector4d p, n;
        randPSurface (mesh, &cumulativeAreas, totalArea, p, n);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
        cloud_out.points[i].normal[0] = n(0);
        cloud_out.points[i].normal[1] = n(1);
        cloud_out.points[i].normal[2] = n(2);
    }
}

