// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "open3d/pipelines/registration/TransformationEstimationf.h"

namespace open3d {

namespace geometry {
class PointCloudf;
}

namespace pipelines {
namespace registration {

/// \class CorrespondenceChecker
///
/// \brief Base class that checks if two (small) point clouds can be aligned.
///
/// This class is used in feature based matching algorithms (such as RANSAC and
/// FastGlobalRegistration) to prune out outlier correspondences.
/// The virtual function Check() must be implemented in subclasses.
class CorrespondenceCheckerf {
public:
    /// \brief Default Constructor.
    ///
    /// \param require_pointcloud_alignment Specifies whether point cloud
    /// alignment is required.
    CorrespondenceCheckerf(bool require_pointcloud_alignment)
        : require_pointcloud_alignment_(require_pointcloud_alignment) {}
    virtual ~CorrespondenceCheckerf() {}

public:
    /// \brief Function to check if two points can be aligned.
    ///
    /// The two input point
    /// clouds must have exact the same number of points.
    /// \param source Source point cloud.
    /// \param target Target point cloud.
    /// \param corres Correspondence set between source and target point cloud.
    /// \param transformation The estimated transformation (inplace).
    virtual bool Check(const geometry::PointCloudf &source,
                       const geometry::PointCloudf &target,
                       const CorrespondenceSet &corres,
                       const Eigen::Matrix4f &transformation) const = 0;

public:
    /// Some checkers do not require point clouds to be aligned, e.g., the edge
    /// length checker. Some checkers do, e.g., the distance checker.
    bool require_pointcloud_alignment_;
};

/// \class CorrespondenceCheckerBasedOnEdgeLength
///
/// \brief Check if two point clouds build the polygons with similar edge
/// lengths.
///
/// That is, checks if the lengths of any two arbitrary edges (line formed by
/// two vertices) individually drawn withinin source point cloud and within the
/// target point cloud with correspondences are similar. The only parameter
/// similarity_threshold is a number between 0 (loose) and 1 (strict).
class CorrespondenceCheckerBasedOnEdgeLengthf : public CorrespondenceCheckerf {
public:
    /// \brief Default Constructor.
    ///
    /// \param similarity_threshold specifies the threshold within which 2
    /// arbitrary edges are similar.
    CorrespondenceCheckerBasedOnEdgeLengthf(float similarity_threshold = 0.9f)
        : CorrespondenceCheckerf(false),
          similarity_threshold_(similarity_threshold) {}
    ~CorrespondenceCheckerBasedOnEdgeLengthf() override {}

public:
    bool Check(const geometry::PointCloudf &source,
               const geometry::PointCloudf &target,
               const CorrespondenceSet &corres,
               const Eigen::Matrix4f &transformation) const override;

public:
    /// For the check to be true,
    /// ||edgesource||>similarity_threshold×||edgetarget|| and
    /// ||edgetarget||>similarity_threshold×||edgesource|| must hold true for
    /// all edges.
    float similarity_threshold_;
};

/// \class CorrespondenceCheckerBasedOnDistance
///
/// \brief Check if two aligned point clouds are close.
class CorrespondenceCheckerBasedOnDistancef : public CorrespondenceCheckerf {
public:
    /// \brief Default Constructor.
    ///
    /// \param distance_threshold Distance threashold for the check.
    CorrespondenceCheckerBasedOnDistancef(float distance_threshold)
        : CorrespondenceCheckerf(true),
          distance_threshold_(distance_threshold) {}
    ~CorrespondenceCheckerBasedOnDistancef() override {}

public:
    bool Check(const geometry::PointCloudf &source,
               const geometry::PointCloudf &target,
               const CorrespondenceSet &corres,
               const Eigen::Matrix4f &transformation) const override;

public:
    /// Distance threashold for the check.
    float distance_threshold_;
};

/// \class CorrespondenceCheckerBasedOnNormal
///
/// \brief Class to check if two aligned point clouds have similar normals.
///
/// It considers vertex normal affinity of any correspondences. It computes dot
/// product of two normal vectors. It takes radian value for the threshold.
class CorrespondenceCheckerBasedOnNormalf : public CorrespondenceCheckerf {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param normal_angle_threshold Radian value for angle threshold.
    CorrespondenceCheckerBasedOnNormalf(float normal_angle_threshold)
        : CorrespondenceCheckerf(true),
          normal_angle_threshold_(normal_angle_threshold) {}
    ~CorrespondenceCheckerBasedOnNormalf() override {}

public:
    bool Check(const geometry::PointCloudf &source,
               const geometry::PointCloudf &target,
               const CorrespondenceSet &corres,
               const Eigen::Matrix4f &transformation) const override;

public:
    /// Radian value for angle threshold.
    float normal_angle_threshold_;
};

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
