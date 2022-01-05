// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 www.open3d.org
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

#include "open3d/geometry/Geometry3Df.h"

#include <Eigen/Dense>
#include <numeric>

#include "open3d/utility/Console.h"

namespace open3d {
namespace geometry {

Geometry3Df& Geometry3Df::Rotate(const Eigen::Matrix3f& R) {
    return Rotate(R, GetCenter());
}

Eigen::Vector3f Geometry3Df::ComputeMinBound(
        const std::vector<Eigen::Vector3f>& points) const {
    if (points.empty()) {
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return std::accumulate(
            points.begin(), points.end(), points[0],
            [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.array().min(b.array()).matrix();
            });
}

Eigen::Vector3f Geometry3Df::ComputeMaxBound(
        const std::vector<Eigen::Vector3f>& points) const {
    if (points.empty()) {
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    return std::accumulate(
            points.begin(), points.end(), points[0],
            [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                return a.array().max(b.array()).matrix();
            });
}
Eigen::Vector3f Geometry3Df::ComputeCenter(
        const std::vector<Eigen::Vector3f>& points) const {
    Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
    if (points.empty()) {
        return center;
    }
    center = std::accumulate(points.begin(), points.end(), center);
    center /= float(points.size());
    return center;
}

void Geometry3Df::ResizeAndPaintUniformColor(
        std::vector<Eigen::Vector3f>& colors,
        const size_t size,
        const Eigen::Vector3f& color) const {
    colors.resize(size);
    Eigen::Vector3f clipped_color = color;
    if (color.minCoeff() < 0 || color.maxCoeff() > 1) {
        utility::LogWarning(
                "invalid color in PaintUniformColor, clipping to [0, 1]");
        clipped_color = clipped_color.array()
                                .max(Eigen::Vector3f(0.0f, 0.0f, 0.0f).array())
                                .matrix();
        clipped_color = clipped_color.array()
                                .min(Eigen::Vector3f(1.0f, 1.0f, 1.0f).array())
                                .matrix();
    }
    for (size_t i = 0; i < size; i++) {
        colors[i] = clipped_color;
    }
}

void Geometry3Df::TransformPoints(const Eigen::Matrix4f& transformation,
                                 std::vector<Eigen::Vector3f>& points) const {
    for (auto& point : points) {
        Eigen::Vector4f new_point =
                transformation *
                Eigen::Vector4f(point(0), point(1), point(2), 1.0f);
        point = new_point.head<3>() / new_point(3);
    }
}

void Geometry3Df::TransformNormals(const Eigen::Matrix4f& transformation,
                                  std::vector<Eigen::Vector3f>& normals) const {
    for (auto& normal : normals) {
        Eigen::Vector4f new_normal =
                transformation *
                Eigen::Vector4f(normal(0), normal(1), normal(2), 0.0f);
        normal = new_normal.head<3>();
    }
}

void Geometry3Df::TranslatePoints(const Eigen::Vector3f& translation,
                                 std::vector<Eigen::Vector3f>& points,
                                 bool relative) const {
    Eigen::Vector3f transform = translation;
    if (!relative) {
        transform -= ComputeCenter(points);
    }
    for (auto& point : points) {
        point += transform;
    }
}

void Geometry3Df::ScalePoints(const float scale,
                             std::vector<Eigen::Vector3f>& points,
                             const Eigen::Vector3f& center) const {
    for (auto& point : points) {
        point = (point - center) * scale + center;
    }
}

void Geometry3Df::RotatePoints(const Eigen::Matrix3f& R,
                              std::vector<Eigen::Vector3f>& points,
                              const Eigen::Vector3f& center) const {
    for (auto& point : points) {
        point = R * (point - center) + center;
    }
}

void Geometry3Df::RotateNormals(const Eigen::Matrix3f& R,
                               std::vector<Eigen::Vector3f>& normals) const {
    for (auto& normal : normals) {
        normal = R * normal;
    }
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromXYZ(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixXf(rotation(0)) *
           open3d::utility::RotationMatrixYf(rotation(1)) *
           open3d::utility::RotationMatrixZf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromYZX(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixYf(rotation(0)) *
           open3d::utility::RotationMatrixZf(rotation(1)) *
           open3d::utility::RotationMatrixXf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromZXY(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixZf(rotation(0)) *
           open3d::utility::RotationMatrixXf(rotation(1)) *
           open3d::utility::RotationMatrixYf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromXZY(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixXf(rotation(0)) *
           open3d::utility::RotationMatrixZf(rotation(1)) *
           open3d::utility::RotationMatrixYf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromZYX(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixZf(rotation(0)) *
           open3d::utility::RotationMatrixYf(rotation(1)) *
           open3d::utility::RotationMatrixXf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromYXZ(
        const Eigen::Vector3f& rotation) {
    return open3d::utility::RotationMatrixYf(rotation(0)) *
           open3d::utility::RotationMatrixXf(rotation(1)) *
           open3d::utility::RotationMatrixZf(rotation(2));
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromAxisAngle(
        const Eigen::Vector3f& rotation) {
    const float phi = rotation.norm();
    return Eigen::AngleAxisf(phi, rotation / phi).toRotationMatrix();
}

Eigen::Matrix3f Geometry3Df::GetRotationMatrixFromQuaternion(
        const Eigen::Vector4f& rotation) {
    return Eigen::Quaternionf(rotation(0), rotation(1), rotation(2),
                              rotation(3))
            .normalized()
            .toRotationMatrix();
}

}  // namespace geometry
}  // namespace open3d
