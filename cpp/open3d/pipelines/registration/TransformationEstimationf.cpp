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

#include "open3d/pipelines/registration/TransformationEstimationf.h"

#include <Eigen/Geometry>

#include "open3d/geometry/PointCloudf.h"
#include "open3d/utility/Eigen.h"

namespace open3d {
namespace pipelines {
namespace registration {

float TransformationEstimationPointToPointf::ComputeRMSE(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return 0.0f;
    float err = 0.0f;
    for (const auto &c : corres) {
        err += (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
    }
    return std::sqrt(err / (float)corres.size());
}

Eigen::Matrix4f TransformationEstimationPointToPointf::ComputeTransformation(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return Eigen::Matrix4f::Identity();
    Eigen::MatrixXf source_mat(3, corres.size());
    Eigen::MatrixXf target_mat(3, corres.size());
    for (size_t i = 0; i < corres.size(); i++) {
        source_mat.block<3, 1>(0, i) = source.points_[corres[i][0]];
        target_mat.block<3, 1>(0, i) = target.points_[corres[i][1]];
    }
    return Eigen::umeyama(source_mat, target_mat, with_scaling_);
}

float TransformationEstimationPointToPlanef::ComputeRMSE(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals()) return 0.0f;
    float err = 0.0f, r;
    for (const auto &c : corres) {
        r = (source.points_[c[0]] - target.points_[c[1]])
                    .dot(target.normals_[c[1]]);
        err += r * r;
    }
    return std::sqrt(err / (float)corres.size());
}

Eigen::Matrix4f TransformationEstimationPointToPlanef::ComputeTransformation(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals())
        return Eigen::Matrix4f::Identity();

    auto compute_jacobian_and_residual = [&](int i, Eigen::Vector6f &J_r,
                                             float &r, float &w) {
        const Eigen::Vector3f &vs = source.points_[corres[i][0]];
        const Eigen::Vector3f &vt = target.points_[corres[i][1]];
        const Eigen::Vector3f &nt = target.normals_[corres[i][1]];
        r = (vs - vt).dot(nt);
        w = kernel_->Weight(r);
        J_r.block<3, 1>(0, 0) = vs.cross(nt);
        J_r.block<3, 1>(3, 0) = nt;
    };

    Eigen::Matrix6f JTJ;
    Eigen::Vector6f JTr;
    float r2;
    std::tie(JTJ, JTr, r2) =
            utility::ComputeJTJandJTrf<Eigen::Matrix6f, Eigen::Vector6f>(
                    compute_jacobian_and_residual, (int)corres.size());

    bool is_success;
    Eigen::Matrix4f extrinsic;
    std::tie(is_success, extrinsic) =
            utility::SolveJacobianSystemAndObtainExtrinsicMatrixf(JTJ, JTr);

    return is_success ? extrinsic : Eigen::Matrix4f::Identity();
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
