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

#include "open3d/pipelines/registration/ColoredICPf.h"

#include <Eigen/Dense>
#include <iostream>

#include "open3d/geometry/KDTreeFlannf.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/geometry/PointCloudf.h"
#include "open3d/pipelines/registration/RobustKernel.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/Eigen.h"

namespace open3d {
namespace pipelines {
namespace registration {

namespace {

class PointCloudForColoredICPf : public geometry::PointCloudf {
public:
    std::vector<Eigen::Vector3f> color_gradient_;
};

std::shared_ptr<PointCloudForColoredICPf> InitializePointCloudForColoredICP(
        const geometry::PointCloudf &target,
        const geometry::KDTreeSearchParamHybrid &search_param) {
    utility::LogDebug("InitializePointCloudForColoredICPf");

    geometry::KDTreeFlannf tree;
    tree.SetGeometry(target);

    auto output = std::make_shared<PointCloudForColoredICPf>();
    output->colors_ = target.colors_;
    output->normals_ = target.normals_;
    output->points_ = target.points_;

    size_t n_points = output->points_.size();
    output->color_gradient_.resize(n_points, Eigen::Vector3f::Zero());

    for (size_t k = 0; k < n_points; k++) {
        const Eigen::Vector3f &vt = output->points_[k];
        const Eigen::Vector3f &nt = output->normals_[k];
        float it = (output->colors_[k](0) + output->colors_[k](1) +
                     output->colors_[k](2)) /
                    3.0f;

        std::vector<int> point_idx;
        std::vector<float> point_squared_distance;

        if (tree.SearchHybrid(vt, search_param.radius_, search_param.max_nn_,
                              point_idx, point_squared_distance) >= 4) {
            // approximate image gradient of vt's tangential plane
            size_t nn = point_idx.size();
            Eigen::MatrixXf A(nn, 3);
            Eigen::MatrixXf b(nn, 1);
            A.setZero();
            b.setZero();
            for (size_t i = 1; i < nn; i++) {
                int P_adj_idx = point_idx[i];
                Eigen::Vector3f vt_adj = output->points_[P_adj_idx];
                Eigen::Vector3f vt_proj = vt_adj - (vt_adj - vt).dot(nt) * nt;
                float it_adj = (output->colors_[P_adj_idx](0) +
                                 output->colors_[P_adj_idx](1) +
                                 output->colors_[P_adj_idx](2)) /
                                3.0f;
                A(i - 1, 0) = (vt_proj(0) - vt(0));
                A(i - 1, 1) = (vt_proj(1) - vt(1));
                A(i - 1, 2) = (vt_proj(2) - vt(2));
                b(i - 1, 0) = (it_adj - it);
            }
            // adds orthogonal constraint
            A(nn - 1, 0) = (nn - 1) * nt(0);
            A(nn - 1, 1) = (nn - 1) * nt(1);
            A(nn - 1, 2) = (nn - 1) * nt(2);
            b(nn - 1, 0) = 0.0f;
            // solving linear equation
            bool is_success = false;
            Eigen::MatrixXf x;
            std::tie(is_success, x) = utility::SolveLinearSystemPSDf(
                    A.transpose() * A, A.transpose() * b);
            if (is_success) {
                output->color_gradient_[k] = x;
            }
        }
    }
    return output;
}

}  // namespace

Eigen::Matrix4f TransformationEstimationForColoredICPf::ComputeTransformation(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) {
        utility::LogError(
                "No correspondences found between source and target "
                "pointcloud.");
    }
    if (!target.HasNormals()) {
        utility::LogError(
                "ColoredICPf requires target pointcloud to have normals.");
    }
    if (!target.HasColors()) {
        utility::LogError(
                "ColoredICPf requires target pointcloud to have colors.");
    }
    if (!source.HasColors()) {
        utility::LogError(
                "ColoredICPf requires source pointcloud to have colors.");
    }

    float sqrt_lambda_geometric = sqrt(lambda_geometric_);
    float lambda_photometric = 1.0f - lambda_geometric_;
    float sqrt_lambda_photometric = sqrt(lambda_photometric);

    const auto &target_c = (const PointCloudForColoredICPf &)target;

    auto compute_jacobian_and_residual =
            [&](int i,
                std::vector<Eigen::Vector6f, utility::Vector6f_allocator> &J_r,
                std::vector<float> &r, std::vector<float> &w) {
                size_t cs = corres[i][0];
                size_t ct = corres[i][1];
                const Eigen::Vector3f &vs = source.points_[cs];
                const Eigen::Vector3f &vt = target.points_[ct];
                const Eigen::Vector3f &nt = target.normals_[ct];

                J_r.resize(2);
                r.resize(2);
                w.resize(2);

                J_r[0].block<3, 1>(0, 0) = sqrt_lambda_geometric * vs.cross(nt);
                J_r[0].block<3, 1>(3, 0) = sqrt_lambda_geometric * nt;
                r[0] = sqrt_lambda_geometric * (vs - vt).dot(nt);
                w[0] = kernel_->Weight(r[0]);

                // project vs into vt's tangential plane
                Eigen::Vector3f vs_proj = vs - (vs - vt).dot(nt) * nt;
                float is = (source.colors_[cs](0) + source.colors_[cs](1) +
                             source.colors_[cs](2)) /
                            3.0f;
                float it = (target.colors_[ct](0) + target.colors_[ct](1) +
                             target.colors_[ct](2)) /
                            3.0f;
                const Eigen::Vector3f &dit = target_c.color_gradient_[ct];
                float is0_proj = (dit.dot(vs_proj - vt)) + it;

                const Eigen::Matrix3f M =
                        (Eigen::Matrix3f() << 1.0 - nt(0) * nt(0),
                         -nt(0) * nt(1), -nt(0) * nt(2), -nt(0) * nt(1),
                         1.0 - nt(1) * nt(1), -nt(1) * nt(2), -nt(0) * nt(2),
                         -nt(1) * nt(2), 1.0 - nt(2) * nt(2))
                                .finished();

                const Eigen::Vector3f &ditM = -dit.transpose() * M;
                J_r[1].block<3, 1>(0, 0) =
                        sqrt_lambda_photometric * vs.cross(ditM);
                J_r[1].block<3, 1>(3, 0) = sqrt_lambda_photometric * ditM;
                r[1] = sqrt_lambda_photometric * (is - is0_proj);
                w[1] = kernel_->Weight(r[1]);
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

float TransformationEstimationForColoredICPf::ComputeRMSE(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres) const {
    float sqrt_lambda_geometric = sqrt(lambda_geometric_);
    float lambda_photometric = 1.0f - lambda_geometric_;
    float sqrt_lambda_photometric = sqrt(lambda_photometric);
    const auto &target_c = (const PointCloudForColoredICPf &)target;

    float residual = 0.0f;
    for (size_t i = 0; i < corres.size(); i++) {
        size_t cs = corres[i][0];
        size_t ct = corres[i][1];
        const Eigen::Vector3f &vs = source.points_[cs];
        const Eigen::Vector3f &vt = target.points_[ct];
        const Eigen::Vector3f &nt = target.normals_[ct];
        Eigen::Vector3f vs_proj = vs - (vs - vt).dot(nt) * nt;
        float is = (source.colors_[cs](0) + source.colors_[cs](1) +
                     source.colors_[cs](2)) /
                    3.0f;
        float it = (target.colors_[ct](0) + target.colors_[ct](1) +
                     target.colors_[ct](2)) /
                    3.0f;
        const Eigen::Vector3f &dit = target_c.color_gradient_[ct];
        float is0_proj = (dit.dot(vs_proj - vt)) + it;
        float residual_geometric = sqrt_lambda_geometric * (vs - vt).dot(nt);
        float residual_photometric = sqrt_lambda_photometric * (is - is0_proj);
        residual += residual_geometric * residual_geometric +
                    residual_photometric * residual_photometric;
    }
    return residual;
};

RegistrationResultf RegistrationColoredICP(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        float max_distance,
        const Eigen::Matrix4f &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimationForColoredICPf &estimation
        /* = TransformationEstimationForColoredICP()*/,
        const ICPConvergenceCriteriaf
                &criteria /* = ICPConvergenceCriteria()*/) {
    if (!target.HasNormals()) {
        utility::LogError(
                "ColoredICPf requires target pointcloud to have normals.");
    }
    if (!target.HasColors()) {
        utility::LogError(
                "ColoredICPf requires target pointcloud to have colors.");
    }
    if (!source.HasColors()) {
        utility::LogError(
                "ColoredICPf requires source pointcloud to have colors.");
    }

    auto target_c = InitializePointCloudForColoredICP(
            target, geometry::KDTreeSearchParamHybrid(max_distance * 2.0f, 30));
    return RegistrationICP(source, *target_c, max_distance, init, estimation,
                           criteria);
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
