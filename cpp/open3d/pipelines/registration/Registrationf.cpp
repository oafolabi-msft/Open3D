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

#include "open3d/pipelines/registration/Registrationf.h"

#include "open3d/geometry/KDTreeFlannf.h"
#include "open3d/geometry/PointCloudf.h"
#include "open3d/pipelines/registration/Feature.h"
#include "open3d/utility/Console.h"
#include "open3d/utility/Helper.h"

namespace open3d {
namespace pipelines {
namespace registration {

static RegistrationResultf GetRegistrationResultAndCorrespondences(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const geometry::KDTreeFlannf &target_kdtree,
        float max_correspondence_distance,
        const Eigen::Matrix4f &transformation) {
    RegistrationResultf result(transformation);
    if (max_correspondence_distance <= 0.0f) {
        return result;
    }

    float error2 = 0.0f;

#pragma omp parallel
    {
        float error2_private = 0.0f;
        CorrespondenceSet correspondence_set_private;
#pragma omp for nowait
        for (int i = 0; i < (int)source.points_.size(); i++) {
            std::vector<int> indices(1);
            std::vector<float> dists(1);
            const auto &point = source.points_[i];
            if (target_kdtree.SearchHybrid(point, max_correspondence_distance,
                                           1, indices, dists) > 0) {
                error2_private += dists[0];
                correspondence_set_private.push_back(
                        Eigen::Vector2i(i, indices[0]));
            }
        }
#pragma omp critical
        {
            for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                result.correspondence_set_.push_back(
                        correspondence_set_private[i]);
            }
            error2 += error2_private;
        }
    }

    if (result.correspondence_set_.empty()) {
        result.fitness_ = 0.0f;
        result.inlier_rmse_ = 0.0f;
    } else {
        size_t corres_number = result.correspondence_set_.size();
        result.fitness_ = (float)corres_number / (float)source.points_.size();
        result.inlier_rmse_ = std::sqrt(error2 / (float)corres_number);
    }
    return result;
}

static RegistrationResultf EvaluateRANSACBasedOnCorrespondence(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres,
        float max_correspondence_distance,
        const Eigen::Matrix4f &transformation) {
    RegistrationResultf result(transformation);
    float error2 = 0.0;
    int good = 0;
    float max_dis2 = max_correspondence_distance * max_correspondence_distance;
    for (const auto &c : corres) {
        float dis2 =
                (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
        if (dis2 < max_dis2) {
            good++;
            error2 += dis2;
            result.correspondence_set_.push_back(c);
        }
    }
    if (good == 0) {
        result.fitness_ = 0.0f;
        result.inlier_rmse_ = 0.0f;
    } else {
        result.fitness_ = (float)good / (float)corres.size();
        result.inlier_rmse_ = std::sqrt(error2 / (float)good);
    }
    return result;
}

RegistrationResultf EvaluateRegistrationf(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        float max_correspondence_distance,
        const Eigen::Matrix4f
                &transformation /* = Eigen::Matrix4d::Identity()*/) {
    geometry::KDTreeFlannf kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloudf pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    return GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
}

RegistrationResultf RegistrationICP(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        float max_correspondence_distance,
        const Eigen::Matrix4f &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimationf &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        const ICPConvergenceCriteriaf
                &criteria /* = ICPConvergenceCriteria()*/) {
    if (max_correspondence_distance <= 0.0f) {
        utility::LogError("Invalid max_correspondence_distance.");
    }
    if ((estimation.GetTransformationEstimationType() ==
                 TransformationEstimationTypef::PointToPlanef ||
         estimation.GetTransformationEstimationType() ==
                 TransformationEstimationTypef::ColoredICPf) &&
        (!target.HasNormals())) {
        utility::LogError(
                "TransformationEstimationPointToPlanef and "
                "TransformationEstimationColoredICPf "
                "require pre-computed normal vectors for target PointCloudf.");
    }

    Eigen::Matrix4f transformation = init;
    geometry::KDTreeFlannf kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloudf pcd = source;
    if (!init.isIdentity()) {
        pcd.Transform(init);
    }
    RegistrationResultf result;
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
    for (int i = 0; i < criteria.max_iteration_; i++) {
        utility::LogDebug("ICP Iteration #{:d}: Fitness {:.4f}, RMSE {:.4f}", i,
                          result.fitness_, result.inlier_rmse_);
        Eigen::Matrix4f update = estimation.ComputeTransformation(
                pcd, target, result.correspondence_set_);
        transformation = update * transformation;
        pcd.Transform(update);
        RegistrationResultf backup = result;
        result = GetRegistrationResultAndCorrespondences(
                pcd, target, kdtree, max_correspondence_distance,
                transformation);

        if (std::abs(backup.fitness_ - result.fitness_) <
                    criteria.relative_fitness_ &&
            std::abs(backup.inlier_rmse_ - result.inlier_rmse_) <
                    criteria.relative_rmse_) {
            break;
        }
    }
    return result;
}

RegistrationResultf RegistrationRANSACBasedOnCorrespondence(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        const CorrespondenceSet &corres,
        float max_correspondence_distance,
        const TransformationEstimationf &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<const CorrespondenceCheckerf>>
                &checkers /* = {}*/,
        const RANSACConvergenceCriteriaf &criteria
        /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || (int)corres.size() < ransac_n ||
        max_correspondence_distance <= 0.0f) {
        return RegistrationResultf();
    }

    RegistrationResultf best_result;
    int exit_itr = -1;

#pragma omp parallel
    {
        CorrespondenceSet ransac_corres(ransac_n);
        RegistrationResultf best_result_local;
        int exit_itr_local = criteria.max_iteration_;

#pragma omp for nowait
        for (int itr = 0; itr < criteria.max_iteration_; itr++) {
            if (itr < exit_itr_local) {
                for (int j = 0; j < ransac_n; j++) {
                    ransac_corres[j] = corres[utility::UniformRandInt(
                            0, static_cast<int>(corres.size()) - 1)];
                }

                Eigen::Matrix4f transformation =
                        estimation.ComputeTransformation(source, target,
                                                         ransac_corres);

                // Check transformation: inexpensive
                bool check = true;
                for (const auto &checker : checkers) {
                    if (!checker.get().Check(source, target, ransac_corres,
                                             transformation)) {
                        check = false;
                        break;
                    }
                }
                if (!check) continue;

                geometry::PointCloudf pcd = source;
                pcd.Transform(transformation);
                auto result = EvaluateRANSACBasedOnCorrespondence(
                        pcd, target, corres, max_correspondence_distance,
                        transformation);

                if (result.IsBetterRANSACThan(best_result_local)) {
                    best_result_local = result;

                    // Update exit condition if necessary
                    float exit_itr_d =
                            std::log(1.0f - criteria.confidence_) /
                            std::log(1.0f - std::pow(result.fitness_, ransac_n));
                    exit_itr_local =
                            exit_itr_d < float(criteria.max_iteration_)
                                    ? static_cast<int>(std::ceil(exit_itr_d))
                                    : exit_itr_local;
                }
            }  // if < exit_itr_local
        }      // for loop
#pragma omp critical
        {
            if (best_result_local.IsBetterRANSACThan(best_result)) {
                best_result = best_result_local;
            }
            if (exit_itr_local > exit_itr) {
                exit_itr = exit_itr_local;
            }
        }
    }
    utility::LogDebug(
            "RANSAC exits at {:d}-th iteration: inlier ratio {:e}, "
            "RMSE {:e}",
            exit_itr, best_result.fitness_, best_result.inlier_rmse_);
    return best_result;
}

// RegistrationResultf RegistrationRANSACBasedOnFeatureMatching(
//         const geometry::PointCloud &source,
//         const geometry::PointCloud &target,
//         const Feature &source_feature,
//         const Feature &target_feature,
//         bool mutual_filter,
//         double max_correspondence_distance,
//         const TransformationEstimation &estimation
//         /* = TransformationEstimationPointToPoint(false)*/,
//         int ransac_n /* = 3*/,
//         const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
//                 &checkers /* = {}*/,
//         const RANSACConvergenceCriteria &criteria
//         /* = RANSACConvergenceCriteria()*/) {
//     if (ransac_n < 3 || max_correspondence_distance <= 0.0) {
//         return RegistrationResult();
//     }

//     int num_src_pts = int(source.points_.size());
//     int num_tgt_pts = int(target.points_.size());

//     geometry::KDTreeFlann kdtree_target(target_feature);
//     pipelines::registration::CorrespondenceSet corres_ij(num_src_pts);

// #pragma omp parallel for
//     for (int i = 0; i < num_src_pts; i++) {
//         std::vector<int> corres_tmp(1);
//         std::vector<double> dist_tmp(1);

//         kdtree_target.SearchKNN(Eigen::VectorXd(source_feature.data_.col(i)), 1,
//                                 corres_tmp, dist_tmp);
//         int j = corres_tmp[0];
//         corres_ij[i] = Eigen::Vector2i(i, j);
//     }

//     // Do reverse check if mutual_filter is enabled
//     if (mutual_filter) {
//         geometry::KDTreeFlann kdtree_source(source_feature);
//         pipelines::registration::CorrespondenceSet corres_ji(num_tgt_pts);

// #pragma omp parallel for
//         for (int j = 0; j < num_tgt_pts; ++j) {
//             std::vector<int> corres_tmp(1);
//             std::vector<double> dist_tmp(1);
//             kdtree_source.SearchKNN(
//                     Eigen::VectorXd(target_feature.data_.col(j)), 1, corres_tmp,
//                     dist_tmp);
//             int i = corres_tmp[0];
//             corres_ji[j] = Eigen::Vector2i(i, j);
//         }

//         pipelines::registration::CorrespondenceSet corres_mutual;
//         for (int i = 0; i < num_src_pts; ++i) {
//             int j = corres_ij[i](1);
//             if (corres_ji[j](0) == i) {
//                 corres_mutual.emplace_back(i, j);
//             }
//         }

//         // Empirically mutual correspondence set should not be too small
//         if (int(corres_mutual.size()) >= ransac_n * 3) {
//             utility::LogDebug("{:d} correspondences remain after mutual filter",
//                               corres_mutual.size());
//             return RegistrationRANSACBasedOnCorrespondence(
//                     source, target, corres_mutual, max_correspondence_distance,
//                     estimation, ransac_n, checkers, criteria);
//         }
//         utility::LogDebug(
//                 "Too few correspondences after mutual filter, fall back to "
//                 "original correspondences.");
//     }

//     return RegistrationRANSACBasedOnCorrespondence(
//             source, target, corres_ij, max_correspondence_distance, estimation,
//             ransac_n, checkers, criteria);
// }

Eigen::Matrix6f GetInformationMatrixFromPointClouds(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        float max_correspondence_distance,
        const Eigen::Matrix4f &transformation) {
    geometry::PointCloudf pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    RegistrationResultf result;
    geometry::KDTreeFlannf target_kdtree(target);
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, target_kdtree, max_correspondence_distance,
            transformation);

    // write q^*
    // see http://redwood-data.org/indoor/registration.html
    // note: I comes first in this implementation
    Eigen::Matrix6f GTG = Eigen::Matrix6f::Zero();
#pragma omp parallel
    {
        Eigen::Matrix6f GTG_private = Eigen::Matrix6f::Zero();
        Eigen::Vector6f G_r_private = Eigen::Vector6f::Zero();
#pragma omp for nowait
        for (int c = 0; c < int(result.correspondence_set_.size()); c++) {
            int t = result.correspondence_set_[c](1);
            float x = target.points_[t](0);
            float y = target.points_[t](1);
            float z = target.points_[t](2);
            G_r_private.setZero();
            G_r_private(1) = z;
            G_r_private(2) = -y;
            G_r_private(3) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = -z;
            G_r_private(2) = x;
            G_r_private(4) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = y;
            G_r_private(1) = -x;
            G_r_private(5) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
        }
#pragma omp critical
        { GTG += GTG_private; }
    }
    return GTG;
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
