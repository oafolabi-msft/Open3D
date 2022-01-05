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

#include "open3d/pipelines/registration/Registrationf.h"
#include "open3d/pipelines/registration/RobustKernel.h"
#include "open3d/pipelines/registration/TransformationEstimationf.h"

namespace open3d {

namespace geometry {
class PointCloudf;
}

namespace pipelines {
namespace registration {

class RegistrationResultf;

class TransformationEstimationForColoredICPf : public TransformationEstimationf {
public:
    ~TransformationEstimationForColoredICPf() override{};

    TransformationEstimationTypef GetTransformationEstimationType()
            const override {
        return type_;
    };
    explicit TransformationEstimationForColoredICPf(
            float lambda_geometric = 0.968f,
            std::shared_ptr<RobustKernel> kernel = std::make_shared<L2Loss>())
        : lambda_geometric_(lambda_geometric), kernel_(std::move(kernel)) {
        if (lambda_geometric_ < 0.0f || lambda_geometric_ > 1.0f) {
            lambda_geometric_ = 0.968f;
        }
    }

public:
    float ComputeRMSE(const geometry::PointCloudf &source,
                       const geometry::PointCloudf &target,
                       const CorrespondenceSet &corres) const override;
    Eigen::Matrix4f ComputeTransformation(
            const geometry::PointCloudf &source,
            const geometry::PointCloudf &target,
            const CorrespondenceSet &corres) const override;

public:
    float lambda_geometric_ = 0.968f;
    /// shared_ptr to an Abstract RobustKernel that could mutate at runtime.
    std::shared_ptr<RobustKernel> kernel_ = std::make_shared<L2Loss>();

private:
    const TransformationEstimationTypef type_ =
            TransformationEstimationTypef::ColoredICPf;
};

/// \brief Function for Colored ICP registration.
///
/// This is implementation of following paper
/// J. Park, Q.-Y. Zhou, V. Koltun,
/// Colored Point Cloud Registration Revisited, ICCV 2017.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param max_distance Maximum correspondence points-pair distance.
/// \param init Initial transformation estimation.
/// Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
/// [0., 0., 0., 1.]]).
/// \param estimation TransformationEstimationForColoredICP method. Can only
/// change the lambda_geometric value and the robust kernel used in the
/// optimization
/// \param criteria  Convergence criteria.
RegistrationResultf RegistrationColoredICP(
        const geometry::PointCloudf &source,
        const geometry::PointCloudf &target,
        float max_distance,
        const Eigen::Matrix4f &init = Eigen::Matrix4f::Identity(),
        const TransformationEstimationForColoredICPf &estimation =
                TransformationEstimationForColoredICPf(),
        const ICPConvergenceCriteriaf &criteria = ICPConvergenceCriteriaf());

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
