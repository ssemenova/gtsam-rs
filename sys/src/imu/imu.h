#pragma once

#include "rust/cxx.h"
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <memory>

namespace gtsam
{
namespace imuBias
{
    std::unique_ptr<ConstantBias> default_constantbias();

    std::unique_ptr<ConstantBias> new_constantbias(const Vector3 &biasAcc, const Vector3 &biasGyro);

    // void point3_to_raw(const Point3 &src, rust::Slice<double> dst);
}

std::unique_ptr<ImuFactor> default_imufactor();

std::unique_ptr<ImuFactor> new_imufactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
                                         const PreintegratedImuMeasurements &preintegratedMeasurements);

std::unique_ptr<PreintegratedImuMeasurements> default_preintegratedimumeasurements();

std::unique_ptr<PreintegratedImuMeasurements> new_preintegratedimumeasurements();

} // namespace gtsam
