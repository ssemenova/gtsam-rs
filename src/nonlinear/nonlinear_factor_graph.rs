use cxx::UniquePtr;

use crate::{
    base::vector::Vector3, geometry::pose3::Pose3, imu::{imu_bias::ConstantBias, imu_factor::PreintegratedImuMeasurements}, inference::key::IntoKey, linear::noise_model::{DiagonalNoiseModel, IsotropicNoiseModel}
};

pub struct NonlinearFactorGraph {
    pub(super) inner: UniquePtr<::sys::NonlinearFactorGraph>,
}

impl Default for NonlinearFactorGraph {
    fn default() -> Self {
        Self {
            inner: ::sys::default_nonlinear_factor_graph(),
        }
    }
}

impl NonlinearFactorGraph {
    pub fn add_between_factor_pose3(
        &mut self,
        from: impl IntoKey,
        to: impl IntoKey,
        measured: &Pose3,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_between_factor_pose3(
            self.inner.pin_mut(),
            from.into_key(),
            to.into_key(),
            &measured.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_pose3(
        &mut self,
        symbol: impl IntoKey,
        prior: &Pose3,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_pose3(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_constant_bias(
        &mut self,
        symbol: impl IntoKey,
        prior: &ConstantBias,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_constant_bias(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_vector3(
        &mut self,
        symbol: impl IntoKey,
        prior: &Vector3,
        model: &IsotropicNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_vector3(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }


    // pub fn add_imu_factor(
    //     &mut self,
    //     pose_i: impl IntoKey,
    //     vel_i: impl IntoKey,
    //     pose_j: impl IntoKey,
    //     vel_j: impl IntoKey,
    //     bias: impl IntoKey,
    //     preintegrated_measurements: PreintegratedImuMeasurements,
    // ) {
    //     ::sys::nonlinear_factor_graph_add_imu_factor(
    //         self.inner.pin_mut(),
    //         pose_i.into_key(),
    //         vel_i.into_key(),
    //         pose_j.into_key(),
    //         vel_j.into_key(),
    //         bias.into_key(),
    //         preintegrated_measurements,
    //     )

    // }
}
