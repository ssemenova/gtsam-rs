pub use self::ffi::*;

// Common types
pub type Key = u64;

#[cxx::bridge(namespace = "gtsam")]
mod ffi {
    unsafe extern "C++" {
        include!("base/vector.h");

        type Vector3;

        fn default_vector() -> UniquePtr<Vector3>;

        fn new_vector3(x: f64, y: f64, z: f64) -> UniquePtr<Vector3>;

        fn vector3_to_raw(src: &Vector3, dst: &mut [f64]);
    }


    unsafe extern "C++" {
        include!("geometry/point3.h");

        type Point3;

        fn default_point3() -> UniquePtr<Point3>;

        fn new_point3(x: f64, y: f64, z: f64) -> UniquePtr<Point3>;

        fn point3_to_raw(src: &Point3, dst: &mut [f64]);
    }

    unsafe extern "C++" {
        include!("geometry/pose3.h");

        type Pose3;

        fn default_pose3() -> UniquePtr<Pose3>;

        fn new_pose3(rotation: &Rot3, point: &Point3) -> UniquePtr<Pose3>;

        fn pose3_rotation(pose: &Pose3) -> &Rot3;

        fn pose3_translation(pose: &Pose3) -> &Point3;
    }

    unsafe extern "C++" {
        include!("geometry/rot3.h");

        type Rot3;

        fn default_rot3() -> UniquePtr<Rot3>;

        fn from_rot3_quaternion(w: f64, x: f64, y: f64, z: f64) -> UniquePtr<Rot3>;

        fn rot3_to_raw(src: &Rot3, dst: &mut [f64]);
    }

    #[namespace = "gtsam::imuBias"]
    unsafe extern "C++" {
        include!("imu/imu.h");
        type ConstantBias;

        fn default_constantbias() -> UniquePtr<ConstantBias>;
    }

    unsafe extern "C++" {
        include!("imu/imu.h");
        type ImuFactor;
        type PreintegratedImuMeasurements;

        fn default_imufactor() -> UniquePtr<ImuFactor>;
        fn default_preintegratedimumeasurements() -> UniquePtr<PreintegratedImuMeasurements>;

    }

    
    unsafe extern "C++" {
        include!("inference/symbol.h");

        type Symbol;

        fn default_symbol() -> UniquePtr<Symbol>;

        fn new_symbol(index: u64) -> UniquePtr<Symbol>;

        fn key(&self) -> u64;
    }

    unsafe extern "C++" {
        include!("linear/noise_model.h");

        type BaseNoiseModel;
    }

    unsafe extern "C++" {
        include!("linear/noise_model.h");

        type DiagonalNoiseModel;
        type IsotropicNoiseModel;

        fn from_diagonal_noise_model_sigmas(sigmas: &mut [f64]) -> SharedPtr<DiagonalNoiseModel>;

        fn cast_diagonal_noise_model_to_base_noise_model(
            a: &SharedPtr<DiagonalNoiseModel>,
        ) -> SharedPtr<BaseNoiseModel>;


        fn from_isotropic_noise_model_dim_and_sigma(dim: usize, sigma: f64) -> SharedPtr<IsotropicNoiseModel>;

        fn cast_isotropic_noise_model_to_base_noise_model(
            a: &SharedPtr<IsotropicNoiseModel>,
        ) -> SharedPtr<BaseNoiseModel>;
    }

    unsafe extern "C++" {
        include!("nonlinear/levenberg_marquardt_optimizer.h");

        type LevenbergMarquardtOptimizer;

        fn new_levenberg_marquardt_optimizer(
            graph: &NonlinearFactorGraph,
            initial_values: &Values,
            params: &LevenbergMarquardtParams,
        ) -> UniquePtr<LevenbergMarquardtOptimizer>;

        fn optimizeSafely(self: Pin<&mut Self>) -> &Values;
    }

    unsafe extern "C++" {
        include!("nonlinear/levenberg_marquardt_params.h");

        type LevenbergMarquardtParams;

        fn default_levenberg_marquardt_params() -> UniquePtr<LevenbergMarquardtParams>;

        fn levenberg_marquardt_params_set_max_iterations(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: u32,
        );

        fn levenberg_marquardt_params_set_lambda_upper_bound(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );

        fn levenberg_marquardt_params_set_lambda_lower_bound(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );


        fn levenberg_marquardt_params_set_diagonal_damping(
            params: Pin<&mut LevenbergMarquardtParams>,
            flag: bool,
        );


        fn levenberg_marquardt_params_set_relative_error_to_l(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );


        fn levenberg_marquardt_params_set_absolute_error_to_l(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );

    }

    unsafe extern "C++" {
        include!("nonlinear/nonlinear_factor_graph.h");

        type NonlinearFactorGraph;

        fn default_nonlinear_factor_graph() -> UniquePtr<NonlinearFactorGraph>;

        fn nonlinear_factor_graph_add_between_factor_pose3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key1: u64,
            key2: u64,
            measured: &Pose3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_prior_factor_pose3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &Pose3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_prior_factor_constant_bias(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &ConstantBias,
            model: &SharedPtr<BaseNoiseModel>,
        );


        fn nonlinear_factor_graph_add_prior_factor_vector3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &Vector3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_imu_factor(
            graph: Pin<&mut NonlinearFactorGraph>,
            pose_i: u64,
            vel_i: u64,
            pose_j: u64,
            vel_j: u64,
            bias: u64,
            preintegrated_measurements: &SharedPtr<PreintegratedImuMeasurements>,
        );
    }

    unsafe extern "C++" {
        include!("nonlinear/values.h");

        type Values;

        fn default_values() -> UniquePtr<Values>;

        fn values_at_pose3(values: &Values, key: u64) -> &Pose3;

        fn values_exists(values: &Values, key: u64) -> bool;

        fn values_insert_pose3(values: Pin<&mut Values>, key: u64, value: &Pose3);

        fn values_insert_constant_bias(values: Pin<&mut Values>, key: u64, value: &ConstantBias);

        fn values_insert_vector3(values: Pin<&mut Values>, key: u64, value: &Vector3);

    }
}
