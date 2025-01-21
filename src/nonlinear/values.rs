use cxx::UniquePtr;

use crate::{
    geometry::pose3::{Pose3, Pose3Ref}, imu::imu_bias, inference::key::IntoKey, base::vector::Vector3,
};

pub struct Values {
    pub(super) inner: UniquePtr<::sys::Values>,
}

impl Default for Values {
    fn default() -> Self {
        Self {
            inner: ::sys::default_values(),
        }
    }
}

impl Values {
    pub fn insert_pose3(&mut self, key: impl IntoKey, value: &Pose3) {
        ::sys::values_insert_pose3(self.inner.pin_mut(), key.into_key(), &value.inner)
    }
    pub fn insert_constant_bias(&mut self, key: impl IntoKey, value: &imu_bias::ConstantBias) {
        ::sys::values_insert_constant_bias(self.inner.pin_mut(), key.into_key(), &value.inner)
    }
    pub fn insert_vector3(&mut self, key: impl IntoKey, value: &Vector3) {
        ::sys::values_insert_vector3(self.inner.pin_mut(), key.into_key(), &value.inner)
    }

}

pub struct ValuesRef<'a> {
    pub(super) inner: &'a ::sys::Values,
}

impl<'a> ValuesRef<'a> {
    pub fn get_pose3(&self, key: impl IntoKey) -> Option<Pose3Ref> {
        let key = key.into_key();

        if ::sys::values_exists(self.inner, key) {
            Some(Pose3Ref {
                inner: ::sys::values_at_pose3(self.inner, key),
            })
        } else {
            None
        }
    }
}
