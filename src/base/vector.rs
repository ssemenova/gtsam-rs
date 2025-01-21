use cxx::UniquePtr;

pub struct Vector3 {
    pub(crate) inner: UniquePtr<::sys::Vector3>,
}

impl Default for Vector3 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_vector(),
        }
    }
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: ::sys::new_vector3(x, y, z),
        }
    }
}


impl From<::nalgebra::Vector3<f64>> for Vector3 {
    fn from(value: ::nalgebra::Vector3<f64>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}
