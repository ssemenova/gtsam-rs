use cxx::UniquePtr;

pub struct ConstantBias {
    pub(crate) inner: UniquePtr<::sys::ConstantBias>,
}

impl Default for ConstantBias {
    fn default() -> Self {
        Self {
            inner: ::sys::default_constantbias(),
        }
    }
}