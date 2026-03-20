use std::{collections::HashMap, hash::Hash, time::Duration};

use half;

pub trait Compress: Sized {
    type Compressed;

    fn compress(&self) -> Self::Compressed;
    fn decompress(val: Self::Compressed) -> Self;
}

impl Compress for f64 {
    type Compressed = [u8; 2];

    fn compress(&self) -> Self::Compressed {
        half::f16::from_f64(*self).to_bits().to_le_bytes()
    }

    fn decompress(val: Self::Compressed) -> Self {
        half::f16::from_bits(u16::from_le_bytes(val)).to_f64()
    }
}

impl Compress for f32 {
    type Compressed = [u8; 2];

    fn compress(&self) -> Self::Compressed {
        half::f16::from_f32(*self).to_bits().to_le_bytes()
    }

    fn decompress(val: Self::Compressed) -> Self {
        half::f16::from_bits(u16::from_le_bytes(val)).to_f32()
    }
}

impl<T: Compress + Clone, const N: usize> Compress for [T; N] {
    type Compressed = [T::Compressed; N];

    fn compress(&self) -> Self::Compressed {
        self.clone().map(|val| val.compress())
    }

    fn decompress(val: Self::Compressed) -> Self {
        val.map(T::decompress)
    }
}

impl<T: Compress> Compress for Option<T> {
    type Compressed = Option<T::Compressed>;

    fn compress(&self) -> Self::Compressed {
        self.as_ref().map(T::compress)
    }

    fn decompress(val: Self::Compressed) -> Self {
        val.map(T::decompress)
    }
}

impl<T: Compress> Compress for Vec<T> {
    type Compressed = Vec<T::Compressed>;

    fn compress(&self) -> Self::Compressed {
        self.iter().map(T::compress).collect()
    }

    fn decompress(val: Self::Compressed) -> Self {
        val.into_iter().map(T::decompress).collect()
    }
}

impl<K, V> Compress for HashMap<K, V>
where
    K: Clone + Eq + Hash,
    V: Compress,
{
    type Compressed = HashMap<K, V::Compressed>;

    fn compress(&self) -> Self::Compressed {
        self.iter().map(|(k, v)| (k.clone(), v.compress())).collect()
    }

    fn decompress(val: Self::Compressed) -> Self {
        val.into_iter().map(|(k, v)| (k, V::decompress(v))).collect()
    }
}

/// Used to automatically derive Compress for a type that doesn't need to be
/// compressed; compress() and decompress() will just return the type without
/// any modifications.
#[macro_export]
macro_rules! compress_identity_impl {
    ($ty:ty) => {
        impl ::compaq::Compress for $ty {
            type Compressed = Self;

            fn compress(&self) -> Self::Compressed {
                (*self).clone()
            }

            fn decompress(val: Self::Compressed) -> Self {
                val
            }
        }
    };
}

macro_rules! compress_identity_internal_impl {
    ($ty:ty) => {
        impl Compress for $ty {
            type Compressed = Self;

            fn compress(&self) -> Self::Compressed {
                (*self).clone()
            }

            fn decompress(val: Self::Compressed) -> Self {
                val
            }
        }
    };
}

compress_identity_internal_impl!(half::f16);

compress_identity_internal_impl!(i8);
compress_identity_internal_impl!(i16);
compress_identity_internal_impl!(i32);
compress_identity_internal_impl!(i64);
compress_identity_internal_impl!(i128);
compress_identity_internal_impl!(isize);

compress_identity_internal_impl!(u8);
compress_identity_internal_impl!(u16);
compress_identity_internal_impl!(u32);
compress_identity_internal_impl!(u64);
compress_identity_internal_impl!(u128);
compress_identity_internal_impl!(usize);

compress_identity_internal_impl!(bool);
compress_identity_internal_impl!(char);
compress_identity_internal_impl!(());

compress_identity_internal_impl!(String);
compress_identity_internal_impl!(Duration);
