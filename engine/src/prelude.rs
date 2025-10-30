/// This module unifies types between std and no_std usage.

mod core {
    #[cfg(feature = "std")]
    pub use std::*;

    #[cfg(not(feature = "std"))]
    pub use core::*;
}

pub use self::core::{cmp, iter, mem, ops, str};
pub use self::core::{i8, i16, i32, i64, isize};
pub use self::core::{u8, u16, u32, u64, usize};
pub use self::core::{f32, f64};

pub use self::core::cell::{Cell, RefCell};
pub use self::core::clone::{self, Clone};
pub use self::core::convert::{self, From, Into};
pub use self::core::default::{self, Default};
pub use self::core::fmt::{self, Debug, Display};
pub use self::core::marker::{self, PhantomData, Send};
pub use self::core::option::{self, Option};
pub use self::core::result::{self, Result};
pub use self::core::future::{self, Future};
