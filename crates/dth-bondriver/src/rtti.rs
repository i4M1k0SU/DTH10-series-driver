//! Hand-made MSVC RTTI so that `dynamic_cast<IBonDriver2*>` (used by
//! TVTest / LibISDB on the object returned by `CreateBonDriver`) succeeds.
//!
//! MSVC's `__RTDynamicCast` reads the Complete Object Locator pointer stored
//! just before the vtable (`vptr[-1]`), follows it to the class hierarchy
//! descriptor and compares *type names* (`.?AVIBonDriver2@@`) of the base
//! class descriptors. Without these structures the runtime throws
//! `__non_rtti_object`, which TVTest does not catch (abort, 0xC0000409/7).
//!
//! x64 uses image-relative 32-bit offsets. The COL signature 1 makes the
//! runtime derive the "image base" as `&col - col.self_offset`, so with all
//! structures packed into one static blob and `self_offset = 0` every offset
//! is simply the field offset inside the blob. x86 uses absolute pointers.

use core::ffi::c_void;

#[repr(C)]
pub struct TypeDescriptor<const N: usize> {
    pub vftable: *const c_void,
    pub spare: *const c_void,
    pub name: [u8; N],
}
unsafe impl<const N: usize> Sync for TypeDescriptor<N> {}

#[repr(C)]
pub struct Pmd {
    pub mdisp: i32,
    pub pdisp: i32,
    pub vdisp: i32,
}

const NAME_SELF: [u8; 24] = *b".?AVBonDriverDTH10@@\0\0\0\0";
const NAME_BON2: [u8; 24] = *b".?AVIBonDriver2@@\0\0\0\0\0\0\0";
const NAME_BON1: [u8; 24] = *b".?AVIBonDriver@@\0\0\0\0\0\0\0\0";

#[cfg(target_pointer_width = "64")]
mod imp {
    use super::*;
    use core::mem::{offset_of, size_of};

    #[repr(C)]
    pub struct BaseClassDescriptor {
        pub type_descriptor: u32,
        pub num_contained_bases: u32,
        pub where_: Pmd,
        pub attributes: u32,
        pub class_descriptor: u32,
    }

    #[repr(C)]
    pub struct ClassHierarchyDescriptor {
        pub signature: u32,
        pub attributes: u32,
        pub num_base_classes: u32,
        pub base_class_array: u32,
    }

    #[repr(C)]
    pub struct CompleteObjectLocator {
        pub signature: u32,
        pub offset: u32,
        pub cd_offset: u32,
        pub type_descriptor: u32,
        pub class_descriptor: u32,
        pub self_offset: u32,
    }

    #[repr(C)]
    pub struct Blob {
        pub col: CompleteObjectLocator,
        pub chd: ClassHierarchyDescriptor,
        pub bca: [u32; 3],
        pub bcd: [BaseClassDescriptor; 3],
        pub td_self: TypeDescriptor<24>,
        pub td_bon2: TypeDescriptor<24>,
        pub td_bon1: TypeDescriptor<24>,
    }
    unsafe impl Sync for Blob {}

    const BCD_SIZE: u32 = size_of::<BaseClassDescriptor>() as u32;
    const fn bcd(td: u32, bases: u32) -> BaseClassDescriptor {
        BaseClassDescriptor {
            type_descriptor: td,
            num_contained_bases: bases,
            where_: Pmd { mdisp: 0, pdisp: -1, vdisp: 0 },
            attributes: 0x40,
            class_descriptor: offset_of!(Blob, chd) as u32,
        }
    }

    pub static RTTI: Blob = Blob {
        col: CompleteObjectLocator {
            signature: 1,
            offset: 0,
            cd_offset: 0,
            type_descriptor: offset_of!(Blob, td_self) as u32,
            class_descriptor: offset_of!(Blob, chd) as u32,
            self_offset: 0,
        },
        chd: ClassHierarchyDescriptor {
            signature: 0,
            attributes: 0,
            num_base_classes: 3,
            base_class_array: offset_of!(Blob, bca) as u32,
        },
        bca: [
            offset_of!(Blob, bcd) as u32,
            offset_of!(Blob, bcd) as u32 + BCD_SIZE,
            offset_of!(Blob, bcd) as u32 + 2 * BCD_SIZE,
        ],
        bcd: [
            bcd(offset_of!(Blob, td_self) as u32, 2),
            bcd(offset_of!(Blob, td_bon2) as u32, 1),
            bcd(offset_of!(Blob, td_bon1) as u32, 0),
        ],
        td_self: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_SELF },
        td_bon2: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_BON2 },
        td_bon1: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_BON1 },
    };

    /// Pointer to store at `vptr[-1]`.
    pub const fn col_ptr() -> *const c_void {
        &RTTI.col as *const CompleteObjectLocator as *const c_void
    }
}

#[cfg(target_pointer_width = "32")]
mod imp {
    use super::*;

    #[repr(C)]
    pub struct BaseClassDescriptor {
        pub type_descriptor: *const TypeDescriptor<24>,
        pub num_contained_bases: u32,
        pub where_: Pmd,
        pub attributes: u32,
        pub class_descriptor: *const ClassHierarchyDescriptor,
    }

    #[repr(C)]
    pub struct ClassHierarchyDescriptor {
        pub signature: u32,
        pub attributes: u32,
        pub num_base_classes: u32,
        pub base_class_array: *const [*const BaseClassDescriptor; 3],
    }

    #[repr(C)]
    pub struct CompleteObjectLocator {
        pub signature: u32,
        pub offset: u32,
        pub cd_offset: u32,
        pub type_descriptor: *const TypeDescriptor<24>,
        pub class_descriptor: *const ClassHierarchyDescriptor,
    }

    #[repr(C)]
    pub struct Blob {
        pub col: CompleteObjectLocator,
        pub chd: ClassHierarchyDescriptor,
        pub bca: [*const BaseClassDescriptor; 3],
        pub bcd: [BaseClassDescriptor; 3],
        pub td_self: TypeDescriptor<24>,
        pub td_bon2: TypeDescriptor<24>,
        pub td_bon1: TypeDescriptor<24>,
    }
    unsafe impl Sync for Blob {}

    const fn bcd(td: *const TypeDescriptor<24>, bases: u32) -> BaseClassDescriptor {
        BaseClassDescriptor {
            type_descriptor: td,
            num_contained_bases: bases,
            where_: Pmd { mdisp: 0, pdisp: -1, vdisp: 0 },
            attributes: 0x40,
            class_descriptor: &RTTI.chd,
        }
    }

    pub static RTTI: Blob = Blob {
        col: CompleteObjectLocator {
            signature: 0,
            offset: 0,
            cd_offset: 0,
            type_descriptor: &RTTI.td_self,
            class_descriptor: &RTTI.chd,
        },
        chd: ClassHierarchyDescriptor { signature: 0, attributes: 0, num_base_classes: 3, base_class_array: &RTTI.bca },
        bca: [&RTTI.bcd[0], &RTTI.bcd[1], &RTTI.bcd[2]],
        bcd: [bcd(&RTTI.td_self, 2), bcd(&RTTI.td_bon2, 1), bcd(&RTTI.td_bon1, 0)],
        td_self: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_SELF },
        td_bon2: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_BON2 },
        td_bon1: TypeDescriptor { vftable: core::ptr::null(), spare: core::ptr::null(), name: NAME_BON1 },
    };

    pub const fn col_ptr() -> *const c_void {
        &RTTI.col as *const CompleteObjectLocator as *const c_void
    }
}

#[allow(unused_imports)]
pub use imp::RTTI;
pub use imp::col_ptr;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_nul_terminated_decorated_names() {
        assert!(RTTI.td_bon2.name.starts_with(b".?AVIBonDriver2@@\0"));
        assert!(RTTI.td_bon1.name.starts_with(b".?AVIBonDriver@@\0"));
        assert!(!col_ptr().is_null());
    }
}
