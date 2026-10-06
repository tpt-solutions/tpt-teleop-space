//! Volatile register-cell abstraction — the foundation of every
//! peripheral driver on the edge layer.
//!
//! A [`RegCell`] binds a raw address to typed access semantics
//! (`ReadOnly` / `ReadWrite`). All accesses are volatile; `modify` is a
//! read-modify-write with the mask applied by the caller's closure, which
//! keeps the RMW atomicity story explicit at the driver level (peripheral
//! bit-banding or critical sections are the hardware's concern, and the
//! sched layer provides the window).
//!
//! On real hardware, cells are constructed from a datasheet address via
//! [`RegCell::at`] inside a driver's `new` (an `unsafe` boundary the
//! driver owns and documents); tests drive cells through the
//! `mirror`-backed constructor so the logic is exercised without a
//! device.

#![allow(unsafe_code)]

/// Access qualifier for a [`RegCell`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// Reads only (status registers).
    ReadOnly,
    /// Reads and writes (control/data registers).
    ReadWrite,
}

/// One memory-mapped register cell of `u32` width.
///
/// The `mirror` indirection exists so host tests can drive the same code
/// paths as the target: on hardware, `ptr` points at the peripheral;
/// in tests, at a `Cell<u32>` in the test's memory.
pub struct RegCell {
    ptr: *mut u32,
    access: Access,
}

// SAFETY: a register cell is stateless beyond its raw pointer; the
// platform memory map guarantees the address is device memory accessed
// only through this API (drivers are single-owner per cell by contract).
unsafe impl Send for RegCell {}

impl RegCell {
    /// Bind a cell to a raw device address. The caller (the driver that
    /// owns this peripheral) guarantees the address is valid and
    /// exclusively owned for the cell's lifetime.
    ///
    /// # Safety
    /// `addr` must be a valid, aligned, device-memory `u32` register that
    /// nothing else accesses.
    pub unsafe fn at(addr: usize, access: Access) -> Self {
        Self {
            ptr: addr as *mut u32,
            access,
        }
    }

    /// Bind a cell to a test mirror (host-side unit testing).
    #[must_use]
    pub fn mirror(mirror: &core::cell::Cell<u32>, access: Access) -> Self {
        Self {
            ptr: mirror.as_ptr(),
            access,
        }
    }

    /// Volatile read.
    #[must_use]
    pub fn read(&self) -> u32 {
        // SAFETY: volatile read of the bound cell; validity is the
        // constructor's documented contract.
        unsafe { self.ptr.read_volatile() }
    }

    /// Volatile write (allowed only on `ReadWrite` cells; read-only cells
    /// silently drop the write so a driver bug cannot corrupt status).
    pub fn write(&self, value: u32) {
        if self.access == Access::ReadWrite {
            // SAFETY: volatile write of the bound cell (see `read`).
            unsafe { self.ptr.write_volatile(value) }
        }
    }

    /// Volatile read-modify-write through `f` (ReadWrite cells only).
    pub fn modify(&self, f: impl FnOnce(u32) -> u32) {
        if self.access == Access::ReadWrite {
            let v = self.read();
            self.write(f(v));
        }
    }

    /// Set the bits of `mask` (read-modify-write).
    pub fn set_bits(&self, mask: u32) {
        self.modify(|v| v | mask);
    }

    /// Clear the bits of `mask` (read-modify-write).
    pub fn clear_bits(&self, mask: u32) {
        self.modify(|v| v & !mask);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_write_roundtrip() {
        let m = core::cell::Cell::new(0u32);
        let cell = RegCell::mirror(&m, Access::ReadWrite);
        cell.write(0xDEAD_BEEF);
        assert_eq!(cell.read(), 0xDEAD_BEEF);
    }

    #[test]
    fn read_only_cells_drop_writes() {
        let m = core::cell::Cell::new(7u32);
        let cell = RegCell::mirror(&m, Access::ReadOnly);
        cell.write(0xFFFF);
        assert_eq!(cell.read(), 7, "status register is write-protected");
        cell.set_bits(0xFF);
        cell.clear_bits(0xFF);
        assert_eq!(cell.read(), 7);
    }

    #[test]
    fn modify_and_bit_ops() {
        let m = core::cell::Cell::new(0b1010u32);
        let cell = RegCell::mirror(&m, Access::ReadWrite);
        cell.set_bits(0b0101);
        assert_eq!(cell.read(), 0b1111);
        cell.clear_bits(0b0010);
        assert_eq!(cell.read(), 0b1101);
        cell.modify(|v| v.rotate_left(4));
        assert_eq!(cell.read(), 0xD0);
    }
}
