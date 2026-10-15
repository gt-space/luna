// Regmem commands API

use crate::status::Status;
/// Writes a block of 32-bit words in register/memory space starting at a specific address. The address is auto-incremented after each data word so that data is stored in contiguous register/memory locations. Address must be 32-bit aligned and data length must be a multiple of 4. Maximum 32 words
pub fn write_reg_mem32_cmd(addr: u32, data: u32) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x01;
    cmd[1] = 0x04;

    cmd[2] |= ((addr >> 16) & 0xFF) as u8;
    cmd[3] |= ((addr >> 8) & 0xFF) as u8;
    cmd[4] |= (addr & 0xFF) as u8;
    cmd[5] |= ((data >> 24) & 0xFF) as u8;
    cmd[6] |= ((data >> 16) & 0xFF) as u8;
    cmd[7] |= ((data >> 8) & 0xFF) as u8;
    cmd[8] |= (data & 0xFF) as u8;
    cmd
}

/// Read-Modify-Writes data at given register/memory address. Only the masked bits (Mask bits = 1) of a single 32-bit word are modified. Address must be 32-bit aligned
pub fn write_reg_mem_mask32_cmd(addr: u32, mask: u32, data: u32) -> [u8; 13] {
    let mut cmd = [0u8; 13];
    cmd[0] = 0x01;
    cmd[1] = 0x05;

    cmd[2] |= ((addr >> 16) & 0xFF) as u8;
    cmd[3] |= ((addr >> 8) & 0xFF) as u8;
    cmd[4] |= (addr & 0xFF) as u8;
    cmd[5] |= ((mask >> 24) & 0xFF) as u8;
    cmd[6] |= ((mask >> 16) & 0xFF) as u8;
    cmd[7] |= ((mask >> 8) & 0xFF) as u8;
    cmd[8] |= (mask & 0xFF) as u8;
    cmd[9] |= ((data >> 24) & 0xFF) as u8;
    cmd[10] |= ((data >> 16) & 0xFF) as u8;
    cmd[11] |= ((data >> 8) & 0xFF) as u8;
    cmd[12] |= (data & 0xFF) as u8;
    cmd
}

/// Reads a block of 32-bit words in register/memory space starting at a specific address. The address is auto-incremented after each data word so that data is read from contiguous register locations. Address must be 32-bit aligned. Maximum 64 words
pub fn read_reg_mem32_req(addr: u32, len: u8) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x01;
    cmd[1] = 0x06;

    cmd[2] |= ((addr >> 16) & 0xFF) as u8;
    cmd[3] |= ((addr >> 8) & 0xFF) as u8;
    cmd[4] |= (addr & 0xFF) as u8;
    cmd[5] |= len;
    cmd
}

// Response structs

/// Response for ReadRegMem32 command
#[derive(Default)]
pub struct ReadRegMem32Rsp([u8; 6]);

impl ReadRegMem32Rsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Variable length array of 32-bit data words read from memory (len words, each 4 bytes)
    pub fn value(&self) -> u32 {
        (self.0[5] as u32) |
        ((self.0[4] as u32) << 8) |
        ((self.0[3] as u32) << 16) |
        ((self.0[2] as u32) << 24)
    }
}

impl AsMut<[u8]> for ReadRegMem32Rsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
