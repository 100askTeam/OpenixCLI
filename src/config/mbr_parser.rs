//! MBR (Master Boot Record) parser
//!
//! Provides parsers for Allwinner MBR partition table format

#![allow(dead_code)]

use std::time::{SystemTime, UNIX_EPOCH};

/// MBR magic string
pub const MBR_MAGIC: &str = "softw411";
/// MBR version
pub const MBR_VERSION: u32 = 0x00000200;
/// MBR size in bytes (16KB)
pub const MBR_SIZE: usize = 16 * 1024;
/// Maximum partition name length
pub const PART_NAME_MAX_LEN: usize = 16;
/// Reserved size in partition entry
pub const PART_SIZE_RES_LEN: usize = 68;
/// Maximum number of partitions supported
pub const MBR_MAX_PART_CNT: usize = 120;

/// CRC32 valid flag
pub const EFEX_CRC32_VALID_FLAG: u32 = 0x6a617603;

const MBR_HEADER_SIZE: usize = 32;
const PARTITION_ENTRY_SIZE: usize = 128;

/// Build the one-partition Allwinner MBR used by the FES raw-disk flow.
///
/// The external image is exposed to FES as a `raw` partition in class `DISK`.
/// Addresses and lengths are expressed in 512-byte logical sectors.
pub fn build_generic_raw_mbr(
    start_sector: u64,
    sector_count: u64,
) -> Result<Vec<u8>, &'static str> {
    if sector_count == 0 {
        return Err("Raw partition must contain at least one sector");
    }

    let mut mbr = vec![0_u8; MBR_SIZE];
    write_u32(&mut mbr, 4, MBR_VERSION);
    mbr[8..16].copy_from_slice(MBR_MAGIC.as_bytes());
    write_u32(&mut mbr, 16, 1); // one MBR copy
    write_u32(&mut mbr, 20, 0); // copy index
    write_u32(&mut mbr, 24, 1); // one partition
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_secs() as u32)
        .unwrap_or(0);
    write_u32(&mut mbr, 28, stamp);

    let partition = MBR_HEADER_SIZE;
    write_u64_halves(&mut mbr, partition, start_sector);
    write_u64_halves(&mut mbr, partition + 8, sector_count);
    write_name(&mut mbr, partition + 16, "DISK");
    write_name(&mut mbr, partition + 32, "raw");
    debug_assert_eq!(PARTITION_ENTRY_SIZE, SUNXI_PARTITION_SIZE);

    let crc = sunxi_mbr_crc32(&mbr[4..]);
    write_u32(&mut mbr, 0, crc);
    Ok(mbr)
}

fn write_u32(target: &mut [u8], offset: usize, value: u32) {
    target[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn write_u64_halves(target: &mut [u8], offset: usize, value: u64) {
    write_u32(target, offset, (value >> 32) as u32);
    write_u32(target, offset + 4, value as u32);
}

fn write_name(target: &mut [u8], offset: usize, value: &str) {
    let bytes = value.as_bytes();
    target[offset..offset + bytes.len()].copy_from_slice(bytes);
}

fn sunxi_mbr_crc32(data: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & (0_u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}

/// Raw partition entry structure
///
/// Represents a single partition entry in the MBR
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SunxiPartitionRaw {
    pub addrhi: u32,
    pub addrlo: u32,
    pub lenhi: u32,
    pub lenlo: u32,
    pub classname: [u8; PART_NAME_MAX_LEN],
    pub name: [u8; PART_NAME_MAX_LEN],
    pub user_type: u32,
    pub keydata: u32,
    pub ro: u32,
    pub reserved: [u8; PART_SIZE_RES_LEN],
}

/// Size of a partition entry
pub const SUNXI_PARTITION_SIZE: usize = std::mem::size_of::<SunxiPartitionRaw>();

impl SunxiPartitionRaw {
    /// Parse partition entry from raw data
    pub fn parse(data: &[u8]) -> Result<&Self, &'static str> {
        if data.len() < SUNXI_PARTITION_SIZE {
            return Err("Data too short for Sunxi partition");
        }

        let ptr = data.as_ptr() as *const SunxiPartitionRaw;
        Ok(unsafe { &*ptr })
    }

    /// Get class name as string
    pub fn classname_str(&self) -> String {
        String::from_utf8_lossy(&self.classname)
            .trim_end_matches('\0')
            .to_string()
    }

    /// Get partition name as string
    pub fn name_str(&self) -> String {
        String::from_utf8_lossy(&self.name)
            .trim_end_matches('\0')
            .to_string()
    }

    /// Get partition starting address
    pub fn address(&self) -> u64 {
        ((self.addrhi as u64) << 32) | (self.addrlo as u64)
    }

    /// Get partition size
    pub fn length(&self) -> u64 {
        ((self.lenhi as u64) << 32) | (self.lenlo as u64)
    }

    /// Check if partition is read-only
    pub fn readonly(&self) -> bool {
        self.ro != 0
    }
}

/// Parsed partition entry
#[derive(Debug, Clone)]
pub struct SunxiPartition {
    pub addrhi: u32,
    pub addrlo: u32,
    pub lenhi: u32,
    pub lenlo: u32,
    pub classname: String,
    pub name: String,
    pub user_type: u32,
    pub keydata: u32,
    pub ro: u32,
}

impl SunxiPartition {
    /// Create from raw partition entry
    pub fn from_raw(raw: &SunxiPartitionRaw) -> Self {
        Self {
            addrhi: raw.addrhi,
            addrlo: raw.addrlo,
            lenhi: raw.lenhi,
            lenlo: raw.lenlo,
            classname: raw.classname_str(),
            name: raw.name_str(),
            user_type: raw.user_type,
            keydata: raw.keydata,
            ro: raw.ro,
        }
    }

    /// Get partition starting address
    pub fn address(&self) -> u64 {
        ((self.addrhi as u64) << 32) | (self.addrlo as u64)
    }

    /// Get partition size
    pub fn length(&self) -> u64 {
        ((self.lenhi as u64) << 32) | (self.lenlo as u64)
    }

    /// Check if partition is read-only
    pub fn readonly(&self) -> bool {
        self.ro != 0
    }
}

/// Raw MBR structure
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SunxiMbrRaw {
    pub crc32: u32,
    pub version: u32,
    pub magic: [u8; 8],
    pub copy: u32,
    pub index: u32,
    pub part_count: u32,
    pub stamp: u32,
    pub partitions: [SunxiPartitionRaw; MBR_MAX_PART_CNT],
}

impl SunxiMbrRaw {
    /// Parse MBR from raw data
    pub fn parse(data: &[u8]) -> Result<&Self, &'static str> {
        if data.len() < MBR_SIZE {
            return Err("Data too short for Sunxi MBR");
        }

        let ptr = data.as_ptr() as *const SunxiMbrRaw;
        let mbr = unsafe { &*ptr };

        let magic = String::from_utf8_lossy(&mbr.magic).to_string();
        if magic != MBR_MAGIC {
            return Err("Invalid MBR magic");
        }

        Ok(mbr)
    }

    /// Get magic string
    pub fn magic_str(&self) -> String {
        String::from_utf8_lossy(&self.magic).to_string()
    }
}

/// Parsed MBR structure
#[derive(Debug, Clone)]
pub struct SunxiMbr {
    pub crc32: u32,
    pub version: u32,
    pub magic: String,
    pub copy: u32,
    pub index: u32,
    pub part_count: u32,
    pub stamp: u32,
    pub partitions: Vec<SunxiPartition>,
}

impl SunxiMbr {
    /// Parse MBR from raw data
    pub fn parse(data: &[u8]) -> Result<Self, &'static str> {
        let raw = SunxiMbrRaw::parse(data)?;

        let mut partitions = Vec::with_capacity(raw.part_count as usize);
        for i in 0..raw.part_count as usize {
            let partition = SunxiPartition::from_raw(&raw.partitions[i]);
            partitions.push(partition);
        }

        Ok(Self {
            crc32: raw.crc32,
            version: raw.version,
            magic: raw.magic_str(),
            copy: raw.copy,
            index: raw.index,
            part_count: raw.part_count,
            stamp: raw.stamp,
            partitions,
        })
    }

    /// Convert to MbrInfo
    pub fn to_mbr_info(&self) -> MbrInfo {
        MbrInfo {
            part_count: self.part_count,
            partitions: self.partitions.clone(),
        }
    }
}

/// MBR information container
#[derive(Debug, Clone)]
pub struct MbrInfo {
    pub part_count: u32,
    pub partitions: Vec<SunxiPartition>,
}

/// Check if data contains a valid MBR
pub fn is_valid_mbr(data: &[u8]) -> bool {
    if data.len() < std::mem::size_of::<SunxiMbrRaw>() {
        return false;
    }

    let raw = SunxiMbrRaw::parse(data);
    raw.is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference_crc32(data: &[u8]) -> u32 {
        let mut crc = u32::MAX;
        for byte in data {
            crc ^= u32::from(*byte);
            for _ in 0..8 {
                crc = (crc >> 1) ^ (0xedb8_8320 & (0_u32.wrapping_sub(crc & 1)));
            }
        }
        !crc
    }

    #[test]
    fn generic_raw_mbr_matches_fes_single_partition_contract() {
        let sectors = 56_832;
        let bytes = build_generic_raw_mbr(0, sectors).unwrap();
        assert_eq!(bytes.len(), MBR_SIZE);

        let parsed = SunxiMbr::parse(&bytes).unwrap();
        assert_eq!(parsed.version, MBR_VERSION);
        assert_eq!(parsed.magic, MBR_MAGIC);
        assert_eq!(parsed.copy, 1);
        assert_eq!(parsed.index, 0);
        assert_eq!(parsed.part_count, 1);
        assert_eq!(parsed.partitions.len(), 1);
        assert_eq!(parsed.partitions[0].classname, "DISK");
        assert_eq!(parsed.partitions[0].name, "raw");
        assert_eq!(parsed.partitions[0].address(), 0);
        assert_eq!(parsed.partitions[0].length(), sectors);

        let stored_crc = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        assert_eq!(stored_crc, reference_crc32(&bytes[4..]));
    }

    #[test]
    fn generic_raw_mbr_rejects_empty_partition() {
        assert!(build_generic_raw_mbr(0, 0).is_err());
    }
}
