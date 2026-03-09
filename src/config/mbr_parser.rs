#![allow(dead_code)]

pub const MBR_MAGIC: &str = "softw411";
pub const MBR_VERSION: u32 = 0x00000200;
pub const MBR_SIZE: usize = 16 * 1024;
pub const PART_NAME_MAX_LEN: usize = 16;
pub const PART_SIZE_RES_LEN: usize = 68;
pub const MBR_MAX_PART_CNT: usize = 120;

pub const EFEX_CRC32_VALID_FLAG: u32 = 0x6a617603;

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

pub const SUNXI_PARTITION_SIZE: usize = std::mem::size_of::<SunxiPartitionRaw>();

impl SunxiPartitionRaw {
    pub fn parse(data: &[u8]) -> Result<&Self, &'static str> {
        if data.len() < SUNXI_PARTITION_SIZE {
            return Err("Data too short for Sunxi partition");
        }

        let ptr = data.as_ptr() as *const SunxiPartitionRaw;
        Ok(unsafe { &*ptr })
    }

    pub fn classname_str(&self) -> String {
        String::from_utf8_lossy(&self.classname)
            .trim_end_matches('\0')
            .to_string()
    }

    pub fn name_str(&self) -> String {
        String::from_utf8_lossy(&self.name)
            .trim_end_matches('\0')
            .to_string()
    }

    pub fn address(&self) -> u64 {
        ((self.addrhi as u64) << 32) | (self.addrlo as u64)
    }

    pub fn length(&self) -> u64 {
        ((self.lenhi as u64) << 32) | (self.lenlo as u64)
    }

    pub fn readonly(&self) -> bool {
        self.ro != 0
    }
}

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

    pub fn address(&self) -> u64 {
        ((self.addrhi as u64) << 32) | (self.addrlo as u64)
    }

    pub fn length(&self) -> u64 {
        ((self.lenhi as u64) << 32) | (self.lenlo as u64)
    }

    pub fn readonly(&self) -> bool {
        self.ro != 0
    }
}

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

    pub fn magic_str(&self) -> String {
        String::from_utf8_lossy(&self.magic).to_string()
    }
}

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

    pub fn to_mbr_info(&self) -> MbrInfo {
        MbrInfo {
            part_count: self.part_count,
            partitions: self.partitions.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MbrInfo {
    pub part_count: u32,
    pub partitions: Vec<SunxiPartition>,
}

pub fn is_valid_mbr(data: &[u8]) -> bool {
    if data.len() < std::mem::size_of::<SunxiMbrRaw>() {
        return false;
    }

    let raw = SunxiMbrRaw::parse(data);
    raw.is_ok()
}
