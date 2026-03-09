#![allow(dead_code)]

use crate::firmware::StorageType;

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct DramParamInfo {
    pub dram_init_flag: u32,
    pub dram_update_flag: u32,
    pub dram_para: [u32; 32],
}

impl DramParamInfo {
    pub fn create_empty() -> Self {
        Self {
            dram_init_flag: 0,
            dram_update_flag: 0,
            dram_para: [0u32; 32],
        }
    }

    pub fn parse(data: &[u8]) -> Result<&Self, &'static str> {
        if data.len() < std::mem::size_of::<DramParamInfo>() {
            return Err("Data too short for DRAM param");
        }

        let ptr = data.as_ptr() as *const DramParamInfo;
        Ok(unsafe { &*ptr })
    }

    pub fn parse_mut(data: &mut [u8]) -> Result<&mut Self, &'static str> {
        if data.len() < std::mem::size_of::<DramParamInfo>() {
            return Err("Data too short for DRAM param");
        }

        let ptr = data.as_mut_ptr() as *mut DramParamInfo;
        Ok(unsafe { &mut *ptr })
    }

    pub fn serialize(&self) -> Vec<u8> {
        let size = std::mem::size_of::<DramParamInfo>();
        let mut data = vec![0u8; size];
        unsafe {
            std::ptr::copy_nonoverlapping(
                self as *const DramParamInfo as *const u8,
                data.as_mut_ptr(),
                size,
            );
        }
        data
    }
}

pub struct SysConfigParser;

impl SysConfigParser {
    pub fn parse(data: &[u8]) -> SysConfig {
        SysConfig {
            storage_type: Self::get_storage_type(data),
        }
    }

    fn get_storage_type(data: &[u8]) -> u32 {
        if data.len() < 4 {
            return 0;
        }
        let ptr = data.as_ptr() as *const u32;
        unsafe { u32::from_le(*ptr) }
    }

    pub fn get_storage_type_from_num(num: u32) -> StorageType {
        StorageType::from(num)
    }
}

#[derive(Debug, Clone)]
pub struct SysConfig {
    pub storage_type: u32,
}
