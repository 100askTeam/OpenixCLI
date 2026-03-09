#![allow(dead_code)]

use crate::config::mbr_parser::SunxiPartition;

#[derive(Debug, Clone)]
pub struct PartitionInfo {
    pub name: String,
    pub classname: String,
    pub address: u64,
    pub length: u64,
    pub user_type: u32,
    pub keydata: u32,
    pub readonly: bool,
}

impl From<SunxiPartition> for PartitionInfo {
    fn from(partition: SunxiPartition) -> Self {
        let address = partition.address();
        let length = partition.length();
        let readonly = partition.readonly();
        Self {
            name: partition.name,
            classname: partition.classname,
            address,
            length,
            user_type: partition.user_type,
            keydata: partition.keydata,
            readonly,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PartitionDownloadInfo {
    pub partition: PartitionInfo,
    pub data_offset: u64,
    pub data_length: u64,
    pub need_verify: bool,
}
