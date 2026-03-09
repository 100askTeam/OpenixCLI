#![allow(dead_code)]

use thiserror::Error;

#[derive(Debug, Error)]
pub enum FlashError {
    #[error("Firmware file not found: {0}")]
    FirmwareNotFound(String),

    #[error("Invalid firmware format: {0}")]
    InvalidFirmwareFormat(String),

    #[error("Encrypted firmware not supported")]
    EncryptedNotSupported,

    #[error("Device not found")]
    DeviceNotFound,

    #[error("Failed to open device: {0}")]
    DeviceOpenFailed(String),

    #[error("DRAM initialization failed")]
    DramInitFailed,

    #[error("U-Boot download failed")]
    UbootDownloadFailed,

    #[error("MBR download failed")]
    MbrDownloadFailed,

    #[error("Partition download failed: {0}")]
    PartitionDownloadFailed(String),

    #[error("Device reconnect failed")]
    ReconnectFailed,

    #[error("Storage type mismatch: device={device}, firmware={firmware}")]
    StorageTypeMismatch { device: String, firmware: String },

    #[error("FES not found in firmware")]
    FesNotFound,

    #[error("U-Boot not found in firmware")]
    UbootNotFound,

    #[error("SysConfig not found in firmware")]
    SysConfigNotFound,

    #[error("MBR not found in firmware")]
    MbrNotFound,

    #[error("Boot0 not found in firmware")]
    Boot0NotFound,

    #[error("Boot1 not found in firmware")]
    Boot1NotFound,

    #[error("USB transfer error: {0}")]
    UsbTransferError(String),

    #[error("Operation cancelled")]
    Cancelled,

    #[error("Timeout: {0}")]
    Timeout(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Packer error: {0}")]
    Packer(#[from] crate::firmware::PackerError),

    #[error("Libefex error: {0}")]
    Libefex(#[from] libefex::EfexError),

    #[error("Unknown error: {0}")]
    Unknown(String),
}

pub type FlashResult<T> = Result<T, FlashError>;
