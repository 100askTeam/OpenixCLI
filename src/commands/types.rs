//! Command type definitions
//!
//! Defines types and structures used by CLI commands

use std::path::PathBuf;

use crate::flash::{DeviceSelector, FlashMode, FlashRequest, PostAction, RawMode};

/// Arguments for the flash command
///
/// # Fields
/// * `firmware_path` - Path to the firmware file
/// * `bus` - USB bus number (optional)
/// * `port` - USB port number (optional)
/// * `verify` - Enable verification after write
/// * `mode` - Flash mode
/// * `partitions` - Specific partitions to flash (optional)
/// * `post_action` - Action to perform after flashing
/// * `verbose` - Enable verbose output
pub struct FlashArgs {
    pub firmware_path: PathBuf,
    pub raw: bool,
    pub nand_components: bool,
    pub raw_mode: RawMode,
    pub bootstrap_firmware_path: Option<PathBuf>,
    pub storage_geometry_serial_port: Option<String>,
    pub storage_geometry_serial_baud: u32,
    pub bus: Option<u8>,
    pub port: Option<u8>,
    pub device_location: Option<String>,
    pub verify: bool,
    pub mode: FlashMode,
    pub partitions: Option<Vec<String>>,
    pub post_action: PostAction,
    pub verbose: bool,
    pub jsonl: bool,
}

impl FlashArgs {
    pub fn request(&self) -> FlashRequest {
        FlashRequest::new(
            DeviceSelector::new(self.bus, self.port).with_location(self.device_location.clone()),
            self.verify,
            self.mode,
            self.partitions.clone(),
            self.post_action,
        )
    }
}

/// Arguments for the unpack command
///
/// # Fields
/// * `firmware_path` - Path to the firmware file
/// * `output` - Optional output directory (defaults to ./<firmware>_unpacked)
pub struct UnpackArgs {
    pub firmware_path: PathBuf,
    pub output: Option<PathBuf>,
}

pub fn parse_partition_list(partitions: Option<String>) -> Option<Vec<String>> {
    partitions.map(|value| {
        value
            .split(',')
            .map(str::trim)
            .filter(|part| !part.is_empty())
            .map(str::to_string)
            .collect()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_partition_list_trims_and_drops_empty_values() {
        assert_eq!(
            parse_partition_list(Some(" boot, system ,,vendor ".to_string())),
            Some(vec![
                "boot".to_string(),
                "system".to_string(),
                "vendor".to_string()
            ])
        );
    }

    #[test]
    fn flash_args_builds_shared_request() {
        let args = FlashArgs {
            firmware_path: PathBuf::from("firmware.img"),
            raw: false,
            nand_components: false,
            raw_mode: RawMode::Command,
            bootstrap_firmware_path: None,
            storage_geometry_serial_port: None,
            storage_geometry_serial_baud: 115_200,
            bus: Some(1),
            port: Some(2),
            device_location: None,
            verify: false,
            mode: FlashMode::Partition,
            partitions: Some(vec!["boot".to_string()]),
            post_action: PostAction::PowerOff,
            verbose: true,
            jsonl: false,
        };

        let request = args.request();
        assert_eq!(request.device.selected_pair(), Some((1, 2)));
        assert_eq!(request.mode, FlashMode::Partition);
        assert_eq!(request.post_action, PostAction::PowerOff);
        assert!(!request.verify);
        assert_eq!(request.partitions, Some(vec!["boot".to_string()]));
    }
}
