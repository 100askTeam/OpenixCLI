//! Shared flash request and option types.

use std::fmt;
use std::str::FromStr;

/// Flash mode options.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashMode {
    /// Flash only specified partitions.
    Partition,
    /// Flash while preserving common user data partitions.
    KeepData,
    /// Erase selected partitions before flashing.
    PartitionErase,
    /// Full erase before flashing.
    FullErase,
}

impl FlashMode {
    /// Get erase flag for this mode.
    pub fn erase_flag(self) -> u32 {
        match self {
            FlashMode::Partition => 0x0,
            FlashMode::KeepData => 0x0,
            FlashMode::PartitionErase => 0x1,
            FlashMode::FullErase => 0x12,
        }
    }

    pub fn display_name(self) -> &'static str {
        match self {
            FlashMode::Partition => "Partition",
            FlashMode::KeepData => "Keep Data",
            FlashMode::PartitionErase => "Part. Erase",
            FlashMode::FullErase => "Full Erase",
        }
    }

    pub fn next(self) -> Self {
        match self {
            FlashMode::FullErase => FlashMode::PartitionErase,
            FlashMode::PartitionErase => FlashMode::KeepData,
            FlashMode::KeepData => FlashMode::Partition,
            FlashMode::Partition => FlashMode::FullErase,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            FlashMode::FullErase => FlashMode::Partition,
            FlashMode::Partition => FlashMode::KeepData,
            FlashMode::KeepData => FlashMode::PartitionErase,
            FlashMode::PartitionErase => FlashMode::FullErase,
        }
    }
}

impl FromStr for FlashMode {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "partition" => Ok(Self::Partition),
            "keep_data" => Ok(Self::KeepData),
            "partition_erase" => Ok(Self::PartitionErase),
            "full_erase" => Ok(Self::FullErase),
            _ => Err(format!("Invalid flash mode: {}", value)),
        }
    }
}

impl fmt::Display for FlashMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlashMode::Partition => write!(f, "partition"),
            FlashMode::KeepData => write!(f, "keep_data"),
            FlashMode::PartitionErase => write!(f, "partition_erase"),
            FlashMode::FullErase => write!(f, "full_erase"),
        }
    }
}

/// Action to perform after flashing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PostAction {
    Reboot,
    PowerOff,
    Shutdown,
    None,
}

impl PostAction {
    pub fn name(self) -> &'static str {
        match self {
            PostAction::Reboot => "Reboot",
            PostAction::PowerOff => "Power Off",
            PostAction::Shutdown => "Shutdown",
            PostAction::None => "None",
        }
    }

    pub fn next(self) -> Self {
        match self {
            PostAction::Reboot => PostAction::PowerOff,
            PostAction::PowerOff => PostAction::Shutdown,
            PostAction::Shutdown => PostAction::Reboot,
            PostAction::None => PostAction::Reboot,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            PostAction::Reboot => PostAction::Shutdown,
            PostAction::PowerOff => PostAction::Reboot,
            PostAction::Shutdown => PostAction::PowerOff,
            PostAction::None => PostAction::Shutdown,
        }
    }

    pub fn fes_tool_mode(self) -> libefex::FesToolMode {
        match self {
            PostAction::Reboot => libefex::FesToolMode::Reboot,
            PostAction::PowerOff | PostAction::Shutdown => libefex::FesToolMode::PowerOff,
            PostAction::None => libefex::FesToolMode::Normal,
        }
    }
}

impl FromStr for PostAction {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "reboot" => Ok(Self::Reboot),
            "poweroff" => Ok(Self::PowerOff),
            "shutdown" => Ok(Self::Shutdown),
            "none" => Ok(Self::None),
            _ => Err(format!("Invalid post-flash action: {}", value)),
        }
    }
}

impl fmt::Display for PostAction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PostAction::Reboot => write!(f, "reboot"),
            PostAction::PowerOff => write!(f, "poweroff"),
            PostAction::Shutdown => write!(f, "shutdown"),
            PostAction::None => write!(f, "none"),
        }
    }
}

/// Device selection requested by the caller.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceSelector {
    pub bus: Option<u8>,
    pub port: Option<u8>,
    pub location: Option<String>,
}

impl DeviceSelector {
    pub fn new(bus: Option<u8>, port: Option<u8>) -> Self {
        Self {
            bus,
            port,
            location: None,
        }
    }

    pub fn with_location(mut self, location: Option<String>) -> Self {
        self.location = location.filter(|value| !value.trim().is_empty());
        self
    }

    pub fn selected_pair(&self) -> Option<(u8, u8)> {
        match (self.bus, self.port) {
            (Some(bus), Some(port)) => Some((bus, port)),
            _ => None,
        }
    }

    pub fn selected_location(&self) -> Option<&str> {
        self.location.as_deref()
    }
}

/// Fully resolved flash request used by CLI and TUI.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashRequest {
    pub device: DeviceSelector,
    pub verify: bool,
    pub mode: FlashMode,
    pub partitions: Option<Vec<String>>,
    pub post_action: PostAction,
}

impl FlashRequest {
    pub fn new(
        device: DeviceSelector,
        verify: bool,
        mode: FlashMode,
        partitions: Option<Vec<String>>,
        post_action: PostAction,
    ) -> Self {
        Self {
            device,
            verify,
            mode,
            partitions,
            post_action,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flash_mode_round_trips_cli_values() {
        let cases = [
            ("partition", FlashMode::Partition),
            ("keep_data", FlashMode::KeepData),
            ("partition_erase", FlashMode::PartitionErase),
            ("full_erase", FlashMode::FullErase),
        ];

        for (value, mode) in cases {
            assert_eq!(value.parse::<FlashMode>().unwrap(), mode);
            assert_eq!(mode.to_string(), value);
        }
    }

    #[test]
    fn post_action_round_trips_cli_values() {
        let cases = [
            ("reboot", PostAction::Reboot),
            ("poweroff", PostAction::PowerOff),
            ("shutdown", PostAction::Shutdown),
            ("none", PostAction::None),
        ];

        for (value, action) in cases {
            assert_eq!(value.parse::<PostAction>().unwrap(), action);
            assert_eq!(action.to_string(), value);
        }
    }

    #[test]
    fn device_selector_requires_bus_and_port_for_explicit_selection() {
        assert_eq!(
            DeviceSelector::new(Some(1), Some(5)).selected_pair(),
            Some((1, 5))
        );
        assert_eq!(DeviceSelector::new(Some(1), None).selected_pair(), None);
        assert_eq!(DeviceSelector::new(None, Some(5)).selected_pair(), None);
    }

    #[test]
    fn physical_location_takes_a_stable_explicit_selector() {
        let selector = DeviceSelector::new(Some(0), Some(10))
            .with_location(Some("pciroot(0)#usbroot(0)#usb(4)".to_owned()));
        assert_eq!(
            selector.selected_location(),
            Some("pciroot(0)#usbroot(0)#usb(4)")
        );
        assert_eq!(selector.selected_pair(), Some((0, 10)));
    }
}
