//! Flash module
//!
//! Provides flash functionality for writing firmware to Allwinner devices
//! Supports both FEL mode (USB boot) and FES mode (U-Boot)

#![allow(dead_code)]

pub mod events;
pub mod fel_handler;
pub mod fes_handler;
mod raw_disk;
pub mod request;

pub use events::{FlashEvent, FlashEventSink, FlashLogLevel};
pub use fel_handler::FelHandler;
pub use fes_handler::FesHandler;
pub use raw_disk::{RawDiskFlasher, RawMode, RuntimeStorageGeometrySerial};
pub use request::{DeviceSelector, FlashMode, FlashRequest, PostAction};

use crate::firmware::OpenixPacker;
use crate::process::{FlashStages, StageType};
use crate::utils::{FlashError, FlashResult, Logger};

struct BootstrapArtifacts {
    uboot: Vec<u8>,
    dtb: Option<Vec<u8>>,
    sys_config: Vec<u8>,
    board_config: Option<Vec<u8>>,
}

/// Main flash controller
///
/// Coordinates the flashing process including FEL initialization,
/// FES handling, and partition flashing
pub struct Flasher {
    packer: OpenixPacker,
    bootstrap_packer: Option<OpenixPacker>,
    nand_components: bool,
    request: FlashRequest,
    logger: Logger,
}

impl Flasher {
    /// Create a new flasher instance
    pub fn new(packer: OpenixPacker, request: FlashRequest, logger: Logger) -> Self {
        Self {
            packer,
            bootstrap_packer: None,
            nand_components: false,
            request,
            logger,
        }
    }

    /// Use a distinct vendor package only for the RAM DRAM/FES bootstrap.
    /// All media layout and component data continue to come from `packer`.
    pub fn with_bootstrap(mut self, bootstrap_packer: OpenixPacker) -> Self {
        self.bootstrap_packer = Some(bootstrap_packer);
        self
    }

    /// Restrict this run to bad-block-aware NAND/SPI-NAND component writes.
    pub fn for_nand_components(mut self) -> Self {
        self.nand_components = true;
        self.request.verify = true;
        self
    }

    /// Execute the flash process
    ///
    /// This is the main entry point for the flashing process.
    /// It handles both FEL and FES mode devices.
    pub async fn execute(&mut self) -> FlashResult<()> {
        let (ctx, _) = self.open_fes_service(false).await?;

        match self.run_fes_mode(&ctx).await {
            Ok(()) => {
                if self.request.verify {
                    self.logger.verification("success", Some(0));
                } else {
                    self.logger.verification("skipped", None);
                }
            }
            Err(error) => {
                if self.request.verify {
                    self.logger.verification("failed", Some(-1));
                }
                return Err(error);
            }
        }
        self.apply_post_action(&ctx).await?;

        Ok(())
    }

    /// Open the selected device and return a running FES service. When the
    /// target is still in BootROM FEL mode, the loaded vendor package is used
    /// only to initialize DRAM and boot its USB-product U-Boot in RAM. This
    /// method never writes MBR, partitions, Boot0, or Boot1 to target media.
    pub(crate) async fn open_fes_service(
        &mut self,
        raw_write: bool,
    ) -> FlashResult<(libefex::Context, bool)> {
        let bootstrap = self.bootstrap_packer.as_mut().unwrap_or(&mut self.packer);
        let fes_data = bootstrap.get_fes().map_err(|_| FlashError::FesNotFound)?;
        let bootstrap_artifacts = BootstrapArtifacts {
            uboot: bootstrap
                .get_uboot()
                .map_err(|_| FlashError::UbootNotFound)?,
            dtb: bootstrap.get_dtb().ok(),
            sys_config: bootstrap
                .get_sys_config_bin()
                .map_err(|_| FlashError::SysConfigNotFound)?,
            board_config: bootstrap.get_board_config().ok(),
        };
        let ctx = self.open_device()?;
        let mode = ctx.get_device_mode();
        self.logger.info(&format!("Device mode: {:?}", mode));
        let has_fel = mode == libefex::DeviceMode::Fel;

        let stages = match (raw_write, has_fel) {
            (true, true) => FlashStages::for_raw_fel_mode(),
            (true, false) => FlashStages::for_raw_fes_mode(),
            (false, true) => FlashStages::for_fel_mode(),
            (false, false) => FlashStages::for_fes_mode(),
        };
        self.logger.define_stages(stages.stages());
        self.logger.start_global_progress();

        self.logger.begin_stage(StageType::Init);
        self.logger
            .info(&format!("FES data loaded ({} bytes)", fes_data.len()));
        self.logger.complete_stage();

        if has_fel {
            self.logger.info("FES_BOOTSTRAP_RAM_ONLY:begin");
            let ctx = self
                .prepare_fel_mode(ctx, &fes_data, &bootstrap_artifacts)
                .await?;
            self.logger.info("FES_BOOTSTRAP_RAM_ONLY:ready");
            Ok((ctx, true))
        } else {
            self.logger.info("FES_BOOTSTRAP_RAM_ONLY:already_ready");
            Ok((ctx, false))
        }
    }

    /// Open the selected device, or the first detected device when no full selector is provided.
    fn open_device(&self) -> FlashResult<libefex::Context> {
        let mut ctx = if let Some(location) = self.request.device.selected_location() {
            let mut ctx = libefex::Context::new();
            ctx.scan_usb_device_by_location(location)
                .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;
            ctx
        } else if let Some((bus, port)) = self.request.device.selected_pair() {
            let mut ctx = libefex::Context::new();
            ctx.scan_usb_device_at(bus, port)
                .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;
            ctx
        } else {
            let devices = libefex::Context::scan_usb_devices()
                .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;

            if devices.is_empty() {
                return Err(FlashError::DeviceNotFound);
            }

            let mut ctx = libefex::Context::new();
            ctx.scan_usb_device_at(devices[0].bus, devices[0].port)
                .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;
            ctx
        };

        ctx.usb_init()
            .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;

        ctx.efex_init()
            .map_err(|e| FlashError::DeviceOpenFailed(e.to_string()))?;

        Ok(ctx)
    }

    async fn prepare_fel_mode(
        &mut self,
        mut ctx: libefex::Context,
        fes_data: &[u8],
        bootstrap: &BootstrapArtifacts,
    ) -> FlashResult<libefex::Context> {
        self.logger.begin_stage(StageType::FelDram);
        let fel_handler = FelHandler::new(&self.logger);
        fel_handler.handle(&mut ctx, fes_data).await?;
        self.logger.complete_stage();

        self.logger.begin_stage(StageType::FelUboot);

        fel_handler
            .download_uboot(
                &ctx,
                &bootstrap.uboot,
                bootstrap.dtb.as_deref(),
                &bootstrap.sys_config,
                bootstrap.board_config.as_deref(),
            )
            .await?;

        self.logger.info(&format!(
            "U-Boot downloaded ({} bytes)",
            bootstrap.uboot.len()
        ));
        self.logger.complete_stage();

        let expected_device_id = ctx.device_id();

        // This handle belongs to the FEL incarnation that just executed
        // U-Boot. Release it before opening the newly enumerated FES device.
        // On Windows, retaining the stale handle can make the replacement
        // interface visible to scan but unavailable to a second open.
        drop(ctx);
        self.logger.info("FES_RECONNECT:released_stale_fel_handle");

        self.logger.begin_stage(StageType::FelReconnect);
        let ctx = self.reconnect_device(expected_device_id).await?;
        self.logger.complete_stage();

        Ok(ctx)
    }

    async fn run_fes_mode(&mut self, ctx: &libefex::Context) -> FlashResult<()> {
        let mut fes_handler = FesHandler::new(&mut self.logger);
        fes_handler
            .handle(ctx, &mut self.packer, &self.request, self.nand_components)
            .await
    }

    async fn apply_post_action(&self, ctx: &libefex::Context) -> FlashResult<()> {
        self.logger.begin_stage(StageType::FesMode);
        if self.request.post_action == PostAction::None {
            self.logger
                .post_action(self.request.post_action, "skipped", 0, None);
            self.logger.complete_stage();
            self.logger.stage_complete("Device will remain in FES mode");
            self.logger.flash_finished(self.request.post_action);
            self.logger.finish_progress();
            return Ok(());
        }

        self.logger
            .post_action(self.request.post_action, "running", 1, None);
        if let Err(error) = self.set_device_mode(ctx).await {
            self.logger.post_action(
                self.request.post_action,
                "failed",
                1,
                Some(error.to_string()),
            );
            return Err(error);
        }
        let mut attempts = 1;
        if !self.wait_for_fes_exit().await {
            attempts = 2;
            self.logger
                .post_action(self.request.post_action, "running", 2, None);
            let retry = self.open_device().and_then(|retry_ctx| {
                retry_ctx
                    .fes_tool_mode(
                        libefex::FesToolMode::Normal,
                        self.request.post_action.fes_tool_mode(),
                    )
                    .map_err(|error| FlashError::UsbTransferError(error.to_string()))
            });
            if let Err(error) = retry {
                self.logger.post_action(
                    self.request.post_action,
                    "failed",
                    2,
                    Some(error.to_string()),
                );
                return Err(error);
            }
            if !self.wait_for_fes_exit().await {
                let message = format!(
                    "POST_FLASH_ACTION_FAILED:{} device remained in FEL/FES",
                    self.request.post_action
                );
                self.logger.post_action(
                    self.request.post_action,
                    "failed",
                    2,
                    Some(message.clone()),
                );
                return Err(FlashError::Timeout(message));
            }
        }
        self.logger
            .post_action(self.request.post_action, "success", attempts, None);
        self.logger.complete_stage();

        self.logger
            .stage_complete(&format!("Device will {}", self.request.post_action));
        self.logger.flash_finished(self.request.post_action);
        self.logger.finish_progress();

        Ok(())
    }

    async fn wait_for_fes_exit(&self) -> bool {
        for _ in 0..20 {
            tokio::time::sleep(tokio::time::Duration::from_millis(250)).await;
            let Ok(devices) = libefex::Context::scan_usb_devices() else {
                continue;
            };
            let present = if let Some(location) = self.request.device.selected_location() {
                devices.iter().any(|device| {
                    device
                        .location_path
                        .as_deref()
                        .is_some_and(|candidate| locations_match(location, candidate))
                })
            } else if let Some((bus, port)) = self.request.device.selected_pair() {
                devices
                    .iter()
                    .any(|device| device.bus == bus && device.port == port)
            } else {
                !devices.is_empty()
            };
            if !present {
                return true;
            }
        }
        false
    }

    /// Reconnect to device after FEL mode operations
    async fn reconnect_device(&self, expected_device_id: u32) -> FlashResult<libefex::Context> {
        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;

        let max_retries = 25;
        let mut retries = 0;
        let mut last_snapshot = "none".to_owned();
        let mut last_open_error = "not_attempted".to_owned();

        while retries < max_retries {
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

            let devices = match libefex::Context::scan_usb_devices() {
                Ok(d) => d,
                Err(error) => {
                    retries += 1;
                    last_snapshot = format!("scan_error={error}");
                    self.logger.debug(&format!(
                        "FES_RECONNECT_SCAN attempt={}/{} error={error}",
                        retries, max_retries,
                    ));
                    continue;
                }
            };

            last_snapshot = format_device_snapshot(&devices);
            self.logger.debug(&format!(
                "FES_RECONNECT_SCAN attempt={}/{} expected={} devices=[{}]",
                retries + 1,
                max_retries,
                self.request
                    .device
                    .selected_location()
                    .unwrap_or("unscoped"),
                last_snapshot
            ));

            let (matching, migration_rule) =
                if let Some(expected) = self.request.device.selected_location() {
                    reconnect_location_candidates(expected, &devices)
                } else if let Some((bus, port)) = self.request.device.selected_pair() {
                    (
                        devices
                            .iter()
                            .filter(|device| device.bus == bus && device.port == port)
                            .collect::<Vec<_>>(),
                        None,
                    )
                } else {
                    (devices.iter().collect::<Vec<_>>(), None)
                };
            if matching.len() != 1 {
                retries += 1;
                self.logger.debug(&format!(
                    "Reconnect attempt {}/{} resolved {} candidates",
                    retries,
                    max_retries,
                    matching.len()
                ));
                continue;
            }

            if let Some(rule) = migration_rule {
                self.logger.info(&format!(
                    "FES_RECONNECT_TOPOLOGY_MIGRATION expected={} current={} rule={rule}",
                    self.request
                        .device
                        .selected_location()
                        .unwrap_or("unscoped"),
                    matching[0]
                        .location_path
                        .as_deref()
                        .unwrap_or("unavailable")
                ));
            }

            for dev in matching {
                let mut new_ctx = libefex::Context::new();
                let current_location = dev.location_path.as_deref().unwrap_or("unavailable");
                let selected = if self.request.device.selected_location().is_some() {
                    new_ctx.scan_usb_device_by_location(
                        dev.location_path
                            .as_deref()
                            .expect("location-matched devices have a location"),
                    )
                } else {
                    new_ctx.scan_usb_device_at(dev.bus, dev.port)
                };
                if let Err(error) = selected {
                    last_open_error = format!(
                        "stage=select location={current_location} bus={} port={} error={error}",
                        dev.bus, dev.port
                    );
                    self.logger
                        .debug(&format!("FES_RECONNECT_OPEN {last_open_error}"));
                    continue;
                }
                if let Err(error) = new_ctx.usb_init() {
                    last_open_error = format!(
                        "stage=usb_init location={current_location} bus={} port={} error={error}",
                        dev.bus, dev.port
                    );
                    self.logger
                        .debug(&format!("FES_RECONNECT_OPEN {last_open_error}"));
                    continue;
                }
                if let Err(error) = new_ctx.efex_init() {
                    last_open_error = format!(
                        "stage=efex_init location={current_location} bus={} port={} error={error}",
                        dev.bus, dev.port
                    );
                    self.logger
                        .debug(&format!("FES_RECONNECT_OPEN {last_open_error}"));
                    continue;
                }

                let actual_device_id = new_ctx.device_id();
                if !fel_fes_device_ids_match(expected_device_id, actual_device_id) {
                    last_open_error = format!(
                        "stage=device_id_check location={current_location} bus={} port={} expected=0x{expected_device_id:08x} actual=0x{actual_device_id:08x}",
                        dev.bus, dev.port
                    );
                    self.logger
                        .debug(&format!("FES_RECONNECT_OPEN {last_open_error}"));
                    continue;
                }
                if actual_device_id != expected_device_id {
                    self.logger.info(&format!(
                        "FES_RECONNECT_DEVICE_ID_MAPPING fel=0x{expected_device_id:08x} fes=0x{actual_device_id:08x} rule=r528-vendor-fes-v1"
                    ));
                }

                let mode = new_ctx.get_device_mode();
                if mode == libefex::DeviceMode::Srv {
                    self.logger.info(&format!(
                        "FES_RECONNECT_READY vid=0x{:04x} pid=0x{:04x} device_id=0x{actual_device_id:08x} mode={mode:?} bus={} port={} location={current_location}",
                        dev.vid, dev.pid, dev.bus, dev.port
                    ));
                    return Ok(new_ctx);
                }
                last_open_error = format!(
                    "stage=mode_check location={current_location} bus={} port={} mode={mode:?}",
                    dev.bus, dev.port
                );
                self.logger
                    .debug(&format!("FES_RECONNECT_OPEN {last_open_error}"));
            }

            retries += 1;
            self.logger
                .debug(&format!("Reconnect attempt {}/{}", retries, max_retries));
        }

        Err(FlashError::ReconnectFailed(format!(
            "expected={}; devices=[{}]; last_open=[{}]",
            self.request
                .device
                .selected_location()
                .unwrap_or("unscoped"),
            last_snapshot,
            last_open_error
        )))
    }

    /// Set device mode after flashing
    async fn set_device_mode(&self, ctx: &libefex::Context) -> FlashResult<()> {
        let tool_mode = self.request.post_action.fes_tool_mode();

        ctx.fes_tool_mode(libefex::FesToolMode::Normal, tool_mode)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        Ok(())
    }
}

fn format_device_snapshot(devices: &[libefex::ScannedDevice]) -> String {
    if devices.is_empty() {
        return "none".to_owned();
    }
    devices
        .iter()
        .take(16)
        .map(|device| {
            format!(
                "vid=0x{:04x},pid=0x{:04x},bus={},port={},location={},device_path={}",
                device.vid,
                device.pid,
                device.bus,
                device.port,
                device.location_path.as_deref().unwrap_or("unavailable"),
                device.device_path.as_deref().unwrap_or("unavailable")
            )
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

fn locations_match(expected: &str, candidate: &str) -> bool {
    if expected.eq_ignore_ascii_case(candidate) {
        return true;
    }
    let Some(legacy) = expected.strip_prefix("libusb:") else {
        return false;
    };
    let Some((expected_bus, expected_leaf)) = legacy.split_once(':') else {
        return false;
    };
    if expected_leaf.contains('.') {
        return false;
    }
    let Some(current) = candidate.strip_prefix("libusb:") else {
        return false;
    };
    let Some((candidate_bus, candidate_path)) = current.split_once(':') else {
        return false;
    };
    expected_bus == candidate_bus
        && candidate_path
            .rsplit('.')
            .next()
            .is_some_and(|leaf| leaf == expected_leaf)
}

fn reconnect_location_candidates<'a>(
    expected: &str,
    devices: &'a [libefex::ScannedDevice],
) -> (Vec<&'a libefex::ScannedDevice>, Option<&'static str>) {
    let exact = devices
        .iter()
        .filter(|device| {
            is_allwinner_fel_fes_usb(device)
                && device
                    .location_path
                    .as_deref()
                    .is_some_and(|candidate| locations_match(expected, candidate))
        })
        .collect::<Vec<_>>();
    if !exact.is_empty() {
        return (exact, None);
    }

    let same_leaf = devices
        .iter()
        .filter(|device| {
            is_allwinner_fel_fes_usb(device)
                && device
                    .location_path
                    .as_deref()
                    .is_some_and(|candidate| same_libusb_bus_and_leaf(expected, candidate))
        })
        .collect::<Vec<_>>();
    if !same_leaf.is_empty() {
        return (same_leaf, Some("same-bus-same-leaf-unique"));
    }

    let same_bus = devices
        .iter()
        .filter(|device| {
            is_allwinner_fel_fes_usb(device)
                && device
                    .location_path
                    .as_deref()
                    .is_some_and(|candidate| same_libusb_bus(expected, candidate))
        })
        .collect::<Vec<_>>();
    (same_bus, Some("same-bus-unique-fes-transition"))
}

fn is_allwinner_fel_fes_usb(device: &libefex::ScannedDevice) -> bool {
    device.vid == 0x1f3a && device.pid == 0xefe8
}

fn fel_fes_device_ids_match(fel_device_id: u32, fes_device_id: u32) -> bool {
    fel_device_id == fes_device_id
        || matches!((fel_device_id, fes_device_id), (0x0018_5900, 0x0016_1000))
}

fn same_libusb_bus_and_leaf(expected: &str, candidate: &str) -> bool {
    let Some(expected) = expected.strip_prefix("libusb:") else {
        return false;
    };
    let Some(candidate) = candidate.strip_prefix("libusb:") else {
        return false;
    };
    let Some((expected_bus, expected_path)) = expected.split_once(':') else {
        return false;
    };
    let Some((candidate_bus, candidate_path)) = candidate.split_once(':') else {
        return false;
    };
    let expected_leaf = expected_path.rsplit('.').next();
    let candidate_leaf = candidate_path.rsplit('.').next();
    expected_bus == candidate_bus && expected_leaf.is_some() && expected_leaf == candidate_leaf
}

fn same_libusb_bus(expected: &str, candidate: &str) -> bool {
    let Some(expected) = expected.strip_prefix("libusb:") else {
        return false;
    };
    let Some(candidate) = candidate.strip_prefix("libusb:") else {
        return false;
    };
    let Some((expected_bus, _)) = expected.split_once(':') else {
        return false;
    };
    let Some((candidate_bus, _)) = candidate.split_once(':') else {
        return false;
    };
    expected_bus == candidate_bus
}

#[cfg(test)]
mod location_tests {
    use super::{
        fel_fes_device_ids_match, format_device_snapshot, locations_match,
        reconnect_location_candidates, same_libusb_bus, same_libusb_bus_and_leaf,
    };

    fn scanned(location: &str) -> libefex::ScannedDevice {
        libefex::ScannedDevice {
            bus: 2,
            port: 2,
            vid: 0x1f3a,
            pid: 0xefe8,
            location_path: Some(location.to_owned()),
            device_path: None,
        }
    }

    #[test]
    fn legacy_libusb_leaf_matches_a_full_physical_path_only_by_bus_and_leaf() {
        assert!(locations_match("libusb:2:2", "libusb:2:3.2"));
        assert!(locations_match("libusb:2:3.2", "libusb:2:3.2"));
        assert!(!locations_match("libusb:2:2", "libusb:3:3.2"));
        assert!(!locations_match("libusb:2:2", "libusb:2:3.1"));
    }

    #[test]
    fn reconnect_snapshot_includes_usb_identity_and_full_topology() {
        let devices = vec![libefex::ScannedDevice {
            bus: 2,
            port: 2,
            vid: 0x1f3a,
            pid: 0xefe8,
            location_path: Some("libusb:2:3.2".to_owned()),
            device_path: Some("node-current".to_owned()),
        }];
        let snapshot = format_device_snapshot(&devices);
        assert!(snapshot.contains("vid=0x1f3a,pid=0xefe8"));
        assert!(snapshot.contains("bus=2,port=2"));
        assert!(snapshot.contains("location=libusb:2:3.2"));
        assert!(snapshot.contains("device_path=node-current"));
    }

    #[test]
    fn controlled_reconnect_accepts_parent_change_only_on_same_bus_and_leaf() {
        assert!(same_libusb_bus_and_leaf("libusb:2:3.2", "libusb:2:4.2"));
        assert!(!same_libusb_bus_and_leaf("libusb:2:3.2", "libusb:3:4.2"));
        assert!(!same_libusb_bus_and_leaf("libusb:2:3.2", "libusb:2:4.1"));
        assert!(!same_libusb_bus_and_leaf(
            "windows:pciroot#usb(2)",
            "libusb:2:4.2"
        ));
    }

    #[test]
    fn reconnect_prefers_an_exact_location_over_a_topology_migration() {
        let devices = vec![scanned("libusb:2:4.2"), scanned("libusb:2:3.2")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert!(rule.is_none());
        assert_eq!(matching.len(), 1);
        assert_eq!(matching[0].location_path.as_deref(), Some("libusb:2:3.2"));
    }

    #[test]
    fn reconnect_accepts_one_same_bus_and_leaf_migration() {
        let devices = vec![scanned("libusb:2:4.2")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-same-leaf-unique"));
        assert_eq!(matching.len(), 1);
        assert_eq!(matching[0].location_path.as_deref(), Some("libusb:2:4.2"));
    }

    #[test]
    fn reconnect_exposes_ambiguous_migrations_for_the_caller_to_reject() {
        let devices = vec![scanned("libusb:2:4.2"), scanned("libusb:2:5.2")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-same-leaf-unique"));
        assert_eq!(matching.len(), 2);
    }

    #[test]
    fn reconnect_does_not_migrate_across_usb_buses() {
        let devices = vec![scanned("libusb:3:4.2")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-unique-fes-transition"));
        assert!(matching.is_empty());
    }

    #[test]
    fn reconnect_ignores_a_topology_match_with_the_wrong_usb_identity() {
        let mut device = scanned("libusb:2:4.2");
        device.vid = 0x1234;
        let devices = vec![device];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-unique-fes-transition"));
        assert!(matching.is_empty());
    }

    #[test]
    fn reconnect_accepts_one_same_bus_fes_when_the_leaf_changes() {
        assert!(same_libusb_bus("libusb:2:3.2", "libusb:2:4.1"));
        let devices = vec![scanned("libusb:2:4.1")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-unique-fes-transition"));
        assert_eq!(matching.len(), 1);
    }

    #[test]
    fn reconnect_rejects_multiple_same_bus_fes_transition_candidates() {
        let devices = vec![scanned("libusb:2:4.1"), scanned("libusb:2:5.3")];
        let (matching, rule) = reconnect_location_candidates("libusb:2:3.2", &devices);
        assert_eq!(rule, Some("same-bus-unique-fes-transition"));
        assert_eq!(matching.len(), 2);
    }

    #[test]
    fn r528_bootrom_to_vendor_fes_is_a_one_way_explicit_mapping() {
        assert!(fel_fes_device_ids_match(0x0018_5900, 0x0016_1000));
        assert!(fel_fes_device_ids_match(0x0018_5900, 0x0018_5900));
        assert!(!fel_fes_device_ids_match(0x0016_1000, 0x0018_5900));
        assert!(!fel_fes_device_ids_match(0x0018_5900, 0x0016_1001));
    }
}
