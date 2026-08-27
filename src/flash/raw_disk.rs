//! Whole-disk raw image writer for an already-running Allwinner FES service.

use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use libefex::FesDataType;
use sha2::{Digest, Sha256};

use crate::config::mbr_parser::{build_generic_raw_mbr, EFEX_CRC32_VALID_FLAG};
use crate::firmware::{OpenixPacker, StorageType};
use crate::flash::fes_handler::{IncrementalChecksum, MbrDownload};
use crate::flash::{FlashRequest, Flasher, PostAction};
use crate::utils::{FlashError, FlashResult, Logger};

const SECTOR_SIZE: u64 = 512;
/// Read external partitions in 10 MiB windows. libefex splits every window into
/// the FES protocol's 64 KiB USB transfers.
const RAW_READ_WINDOW_SIZE: usize = 10 * 1024 * 1024;
const LYNX_CANCEL_FILE_ENV: &str = "LYNX_FLASH_CANCEL_FILE";
/// Tina FES exposes SD/eMMC user storage after a 20 MiB logical prefix.  A
/// command image is a physical-disk image, so its FES address must wrap by the
/// inverse offset: 0xffff6000 + 0x0000a000 == physical sector zero.
const FES_SDMMC_LOGICAL_OFFSET_SECTORS: u32 = 40_960;
const MAINLINE_SPL_DISK_OFFSET: u64 = 8 * 1024;
const MAINLINE_SPL_MAX_BYTES: usize = 1024 * 1024;
const FES_BOOT0_VERIFY_TAG: u32 = 0x7f03;

#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum RawMode {
    /// Treat the input as a whole-disk command image beginning at sector 0.
    Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RawLayout {
    start_sector: u64,
    copies: u32,
    wrapping_address: bool,
}

impl RawMode {
    const fn layout(self, storage_type: StorageType) -> RawLayout {
        match self {
            Self::Command => {
                let sdmmc = matches!(
                    storage_type,
                    StorageType::Emmc
                        | StorageType::Emmc3
                        | StorageType::Emmc0
                        | StorageType::Sdcard
                        | StorageType::Sd1
                );
                RawLayout {
                    start_sector: if sdmmc {
                        u32::MAX.wrapping_sub(FES_SDMMC_LOGICAL_OFFSET_SECTORS - 1) as u64
                    } else {
                        0
                    },
                    copies: crate::config::mbr_parser::MBR_COPY_NUM as u32,
                    wrapping_address: sdmmc,
                }
            }
        }
    }

    const fn physical_start_sector(layout: RawLayout) -> u64 {
        if layout.wrapping_address {
            0
        } else {
            layout.start_sector
        }
    }
}

pub struct RawDiskFlasher {
    image_path: PathBuf,
    bootstrap_packer: Option<OpenixPacker>,
    geometry_serial: Option<RuntimeStorageGeometrySerial>,
    raw_mode: RawMode,
    emmc_boot0_from_image: bool,
    request: FlashRequest,
    logger: Logger,
}

#[derive(Clone, Debug)]
pub struct RuntimeStorageGeometrySerial {
    pub port: String,
    pub baud: u32,
}

impl RawDiskFlasher {
    pub fn new(
        image_path: PathBuf,
        bootstrap_packer: Option<OpenixPacker>,
        geometry_serial: Option<RuntimeStorageGeometrySerial>,
        raw_mode: RawMode,
        emmc_boot0_from_image: bool,
        request: FlashRequest,
        logger: Logger,
    ) -> Self {
        Self {
            image_path,
            bootstrap_packer,
            geometry_serial,
            raw_mode,
            emmc_boot0_from_image,
            request,
            logger,
        }
    }

    pub async fn execute(&mut self) -> FlashResult<()> {
        ensure_not_cancelled()?;
        self.logger.info("RAW_PREFLIGHT_STAGE:metadata");
        let image_len = std::fs::metadata(&self.image_path)
            .map_err(FlashError::Io)?
            .len();
        if image_len == 0 || image_len % SECTOR_SIZE != 0 {
            return Err(FlashError::InvalidFirmwareFormat(format!(
                "RAW_IMAGE_ALIGNMENT: image size must be a non-zero multiple of {SECTOR_SIZE} bytes"
            )));
        }
        let emmc_boot0 = self
            .emmc_boot0_from_image
            .then(|| extract_mainline_spl(&self.image_path, image_len))
            .transpose()?;
        if let Some(spl) = emmc_boot0.as_ref() {
            self.logger.info(&format!(
                "EMMC_BOOT0_ARTIFACT:source=image@0x{MAINLINE_SPL_DISK_OFFSET:X},bytes={},sha256={:x}",
                spl.len(),
                Sha256::digest(spl)
            ));
        }

        let mut geometry_capture = self
            .geometry_serial
            .as_ref()
            .map(RuntimeGeometryCapture::start)
            .transpose()?;
        let ctx = if let Some(bootstrap_packer) = self.bootstrap_packer.take() {
            self.logger.info("RAW_PREFLIGHT_STAGE:bootstrap_vendor_fes");
            let mut bootstrap =
                Flasher::new(bootstrap_packer, self.request.clone(), self.logger.clone());
            bootstrap.open_fes_service(true).await?.0
        } else {
            let ctx = self.open_device()?;
            let stages = crate::process::FlashStages::for_raw_fes_mode();
            self.logger.define_stages(stages.stages());
            self.logger.start_global_progress();
            self.logger.begin_stage(crate::process::StageType::Init);
            self.logger.complete_stage();
            ctx
        };
        ensure_not_cancelled()?;
        if ctx.get_device_mode() == libefex::DeviceMode::Fel {
            return Err(FlashError::Unknown(
                "RAW_FLASH_REQUIRES_FES: boot the board-scoped RAM FES/installer before writing a raw disk image"
                    .to_owned(),
            ));
        }
        // Besides reporting the boot security type, this query precedes
        // erase-policy and MBR commands in every supported FES write sequence.
        ensure_not_cancelled()?;
        self.logger.info("RAW_PREFLIGHT_STAGE:query_secure");
        let secure_type = ctx
            .fes_query_secure()
            .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
        self.logger
            .info(&format!("RAW_TARGET_SECURE_TYPE:{secure_type}"));

        // Storage policy must be resolved before any destructive command.
        // NAND media require component-aware writes so their bad-block model
        // is not flattened into a whole-disk byte stream.
        self.logger.info("RAW_PREFLIGHT_STAGE:query_storage");
        self.logger.begin_stage(crate::process::StageType::FesQuery);
        let storage_code = ctx
            .fes_query_storage()
            .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
        let storage_type = StorageType::from(storage_code);
        validate_raw_storage(storage_type)?;
        if emmc_boot0.is_some() && !is_emmc_storage(storage_type) {
            return Err(FlashError::InvalidFirmwareFormat(format!(
                "EMMC_BOOT0_STORAGE_MISMATCH: requested eMMC Boot0 installation but FES detected {storage_type}"
            )));
        }
        self.logger
            .info(&format!("RAW_TARGET_STORAGE:{storage_type}"));

        self.logger.info("RAW_PREFLIGHT_STAGE:probe_capacity");
        let probed_sectors = ctx
            .fes_probe_flash_size()
            .map_err(|error| FlashError::UsbTransferError(error.to_string()))?
            as u64;
        let geometry_observation = geometry_capture
            .as_mut()
            .map(RuntimeGeometryCapture::finish);
        if probed_sectors == 0 {
            if let Some(observation) = geometry_observation.as_ref() {
                self.logger.info(&format!(
                    "RAW_TARGET_RUNTIME_GEOMETRY:model={},totalBytes={},sectorBytes={},queuePageBytes={}",
                    observation.model.as_deref().unwrap_or("unreported"),
                    observation.total_bytes.unwrap_or(0),
                    observation.sector_bytes.unwrap_or(0),
                    observation.queue_page_bytes.unwrap_or(0)
                ));
            }
        }
        let (flash_sectors, capacity_source) =
            resolve_capacity_sectors(storage_type, probed_sectors, geometry_observation.clone())?;
        let flash_bytes = capacity_bytes_from_sectors(flash_sectors)?;
        let raw_layout = self.raw_mode.layout(storage_type);
        let write_plan = RawWritePlan::new(image_len);
        self.logger
            .info(&format!("RAW_TARGET_CAPACITY_SOURCE:{capacity_source}"));
        self.logger
            .info(&format!("RAW_TARGET_CAPACITY_SECTORS:{flash_sectors}"));
        self.logger
            .info(&format!("RAW_TARGET_CAPACITY:{flash_bytes}"));
        let image_sectors = image_len / SECTOR_SIZE;
        let required_sectors = RawMode::physical_start_sector(raw_layout)
            .checked_add(image_sectors)
            .ok_or_else(|| {
                FlashError::InvalidFirmwareFormat(
                    "RAW_IMAGE_ADDRESS_OVERFLOW: raw disk image exceeds host limits".to_owned(),
                )
            })?;
        if required_sectors > flash_sectors {
            return Err(FlashError::InvalidFirmwareFormat(format!(
                "RAW_IMAGE_TOO_LARGE: image is {image_len} bytes but target capacity is {flash_bytes} bytes"
            )));
        }

        self.logger.info(&format!(
            "Raw image: {} bytes; target capacity: {} bytes",
            image_len, flash_bytes
        ));
        self.logger.info(&format!(
            "RAW_WRITE_PLAN:totalBytes={},chunkSize={},chunkCount={},chunkSource={},ack=awus_csw",
            write_plan.total_bytes,
            write_plan.chunk_size,
            write_plan.chunk_count,
            write_plan.chunk_source
        ));
        self.logger.complete_stage();

        // FES requires the erase-policy command before it will accept an MBR.
        // It is deliberately sent only after media type, capacity and image
        // bounds have all passed their non-destructive preflight checks.
        ensure_not_cancelled()?;
        self.logger.begin_stage(crate::process::StageType::FesErase);
        self.logger.info(&format!(
            "RAW_FES_ERASE_POLICY:mode={},flag=0x{:x}",
            self.request.mode,
            self.request.mode.erase_flag()
        ));
        crate::flash::fes_handler::EraseFlag::new(&self.logger)
            .execute(&ctx, self.request.mode)
            .await?;
        self.logger.complete_stage();

        ensure_not_cancelled()?;
        let mbr_data = build_generic_raw_mbr(raw_layout.start_sector, image_sectors)
            .map_err(|error| FlashError::InvalidFirmwareFormat(error.to_owned()))?;
        self.logger.begin_stage(crate::process::StageType::FesMbr);
        self.logger.info(&format!(
            "RAW_MBR_GENERATED:name=raw,class=DISK,startSector={},sectorCount={image_sectors},copies={},wrapping_address={},bytes={}",
            raw_layout.start_sector,
            raw_layout.copies,
            raw_layout.wrapping_address,
            mbr_data.len(),
        ));
        self.logger.info("Downloading MBR...");
        self.logger.info("RAW_MBR_DOWNLOAD_START");
        MbrDownload::new(&self.logger)
            .execute(&ctx, &mbr_data)
            .await?;
        self.logger.info("RAW_MBR_DOWNLOAD_READY");
        self.logger.complete_stage();

        // Tina FES Boot0 writes can alias a copy of the SPL into the eMMC user
        // area (observed at physical 0x20000 on T527). Install Boot0 before the
        // raw user-area image so the following full-disk write repairs that
        // alias. The final raw verification then also covers alias damage.
        if let Some(spl) = emmc_boot0.as_ref() {
            ensure_not_cancelled()?;
            self.install_emmc_boot0(&ctx, spl)?;
        }

        // The generated MBR establishes the partition layout; flash access is
        // enabled once for the following partition phase.
        let flash_access = FlashAccessGuard::enable(&ctx, storage_code)?;

        let file = File::open(&self.image_path).map_err(FlashError::Io)?;
        let mut reader = BufReader::new(file);
        let mut buffer = vec![0_u8; write_plan.chunk_size];
        let mut offset = 0_u64;
        let mut successful_chunks = 0_u64;
        let mut checksum = IncrementalChecksum::new();
        self.logger
            .begin_stage(crate::process::StageType::FesPartitions);
        self.logger.set_current_partition("raw");
        self.logger
            .set_partition_stage_weight(write_plan.total_bytes);
        self.logger.update_progress_with_speed(0);
        self.logger.info("Downloading raw image...");
        self.logger.info("RAW_WRITE_START");

        loop {
            ensure_not_cancelled()?;
            let read = reader.read(&mut buffer).map_err(FlashError::Io)?;
            if read == 0 {
                break;
            }
            let chunk = &buffer[..read];
            checksum.update(chunk);
            let relative_sector = u32::try_from(offset / SECTOR_SIZE).map_err(|_| {
                FlashError::InvalidFirmwareFormat(
                    "RAW_IMAGE_ADDRESS_OVERFLOW: target sector exceeds the FES protocol".to_owned(),
                )
            })?;
            let sector = (raw_layout.start_sector as u32).wrapping_add(relative_sector);
            let chunk_index = successful_chunks;
            let sector_count = read as u64 / SECTOR_SIZE;
            let transfer = ctx.fes_down_with_progress(chunk, sector, FesDataType::Flash, {
                let logger = self.logger.clone();
                move |transferred, _| logger.update_progress_with_speed(offset + transferred)
            });
            let acknowledged = match transfer {
                Ok(acknowledged) => acknowledged,
                Err(error) => {
                    let fes_status = fes_status_label(&error);
                    let detail = format!(
                        "RAW_WRITE_CHUNK_FAILED:chunkIndex={chunk_index},offset=0x{offset:X},startSector={sector},sectorCount={sector_count},chunkSize={read},successfulChunks={successful_chunks},acknowledgedBytes={offset},fesStatus={fes_status},error={error}"
                    );
                    self.logger.error(&detail);
                    return Err(FlashError::UsbTransferError(detail));
                }
            };
            if acknowledged != read as u64 {
                let detail = format!(
                    "RAW_WRITE_SHORT_ACK:chunkIndex={chunk_index},offset=0x{offset:X},startSector={sector},sectorCount={sector_count},chunkSize={read},successfulChunks={successful_chunks},acknowledgedChunkBytes={acknowledged},acknowledgedBytes={offset}"
                );
                self.logger.error(&detail);
                return Err(FlashError::UsbTransferError(detail));
            }
            offset += read as u64;
            successful_chunks += 1;
        }
        self.logger.complete_stage();

        if self.request.verify {
            ensure_not_cancelled()?;
            let verify = ctx
                .fes_verify_value(raw_layout.start_sector as u32, image_len)
                .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
            let local = checksum.finalize();
            if verify.flag != EFEX_CRC32_VALID_FLAG || verify.media_crc as u32 != local {
                return Err(FlashError::Unknown(format!(
                    "RAW_IMAGE_VERIFY_FAILED: local=0x{local:08x}, media=0x{:08x}, flag=0x{:04x}",
                    verify.media_crc, verify.flag
                )));
            }
            self.logger.verification("success", Some(0));
            self.logger.info("verifyState=success");
        } else {
            self.logger.verification("skipped", None);
        }

        flash_access.disable()?;
        ensure_not_cancelled()?;
        self.logger.begin_stage(crate::process::StageType::FesMode);
        self.apply_post_action(&ctx)?;
        self.logger.complete_stage();
        self.logger.flash_finished(self.request.post_action);
        self.logger.finish_progress();
        Ok(())
    }

    fn open_device(&self) -> FlashResult<libefex::Context> {
        let mut ctx = libefex::Context::new();
        self.logger.info("RAW_PREFLIGHT_STAGE:scan_target");
        if let Some(location) = self.request.device.selected_location() {
            ctx.scan_usb_device_by_location(location)
                .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?;
        } else if let Some((bus, port)) = self.request.device.selected_pair() {
            ctx.scan_usb_device_at(bus, port)
                .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?;
        } else {
            let device = libefex::Context::scan_usb_devices()
                .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?
                .into_iter()
                .next()
                .ok_or(FlashError::DeviceNotFound)?;
            ctx.scan_usb_device_at(device.bus, device.port)
                .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?;
        }
        self.logger.info("RAW_PREFLIGHT_STAGE:open_usb");
        ctx.usb_init()
            .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?;
        self.logger.info("RAW_PREFLIGHT_STAGE:fes_handshake");
        ctx.efex_init()
            .map_err(|error| FlashError::DeviceOpenFailed(error.to_string()))?;
        self.logger.info("RAW_PREFLIGHT_STAGE:fes_ready");
        Ok(ctx)
    }

    fn apply_post_action(&self, ctx: &libefex::Context) -> FlashResult<()> {
        if self.request.post_action == PostAction::None {
            self.logger
                .post_action(PostAction::None, "skipped", 0, None);
            return Ok(());
        }
        ctx.fes_tool_mode(
            libefex::FesToolMode::Normal,
            self.request.post_action.fes_tool_mode(),
        )
        .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
        self.logger
            .post_action(self.request.post_action, "success", 1, None);
        Ok(())
    }

    fn install_emmc_boot0(&self, ctx: &libefex::Context, spl: &[u8]) -> FlashResult<()> {
        self.logger
            .info("EMMC_BOOT0_WRITE_START:method=fes_boot0_component");
        ctx.fes_down(spl, 0, FesDataType::Boot0).map_err(|error| {
            FlashError::UsbTransferError(format!("EMMC_BOOT0_WRITE_FAILED:{error}"))
        })?;
        let status = ctx
            .fes_verify_status(FES_BOOT0_VERIFY_TAG)
            .map_err(|error| {
                FlashError::UsbTransferError(format!("EMMC_BOOT0_STATUS_FAILED:{error}"))
            })?;
        if status.flag != EFEX_CRC32_VALID_FLAG || status.media_crc != 0 {
            return Err(FlashError::Unknown(format!(
                "EMMC_BOOT0_STATUS_INVALID:flag=0x{:04x},status=0x{:08x}",
                status.flag, status.media_crc
            )));
        }

        let expected = Sha256::digest(spl);
        self.logger.info(&format!(
            "EMMC_BOOT0_VERIFIED:method=fes_boot0_status,bytes={},sourceSha256={expected:x}",
            spl.len()
        ));
        self.logger.info(
            "EMMC_BOOT0_READBACK_UNAVAILABLE:FES FlashBoot0 upload aliases the eMMC user area and cannot prove hardware boot-partition byte identity",
        );
        Ok(())
    }
}

fn is_emmc_storage(storage_type: StorageType) -> bool {
    matches!(
        storage_type,
        StorageType::Emmc | StorageType::Emmc3 | StorageType::Emmc0
    )
}

fn extract_mainline_spl(path: &Path, image_len: u64) -> FlashResult<Vec<u8>> {
    if image_len < MAINLINE_SPL_DISK_OFFSET + 0x20 {
        return Err(FlashError::InvalidFirmwareFormat(
            "EMMC_BOOT0_IMAGE_TOO_SMALL".to_owned(),
        ));
    }
    let mut file = File::open(path).map_err(FlashError::Io)?;
    file.seek(SeekFrom::Start(MAINLINE_SPL_DISK_OFFSET))
        .map_err(FlashError::Io)?;
    let mut header = [0_u8; 0x20];
    file.read_exact(&mut header).map_err(FlashError::Io)?;
    if header.get(4..12) != Some(b"eGON.BT0") || header.get(0x14..0x17) != Some(b"SPL") {
        return Err(FlashError::InvalidFirmwareFormat(
            "EMMC_BOOT0_MAINLINE_SPL_REQUIRED:image offset 0x2000 has no eGON.BT0 SPL".to_owned(),
        ));
    }
    let version = header[0x17];
    if !(1..=31).contains(&version) {
        return Err(FlashError::InvalidFirmwareFormat(format!(
            "EMMC_BOOT0_SPL_VERSION_UNSUPPORTED:{version}"
        )));
    }
    let length = u32::from_le_bytes(header[0x10..0x14].try_into().expect("four bytes")) as usize;
    if !(0x20..=MAINLINE_SPL_MAX_BYTES).contains(&length)
        || !length.is_multiple_of(4)
        || MAINLINE_SPL_DISK_OFFSET + length as u64 > image_len
    {
        return Err(FlashError::InvalidFirmwareFormat(format!(
            "EMMC_BOOT0_SPL_LENGTH_INVALID:{length}"
        )));
    }
    let mut spl = vec![0_u8; length];
    spl[..header.len()].copy_from_slice(&header);
    file.read_exact(&mut spl[header.len()..])
        .map_err(FlashError::Io)?;
    let stored = u32::from_le_bytes(spl[0x0c..0x10].try_into().expect("four bytes"));
    let expected_sum = stored.wrapping_mul(2).wrapping_sub(0x5f0a_6c39);
    let actual_sum = spl.chunks_exact(4).fold(0_u32, |sum, word| {
        sum.wrapping_add(u32::from_le_bytes(word.try_into().expect("four bytes")))
    });
    if actual_sum != expected_sum {
        return Err(FlashError::InvalidFirmwareFormat(
            "EMMC_BOOT0_SPL_CHECKSUM_INVALID".to_owned(),
        ));
    }

    Ok(spl)
}

struct FlashAccessGuard<'a> {
    ctx: &'a libefex::Context,
    storage_code: u32,
    active: bool,
}

impl<'a> FlashAccessGuard<'a> {
    fn enable(ctx: &'a libefex::Context, storage_code: u32) -> FlashResult<Self> {
        ctx.fes_flash_set_onoff(storage_code, true)
            .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
        Ok(Self {
            ctx,
            storage_code,
            active: true,
        })
    }

    fn disable(mut self) -> FlashResult<()> {
        self.ctx
            .fes_flash_set_onoff(self.storage_code, false)
            .map_err(|error| FlashError::UsbTransferError(error.to_string()))?;
        self.active = false;
        Ok(())
    }
}

impl Drop for FlashAccessGuard<'_> {
    fn drop(&mut self) {
        if self.active {
            let _ = self.ctx.fes_flash_set_onoff(self.storage_code, false);
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct RuntimeGeometryObservation {
    model: Option<String>,
    total_bytes: Option<u64>,
    sector_bytes: Option<u64>,
    queue_page_bytes: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RawWritePlan {
    total_bytes: u64,
    chunk_size: usize,
    chunk_count: u64,
    chunk_source: &'static str,
}

impl RawWritePlan {
    fn new(total_bytes: u64) -> Self {
        Self {
            total_bytes,
            chunk_size: RAW_READ_WINDOW_SIZE,
            chunk_count: total_bytes.div_ceil(RAW_READ_WINDOW_SIZE as u64),
            chunk_source: "fes_external_partition_window",
        }
    }
}

fn fes_status_label(error: &libefex::EfexError) -> String {
    match error {
        libefex::EfexError::FesStatus(status) => format!("0x{status:02X}"),
        _ => "transport_or_invalid_csw".to_owned(),
    }
}

fn resolve_capacity_sectors(
    storage_type: StorageType,
    probed_sectors: u64,
    observation: Option<RuntimeGeometryObservation>,
) -> FlashResult<(u64, &'static str)> {
    if probed_sectors != 0 {
        return Ok((probed_sectors, "fes_flash_size_probe"));
    }
    let observation = observation.ok_or_else(unknown_capacity)?;
    let sector_bytes = observation.sector_bytes.ok_or_else(unknown_capacity)?;
    let total_bytes = observation.total_bytes.ok_or_else(unknown_capacity)?;
    if sector_bytes != SECTOR_SIZE {
        return Err(FlashError::Unknown(
            "RAW_TARGET_GEOMETRY_UNSUPPORTED: runtime logical sector size is not 512 bytes"
                .to_owned(),
        ));
    }
    if total_bytes == 0 || total_bytes % sector_bytes != 0 {
        return Err(FlashError::Unknown(
            "RAW_TARGET_GEOMETRY_INVALID: runtime capacity is zero or not sector aligned"
                .to_owned(),
        ));
    }
    let sectors = total_bytes / sector_bytes;
    if sectors > u32::MAX as u64 {
        return Err(FlashError::Unknown(
            "RAW_TARGET_CAPACITY_OVERFLOW: runtime geometry exceeds the FES protocol".to_owned(),
        ));
    }
    let source = match storage_type {
        StorageType::Nand | StorageType::Spinand => "bootstrap_uart_nand_geometry",
        _ => "bootstrap_uart_storage_geometry",
    };
    Ok((sectors, source))
}

fn unknown_capacity() -> FlashError {
    FlashError::Unknown(
        "RAW_TARGET_CAPACITY_UNKNOWN: FES returned zero sectors and no verified runtime storage geometry was captured"
            .to_owned(),
    )
}

fn capacity_bytes_from_sectors(sectors: u64) -> FlashResult<u64> {
    if sectors == 0 {
        return Err(unknown_capacity());
    }
    sectors.checked_mul(SECTOR_SIZE).ok_or_else(|| {
        FlashError::Unknown(
            "RAW_TARGET_CAPACITY_OVERFLOW: FES sector geometry exceeds host limits".to_owned(),
        )
    })
}

struct RuntimeGeometryCapture {
    observation: Arc<Mutex<RuntimeGeometryObservation>>,
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl RuntimeGeometryCapture {
    fn start(config: &RuntimeStorageGeometrySerial) -> FlashResult<Self> {
        let mut port = serialport::new(&config.port, config.baud.max(1_200))
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|error| {
                FlashError::Unknown(format!(
                    "RAW_STORAGE_GEOMETRY_SERIAL_OPEN:port={} error={error}",
                    config.port
                ))
            })?;
        let observation = Arc::new(Mutex::new(RuntimeGeometryObservation::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let worker_observation = observation.clone();
        let worker_stop = stop.clone();
        let worker = std::thread::spawn(move || {
            let mut pending = String::new();
            let mut buffer = [0_u8; 512];
            while !worker_stop.load(Ordering::Relaxed) {
                match port.read(&mut buffer) {
                    Ok(read) if read > 0 => {
                        pending.push_str(&String::from_utf8_lossy(&buffer[..read]));
                        while let Some(newline) = pending.find(['\r', '\n']) {
                            let line = pending[..newline].to_owned();
                            pending.drain(..=newline);
                            if let Ok(mut current) = worker_observation.lock() {
                                observe_geometry_line(&mut current, &line);
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(_) => break,
                }
            }
            if let Ok(mut current) = worker_observation.lock() {
                observe_geometry_line(&mut current, &pending);
            }
        });
        Ok(Self {
            observation,
            stop,
            worker: Some(worker),
        })
    }

    fn finish(&mut self) -> RuntimeGeometryObservation {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
        self.observation
            .lock()
            .map(|value| value.clone())
            .unwrap_or_default()
    }
}

impl Drop for RuntimeGeometryCapture {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn observe_geometry_line(observation: &mut RuntimeGeometryObservation, line: &str) {
    let line = line.trim();
    if let Some(model) = line
        .find("SPI NAND:")
        .map(|offset| line[offset + "SPI NAND:".len()..].trim())
    {
        if !model.is_empty()
            && model.len() <= 64
            && model
                .chars()
                .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '-' | '_' | '.'))
        {
            observation.model = Some(model.to_owned());
        }
    }
    if let Some(value) = line
        .find("TotalSize:")
        .and_then(|offset| parse_binary_size(&line[offset + "TotalSize:".len()..]))
    {
        observation.total_bytes = Some(value);
    }
    if let Some(value) = line
        .find("SectorSize:")
        .and_then(|offset| parse_binary_size(&line[offset + "SectorSize:".len()..]))
    {
        observation.sector_bytes = Some(value);
    }
    let lower = line.to_ascii_lowercase();
    if let Some(offset) = lower.find("buf queue page size") {
        let value = line[offset + "buf queue page size".len()..]
            .trim_start_matches(|ch: char| ch.is_ascii_whitespace() || matches!(ch, ':' | '='))
            .split_whitespace()
            .next()
            .and_then(|value| value.parse::<u64>().ok());
        if let Some(value) = value {
            observation.queue_page_bytes = Some(value);
        }
    }
}

fn parse_binary_size(value: &str) -> Option<u64> {
    let mut fields = value.split_whitespace();
    let number = fields.next()?.parse::<u64>().ok()?;
    let unit = fields.next().unwrap_or("B").to_ascii_uppercase();
    let multiplier = match unit.as_str() {
        "B" | "BYTE" | "BYTES" => 1,
        "K" | "KB" | "KIB" => 1 << 10,
        "M" | "MB" | "MIB" => 1 << 20,
        "G" | "GB" | "GIB" => 1 << 30,
        _ => return None,
    };
    number.checked_mul(multiplier)
}

fn validate_raw_storage(storage_type: StorageType) -> FlashResult<()> {
    match storage_type {
        StorageType::Nand | StorageType::Spinand => Err(FlashError::Unknown(
            "RAW_FLASH_UNSUPPORTED_ON_NAND_USE_COMPONENTS: whole-disk raw bypasses NAND component and bad-block semantics"
                .to_owned(),
        )),
        StorageType::Auto => Err(FlashError::Unknown(
            "RAW_TARGET_STORAGE_UNKNOWN: FES did not identify the target storage".to_owned(),
        )),
        _ => Ok(()),
    }
}

fn ensure_not_cancelled() -> FlashResult<()> {
    let Some(path) = std::env::var_os(LYNX_CANCEL_FILE_ENV) else {
        return Ok(());
    };
    if Path::new(&path).is_file() {
        Err(FlashError::Unknown(
            "RAW_FLASH_CANCELLED: cancellation requested by LYNX before the next media operation"
                .to_owned(),
        ))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_raw_contract_is_sector_aligned_and_starts_at_disk_zero() {
        assert_eq!(SECTOR_SIZE, 512);
        let layout = RawMode::Command.layout(StorageType::Emmc);
        assert_eq!(layout.start_sector, 0xffff_6000);
        assert_eq!(layout.copies, 4);
        assert!(layout.wrapping_address);
        assert_eq!(RawMode::physical_start_sector(layout), 0);
        assert_eq!(
            (layout.start_sector as u32).wrapping_add(FES_SDMMC_LOGICAL_OFFSET_SECTORS),
            0
        );
        let spinor_layout = RawMode::Command.layout(StorageType::Spinor);
        assert_eq!(spinor_layout.start_sector, 0);
        assert!(!spinor_layout.wrapping_address);
        assert_eq!(RAW_READ_WINDOW_SIZE, 10 * 1024 * 1024);
        assert_eq!(RAW_READ_WINDOW_SIZE % SECTOR_SIZE as usize, 0);
    }

    #[test]
    fn missing_cancel_marker_keeps_raw_operation_enabled() {
        std::env::remove_var(LYNX_CANCEL_FILE_ENV);
        assert!(ensure_not_cancelled().is_ok());
    }

    #[test]
    fn raw_accepts_only_block_like_media_and_spinor() {
        for storage in [
            StorageType::Spinor,
            StorageType::Emmc,
            StorageType::Emmc3,
            StorageType::Emmc0,
            StorageType::Sdcard,
            StorageType::Sd1,
            StorageType::Ufs,
        ] {
            assert!(validate_raw_storage(storage).is_ok());
        }
        for storage in [StorageType::Nand, StorageType::Spinand] {
            assert!(validate_raw_storage(storage)
                .unwrap_err()
                .to_string()
                .contains("RAW_FLASH_UNSUPPORTED_ON_NAND_USE_COMPONENTS"));
        }
        assert!(validate_raw_storage(StorageType::Auto)
            .unwrap_err()
            .to_string()
            .contains("RAW_TARGET_STORAGE_UNKNOWN"));
    }

    fn test_mainline_spl() -> Vec<u8> {
        let mut spl = vec![0_u8; 0xc000];
        spl[4..12].copy_from_slice(b"eGON.BT0");
        spl[0x0c..0x10].copy_from_slice(&0x5f0a_6c39_u32.to_le_bytes());
        let length = spl.len() as u32;
        spl[0x10..0x14].copy_from_slice(&length.to_le_bytes());
        spl[0x14..0x18].copy_from_slice(b"SPL\x02");
        let checksum = spl.chunks_exact(4).fold(0_u32, |sum, word| {
            sum.wrapping_add(u32::from_le_bytes(word.try_into().unwrap()))
        });
        spl[0x0c..0x10].copy_from_slice(&checksum.to_le_bytes());
        spl
    }

    #[test]
    fn extracts_only_a_checksum_valid_mainline_spl_at_the_emmc_boot_offset() {
        let spl = test_mainline_spl();
        let mut image = vec![0_u8; MAINLINE_SPL_DISK_OFFSET as usize];
        image.extend_from_slice(&spl);
        image.resize(2 * 1024 * 1024, 0);
        let path = std::env::temp_dir().join(format!(
            "openix-emmc-boot0-{}-{}.img",
            std::process::id(),
            std::thread::current().name().unwrap_or("test")
        ));
        std::fs::write(&path, &image).unwrap();
        assert_eq!(
            extract_mainline_spl(&path, image.len() as u64).unwrap(),
            spl
        );

        image[MAINLINE_SPL_DISK_OFFSET as usize + 0x100] ^= 0x80;
        std::fs::write(&path, &image).unwrap();
        assert!(extract_mainline_spl(&path, image.len() as u64)
            .unwrap_err()
            .to_string()
            .contains("EMMC_BOOT0_SPL_CHECKSUM_INVALID"));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn emmc_boot0_route_rejects_non_emmc_media() {
        for storage in [StorageType::Emmc, StorageType::Emmc3, StorageType::Emmc0] {
            assert!(is_emmc_storage(storage));
        }
        for storage in [
            StorageType::Sdcard,
            StorageType::Sd1,
            StorageType::Ufs,
            StorageType::Spinor,
            StorageType::Nand,
            StorageType::Spinand,
        ] {
            assert!(!is_emmc_storage(storage));
        }
    }

    #[test]
    fn nand_geometry_converts_sectors_to_bytes_and_zero_is_unknown() {
        assert_eq!(capacity_bytes_from_sectors(524_288).unwrap(), 268_435_456);
        assert!(capacity_bytes_from_sectors(0)
            .unwrap_err()
            .to_string()
            .contains("RAW_TARGET_CAPACITY_UNKNOWN"));
    }

    #[test]
    fn runtime_observation_resolves_capacity_without_a_model_table() {
        let mut observation = RuntimeGeometryObservation::default();
        observe_geometry_line(&mut observation, "[boot] SPI NAND: W25N02KVZEIR");
        observe_geometry_line(&mut observation, "[boot] TotalSize: 256 M");
        observe_geometry_line(&mut observation, "[boot] SectorSize: 512 B");
        let (sectors, source) =
            resolve_capacity_sectors(StorageType::Spinand, 0, Some(observation)).unwrap();
        assert_eq!(source, "bootstrap_uart_nand_geometry");
        assert_eq!(sectors, 524_288);
        assert_eq!(capacity_bytes_from_sectors(sectors).unwrap(), 268_435_456);
    }

    #[test]
    fn runtime_geometry_supports_other_models_sizes_and_storage_types() {
        let observation = RuntimeGeometryObservation {
            model: Some("UNKNOWN".to_owned()),
            total_bytes: Some(512 << 20),
            sector_bytes: Some(512),
            queue_page_bytes: None,
        };
        let (sectors, source) =
            resolve_capacity_sectors(StorageType::Emmc, 0, Some(observation)).unwrap();
        assert_eq!(sectors, 1_048_576);
        assert_eq!(source, "bootstrap_uart_storage_geometry");
    }

    #[test]
    fn runtime_geometry_rejects_non_fes_sector_sizes_and_unaligned_capacity() {
        let unsupported_sector = RuntimeGeometryObservation {
            model: None,
            total_bytes: Some(256 << 20),
            sector_bytes: Some(4_096),
            queue_page_bytes: None,
        };
        assert!(
            resolve_capacity_sectors(StorageType::Spinand, 0, Some(unsupported_sector))
                .unwrap_err()
                .to_string()
                .contains("RAW_TARGET_GEOMETRY_UNSUPPORTED")
        );

        let unaligned = RuntimeGeometryObservation {
            model: None,
            total_bytes: Some(1000),
            sector_bytes: Some(512),
            queue_page_bytes: None,
        };
        assert!(
            resolve_capacity_sectors(StorageType::Spinand, 0, Some(unaligned))
                .unwrap_err()
                .to_string()
                .contains("RAW_TARGET_GEOMETRY_INVALID")
        );
    }

    #[test]
    fn nonzero_fes_capacity_remains_authoritative() {
        let (sectors, source) =
            resolve_capacity_sectors(StorageType::Spinand, 123_456, None).unwrap();
        assert_eq!(sectors, 123_456);
        assert_eq!(source, "fes_flash_size_probe");
    }

    #[test]
    fn write_plan_matches_fes_external_partition_windows() {
        let plan = RawWritePlan::new(29_097_984);
        assert_eq!(plan.chunk_size, 10 * 1024 * 1024);
        assert_eq!(plan.chunk_count, 3);
        assert_eq!(plan.chunk_source, "fes_external_partition_window");
    }

    #[test]
    fn fes_status_is_preserved_for_chunk_failure_telemetry() {
        assert_eq!(fes_status_label(&libefex::EfexError::FesStatus(7)), "0x07");
        assert_eq!(
            fes_status_label(&libefex::EfexError::UsbTransfer),
            "transport_or_invalid_csw"
        );
    }
}
