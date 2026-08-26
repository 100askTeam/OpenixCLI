//! FES (Flash Eraser Script) handler
//!
//! Handles FES mode operations for devices in U-Boot mode
//! FES mode is used for flashing partitions and boot images to storage

mod boot_download;
mod constants;
mod erase_flag;
mod mbr_download;
mod partition;
mod partition_planner;
mod types;
mod ubifs_config;

pub use boot_download::BootDownload;
pub use erase_flag::EraseFlag;
pub use mbr_download::MbrDownload;
pub use partition::PartitionDownload;
pub use partition_planner::PartitionPlanner;
pub(crate) use types::IncrementalChecksum;
pub use ubifs_config::UbifsConfig;

use crate::config::boot_header::get_sunxi_boot_file_mode_string;
use crate::config::mbr_parser::SunxiMbr;
use crate::firmware::{OpenixPacker, StorageType};
use crate::flash::{FlashMode, FlashRequest};
use crate::process::StageType;
use crate::utils::{FlashError, FlashResult, Logger};

/// FES handler for devices in U-Boot mode
///
/// Handles partition flashing, MBR writing, and boot image downloading
/// for devices that are in FES mode (U-Boot)
pub struct FesHandler<'a> {
    logger: &'a mut Logger,
}

impl<'a> FesHandler<'a> {
    /// Create a new FES handler
    pub fn new(logger: &'a mut Logger) -> Self {
        Self { logger }
    }

    /// Handle FES mode operations
    ///
    /// Executes the full flashing process:
    /// 1. Query device information (boot mode, storage type, flash size)
    /// 2. Erase flash if required
    /// 3. Download MBR
    /// 4. Download partitions
    /// 5. Download boot images
    pub async fn handle(
        &mut self,
        ctx: &libefex::Context,
        packer: &mut OpenixPacker,
        request: &FlashRequest,
        nand_components: bool,
    ) -> FlashResult<()> {
        self.logger.begin_stage(StageType::FesQuery);

        let secure = ctx
            .fes_query_secure()
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
        self.logger.info(&format!(
            "Boot mode: {}",
            get_sunxi_boot_file_mode_string(secure)
        ));

        let storage_type = ctx
            .fes_query_storage()
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
        let storage_kind = StorageType::from(storage_type);
        self.logger.info(&format!("Storage type: {}", storage_kind));

        if nand_components {
            validate_nand_component_storage(storage_kind)?;
            validate_nand_component_mode(request.mode)?;
            self.logger
                .info("NAND_COMPONENT_STORAGE_PREFLIGHT:accepted");
            self.logger
                .info(&format!("NAND_COMPONENT_LAYOUT_POLICY:{}", request.mode));
        }

        let flash_size = ctx
            .fes_probe_flash_size()
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
        self.logger.info(&format!(
            "Flash size: {} MB",
            (flash_size as u64) * 512 / 1024 / 1024
        ));

        self.logger.complete_stage();

        // Lab-only, read-only SPI-NAND dump used to compare the physical boot
        // area with the packaged SPL/U-Boot images. LYNX never sets this.
        if let Some(path) = std::env::var_os("OPENIXCLI_DIAGNOSTIC_DUMP_SPINAND_BOOT") {
            let mut boot0 = vec![0u8; 24 * 1024];
            ctx.fes_spinand_up(&mut boot0, 0, libefex::FesDataType::NandBoot0)
                .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
            let mut boot1_head = vec![0u8; 64 * 1024];
            ctx.fes_spinand_up(&mut boot1_head, 0, libefex::FesDataType::NandBoot1)
                .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
            std::fs::write(&path, &boot0)
                .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;
            let boot1_path = std::path::PathBuf::from(&path).with_extension("boot1-head.bin");
            std::fs::write(&boot1_path, &boot1_head)
                .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;
            self.logger.stage_complete(&format!(
                "DIAGNOSTIC_DUMP_SPINAND_BOOT:wrote:{}:{};{}:{}",
                std::path::Path::new(&path).display(),
                boot0.len(),
                boot1_path.display(),
                boot1_head.len()
            ));
            return Ok(());
        }

        // Explicit lab-recovery gate. LYNX never sets this variable during a
        // production flash; it is used only with a recovery loader whose force
        // erase implementation scrubs false NAND bad-block markers.
        if std::env::var_os("OPENIXCLI_DIAGNOSTIC_FORCE_ERASE_FLASH").is_some() {
            self.logger
                .warn("DIAGNOSTIC_FORCE_ERASE_FLASH: scrubbing the complete selected medium");
            ctx.fes_force_erase_flash()
                .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
            self.logger
                .stage_complete("DIAGNOSTIC_FORCE_ERASE_FLASH: device reported success");
            return Ok(());
        }

        if request.mode != FlashMode::Partition {
            self.logger.begin_stage(StageType::FesErase);
            let erase_flag = EraseFlag::new(&*self.logger);
            erase_flag.execute(ctx, request.mode).await?;
            self.logger.complete_stage();
        }

        self.logger.begin_stage(StageType::FesMbr);

        let mbr_data = packer.get_mbr().map_err(|_| FlashError::MbrNotFound)?;
        let mbr = SunxiMbr::parse(&mbr_data)
            .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;
        let mbr_info = mbr.to_mbr_info();

        self.logger
            .info(&format!("Found {} partitions in MBR", mbr_info.part_count));

        let partition_planner = PartitionPlanner::new(&*self.logger);
        let download_list = partition_planner.prepare(packer, &mbr_info, request)?;

        let ubifs_config = UbifsConfig::new(&*self.logger);
        ubifs_config.execute(ctx, &mut *packer, &download_list, storage_kind)?;

        let mbr_download = MbrDownload::new(&*self.logger);
        mbr_download.execute(ctx, &mbr_data).await?;

        self.logger.complete_stage();

        // Lab-only discriminator for FES compatibility investigations. This is
        // deliberately environment-gated and is never set by the LYNX worker.
        // It lets us prove that a loader accepts a package MBR without writing
        // any partition or boot payload from that package.
        if std::env::var_os("OPENIXCLI_DIAGNOSTIC_STOP_AFTER_MBR").is_some() {
            self.logger
                .info("DIAGNOSTIC_STOP_AFTER_MBR: no partition or boot payload was written");
            return Ok(());
        }

        if !download_list.is_empty() {
            self.logger.begin_stage(StageType::FesPartitions);

            let total_bytes: u64 = download_list.iter().map(|p| p.data_length).sum();
            self.logger.set_partition_stage_weight(total_bytes);

            {
                let mut partition_download = PartitionDownload::new(&mut *self.logger);
                partition_download
                    .execute(ctx, packer, &download_list, request.verify)
                    .await?;
            }

            self.logger.complete_stage();

            self.logger.begin_stage(StageType::FesBoot);
            let boot_download = BootDownload::new(&*self.logger);
            boot_download
                .execute(ctx, packer, secure, storage_type)
                .await?;
            self.logger.complete_stage();
        }

        Ok(())
    }
}

fn validate_nand_component_storage(storage_type: StorageType) -> FlashResult<()> {
    match storage_type {
        StorageType::Nand | StorageType::Spinand => Ok(()),
        other => Err(FlashError::InvalidFirmwareFormat(format!(
            "NAND_COMPONENT_PACKAGE_STORAGE_MISMATCH: device storage is {other}"
        ))),
    }
}

fn validate_nand_component_mode(mode: FlashMode) -> FlashResult<()> {
    match mode {
        FlashMode::PartitionErase | FlashMode::FullErase => Ok(()),
        FlashMode::Partition | FlashMode::KeepData => Err(FlashError::InvalidFirmwareFormat(
            format!(
                "NAND_COMPONENT_LAYOUT_REQUIRES_ERASE_POLICY: mode={mode}; use partition_erase or full_erase"
            ),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::{validate_nand_component_mode, validate_nand_component_storage};
    use crate::firmware::StorageType;
    use crate::flash::FlashMode;

    #[test]
    fn component_package_accepts_only_nand_media() {
        assert!(validate_nand_component_storage(StorageType::Nand).is_ok());
        assert!(validate_nand_component_storage(StorageType::Spinand).is_ok());
        for storage in [
            StorageType::Spinor,
            StorageType::Sdcard,
            StorageType::Sd1,
            StorageType::Emmc,
            StorageType::Emmc3,
            StorageType::Emmc0,
            StorageType::Ufs,
            StorageType::Auto,
        ] {
            let error = validate_nand_component_storage(storage)
                .expect_err("component package must be rejected on non-NAND media");
            assert!(error
                .to_string()
                .contains("NAND_COMPONENT_PACKAGE_STORAGE_MISMATCH"));
        }
    }

    #[test]
    fn nand_component_layout_accepts_bad_block_aware_erase_policies() {
        assert!(validate_nand_component_mode(FlashMode::PartitionErase).is_ok());
        assert!(validate_nand_component_mode(FlashMode::FullErase).is_ok());
    }

    #[test]
    fn nand_component_layout_rejects_modes_that_cannot_replace_the_mbr() {
        for mode in [FlashMode::Partition, FlashMode::KeepData] {
            let error = validate_nand_component_mode(mode)
                .expect_err("component MBR replacement requires an erase policy");
            assert!(error
                .to_string()
                .contains("NAND_COMPONENT_LAYOUT_REQUIRES_ERASE_POLICY"));
        }
    }
}
