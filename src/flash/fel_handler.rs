use crate::config::boot_header::{Boot0Header, UBootHeader, WORK_MODE_USB_PRODUCT};
use crate::config::sys_config::DramParamInfo;
use crate::utils::{FlashError, FlashResult, Logger};
use std::time::Duration;

const DRAM_INIT_CHECK_INTERVAL: Duration = Duration::from_millis(1000);
const DRAM_INIT_TIMEOUT: Duration = Duration::from_secs(60);

const UBOOT_MAX_LEN: usize = 2 * 1024 * 1024;
const DTB_MAX_LEN: usize = 1024 * 1024;
const SYS_CONFIG_BIN00_MAX_LEN: usize = 512 * 1024;

pub struct FelHandler<'a> {
    logger: &'a Logger,
}

impl<'a> FelHandler<'a> {
    pub fn new(logger: &'a Logger) -> Self {
        Self { logger }
    }

    pub async fn handle(&self, ctx: &mut libefex::Context, fes_data: &[u8]) -> FlashResult<()> {
        self.init_dram(ctx, fes_data).await
    }

    async fn init_dram(&self, ctx: &mut libefex::Context, fes_data: &[u8]) -> FlashResult<()> {
        self.logger.info("Initializing DRAM...");

        let fes_head = Boot0Header::parse(fes_data)
            .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;

        let run_addr = fes_head.run_addr;
        let ret_addr = fes_head.ret_addr;

        self.logger.debug(&format!(
            "FES magic: {}, run_addr: 0x{:x}, ret_addr: 0x{:x}",
            fes_head.magic_str(),
            run_addr,
            ret_addr
        ));

        let dram_param = DramParamInfo::create_empty();
        let dram_buffer = dram_param.serialize();

        self.logger
            .debug(&format!("Clearing DRAM param area at 0x{:x}", ret_addr));
        ctx.fel_write(ret_addr, &dram_buffer)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        let timeout_secs = std::cmp::max(3, fes_data.len() / (64 * 1024));
        self.logger.debug(&format!(
            "Downloading {} bytes FES to device (timeout: {}s)...",
            fes_data.len(),
            timeout_secs
        ));

        ctx.fel_write(run_addr, fes_data)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        self.logger
            .debug(&format!("Executing FES at 0x{:x}", run_addr));
        ctx.fel_exec(run_addr)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        self.logger.info("Waiting for DRAM initialization...");
        let start = std::time::Instant::now();
        let mut attempts = 0;
        let mut dram_info = DramParamInfo::create_empty();

        while start.elapsed() < DRAM_INIT_TIMEOUT {
            attempts += 1;
            tokio::time::sleep(DRAM_INIT_CHECK_INTERVAL).await;

            let mut dram_result = vec![0u8; std::mem::size_of::<DramParamInfo>()];
            match ctx.fel_read(ret_addr, &mut dram_result) {
                Ok(_) => {
                    dram_info = *DramParamInfo::parse(&dram_result)
                        .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;

                    let dram_init_flag = dram_info.dram_init_flag;
                    let dram_update_flag = dram_info.dram_update_flag;

                    self.logger.debug(&format!(
                        "DRAM init check #{}: init_flag={}, update_flag={}",
                        attempts, dram_init_flag, dram_update_flag
                    ));

                    if dram_init_flag != 0 {
                        break;
                    }
                }
                Err(e) => {
                    self.logger
                        .debug(&format!("DRAM init check #{} failed: {}", attempts, e));
                }
            }
        }

        let elapsed = start.elapsed();
        self.logger.debug(&format!(
            "DRAM init completed after {} attempts, {:?}",
            attempts, elapsed
        ));

        let dram_init_flag = dram_info.dram_init_flag;
        if dram_init_flag == 1 {
            return Err(FlashError::DramInitFailed);
        }

        self.logger.info("DRAM initialized successfully");
        Ok(())
    }

    pub async fn download_uboot(
        &self,
        ctx: &libefex::Context,
        uboot_data: &[u8],
        dtb_data: Option<&[u8]>,
        sysconfig_data: &[u8],
        board_config_data: Option<&[u8]>,
    ) -> FlashResult<()> {
        self.logger.info(&format!(
            "Downloading U-Boot ({} bytes)...",
            uboot_data.len()
        ));

        let mut uboot_buffer = uboot_data.to_vec();
        UBootHeader::set_work_mode(&mut uboot_buffer, WORK_MODE_USB_PRODUCT);

        let uboot_head = UBootHeader::parse(&uboot_buffer)
            .map_err(|e| FlashError::InvalidFirmwareFormat(e.to_string()))?;

        let run_addr = uboot_head.uboot_head.run_addr;

        self.logger.debug(&format!(
            "U-Boot magic: {}, addr: 0x{:x}",
            uboot_head.uboot_head.magic_str(),
            run_addr
        ));

        let timeout_secs = std::cmp::max(10, uboot_data.len() / (64 * 1024));
        self.logger.debug(&format!(
            "Setting timeout to {}s for {} bytes",
            timeout_secs,
            uboot_data.len()
        ));

        ctx.fel_write(run_addr, &uboot_buffer)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        let dtb_sysconfig_base = run_addr + UBOOT_MAX_LEN as u32;

        if let Some(dtb) = dtb_data {
            ctx.fel_write(dtb_sysconfig_base, dtb)
                .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
            self.logger.debug(&format!(
                "DTB written to 0x{:x} ({} bytes)",
                dtb_sysconfig_base,
                dtb.len()
            ));
        }

        let sys_config_bin_base = dtb_sysconfig_base + DTB_MAX_LEN as u32;
        ctx.fel_write(sys_config_bin_base, sysconfig_data)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
        self.logger.debug(&format!(
            "SysConfig written to 0x{:x} ({} bytes)",
            sys_config_bin_base,
            sysconfig_data.len()
        ));

        if let Some(board_config) = board_config_data {
            let board_config_bin_base = sys_config_bin_base + SYS_CONFIG_BIN00_MAX_LEN as u32;
            ctx.fel_write(board_config_bin_base, board_config)
                .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;
            self.logger.debug(&format!(
                "BoardConfig written to 0x{:x} ({} bytes)",
                board_config_bin_base,
                board_config.len()
            ));
        }

        self.logger
            .debug(&format!("Executing U-Boot at 0x{:x}", run_addr));
        ctx.fel_exec(run_addr)
            .map_err(|e| FlashError::UsbTransferError(e.to_string()))?;

        self.logger.info("U-Boot downloaded and executed");
        Ok(())
    }
}
