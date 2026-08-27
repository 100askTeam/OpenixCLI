//! Flash command implementation

use crate::commands::FlashArgs;
use crate::firmware::LoadedFirmware;
use crate::flash::{
    FlashEvent, FlashEventSink, FlashLogLevel, Flasher, RawDiskFlasher,
    RuntimeStorageGeometrySerial,
};
use crate::utils::logger::Logger;

/// Execute the flash command
///
/// Loads firmware from the specified path and flashes it to the device
///
/// # Arguments
/// * `args` - Flash arguments including firmware path, device selection, and flash options
///
/// # Returns
/// Ok(()) on success, Error on failure
pub async fn execute(args: FlashArgs) -> anyhow::Result<()> {
    let logger = if args.jsonl {
        Logger::for_events(args.verbose, FlashEventSink::from_fn(print_json_event))
    } else {
        Logger::with_verbose(args.verbose)
    };

    logger.info(&format!(
        "Loading firmware: {}",
        args.firmware_path.display()
    ));

    if !args.firmware_path.exists() {
        logger.error(&format!(
            "Firmware file not found: {}",
            args.firmware_path.display()
        ));
        return Err(anyhow::anyhow!("Firmware file not found"));
    }

    let request = args.request();

    if args.raw {
        logger.info("Input format: raw whole-disk image (sector 0)");
        let bootstrap_packer = args
            .bootstrap_firmware_path
            .as_deref()
            .map(LoadedFirmware::load)
            .transpose()?
            .map(LoadedFirmware::into_packer);
        let mut flasher = RawDiskFlasher::new(
            args.firmware_path,
            bootstrap_packer,
            args.storage_geometry_serial_port
                .map(|port| RuntimeStorageGeometrySerial {
                    port,
                    baud: args.storage_geometry_serial_baud,
                }),
            args.raw_mode,
            args.emmc_boot0_from_image,
            request,
            logger.clone(),
        );
        if let Err(error) = flasher.execute().await {
            logger.error(&format!("Raw disk flash failed: {error}"));
            return Err(anyhow::anyhow!("{error}"));
        }
        if !args.jsonl {
            println!();
        }
        logger.stage_complete("Raw disk image flashed successfully");
        return Ok(());
    }

    let loaded = LoadedFirmware::load(&args.firmware_path)?;
    let bootstrap = args
        .bootstrap_firmware_path
        .as_deref()
        .map(LoadedFirmware::load)
        .transpose()?
        .map(LoadedFirmware::into_packer);

    let image_info = loaded.image_info();
    logger.info(&format!(
        "Firmware size: {} MB, {} files",
        image_info.image_size / (1024 * 1024),
        image_info.num_files
    ));

    if let Some(location) = args.device_location.as_deref() {
        logger.info(&format!("Selected physical USB location: {location}"));
    } else if let (Some(bus), Some(port)) = (args.bus, args.port) {
        logger.info(&format!("Selected device: Bus {}, Port {}", bus, port));
    } else {
        logger.info("No device specified, will use first available device");
    }

    let mut flasher = Flasher::new(loaded.into_packer(), request, logger.clone());
    if args.nand_components {
        logger.info("NAND_COMPONENT_FLASH:bad_block_aware");
        flasher = flasher.for_nand_components();
    }
    if let Some(bootstrap) = bootstrap {
        logger.info("FES_BOOTSTRAP_SOURCE:external_ram_only_package");
        flasher = flasher.with_bootstrap(bootstrap);
    }
    if let Err(e) = flasher.execute().await {
        logger.error(&format!("Flash failed: {}", e));
        return Err(anyhow::anyhow!("{}", e));
    }

    if !args.jsonl {
        println!();
    }
    logger.stage_complete("All partitions flashed successfully");

    Ok(())
}

fn print_json_event(event: FlashEvent) {
    let value = match event {
        FlashEvent::Log { level, message } => serde_json::json!({
            "type": "log",
            "level": match level {
                FlashLogLevel::Info => "info",
                FlashLogLevel::Success => "success",
                FlashLogLevel::Warn => "warn",
                FlashLogLevel::Error => "error",
                FlashLogLevel::Debug => "debug",
            },
            "message": message,
        }),
        FlashEvent::StagesDefined(stages) => serde_json::json!({
            "type": "stages",
            "stages": stages.into_iter().map(|stage| stage.name()).collect::<Vec<_>>(),
        }),
        FlashEvent::StageStarted(stage) => {
            serde_json::json!({"type": "stage", "status": "running", "stage": stage.name()})
        }
        FlashEvent::StageCompleted(stage) => {
            serde_json::json!({"type": "stage", "status": "success", "stage": stage.name()})
        }
        FlashEvent::PartitionStageWeight(total) => {
            serde_json::json!({"type": "partition_weight", "total": total})
        }
        FlashEvent::PartitionStarted(partition) => {
            serde_json::json!({"type": "partition", "name": partition})
        }
        FlashEvent::Progress {
            overall_percent,
            stage_progress,
            total,
            speed,
        } => serde_json::json!({
            "type": "progress",
            "percent": overall_percent,
            "current": stage_progress,
            "transferredBytes": stage_progress,
            "queuedBytes": null,
            "committedBytes": null,
            "total": total,
            "bytesPerSecond": speed,
        }),
        FlashEvent::Verification { status, error_code } => serde_json::json!({
            "type": "verify",
            "status": status,
            "errorCode": error_code,
        }),
        FlashEvent::PostAction {
            action,
            status,
            attempt,
            error,
        } => serde_json::json!({
            "type": "post_action",
            "action": action.to_string(),
            "status": status,
            "attempt": attempt,
            "error": error,
        }),
        FlashEvent::Finished { post_action } => serde_json::json!({
            "type": "finished",
            "postAction": post_action.to_string(),
        }),
    };
    println!("{value}");
}
