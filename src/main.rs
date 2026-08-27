//! OpenixCLI-cli - Firmware flashing CLI tool for Allwinner chips
//!
//! This tool provides the following functionality:
//! - Scan for connected Allwinner devices via USB
//! - Flash firmware to device storage (NAND/eMMC/SD card, etc.)
//! - Support multiple flash modes and post-flash actions
//! - Interactive TUI mode (default when no subcommand given)
//!
//! Usage examples:
//!   openixcli              # Launch interactive TUI (default)
//!   openixcli tui          # Launch interactive TUI (explicit)
//!   openixcli scan         # Scan for connected devices
//!   openixcli flash firmware.fex  # Flash firmware to device

use clap::Parser;

mod cli;
mod commands;
mod config;
mod firmware;
mod flash;
mod process;
mod tui;
mod utils;

/// CLI structure parsed from command line arguments
use cli::{Cli, Commands, OutputFormat};
use commands::{parse_partition_list, FlashArgs};
use flash::{FlashMode, PostAction, RawMode};
use utils::TermLogger;

const RAW_GEOMETRY_SERIAL_PORT_ENV: &str = "OPENIXCLI_STORAGE_GEOMETRY_SERIAL_PORT";
const RAW_GEOMETRY_SERIAL_BAUD_ENV: &str = "OPENIXCLI_STORAGE_GEOMETRY_SERIAL_BAUD";

fn parse_raw_geometry_environment(
    port: Option<String>,
    baud: Option<String>,
) -> (Option<String>, u32) {
    let port = port.filter(|value| !value.trim().is_empty());
    let baud = baud
        .and_then(|value| value.parse::<u32>().ok())
        .filter(|value| *value >= 1_200)
        .unwrap_or(115_200);
    (port, baud)
}

fn raw_geometry_environment() -> (Option<String>, u32) {
    parse_raw_geometry_environment(
        std::env::var(RAW_GEOMETRY_SERIAL_PORT_ENV).ok(),
        std::env::var(RAW_GEOMETRY_SERIAL_BAUD_ENV).ok(),
    )
}

/// Command-layout raw flashing must not be promoted to a full-chip erase.
/// The MBR command creates the target raw volume and owns its layout.
const fn command_raw_fes_policy() -> FlashMode {
    FlashMode::Partition
}

/// Initialize the logging system
///
/// # Parameters
/// * `verbose` - Enable verbose output mode
///
/// If initialization fails, error message is printed to stderr but program continues
fn setup_logging(verbose: bool) {
    if let Err(e) = TermLogger::init(verbose) {
        eprintln!("Failed to initialize logger: {}", e);
    }
}

#[tokio::main]
/// Program entry point
///
/// Parses command line arguments and executes corresponding commands:
/// - No subcommand / `tui`: Launch interactive TUI
/// - `scan`: Scan for USB devices
/// - `flash`: Flash firmware to device
///
/// # Returns
/// Ok(()) on success, anyhow::Error on failure
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        None | Some(Commands::Tui) => {
            // TUI mode - don't init the standard logger, TUI has its own
            tui::run().await?;
        }
        Some(Commands::Scan { detailed }) => {
            setup_logging(cli.verbose);
            commands::scan::execute(detailed).await?;
        }
        Some(Commands::Flash {
            firmware,
            raw,
            emmc_boot0_from_image,
            bootstrap_firmware,
            nand_components,
            storage_geometry_serial_port,
            storage_geometry_serial_baud,
            bus,
            port,
            device_location,
            verify,
            mode,
            partitions,
            post_action,
        }) => {
            setup_logging(cli.verbose);

            let args = FlashArgs {
                firmware_path: firmware.into(),
                raw,
                nand_components,
                raw_mode: RawMode::Command,
                emmc_boot0_from_image,
                bootstrap_firmware_path: bootstrap_firmware.map(Into::into),
                storage_geometry_serial_port,
                storage_geometry_serial_baud,
                bus,
                port,
                device_location,
                verify,
                mode,
                partitions: parse_partition_list(partitions),
                post_action,
                verbose: cli.verbose,
                jsonl: cli.output == OutputFormat::Jsonl,
            };

            commands::flash::execute(args).await?;
        }
        Some(Commands::Raw {
            loader,
            firmware,
            mode,
            emmc_boot0_from_image,
            bus,
            port,
            device_location,
        }) => {
            setup_logging(cli.verbose);
            let (storage_geometry_serial_port, storage_geometry_serial_baud) =
                raw_geometry_environment();
            let args = FlashArgs {
                firmware_path: firmware.into(),
                raw: true,
                nand_components: false,
                raw_mode: mode,
                emmc_boot0_from_image,
                bootstrap_firmware_path: Some(loader.into()),
                storage_geometry_serial_port,
                storage_geometry_serial_baud,
                bus,
                port,
                device_location,
                verify: true,
                mode: command_raw_fes_policy(),
                partitions: None,
                post_action: PostAction::None,
                verbose: cli.verbose,
                jsonl: cli.output == OutputFormat::Jsonl,
            };
            commands::flash::execute(args).await?;
        }
        Some(Commands::BootMainline {
            plan,
            device_location,
            bus,
            port,
        }) => {
            setup_logging(cli.verbose);
            commands::mainline::execute(
                plan.into(),
                device_location,
                bus,
                port,
                cli.output == OutputFormat::Jsonl,
            )?;
        }
        Some(Commands::Inspect { firmware }) => {
            setup_logging(cli.verbose);
            commands::inspect::execute(firmware.into()).await?;
        }
        Some(Commands::Unpack {
            firmware,
            output_dir,
        }) => {
            setup_logging(cli.verbose);
            commands::unpack::execute(commands::UnpackArgs {
                firmware_path: firmware.into(),
                output: output_dir.map(std::path::PathBuf::from),
            })
            .await?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{command_raw_fes_policy, parse_raw_geometry_environment};
    use crate::flash::FlashMode;

    #[test]
    fn command_raw_layout_uses_command_fes_policy() {
        assert_eq!(command_raw_fes_policy(), FlashMode::Partition);
        assert_eq!(command_raw_fes_policy().erase_flag(), 0x00);
    }

    #[test]
    fn command_raw_accepts_board_scoped_geometry_out_of_band() {
        assert_eq!(
            parse_raw_geometry_environment(
                Some("/dev/ttyACM0".to_owned()),
                Some("115200".to_owned())
            ),
            (Some("/dev/ttyACM0".to_owned()), 115_200)
        );
    }

    #[test]
    fn invalid_or_empty_geometry_environment_is_not_trusted() {
        assert_eq!(
            parse_raw_geometry_environment(Some(String::new()), Some("0".to_owned())),
            (None, 115_200)
        );
    }
}
