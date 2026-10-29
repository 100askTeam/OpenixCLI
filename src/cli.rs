//! Command-line interface definitions
//!
//! Defines the CLI structure using clap for argument parsing

use clap::{Parser, Subcommand, ValueEnum};

use crate::flash::{FlashMode, PostAction, RawMode};

/// Main CLI structure
///
/// # Fields
/// * `command` - The subcommand to execute (scan, flash, or tui). Defaults to TUI if none given.
/// * `verbose` - Enable verbose output
#[derive(Parser)]
#[command(name = "openixcli")]
#[command(about = "Firmware flashing CLI tool for Allwinner chips", long_about = None)]
#[command(version)]
pub struct Cli {
    /// The subcommand to execute (defaults to TUI if omitted)
    #[command(subcommand)]
    pub command: Option<Commands>,

    /// Enable verbose output
    #[arg(short, long, global = true, help = "Enable verbose output")]
    pub verbose: bool,

    /// Machine-readable output format for automation.
    #[arg(long, global = true, value_enum, default_value = "text")]
    pub output: OutputFormat,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum OutputFormat {
    Text,
    Jsonl,
}

/// Available CLI commands
#[derive(Subcommand)]
pub enum Commands {
    /// Scan for connected devices
    Scan {
        /// Get detailed device information (requires device initialization)
        #[arg(short = 'l', long, help = "Get detailed device information")]
        detailed: bool,
    },

    /// Boot a board-matched loader in RAM, enter FES, and write a whole-disk
    /// raw image using the selected raw layout.
    Raw {
        /// Board-matched IMAGEWTY loader used only for RAM DRAM/FES bootstrap.
        #[arg(help = "RAM-only board loader; this file is never written to target storage")]
        loader: String,

        /// Sector-aligned whole-disk raw image written after FES is ready.
        #[arg(help = "Whole-disk raw image")]
        firmware: String,

        /// Raw disk layout. Command mode always starts at sector 0.
        #[arg(long, value_enum, default_value = "command")]
        mode: RawMode,

        /// Action after a successful raw write and verification. Keep the
        /// existing no-action default unless the caller explicitly requests one.
        #[arg(short = 'a', long, default_value = "none")]
        post_action: PostAction,

        /// Extract a checksum-valid mainline SPL from the standard 8 KiB
        /// offset in the raw image and install it through FES Boot0 semantics.
        #[arg(long)]
        emmc_boot0_from_image: bool,

        #[arg(short, long)]
        bus: Option<u8>,

        #[arg(short = 'P', long)]
        port: Option<u8>,

        #[arg(long)]
        device_location: Option<String>,
    },

    /// Read F101 NOR through a physical XIP window. Never erases or reboots.
    SpinorReadback {
        /// Load the fingerprinted F101 vendor RAM service before physical diagnostics.
        #[arg(long)]
        ram_loader: Option<String>,
        /// Explicitly configure and restore a physical read window in F101 ROM FEL.
        /// Changes controller registers; does not write flash. Default is passive.
        #[arg(long)]
        initialize_window: bool,
        #[arg(long)]
        device_location: String,
        /// New JSON evidence file; never overwrite previous evidence.
        #[arg(long)]
        report: String,
        /// New binary output. Omit to check physical-read readiness only.
        #[arg(long)]
        dump: Option<String>,
        /// Bytes from physical zero (decimal; maximum 16 MiB for this profile).
        #[arg(long, default_value_t = 16 * 1024 * 1024)]
        length: u32,
    },

    /// Flash firmware to device
    Flash {
        /// Path to firmware file
        #[arg(help = "Path to firmware file")]
        firmware: String,

        /// Treat the input as a sector-aligned whole-disk image. The target
        /// must already be running the Allwinner FES write service.
        #[arg(
            long,
            help = "Flash a raw whole-disk image to sector 0 through the FES service"
        )]
        raw: bool,

        /// For an eMMC raw image, extract the mainline SPL at 8 KiB and write
        /// it to hardware Boot0 before the raw user-area write and verification.
        #[arg(long, requires = "raw")]
        emmc_boot0_from_image: bool,

        /// Trusted board-matched vendor package used only to initialize DRAM
        /// and boot the RAM FES service. Its partitions are never written.
        #[arg(
            long,
            value_name = "VENDOR_IMAGE",
            help = "Boot FES from this vendor package before flashing a separate raw image or component package; the bootstrap package is used in RAM only"
        )]
        bootstrap_firmware: Option<String>,

        /// Treat the firmware as a NAND/SPI-NAND component package. This
        /// route writes Boot0, Boot1 and named UBI/partition components and
        /// is rejected on block-like media before erase or MBR download.
        #[arg(
            long,
            requires = "bootstrap_firmware",
            conflicts_with = "raw",
            help = "Flash a bad-block-aware NAND component package; requires a separate RAM-only bootstrap"
        )]
        nand_components: bool,

        /// Board-scoped UART used only to capture storage geometry printed by
        /// the RAM bootstrap before it enters the FES USB service.
        #[arg(long, requires = "bootstrap_firmware")]
        storage_geometry_serial_port: Option<String>,

        /// Baud rate for the board-scoped storage geometry UART.
        #[arg(
            long,
            default_value_t = 115_200,
            requires = "storage_geometry_serial_port"
        )]
        storage_geometry_serial_baud: u32,

        /// USB bus number
        #[arg(short, long, help = "USB bus number")]
        bus: Option<u8>,

        /// USB port number
        #[arg(short = 'P', long, help = "USB port number")]
        port: Option<u8>,

        /// Stable physical USB topology path. Takes precedence over bus/port.
        #[arg(
            long,
            help = "Exact physical USB location returned by `openixcli scan`"
        )]
        device_location: Option<String>,

        /// Enable verification after write
        #[arg(
            short = 'V',
            long,
            default_value_t = true,
            action = clap::ArgAction::Set,
            help = "Enable verification after write"
        )]
        verify: bool,

        /// Flash mode
        /// - partition: Flash only specified partitions
        /// - keep_data: Keep existing data
        /// - partition_erase: Erase partitions before flashing
        /// - full_erase: Erase all data before flashing
        #[arg(
            short,
            long,
            default_value = "full_erase",
            help = "Flash mode: partition, keep_data, partition_erase, full_erase"
        )]
        mode: FlashMode,

        /// Partitions to flash (comma-separated)
        #[arg(short = 'p', long, help = "Partitions to flash (comma-separated)")]
        partitions: Option<String>,

        /// Post-flash action
        /// - reboot: Reboot device after flashing
        /// - poweroff: Power off device after flashing
        /// - none: Leave the device in FES mode
        #[arg(
            short = 'a',
            long,
            default_value = "reboot",
            help = "Post-flash action: reboot, poweroff, shutdown, none"
        )]
        post_action: PostAction,
    },

    /// Execute a bounded, hash-verified mainline FEL memory plan. This command
    /// is intended for the board-scoped LYNX worker process; it does not accept
    /// arbitrary commands or an unconstrained entry address.
    BootMainline {
        /// Path to the validated JSON plan generated by LYNX.
        #[arg(long)]
        plan: String,

        /// Exact physical USB location returned by `openixcli scan`.
        #[arg(long)]
        device_location: String,

        /// Current USB bus resolved by LYNX from the stable physical location.
        #[arg(long)]
        bus: u8,

        /// Current USB port resolved by LYNX from the stable physical location.
        #[arg(long)]
        port: u8,
    },

    /// Inspect firmware contents (image header, embedded files, MBR partitions)
    Inspect {
        /// Path to firmware file
        #[arg(help = "Path to firmware file")]
        firmware: String,
    },

    /// Unpack firmware data to disk (embedded files + partition images)
    Unpack {
        /// Path to firmware file
        #[arg(help = "Path to firmware file")]
        firmware: String,

        /// Output directory (default: ./<firmware>_unpacked)
        #[arg(short = 'o', long = "output-dir", help = "Output directory")]
        output_dir: Option<String>,
    },

    /// Launch interactive TUI mode
    Tui,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_accepts_explicit_spinor_route_with_completion_action() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "raw",
            "loader.img",
            "nekobox-full.img",
            "--mode",
            "spinor-physical",
            "--post-action",
            "reboot",
        ])
        .unwrap();
        assert!(matches!(
            cli.command,
            Some(Commands::Raw {
                mode: RawMode::SpinorPhysical,
                post_action: PostAction::Reboot,
                ..
            })
        ));
    }

    #[test]
    fn physical_readback_requires_exact_scope_and_rejects_destructive_flags() {
        let base = [
            "openixcli",
            "spinor-readback",
            "--device-location",
            "usb-bound",
            "--report",
            "evidence.json",
        ];
        assert!(matches!(
            Cli::try_parse_from(base).unwrap().command,
            Some(Commands::SpinorReadback { .. })
        ));
        assert!(
            Cli::try_parse_from(["openixcli", "spinor-readback", "--report", "evidence.json"])
                .is_err()
        );
        let mut destructive = base.to_vec();
        destructive.extend(["--post-action", "reboot"]);
        assert!(Cli::try_parse_from(destructive).is_err());
    }

    #[test]
    fn raw_accepts_explicit_post_actions() {
        for (action, expected) in [
            ("none", PostAction::None),
            ("reboot", PostAction::Reboot),
            ("poweroff", PostAction::PowerOff),
            ("shutdown", PostAction::Shutdown),
        ] {
            let parsed = Cli::try_parse_from([
                "openixcli",
                "raw",
                "loader.img",
                "disk.img",
                "--post-action",
                action,
            ])
            .expect("explicit raw post action");
            assert!(
                matches!(parsed.command, Some(Commands::Raw { post_action, .. }) if post_action == expected)
            );
        }
    }

    #[test]
    fn raw_defaults_to_no_post_action_and_rejects_invalid_actions() {
        let cli = Cli::try_parse_from(["openixcli", "raw", "loader.img", "disk.img"]).unwrap();
        assert!(matches!(
            cli.command,
            Some(Commands::Raw {
                post_action: PostAction::None,
                ..
            })
        ));
        assert!(Cli::try_parse_from([
            "openixcli",
            "raw",
            "loader.img",
            "disk.img",
            "--post-action",
            "invalid"
        ])
        .is_err());
    }

    #[test]
    fn parses_jsonl_output_for_flash_automation() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "--output",
            "jsonl",
            "flash",
            "firmware.img",
            "--raw",
            "--bootstrap-firmware",
            "vendor.img",
            "--post-action",
            "none",
        ])
        .expect("valid flash automation arguments");

        assert_eq!(cli.output, OutputFormat::Jsonl);
        assert!(matches!(
            cli.command,
            Some(Commands::Flash {
                raw: true,
                bootstrap_firmware: Some(ref bootstrap),
                post_action: PostAction::None,
                ..
            }) if bootstrap == "vendor.img"
        ));
    }

    #[test]
    fn parses_command_raw_layout_with_loader_first() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "raw",
            "avaota-a1-loader.img",
            "avaota-a1-full-mainline.img",
            "--mode",
            "command",
            "--emmc-boot0-from-image",
            "--bus",
            "0",
            "--port",
            "2",
        ])
        .expect("valid dedicated raw command");

        assert!(matches!(
            cli.command,
            Some(Commands::Raw {
                ref loader,
                ref firmware,
                mode: RawMode::Command,
                emmc_boot0_from_image: true,
                bus: Some(0),
                port: Some(2),
                ..
            }) if loader == "avaota-a1-loader.img"
                && firmware == "avaota-a1-full-mainline.img"
        ));
    }

    #[test]
    fn text_output_remains_the_default() {
        let cli =
            Cli::try_parse_from(["openixcli", "scan"]).expect("scan arguments should remain valid");

        assert_eq!(cli.output, OutputFormat::Text);
    }

    #[test]
    fn unpack_output_directory_does_not_collide_with_global_output_format() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "unpack",
            "firmware.img",
            "--output-dir",
            "unpacked",
        ])
        .expect("unpack output directory");

        assert_eq!(cli.output, OutputFormat::Text);
        assert!(matches!(
            cli.command,
            Some(Commands::Unpack {
                output_dir: Some(ref directory),
                ..
            }) if directory == "unpacked"
        ));
    }

    #[test]
    fn parses_nand_component_package_with_ram_only_bootstrap() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "flash",
            "mainline-nand-components.img",
            "--bootstrap-firmware",
            "vendor-loader.img",
            "--nand-components",
            "--mode",
            "partition",
        ])
        .expect("component package with separate bootstrap");
        assert!(matches!(
            cli.command,
            Some(Commands::Flash {
                raw: false,
                nand_components: true,
                bootstrap_firmware: Some(ref bootstrap),
                mode: FlashMode::Partition,
                ..
            }) if bootstrap == "vendor-loader.img"
        ));
    }

    #[test]
    fn parses_stable_physical_usb_location() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "flash",
            "firmware.img",
            "--device-location",
            "pciroot(0)#usbroot(0)#usb(4)",
        ])
        .expect("stable location selector");
        assert!(matches!(
            cli.command,
            Some(Commands::Flash { device_location: Some(location), .. })
                if location == "pciroot(0)#usbroot(0)#usb(4)"
        ));
    }

    #[test]
    fn parses_scoped_mainline_worker_plan() {
        let cli = Cli::try_parse_from([
            "openixcli",
            "--output",
            "jsonl",
            "boot-mainline",
            "--plan",
            "plan.json",
            "--device-location",
            "pciroot(0)#usbroot(0)#usb(10)#usb(2)",
            "--bus",
            "0",
            "--port",
            "10",
        ])
        .expect("scoped mainline worker arguments");
        assert!(matches!(
            cli.command,
            Some(Commands::BootMainline { plan, device_location, bus, port })
                if plan == "plan.json"
                    && device_location.ends_with("#usb(2)")
                    && bus == 0
                    && port == 10
        ));
    }
}
