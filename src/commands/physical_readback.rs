//! Read-only F101 physical-window acquisition, independent of FES Flash LBAs.
//!
//! Register semantics: F101 Zephyr sun252i_f101.dtsi, sun252i-f101-ccu.h,
//! flash_sunxi_spif.c (SPIF_PSA/PEA/PMA and spif_window_on).
//! Default mode never changes hardware. Explicit --initialize-window configures
//! and restores the read window only; no flash write, boot or post-action calls.

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::Path;

use anyhow::{ensure, Context as _, Result};
use libefex::{Context, DeviceMode, ScannedDevice, UsbBackend};
use serde::Serialize;
use serde_json::json;
use sha2::{Digest, Sha256};

const F101_ID: u32 = 0x0019_3700;
const CCU_SPIF_CLOCK: u32 = 0x0200_1980;
const CCU_SPIF_GATE: u32 = 0x0200_1984;
const SPIF: u32 = 0x0c00_0000;
const WINDOW_START: u32 = 0x0e00_0000;
const WINDOW_END: u32 = 0x1000_0000;
const MAX_DUMP: u32 = 16 * 1024 * 1024;
const CHUNK: usize = 64 * 1024;

fn selector(location: &str) -> Result<(u8, u8)> {
    let value = location
        .strip_prefix("libusb:")
        .context("SPINOR_LIBUSB_SELECTOR_REQUIRED")?;
    let (bus, port) = value
        .split_once(':')
        .context("SPINOR_LIBUSB_SELECTOR_REQUIRED")?;
    let bus = bus.parse::<u8>().context("SPINOR_BUS_INVALID")?;
    let port = port.parse::<u8>().context("SPINOR_PORT_INVALID")?;
    ensure!(port != 0, "SPINOR_PORT_INVALID");
    Ok((bus, port))
}

fn unique_device(devices: &[ScannedDevice], bus: u8, port: u8) -> Result<()> {
    // libefex's bus/leaf-port selector is ambiguous across nested hubs. Its
    // free-form path opener can also fall back to the first device. Refuse both
    // cases here rather than changing shared NAND/eMMC selection behavior.
    let count = devices
        .iter()
        .filter(|d| d.vid == 0x1f3a && d.pid == 0xefe8 && d.bus == bus && d.port == port)
        .count();
    ensure!(count == 1, "SPINOR_DEVICE_MISSING_OR_AMBIGUOUS");
    Ok(())
}

// The transport capability intentionally exposes only reads.
trait MemoryRead {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<()>;
    fn word(&mut self, address: u32) -> Result<u32> {
        let mut bytes = [0; 4];
        self.read(address, &mut bytes)?;
        Ok(u32::from_le_bytes(bytes))
    }
}

struct DeviceMemory {
    ctx: Context,
    fes: bool,
}
impl MemoryRead for DeviceMemory {
    fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<()> {
        if self.fes {
            self.ctx
                .fes_up(bytes, address, libefex::FesDataType::Dram)?;
        } else {
            self.ctx.fel_read(address, bytes)?;
        }
        Ok(())
    }
}

// Immutable code regions of the analyzed F101 loader, loaded at 0x40050000.
// The work-mode header and mutable BSS are deliberately excluded. No MMIO or
// runtime function/table modification is allowed before these checks pass.
fn verify_f101_fes_profile(memory: &mut impl MemoryRead) -> Result<()> {
    for (address, length, expected) in [
        (
            0x4008_d3d0,
            0x652,
            "746970bfe87bfe12a458efbff016931e45e03f067bb04086ffbc815d6fde6ba1",
        ),
        (
            0x4008_fffc,
            0x804,
            "885a523872451b767c2202c18b5f608ea2cd0b1bb0806aabc5412008369f3a47",
        ),
    ] {
        let mut bytes = vec![0; length];
        memory.read(address, &mut bytes)?;
        ensure!(
            format!("{:x}", Sha256::digest(&bytes)) == expected,
            "SPINOR_FES_PROFILE_MISMATCH: immutable loader code at {address:#x}"
        );
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
struct Snapshot {
    clock: u32,
    gate: u32,
    version: u32,
    control: u32,
    phases: u32,
    flash_address: u32,
    command: u32,
    counts: u32,
    window_start: u32,
    window_end: u32,
    physical_start: u32,
}

fn snapshot(memory: &mut impl MemoryRead) -> Result<Snapshot> {
    let clock = memory.word(CCU_SPIF_CLOCK)?;
    let gate = memory.word(CCU_SPIF_GATE)?;
    // Do not access a clock-gated / reset peripheral or turn its clock on.
    ensure!(
        clock & (1 << 31) != 0 && gate & 0x10001 == 0x10001,
        "SPINOR_WINDOW_CLOCK_INACTIVE: no peripheral read or reconfiguration performed"
    );
    Ok(Snapshot {
        clock,
        gate,
        version: memory.word(SPIF)?,
        control: memory.word(SPIF + 4)?,
        phases: memory.word(SPIF + 0x20)?,
        flash_address: memory.word(SPIF + 0x24)?,
        command: memory.word(SPIF + 0x28)?,
        counts: memory.word(SPIF + 0x2c)?,
        window_start: memory.word(SPIF + 0x34)?,
        window_end: memory.word(SPIF + 0x38)?,
        physical_start: memory.word(SPIF + 0x3c)?,
    })
}

fn physical_zero_address(state: &Snapshot, length: u32) -> Result<u32> {
    ensure!(
        matches!(state.version, 0x10001..=0x10003),
        "SPINOR_CONTROLLER_UNKNOWN"
    );
    ensure!(
        state.control & 0xf == 0xa,
        "SPINOR_WINDOW_INACTIVE: requires address-map + prefetch, without DMA/normal mode"
    );
    ensure!(
        state.phases & 0x1100_0100 == 0x1100_0100 && state.phases & 0x1000 == 0,
        "SPINOR_WINDOW_NOT_READ: requires command/address/RX without TX"
    );
    ensure!(
        matches!(
            state.command >> 24,
            0x03 | 0x0b | 0x3b | 0x6b | 0xbb | 0xeb | 0xbd | 0xed | 0x0c | 0x6c
        ),
        "SPINOR_WINDOW_OPCODE_UNSUPPORTED"
    );
    ensure!(
        state.counts & ((1 << 28) | 0xffff) == 0,
        "SPINOR_WINDOW_NOT_PREFETCH"
    );
    ensure!(
        length > 0 && length <= MAX_DUMP,
        "SPINOR_READ_RANGE_INVALID"
    );
    ensure!(
        state.physical_start == 0 && state.flash_address == 0,
        "SPINOR_WINDOW_NOT_PHYSICAL_ZERO: refusing an offset window"
    );
    ensure!(
        state.window_start >= WINDOW_START && state.window_end <= WINDOW_END,
        "SPINOR_WINDOW_OUTSIDE_F101_APERTURE"
    );
    let end = state
        .window_start
        .checked_add(length)
        .context("SPINOR_ADDRESS_OVERFLOW")?;
    ensure!(end <= state.window_end, "SPINOR_WINDOW_TOO_SHORT");
    Ok(state.window_start)
}

fn acquire(
    memory: &mut impl MemoryRead,
    state: &Snapshot,
    length: u32,
    output: &mut impl Write,
) -> Result<String> {
    let start = physical_zero_address(state, length)?;
    let mut digest = Sha256::new();
    let mut buffer = vec![0; CHUNK];
    let mut offset = 0;
    while offset < length {
        ensure!(
            &snapshot(memory)? == state,
            "SPINOR_WINDOW_CHANGED_DURING_READ"
        );
        let count = (length - offset).min(CHUNK as u32) as usize;
        memory.read(start + offset, &mut buffer[..count])?;
        // Validate the mapping again before accepting each data block.
        ensure!(
            &snapshot(memory)? == state,
            "SPINOR_WINDOW_CHANGED_DURING_READ"
        );
        output.write_all(&buffer[..count])?;
        digest.update(&buffer[..count]);
        offset += count as u32;
    }
    Ok(format!("{:x}", digest.finalize()))
}

pub async fn execute(
    location: &str,
    report_path: &Path,
    dump: Option<&Path>,
    length: u32,
    initialize_window: bool,
    ram_loader: Option<&Path>,
) -> Result<()> {
    let (bus, port) = selector(location)?;
    ensure!(
        length > 0 && length <= MAX_DUMP,
        "SPINOR_READ_RANGE_INVALID"
    );
    if let Some(path) = dump {
        ensure!(
            path != report_path && !path.exists(),
            "SPINOR_DUMP_EXISTS_OR_EQUALS_REPORT"
        );
    }
    let mut report = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(report_path)
        .context("SPINOR_REPORT_MUST_BE_NEW")?;
    let mut evidence = json!({
        "method":"f101_rom_fel_spif_existing_physical_window", "usbBackend":"libusb",
        "deviceLocation":location, "physicalStart":0, "requestedBytes":length,
        "flashWritesPerformed":false, "controllerWritesPerformed":false,
        "physicalReadback":"not_performed", "bootVerification":"not_verified"
    });
    let result: Result<()> =
        async {
            Context::set_usb_backend_static(UsbBackend::Libusb)?;
            unique_device(&Context::scan_usb_devices()?, bus, port)?;
            let mut ctx = Context::new();
            ctx.scan_usb_device_at(bus, port)?;
            ctx.usb_init()?;
            ctx.efex_init()?;
            if let Some(path) = ram_loader {
                ensure!(
                    ctx.device_id() == F101_ID && ctx.get_device_mode() == DeviceMode::Fel,
                    "SPINOR_RAM_BOOTSTRAP_REQUIRES_F101_FEL"
                );
                let bytes = fs::read(path)?;
                ensure!(
                    format!("{:x}", Sha256::digest(&bytes))
                        == "51bf5c40095fc5f3770cac673e18943972876cad6a84d106d6dc7f042d639186",
                    "SPINOR_RAM_PACKAGE_PROFILE_MISMATCH"
                );
                let packer = crate::firmware::LoadedFirmware::load(path)?.into_packer();
                drop(ctx);
                let request = crate::flash::FlashRequest::new(
                    crate::flash::DeviceSelector::new(Some(bus), Some(port)),
                    true,
                    crate::flash::FlashMode::Partition,
                    None,
                    crate::flash::PostAction::None,
                );
                let mut flasher =
                    crate::flash::Flasher::new(packer, request, crate::utils::Logger::new());
                ctx = flasher.open_fes_service(true).await?.0;
                evidence["ramLoaderExecuted"] = json!(true);
            }
            let chip = ctx.device_id();
            evidence["chipId"] = json!(chip);
            let fes = ctx.get_device_mode() == DeviceMode::Srv && chip == 0x0016_1000;
            ensure!(fes || (chip == F101_ID && ctx.get_device_mode() == DeviceMode::Fel),
            "SPINOR_DEVICE_UNSUPPORTED: requires F101 ROM FEL or fingerprint-checked vendor FES");
            let mut memory = DeviceMemory { ctx, fes };
            if fes {
                verify_f101_fes_profile(&mut memory)?;
                evidence["runtimePointers"] = json!({
                    "active":memory.word(0x400d_0138)?, "boot":memory.word(0x400d_0134)?,
                    "norTable":memory.word(0x400d_5edc)?, "mapping":memory.word(0x400d_5e60)?,
                    "mtd":memory.word(0x400e_e358)?, "gd":memory.word(0x400d_5bc8)?,
                    "dramMiB":memory.word(0x4005_04f8)?, "dramResult":memory.word(0x0002_8340)?
                });
                let contract = crate::flash::f101_spinor::NorContract::inspect(&memory.ctx)?;
                evidence["runtimeContract"] = serde_json::to_value(&contract)?;
                evidence["loaderProfile"] = json!("f101-58ba9839-code-verified");
                evidence["method"] = json!("f101_fes_memory_spif_existing_physical_window");
            }
            evidence["clock"] = json!(memory.word(CCU_SPIF_CLOCK)?);
            evidence["gate"] = json!(memory.word(CCU_SPIF_GATE)?);
            let window = if initialize_window {
                Some(crate::flash::f101_spinor::PhysicalWindow::prepare(
                    &memory.ctx,
                )?)
            } else {
                None
            };
            let read_result = (|| -> Result<()> {
                if let Some(window) = &window {
                    evidence["controllerWritesPerformed"] = json!(true);
                    evidence["method"] = json!(if fes {
                        "f101_fes_memory_configured_physical_window"
                    } else {
                        "f101_fel_configured_physical_window"
                    });
                    window.enable(&memory.ctx, length)?;
                }
                let state = snapshot(&mut memory)?;
                evidence["controllerSnapshot"] = serde_json::to_value(&state)?;
                physical_zero_address(&state, length)?;
                if let Some(path) = dump {
                    let partial = path.with_file_name(format!(
                        "{}.partial",
                        path.file_name()
                            .context("SPINOR_DUMP_FILENAME")?
                            .to_string_lossy()
                    ));
                    let mut file = OpenOptions::new()
                        .write(true)
                        .create_new(true)
                        .open(&partial)?;
                    evidence["partialFile"] = json!(partial);
                    let sha = acquire(&mut memory, &state, length, &mut file)?;
                    file.sync_all()?;
                    drop(file);
                    // Publish without replacing an existing evidence file, even in a race.
                    fs::hard_link(&partial, path)?;
                    fs::remove_file(&partial)?;
                    evidence["physicalReadback"] = json!("acquired_not_yet_compared");
                    evidence["sha256"] = json!(sha);
                    evidence["dump"] = json!(path);
                    evidence["acquiredBytes"] = json!(length);
                }
                Ok(())
            })();
            if let Some(window) = &window {
                let restore_result = window.restore(&memory.ctx);
                evidence["controllerRestored"] = json!(restore_result.is_ok());
                if let Err(error) = restore_result {
                    evidence["restoreError"] = json!(format!("{error:#}"));
                    if read_result.is_ok() {
                        return Err(error);
                    }
                }
            }
            read_result?;
            if fes && initialize_window {
                if let Some(path) = dump {
                    let physical = fs::read(path)?;
                    crate::flash::f101_spinor::NorContract::inspect(&memory.ctx)?
                        .verify_read_translation(&memory.ctx, &physical)?;
                    evidence["addressTranslationVerified"] = json!(true);
                }
            }
            Ok(())
        }
        .await;
    if let Err(error) = &result {
        evidence["error"] = json!(format!("{error:#}"));
    }
    serde_json::to_writer_pretty(&mut report, &evidence)?;
    report.write_all(b"\n")?;
    report.sync_all()?;
    println!("{}", serde_json::to_string(&evidence)?);
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unknown_fes_loader_is_rejected_before_peripheral_access() {
        struct Unknown(Vec<(u32, usize)>);
        impl MemoryRead for Unknown {
            fn read(&mut self, address: u32, bytes: &mut [u8]) -> Result<()> {
                self.0.push((address, bytes.len()));
                bytes.fill(0);
                Ok(())
            }
        }
        let mut memory = Unknown(vec![]);
        let error = verify_f101_fes_profile(&mut memory).unwrap_err();
        assert!(error.to_string().contains("SPINOR_FES_PROFILE_MISMATCH"));
        assert_eq!(memory.0, vec![(0x4008_d3d0, 0x652)]);
    }
    #[test]
    fn selector_rejects_free_form_paths_and_duplicate_leaf_ports() {
        for path in [
            "",
            "location:pciroot(0)#usb(2)",
            "libusb:0:0",
            "libusb:256:1",
            "libusb:0:2:3",
        ] {
            assert!(selector(path).is_err());
        }
        assert_eq!(selector("libusb:0:2").unwrap(), (0, 2));
        let device = || ScannedDevice {
            bus: 0,
            port: 2,
            vid: 0x1f3a,
            pid: 0xefe8,
            location_path: None,
            device_path: None,
        };
        assert!(unique_device(&[], 0, 2).is_err());
        assert!(unique_device(&[device()], 0, 2).is_ok());
        assert!(unique_device(&[device()], 0, 3).is_err());
        assert!(unique_device(&[device(), device()], 0, 2).is_err());
    }
    fn valid() -> Snapshot {
        Snapshot {
            clock: 1 << 31,
            gate: 0x10001,
            version: 0x10002,
            control: 0xa,
            phases: 0x11000100,
            flash_address: 0,
            command: 0x0b000000,
            counts: 0,
            window_start: WINDOW_START,
            window_end: WINDOW_START + MAX_DUMP,
            physical_start: 0,
        }
    }
    struct Mock {
        state: Snapshot,
        addresses: Vec<(u32, usize)>,
        alter_after_data: bool,
    }
    impl MemoryRead for Mock {
        fn read(&mut self, addr: u32, bytes: &mut [u8]) -> Result<()> {
            self.addresses.push((addr, bytes.len()));
            if addr >= WINDOW_START {
                for (i, byte) in bytes.iter_mut().enumerate() {
                    *byte = (addr as usize + i) as u8;
                }
                if self.alter_after_data {
                    self.state.physical_start = 0x80000;
                }
                return Ok(());
            }
            let word = match addr {
                CCU_SPIF_CLOCK => self.state.clock,
                CCU_SPIF_GATE => self.state.gate,
                SPIF => self.state.version,
                a if a == SPIF + 4 => self.state.control,
                a if a == SPIF + 0x20 => self.state.phases,
                a if a == SPIF + 0x24 => self.state.flash_address,
                a if a == SPIF + 0x28 => self.state.command,
                a if a == SPIF + 0x2c => self.state.counts,
                a if a == SPIF + 0x34 => self.state.window_start,
                a if a == SPIF + 0x38 => self.state.window_end,
                a if a == SPIF + 0x3c => self.state.physical_start,
                _ => anyhow::bail!("unexpected address"),
            };
            bytes.copy_from_slice(&word.to_le_bytes());
            Ok(())
        }
    }
    #[test]
    fn inactive_clock_never_touches_peripheral_or_flash() {
        let mut m = Mock {
            state: valid(),
            addresses: vec![],
            alter_after_data: false,
        };
        m.state.clock = 0;
        assert!(snapshot(&mut m).is_err());
        assert_eq!(m.addresses, [(CCU_SPIF_CLOCK, 4), (CCU_SPIF_GATE, 4)]);
    }
    #[test]
    fn rejects_offset_window_and_invalid_or_unsafe_controller_states() {
        let mut bad = valid();
        bad.physical_start = 0xbc000;
        assert!(physical_zero_address(&bad, 512).is_err());
        bad = valid();
        bad.flash_address = 0x80000;
        assert!(physical_zero_address(&bad, 512).is_err());
        for gc in [0, 2, 8, 0xb, 0xe] {
            bad = valid();
            bad.control = gc;
            assert!(physical_zero_address(&bad, 512).is_err());
        }
        bad = valid();
        bad.command = 0x02000000;
        assert!(physical_zero_address(&bad, 512).is_err());
        bad = valid();
        bad.phases |= 0x1000;
        assert!(physical_zero_address(&bad, 512).is_err());
        assert!(physical_zero_address(&valid(), 0).is_err());
        assert!(physical_zero_address(&valid(), MAX_DUMP + 1).is_err());
    }
    #[test]
    fn exact_physical_zero_chunk_addresses_and_sha() {
        let state = valid();
        let mut m = Mock {
            state: state.clone(),
            addresses: vec![],
            alter_after_data: false,
        };
        let mut data = vec![];
        let length = CHUNK as u32 + 512;
        let hash = acquire(&mut m, &state, length, &mut data).unwrap();
        assert_eq!(
            m.addresses
                .iter()
                .copied()
                .filter(|(a, _)| *a >= WINDOW_START)
                .collect::<Vec<_>>(),
            [(WINDOW_START, CHUNK), (WINDOW_START + CHUNK as u32, 512)]
        );
        assert_eq!(data, (0..length).map(|n| n as u8).collect::<Vec<_>>());
        assert_eq!(hash, format!("{:x}", Sha256::digest(&data)));
    }
    #[test]
    fn f101_measured_v3_uses_v2_address_encoding() {
        let mut state = valid();
        state.version = 0x10003;
        state.counts = 2 << 24;
        assert_eq!(physical_zero_address(&state, 4096).unwrap(), WINDOW_START);
        state.version = 0x10004;
        assert!(physical_zero_address(&state, 4096).is_err());
    }
    #[test]
    fn mapping_change_discards_untrusted_chunk() {
        let state = valid();
        let mut m = Mock {
            state: state.clone(),
            addresses: vec![],
            alter_after_data: true,
        };
        let mut data = vec![];
        assert!(acquire(&mut m, &state, 512, &mut data).is_err());
        assert!(data.is_empty());
    }
}
