//! F101 SPIF physical read window. No flash erase/program opcode is issued.
//!
//! Register contract: F101 Zephyr flash_sunxi_spif.c (Apache-2.0), spif_build,
//! spif_set_clock and spif_window_on; YuzukiNeko board.dts PC0..PC5 function 4.
//! This explicit operation changes controller state and restores it afterwards;
//! it is not the passive readback diagnostic's default behavior.

use anyhow::{ensure, Context as _, Result};
use libefex::{Context, DeviceMode};
use sha2::{Digest, Sha256};

const BASE: u32 = 0x0c00_0000;
const CLOCK: u32 = 0x0200_1980;
const GATE: u32 = 0x0200_1984;
const PC_CFG0: u32 = 0x0200_0060;
const WINDOW: u32 = 0x0e00_0000;

pub(crate) fn read_bytes(ctx: &Context, address: u32, bytes: &mut [u8]) -> Result<()> {
    if ctx.get_device_mode() == DeviceMode::Fel {
        ctx.fel_read(address, bytes)?;
    } else {
        ctx.fes_up(bytes, address, libefex::FesDataType::Dram)?;
    }
    Ok(())
}
fn read(ctx: &Context, address: u32) -> Result<u32> {
    let mut bytes = [0; 4];
    read_bytes(ctx, address, &mut bytes)?;
    Ok(u32::from_le_bytes(bytes))
}
fn write(ctx: &Context, address: u32, value: u32) -> Result<()> {
    ensure!(
        address == CLOCK
            || address == PC_CFG0
            || matches!(
                address.wrapping_sub(BASE),
                4 | 0x0c | 0x1c | 0x20 | 0x24 | 0x28 | 0x2c | 0x34 | 0x38 | 0x3c
            ),
        "SPINOR_MMIO_WRITE_NOT_ALLOWED"
    );
    let bytes = value.to_le_bytes();
    if ctx.get_device_mode() == DeviceMode::Fel {
        ctx.fel_write(address, &bytes)?;
    } else {
        // FES_DOWN DRAM+FINISH (0x17f00) saves the byte destination at 0x402b6.
        // The pinned loader's 0x3f834 handler explicitly performs a fenced 32-bit
        // MMIO store for length=4 and address<0x40000000. FES_TRANS does not
        // invoke that finalizer and must not be used for these register writes.
        ctx.fes_down(&bytes, address, libefex::FesDataType::Dram)?;
    }
    Ok(())
}

fn read_window_words(version: u32, length: u32) -> Result<Vec<(u32, u32)>> {
    ensure!(
        matches!(version, 0x10001..=0x10003),
        "SPINOR_CONTROLLER_UNKNOWN"
    );
    ensure!(length != 0 && length <= 0x0100_0000, "SPINOR_WINDOW_RANGE");
    Ok(vec![
        (BASE + 0x34, WINDOW),
        (BASE + 0x38, WINDOW + length),
        (BASE + 0x3c, 0),
        (BASE + 0x20, 0x1100_0100),
        (BASE + 0x24, 0),
        (BASE + 0x28, 0x0300_0000),
        (BASE + 0x2c, if version >= 0x10002 { 2 << 24 } else { 0 }),
        (BASE + 4, 0x10a),
    ])
}

/// Saved register state is owned by the caller so restoration errors are visible.
/// Callers must call restore() after both successful and failed reads.
pub(crate) struct PhysicalWindow {
    saved: Vec<(u32, u32)>,
}
impl PhysicalWindow {
    pub(crate) fn prepare(ctx: &Context) -> Result<Self> {
        if ctx.get_device_mode() == DeviceMode::Fel {
            ensure!(
                ctx.device_id() == 0x0019_3700,
                "SPINOR_WINDOW_INIT_REQUIRES_F101"
            );
        } else {
            verify_loader(ctx)?;
        }
        let clock = read(ctx, CLOCK)?;
        let gate = read(ctx, GATE)?;
        // Cold initialization is a separate capability, never silently inferred.
        ensure!(
            clock & (1 << 31) != 0 && gate & 0x10001 == 0x10001,
            "SPINOR_WINDOW_INIT_REQUIRES_ACTIVE_CLOCK"
        );
        let version = read(ctx, BASE)?;
        read_window_words(version, 1)?;
        ensure!(
            read(ctx, BASE + 4)? & 4 == 0 && read(ctx, BASE + 0x40)? & 1 == 0,
            "SPINOR_CONTROLLER_BUSY"
        );
        let mut saved = vec![(CLOCK, clock), (PC_CFG0, read(ctx, PC_CFG0)?)];
        for offset in [4, 0x0c, 0x1c, 0x20, 0x24, 0x28, 0x2c, 0x34, 0x38, 0x3c] {
            saved.push((BASE + offset, read(ctx, BASE + offset)?));
        }
        Ok(Self { saved })
    }

    pub(crate) fn enable(&self, ctx: &Context, length: u32) -> Result<()> {
        self.enable_at(ctx, length, WINDOW)
    }

    pub(crate) fn enable_at(&self, ctx: &Context, length: u32, cpu_window: u32) -> Result<()> {
        ensure!(
            cpu_window >= WINDOW
                && cpu_window
                    .checked_add(length)
                    .is_some_and(|end| end <= 0x1000_0000),
            "SPINOR_CPU_WINDOW_RANGE"
        );
        let mut words = read_window_words(read(ctx, BASE)?, length)?;
        words[0].1 = cpu_window;
        words[1].1 = cpu_window + length;
        // Quiesce normal/DMA/prefetch operation, then use conservative 24 MHz,
        // single-wire 0x03 reads. No status-register write, WREN, erase or program.
        write(ctx, BASE + 4, 0x100)?;
        write(ctx, CLOCK, 1 << 31)?;
        write(ctx, PC_CFG0, (self.saved[1].1 & 0xff00_0000) | 0x0044_4444)?;
        let timing = self
            .saved
            .iter()
            .find(|(a, _)| *a == BASE + 0x0c)
            .unwrap()
            .1;
        write(
            ctx,
            BASE + 0x0c,
            timing & !((1 << 26) | (1 << 20) | (7 << 16) | 0x7f),
        )?;
        write(ctx, BASE + 0x1c, 0x0005_0606)?;
        for (address, value) in words {
            write(ctx, address, value)?;
            ensure!(
                read(ctx, address)? == value,
                "SPINOR_WINDOW_REGISTER_MISMATCH at {address:#x}"
            );
        }
        Ok(())
    }

    pub(crate) fn restore(&self, ctx: &Context) -> Result<()> {
        // Attempt every restore even after one transfer error. Restore GC last.
        let mut errors = vec![];
        if let Err(e) = write(ctx, BASE + 4, 0x100) {
            errors.push(e.to_string());
        }
        for &(address, value) in self.saved.iter().filter(|(a, _)| *a != BASE + 4) {
            if let Err(e) = write(ctx, address, value) {
                errors.push(e.to_string());
            }
        }
        let control = self
            .saved
            .iter()
            .find(|(a, _)| *a == BASE + 4)
            .context("missing saved GC")?
            .1;
        if let Err(e) = write(ctx, BASE + 4, control) {
            errors.push(e.to_string());
        }
        for &(address, value) in &self.saved {
            match read(ctx, address) {
                Ok(actual) if actual == value => {}
                Ok(actual) => errors.push(format!(
                    "{address:#x}: expected {value:#x}, got {actual:#x}"
                )),
                Err(error) => errors.push(error.to_string()),
            }
        }
        ensure!(
            errors.is_empty(),
            "SPINOR_WINDOW_RESTORE_FAILED: {}",
            errors.join("; ")
        );
        Ok(())
    }
}

pub(crate) fn verify_loader(ctx: &Context) -> Result<()> {
    verify_loader_at(ctx, 0x4005_0000)
}
fn verify_loader_at(ctx: &Context, base: u32) -> Result<()> {
    ensure!(
        ctx.get_device_mode() == DeviceMode::Srv && ctx.device_id() == 0x0016_1000,
        "SPINOR_FES_PROFILE_MODE"
    );
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
        (
            0x4005_0640,
            0x1f8,
            "a7f5bb9a66c4cb9af2968a359f73a94fc5a335293d859cecec2fe336f7053ffe",
        ),
        (
            0x4005_981a,
            0x2c,
            "8c57dcf9d8182bf767a956062c8045a77144c654e3f5427b7a679b281a8774bd",
        ),
        (
            0x4005_9e32,
            0x4c,
            "d4698307e3939ff9d1d730727543a7ea213ecd469f7f7f9126d13c2840d801ec",
        ),
        (
            0x4007_3512,
            0x18,
            "a77227a79cc4c59c298f6eb85c2c8d02e1e6303411e1f9f0cd78ad30e98bd811",
        ),
        (
            0x4007_3bdc,
            0x34,
            "ae164cc6b36a33cb1258c4b5efaf139a64d0d016488b7e314f05f8c4b035ec56",
        ),
        (
            0x4005_1a2e,
            0x4,
            "9c28941eb64aa06520e1ff0e0058871b3bfbcea0b07d72ad58477e8d0d7350cb",
        ),
        (
            0x4008_f834,
            0x2c,
            "c99d513e60b19ac365a2c376615313d924f41a7443e127f1346980bfde68801e",
        ),
    ] {
        let mut bytes = vec![0; length];
        read_bytes(ctx, base + address - 0x4005_0000, &mut bytes)?;
        ensure!(
            format!("{:x}", Sha256::digest(&bytes)) == expected,
            "SPINOR_FES_PROFILE_MISMATCH at {address:#x}"
        );
    }
    Ok(())
}

fn locate_running_loader(ctx: &Context) -> Result<u32> {
    verify_loader(ctx)?;
    // Fingerprinted startup at 0x68c/0x981a puts initial gd at
    // align_down(0x29ec0 - 308,16). 0x9e32 records and copies relocation metadata.
    let initial_gd = 0x29d80;
    let base = read(ctx, initial_gd + 56)?;
    let delta = read(ctx, initial_gd + 76)?;
    let relocated_gd = read(ctx, initial_gd + 80)?;
    ensure!(
        (0x4000_0000..=0x4ff0_0000).contains(&base)
            && base.wrapping_sub(0x4005_0000) == delta
            && (0x4000_0000..0x5000_0000 - 308).contains(&relocated_gd),
        "SPINOR_RELOCATION_INVALID:base={base:#x},delta={delta:#x},gd={relocated_gd:#x}"
    );
    ensure!(
        read(ctx, relocated_gd + 56)? == base && read(ctx, relocated_gd + 76)? == delta,
        "SPINOR_RELOCATION_COPY_MISMATCH"
    );
    verify_loader_at(ctx, base)?;
    Ok(base)
}

/// Capture through the SPIF CPU window, never through FES Flash LBAs. A different
/// unused CPU alias is used after programming to avoid reusing pre-write cache lines.
pub(crate) fn read_physical(ctx: &Context, length: u32, cpu_window: u32) -> Result<Vec<u8>> {
    let window = PhysicalWindow::prepare(ctx)?;
    let result = (|| -> Result<Vec<u8>> {
        window.enable_at(ctx, length, cpu_window)?;
        let mut bytes = vec![0; length as usize];
        for (index, chunk) in bytes.chunks_mut(64 * 1024).enumerate() {
            for (address, value) in [
                (BASE + 0x3c, 0),
                (BASE + 0x24, 0),
                (BASE + 0x34, cpu_window),
                (BASE + 0x28, 0x0300_0000),
                (BASE + 4, 0x10a),
            ] {
                ensure!(read(ctx, address)? == value, "SPINOR_PHYSICAL_MAP_CHANGED");
            }
            read_bytes(ctx, cpu_window + index as u32 * 65536, chunk)?;
        }
        ensure!(
            read(ctx, BASE + 0x3c)? == 0 && read(ctx, BASE + 0x24)? == 0,
            "SPINOR_PHYSICAL_MAP_CHANGED"
        );
        Ok(bytes)
    })();
    let restored = window.restore(ctx);
    match (result, restored) {
        (Ok(bytes), Ok(())) => Ok(bytes),
        (Err(error), Ok(())) | (Ok(_), Err(error)) => Err(error),
        (Err(error), Err(restore)) => anyhow::bail!("{error}; restoration: {restore}"),
    }
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub(crate) struct NorContract {
    pub loader_base: u32,
    pub logical_base_sectors: u32,
    pub capacity_bytes: u32,
    pub erase_alignment: u32,
    pub erase_block: u32,
    pub write_alignment: u32,
    mtd: u32,
}
impl NorContract {
    pub(crate) fn inspect(ctx: &Context) -> Result<Self> {
        let base = locate_running_loader(ctx)?;
        ensure!(ctx.fes_query_storage()? == 3, "SPINOR_STORAGE_MISMATCH");
        ensure!(
            read(ctx, base + 0x80138)? == base + 0x85964,
            "SPINOR_ACTIVE_DRIVER_MISMATCH"
        );
        ensure!(
            read(ctx, base + 0x85978)? == base + 0x3d958
                && read(ctx, base + 0x8597c)? == base + 0x3d930,
            "SPINOR_CALLBACK_MISMATCH"
        );
        ensure!(
            read(ctx, base + 0x85e60)? == base + 0x80188,
            "SPINOR_MAP_TABLE_MISMATCH"
        );
        let logical_base_sectors = read(ctx, base + 0x80190)?;
        // getter(0,1) selects normal/secure entries. Require both to agree rather
        // than guessing the current security state or using a static default.
        ensure!(
            logical_base_sectors == read(ctx, base + 0x80194)?,
            "SPINOR_SECURE_MAP_AMBIGUOUS"
        );
        let mtd = read(ctx, base + 0x9e358)?;
        ensure!(
            (0x4000_0000..0x4100_0000 - 0x170).contains(&mtd) && mtd % 4 == 0,
            "SPINOR_MTD_POINTER_INVALID"
        );
        let contract = Self {
            loader_base: base,
            logical_base_sectors,
            capacity_bytes: read(ctx, mtd + 0x164)?,
            erase_alignment: read(ctx, mtd + 0x10)?,
            erase_block: read(ctx, mtd + 0x168)?,
            write_alignment: read(ctx, mtd + 0x16c)?,
            mtd,
        };
        ensure!(
            contract.capacity_bytes.is_power_of_two()
                && (1024 * 1024..=16 * 1024 * 1024).contains(&contract.capacity_bytes),
            "SPINOR_CAPACITY_UNSUPPORTED"
        );
        for size in [
            contract.erase_alignment,
            contract.erase_block,
            contract.write_alignment,
        ] {
            ensure!(
                size.is_power_of_two() && (256..=65536).contains(&size),
                "SPINOR_ERASE_GEOMETRY_UNSUPPORTED"
            );
        }
        ensure!(
            contract.erase_block >= contract.erase_alignment
                && logical_base_sectors < contract.capacity_bytes / 512,
            "SPINOR_MAPPING_INVALID"
        );
        Ok(contract)
    }

    /// Inverse of the fingerprinted RV32 logical wrapper's unsigned ADD.
    /// Base is read from the running loader, not copied from eMMC or hard-coded.
    /// This may only be used after independent physical-vs-logical sample checks.
    pub(crate) fn fes_sector(&self, physical_byte: u32, length: u32) -> Result<u32> {
        ensure!(
            length != 0
                && length.is_multiple_of(512)
                && physical_byte.is_multiple_of(512)
                && physical_byte
                    .checked_add(length)
                    .is_some_and(|end| end <= self.capacity_bytes),
            "SPINOR_PHYSICAL_RANGE_INVALID"
        );
        Ok((physical_byte / 512).wrapping_sub(self.logical_base_sectors))
    }

    pub(crate) fn assert_unchanged(&self, ctx: &Context) -> Result<()> {
        ensure!(
            read(ctx, self.loader_base + 0x80190)? == self.logical_base_sectors
                && read(ctx, self.loader_base + 0x80194)? == self.logical_base_sectors
                && read(ctx, self.loader_base + 0x9e358)? == self.mtd
                && read(ctx, self.loader_base + 0x80138)? == self.loader_base + 0x85964,
            "SPINOR_RUNTIME_MAPPING_CHANGED"
        );
        Ok(())
    }

    pub(crate) fn verify_read_translation(&self, ctx: &Context, physical: &[u8]) -> Result<()> {
        ensure!(
            physical.len() >= 0x80200,
            "SPINOR_PHYSICAL_SAMPLE_TOO_SHORT"
        );
        for offset in [0, 0x80000, physical.len() as u32 - 512] {
            self.assert_unchanged(ctx)?;
            let mut logical = [0; 512];
            ctx.fes_up(
                &mut logical,
                self.fes_sector(offset, 512)?,
                libefex::FesDataType::Flash,
            )?;
            ensure!(
                logical == physical[offset as usize..offset as usize + 512],
                "SPINOR_PHYSICAL_TRANSLATION_MISMATCH at {offset:#x}"
            );
        }
        Ok(())
    }
}

/// Created only after an independent physical backup and a comparison against
/// the running loader's translated reads. No caller receives a write capability
/// just because it recognized a chip ID or selected a UI route.
pub(crate) struct PreparedWrite {
    contract: NorContract,
    image: Vec<u8>,
    expected: Vec<u8>,
    write_len: usize,
    evidence: std::path::PathBuf,
}

fn expected_contents(before: &[u8], image: &[u8], alignment: u32) -> Result<(Vec<u8>, usize)> {
    ensure!(
        !image.is_empty() && image.len().is_multiple_of(512),
        "SPINOR_IMAGE_ALIGNMENT"
    );
    ensure!(
        alignment.is_power_of_two() && (512..=65536).contains(&alignment),
        "SPINOR_WRITE_ALIGNMENT"
    );
    let write_len = image.len().div_ceil(alignment as usize) * alignment as usize;
    ensure!(write_len <= before.len(), "SPINOR_IMAGE_EXCEEDS_CAPACITY");
    let mut expected = before.to_vec();
    expected[..image.len()].copy_from_slice(image);
    Ok((expected, write_len))
}

fn physical_write_offsets(
    expected: &[u8],
    after_setup: &[u8],
    image_write_len: usize,
    alignment: usize,
) -> Result<Vec<usize>> {
    ensure!(
        alignment.is_power_of_two() && (512..=65536).contains(&alignment),
        "SPINOR_WRITE_ALIGNMENT"
    );
    ensure!(
        !expected.is_empty()
            && expected.len() == after_setup.len()
            && expected.len().is_multiple_of(alignment)
            && image_write_len != 0
            && image_write_len <= expected.len()
            && image_write_len.is_multiple_of(alignment),
        "SPINOR_SETUP_PRESERVATION_GEOMETRY"
    );
    // The loader may materialize partition metadata outside a short image.
    // Discover those erase blocks from physical observation; never guess a
    // fixed GPT offset or extend writes across unchanged gaps.
    Ok((0..expected.len())
        .step_by(alignment)
        .filter(|&offset| {
            offset < image_write_len
                || expected[offset..offset + alignment] != after_setup[offset..offset + alignment]
        })
        .collect())
}

fn save_new(path: &std::path::Path, bytes: &[u8]) -> Result<()> {
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(path)?;
    file.write_all(bytes)?;
    file.sync_all()?;
    Ok(())
}

impl PreparedWrite {
    pub(crate) fn prepare(ctx: &Context, image_path: &std::path::Path) -> Result<Self> {
        let contract = NorContract::inspect(ctx)?;
        let image = std::fs::read(image_path)?;
        ensure!(
            image.len() >= 0x80200 && image.get(4..12) == Some(b"eGON.BT0"),
            "SPINOR_FULL_IMAGE_REQUIRES_BOOT_AT_PHYSICAL_ZERO"
        );
        ensure!(
            image.len() <= contract.capacity_bytes as usize,
            "SPINOR_IMAGE_EXCEEDS_CAPACITY"
        );
        let probed = ctx.fes_probe_flash_size()?;
        ensure!(
            probed == 0 || u64::from(probed) * 512 == u64::from(contract.capacity_bytes),
            "SPINOR_CAPACITY_DISAGREEMENT"
        );
        let before = read_physical(ctx, contract.capacity_bytes, WINDOW)?;
        contract.verify_read_translation(ctx, &before)?;
        let alignment = contract.erase_block.max(contract.write_alignment);
        let (expected, write_len) = expected_contents(&before, &image, alignment)?;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos();
        let parent = std::env::var_os("LYNX_FLASH_EVIDENCE_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join("openixcli-evidence"));
        std::fs::create_dir_all(&parent)?;
        let evidence = parent.join(format!("f101-{}-{stamp}", std::process::id()));
        std::fs::create_dir(&evidence)?;
        save_new(&evidence.join("physical-before.bin"), &before)?;
        let manifest = serde_json::json!({"contract":contract, "imagePath":image_path,
            "imageBytes":image.len(), "writeBytes":write_len,
            "physicalStart":0, "imageSha256":format!("{:x}",Sha256::digest(&image)),
            "beforeSha256":format!("{:x}",Sha256::digest(&before)),
            "expectedFullFlashSha256":format!("{:x}",Sha256::digest(&expected)),
            "addressTranslationVerified":true, "flashWritesPerformed":false,
            "bootVerification":"not_verified"});
        save_new(
            &evidence.join("preflight.json"),
            &serde_json::to_vec_pretty(&manifest)?,
        )?;
        Ok(Self {
            contract,
            image,
            expected,
            write_len,
            evidence,
        })
    }

    pub(crate) fn evidence_dir(&self) -> &std::path::Path {
        &self.evidence
    }
    pub(crate) fn image_len(&self) -> usize {
        self.image.len()
    }
    pub(crate) fn description(&self) -> String {
        format!(
            "loader={:#x},runtimeBaseSectors={},capacity={},eraseAlignment={},writeBytes={}",
            self.contract.loader_base,
            self.contract.logical_base_sectors,
            self.contract.capacity_bytes,
            self.contract.erase_alignment,
            self.write_len
        )
    }
    pub(crate) fn check(&self, ctx: &Context) -> Result<()> {
        self.contract.assert_unchanged(ctx)
    }

    pub(crate) fn write(
        &self,
        ctx: &Context,
        mut before_chunk: impl FnMut() -> Result<()>,
        mut progress: impl FnMut(usize),
    ) -> Result<()> {
        let alignment = self.contract.erase_block.max(self.contract.write_alignment) as usize;
        before_chunk()?;
        self.check(ctx)?;
        let after_setup = read_physical(ctx, self.contract.capacity_bytes, WINDOW)?;
        self.check(ctx)?;
        let offsets =
            physical_write_offsets(&self.expected, &after_setup, self.write_len, alignment)?;
        save_new(
            &self.evidence.join("physical-after-setup.bin"),
            &after_setup,
        )?;
        let plan = serde_json::json!({
            "method":"post-setup physical comparison; preserve original outside-image bytes",
            "postSetupSha256":format!("{:x}",Sha256::digest(&after_setup)),
            "imageWriteBytes":self.write_len,
            "alignment":alignment,
            "physicalWriteOffsets":offsets,
            "outsideRestoreOffsets":offsets.iter().copied().filter(|&offset| offset >= self.write_len).collect::<Vec<_>>(),
            "outsideImageChangedBytes":after_setup[self.image.len()..].iter().zip(&self.expected[self.image.len()..]).filter(|(a,b)| a != b).count(),
            "automaticRetry":false
        });
        save_new(
            &self.evidence.join("physical-write-plan.json"),
            &serde_json::to_vec_pretty(&plan)?,
        )?;
        save_new(
            &self.evidence.join("write-started.json"),
            b"{\"writeStarted\":true,\"automaticRetry\":false}\n",
        )?;
        for offset in offsets {
            before_chunk()?;
            self.check(ctx)?;
            let chunk = &self.expected[offset..offset + alignment];
            let offset = offset as u32;
            let sector = self.contract.fes_sector(offset, chunk.len() as u32)?;
            ctx.fes_down(chunk, sector, libefex::FesDataType::Flash)
                .with_context(|| format!("physical offset {offset:#x}; no retry, no reboot"))?;
            progress((offset as usize + chunk.len()).min(self.image.len()));
        }
        save_new(
            &self.evidence.join("write-complete.json"),
            b"{\"writeCompleted\":true,\"physicalVerification\":\"pending\"}\n",
        )?;
        Ok(())
    }

    pub(crate) fn verify(&self, ctx: &Context) -> Result<String> {
        self.check(ctx)?;
        // This CPU alias is distinct from the pre-write alias. Physical address
        // remains zero and verification never uses the logical Flash wrapper.
        let physical = read_physical(ctx, self.contract.capacity_bytes, 0x0f00_0000)?;
        save_new(&self.evidence.join("physical-after.bin"), &physical)?;
        let first_difference = physical
            .iter()
            .zip(&self.expected)
            .position(|(a, b)| a != b);
        let hash = format!("{:x}", Sha256::digest(&physical[..self.image.len()]));
        let report = serde_json::json!({"physicalVerification":if first_difference.is_none(){"success"}else{"failed"},
            "method":"SPIF physical window via FES DRAM upload", "physicalStart":0,
            "verifiedBytes":physical.len(), "imageBytes":self.image.len(), "imageReadbackSha256":hash,
            "expectedImageSha256":format!("{:x}",Sha256::digest(&self.image)),
            "fullFlashSha256":format!("{:x}",Sha256::digest(&physical)),
            "expectedFullFlashSha256":format!("{:x}",Sha256::digest(&self.expected)),
            "firstDifferentPhysicalByte":first_difference, "bootVerification":"not_verified"});
        save_new(
            &self.evidence.join("verification.json"),
            &serde_json::to_vec_pretty(&report)?,
        )?;
        ensure!(
            first_difference.is_none(),
            "SPINOR_PHYSICAL_CONTENT_MISMATCH: first byte {first_difference:?}"
        );
        Ok(format!(
            "imageSha256={hash},imageBytes={},fullFlashBytes={},outsideImage=unchanged",
            self.image.len(),
            physical.len()
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn small_image_restores_loader_metadata_outside_image_without_writing_gap() {
        let alignment = 65536;
        let before = vec![0xa5; 16 * 1024 * 1024];
        let image = vec![0x5a; 614400];
        let (expected, write_len) = expected_contents(&before, &image, alignment as u32).unwrap();
        let mut after_setup = before.clone();
        // Reproduce the observed loader metadata footprint; the planner must
        // discover changed blocks, not encode this address as a device offset.
        after_setup[0xbc000..0xbc600].fill(0);
        let offsets =
            physical_write_offsets(&expected, &after_setup, write_len, alignment).unwrap();
        assert_eq!(offsets.len(), 11);
        assert_eq!(offsets.last(), Some(&0xb0000));
        assert!(!offsets.contains(&0xa0000));
        for offset in offsets {
            after_setup[offset..offset + alignment]
                .copy_from_slice(&expected[offset..offset + alignment]);
        }
        assert_eq!(after_setup, expected);
        assert_eq!(&after_setup[image.len()..], &before[image.len()..]);
    }

    #[test]
    fn metadata_inside_image_or_partial_last_block_needs_no_extra_write() {
        let before = vec![0xa5; 8 * 65536];
        let image = vec![0x5a; 2 * 65536 + 512];
        let (expected, write_len) = expected_contents(&before, &image, 65536).unwrap();
        let mut setup = before.clone();
        setup[1024..2048].fill(0);
        setup[image.len()..image.len() + 512].fill(0);
        assert_eq!(
            physical_write_offsets(&expected, &setup, write_len, 65536).unwrap(),
            vec![0, 65536, 2 * 65536]
        );
    }

    #[test]
    fn setup_preservation_discovers_sparse_blocks_and_rejects_invalid_geometry() {
        let before = vec![0xa5; 8 * 65536];
        let image = vec![0x5a; 65536];
        let (expected, write_len) = expected_contents(&before, &image, 65536).unwrap();
        let mut setup = before.clone();
        setup[5 * 65536 + 12] = 0;
        setup[7 * 65536 + 99] = 0;
        assert_eq!(
            physical_write_offsets(&expected, &setup, write_len, 65536).unwrap(),
            vec![0, 5 * 65536, 7 * 65536]
        );
        assert!(physical_write_offsets(&expected, &setup[..65536], write_len, 65536).is_err());
        assert!(physical_write_offsets(&expected, &setup, write_len, 0).is_err());
        assert!(physical_write_offsets(&expected, &setup, 512, 65536).is_err());
        assert!(physical_write_offsets(&expected, &setup, expected.len() + 65536, 65536).is_err());
    }

    #[test]
    fn aligned_program_preserves_tail_and_all_bytes_outside_image() {
        let before = vec![0xa5; 3 * 65536];
        let image = vec![0x5a; 65536 + 40960];
        let (expected, write_len) = expected_contents(&before, &image, 65536).unwrap();
        assert_eq!(write_len, 2 * 65536);
        assert_eq!(&expected[..image.len()], &image);
        assert_eq!(&expected[image.len()..], &before[image.len()..]);
        assert!(expected_contents(&before[..65536], &image, 65536).is_err());
        assert!(expected_contents(&before, &[0; 511], 65536).is_err());
        assert!(expected_contents(&before, &image, 0).is_err());
    }
    #[test]
    fn mapping_is_runtime_derived_bounded_and_reversible() {
        for base in [0, 8, 0x5e0, 0x760] {
            let c = NorContract {
                loader_base: 0,
                logical_base_sectors: base,
                capacity_bytes: 16 * 1024 * 1024,
                erase_alignment: 4096,
                erase_block: 65536,
                write_alignment: 4096,
                mtd: 0,
            };
            for physical in [0, 512, 0x80000, 0xbc000, 16 * 1024 * 1024 - 512] {
                assert_eq!(
                    c.fes_sector(physical, 512).unwrap().wrapping_add(base) * 512,
                    physical
                );
            }
            assert!(c.fes_sector(1, 512).is_err());
            assert!(c.fes_sector(0, 0).is_err());
            assert!(c.fes_sector(16 * 1024 * 1024, 512).is_err());
            assert!(c.fes_sector(u32::MAX - 511, 512).is_err());
        }
    }
    #[test]
    fn physical_zero_read_program_has_no_logical_base_or_write_opcode() {
        let words = read_window_words(0x10003, 4_890_624).unwrap();
        assert!(words.contains(&(BASE + 0x3c, 0)));
        assert!(words.contains(&(BASE + 0x24, 0)));
        assert!(words.contains(&(BASE + 0x28, 0x0300_0000)));
        assert!(words.contains(&(BASE + 0x2c, 2 << 24)));
        assert_eq!(words.last(), Some(&(BASE + 4, 0x10a)));
    }
    #[test]
    fn invalid_geometry_or_unknown_controller_has_no_register_program() {
        for length in [0, 0x0100_0001, u32::MAX] {
            assert!(read_window_words(0x10003, length).is_err());
        }
        assert!(read_window_words(0x10004, 4096).is_err());
    }
}
