# T527 mainline eMMC through FES

This route installs a fully mainline persistent system while using a
board-matched Tina/IMAGEWTY loader only in RAM:

```text
BootROM FEL
  → RAM-only vendor loader
  → FES
  → eMMC type/capacity preflight
  → mainline SPL to eMMC Boot0
  → raw mainline disk image to eMMC user area
  → user-area verify
  → explicit cold power cycle
```

## Input contract

The raw image must:

- be non-empty and sector aligned;
- fit the FES-probed target capacity;
- contain an eGON `SPL` image at byte offset `0x2000`;
- carry a valid eGON length and checksum;
- keep the complete U-Boot FIT and partition table in the eMMC user area.

`--emmc-boot0-from-image` is rejected unless FES reports eMMC. The SPL is
written before the raw image because the tested FES Boot0 primitive aliases a
copy into the user area. The subsequent disk write repairs the alias and the
final whole-image verification covers the user area.

The FES Boot0 status response confirms that the component was accepted. The
vendor protocol's Boot0 upload aliases the user area, so it cannot prove byte
identity in the hardware boot partition. A power-cycle cold boot is therefore
an independent release gate.

## Addressing

Tina FES exposes SD/eMMC external partition traffic through a logical 20 MiB
prefix. OpenixCLI uses the protocol's wrapping start sector `0xffff6000`, so
adding the prefix resolves to physical sector zero. Four Allwinner MBR copies
are sent, matching the service contract.

## Safety

- NAND and SPI-NAND are rejected by raw mode.
- NAND components are rejected on eMMC.
- A stable physical USB location should be supplied.
- No automatic retry occurs after a failed FEL/FES transition.
- `--post-action none` is recommended for qualification.
- A FES verification success is not equivalent to a successful cold boot.

The hardware-qualified Avaota A1 source, image layout and cold-boot transcript
are maintained in `dshanpi/AvaotaA1-T527-MainLineLinux`.

