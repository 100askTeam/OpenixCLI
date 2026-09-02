# OpenixCLI change gate

OpenixCLI is a shared flashing tool for multiple Allwinner SoCs and products.
Treat a previously validated OpenixCLI binary and its behavior as immutable.

A change to OpenixCLI is permitted only when at least one of these conditions
is established with concrete evidence:

1. the requested work is an explicit new OpenixCLI feature; or
2. a reproducible defect is proven to be in OpenixCLI's business logic.

Do not modify OpenixCLI to compensate for board firmware, BootROM/FEL state,
FES loader contents, target U-Boot/Linux drivers, image packaging, cabling,
USB passthrough, VMware reconnect behavior, permissions, or other environment
problems. Diagnose and fix those components in their own scope.

Before an allowed OpenixCLI change is accepted, preserve existing chip paths,
add focused regression coverage for the new behavior or defect, run the
existing test suite, and explicitly check that unrelated SoCs and storage
backends have not changed. Never fold an unverified emergency workaround into
the shared OpenixCLI binary.
