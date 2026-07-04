# ODrive Production Firmware

This repository contains a trimmed single-axis ODrive v3 firmware profile for the current production CAN workflow.

## Repository Layout

- `Firmware/`: firmware source, board support, CAN protocol, and firmware-side tests
- `Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md`: current production CAN protocol notes
- `tools/`: minimal generation helpers that are still referenced by the firmware build
- `odrive-cansimple.dbc`: maintained CAN database snapshot

The original upstream Arduino library, GUI, docs site, odrivetool package, and broad analysis scripts have been removed from this tree.

## Build

The firmware build is driven from `Firmware/` with Tup:

```sh
make all
```

`Firmware/Makefile` now only prepares firmware-local generated files and runs Tup. It no longer regenerates Arduino headers, odrivetool files, GUI assets, or DBC files.
