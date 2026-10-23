# Building the firmware

The firmware is built with [PlatformIO](https://platformio.org/) from the command
line. PlatformIO downloads the ARM toolchain, the Beyond Robotix board package
([br_platformio_hwdef](https://github.com/BeyondRobotix/br_platformio_hwdef)) and
the DroneCAN library
([libArduinoDroneCAN](https://github.com/BeyondRobotix/libArduinoDroneCAN)) on the
first build, at the versions pinned in `platformio.ini`. The first build
therefore needs internet access.

## Setup

Install PlatformIO in its own Python virtual environment (Ubuntu 24.04 and
other recent distributions block system-wide `pip install`):

```bash
python3 -m venv ~/venv-pio
~/venv-pio/bin/pip install platformio
```

Put `pio` on your `PATH` (use `~/.bashrc` instead of `~/.zshrc` if you use bash):

```bash
echo 'export PATH="$HOME/venv-pio/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
pio --version
```

Without this step, type `~/venv-pio/bin/pio` instead of `pio` in the commands
below.

Clone the repository:

```bash
git clone git@github.com:flowcopter/AUAV-DroneCAN.git
cd AUAV-DroneCAN
```

## Build

```bash
pio run -e Micro-Node-App
```

`-e Micro-Node-App` selects the build environment from `platformio.ini`. It is
also the default, so `pio run` does the same. The first build takes about a
minute; later builds only recompile changed files. A good build ends with
`[SUCCESS]` and prints the flash and RAM usage.

Output, in `.pio/build/Micro-Node-App/`:

| File | Use |
|---|---|
| `firmware.bin` | Upload this to the module over DroneCAN |
| `firmware.elf` | The same firmware with debug symbols, for an ST-Link debugger |
| `firmware.map` | Linker map: what ended up in the binary (used for `THIRD_PARTY_NOTICES.md`) |

## Other commands

| Command | What it does |
|---|---|
| `pio test -e native` | Runs the pressure filter unit tests on the PC. No hardware needed |
| `pio run -t clean` | Deletes the build output so the next build starts from scratch |
| `pio run -v` | Verbose build: prints every compiler command, useful when a build fails |
| `pio run -t upload` | Flashes over an ST-Link connected to the SWD pads. Not needed normally; see below |

## Uploading to the module

Upload `firmware.bin` over DroneCAN; the Beyond Robotix bootloader on the
module writes it to flash:

- **Mission Planner:** DroneCAN/UAVCAN screen, select the node, update
  firmware, choose `.pio/build/Micro-Node-App/firmware.bin`.
- **DroneCAN GUI Tool:** select the node and start a firmware update with the
  same file.

Use the `.bin`, not an `.apj`: the bootloader writes the file as-is. If an
update fails part-way, the bootloader reappears on the bus as node ID 100 so
the update can be retried.

After an update that changes the parameter list, the node's parameters return
to their defaults. Set `NODEID` back to the module's previous value so
ArduPilot still recognises it as the same barometer.

## Typical workflow

```bash
# edit src/main.cpp or lib/PressureFilter ...
pio test -e native     # filter tests still pass
pio run                # build
# upload .pio/build/Micro-Node-App/firmware.bin over DroneCAN
git add -A && git commit -m "..."
```
