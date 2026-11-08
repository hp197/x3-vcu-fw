# x3-vcu-fw

Open-source application firmware for the Segway-Ninebot **x3** (GT3 / G3 / ZT3 …) VCU,
for the Artery **AT32F415CBT7** (ARM Cortex-M4). It is the sibling of
[x3-vcu-boot](https://github.com/hp197/x3-vcu-boot) (the 4 KB bootloader); this is the
application that lives at `0x08001000` (60 KB) and is launched by that
bootloader.

## Scope

The VCU talks to the motor controller and battery over **CAN**, to the dashboard/BLE
module over a **USART** link, reads buttons and sensors through GPIO and **external
interrupts**, and runs a timer-driven scheduler. The modules:

- boot / clock / `main` and the scheduler
- CAN bus (motor + battery communication) and the bus protocol
- USART link and the Ninebot register protocol
- vehicle state machine, control loop and ride modes
- IMU attitude, tilt and ride assist
- GPIO / external interrupts (buttons, lights, buzzer)
- configuration / identity record handling, odometer, charging
- fault monitor and passcode lock
- OTA (receives an image, stages it into slot 1 for the bootloader)

## Build

PlatformIO pulls its own copy of the Artery platform and BSP, so no local checkout is
needed.

```bash
pio run            # build the default model (gt3) -> .pio/build/gt3/firmware.bin
pio run -e gt3     # one model: gt3, zt3, f3 (include/model.h has the models and their features)
pio run -t clean   # clean
```

The build stamps the first 6 hex digits of the git commit into the image as its build id
(BLE registers `0xE4..0xE6`). `X3VCU_BUILD_ID=none pio run` leaves it unset.

Flashing is disabled in `platformio.ini`; flash the resulting image to `0x08001000`
with the x3 bootloader's OTA path or x3utils over SWD.

## Tests

Unity unit tests run on the target CPU inside the x3emu emulator (`../x3-vcu-emu`), one
image per suite in `test/test_<module>/`:

```bash
pio test -e emu-test                    # every suite
pio test -e emu-test -f test_ninebot    # one suite
```

[test/README.md](test/README.md) explains the setup, the helpers and how to write a suite.

## License

The work in this repository is licensed under **CC BY-NC-SA 4.0** (see
[LICENSE](LICENSE)). The Artery AT32F415 BSP and CMSIS used at build time keep their
own licenses, and so does the test-only fake function framework in
[lib/fff](lib/fff/README.md) (MIT).
