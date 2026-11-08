# On-target unit tests

Unity unit tests for the firmware C code. They run **on the target CPU inside the
[x3emu emulator](https://github.com/hp197/x3-vcu-emu)**, not on the host: the drivers reach
the AT32 registers through absolute-address macros, so the code only works where those
addresses exist. The emulator models the peripherals the firmware uses, so a test can drive
the real product code and the real BSP and look at what reaches the pins and buses.

## Running

```sh
pio test -e emu-test                     # build and run every suite (test/test_*/)
pio test -e emu-test -f test_ninebot     # one suite
pio test -e emu-test -f test_ninebot -v  # with the full Unity output and the x3emu summary
```

`pio test` without `-e emu-test` runs nothing: the product environment ignores the tests.

The first run fetches the emulator from `github.com/hp197/x3-vcu-emu` into `.pio/` and builds a
venv with its dependencies (`tools/run_emu_test.py`, cached between runs), so a fresh checkout
works with no manual setup. Python 3.13+ and `git` must be on `PATH`; set `X3EMU_REF` to pin a
tag or commit instead of `main`.

### Using a local emulator

To use a checkout you already have (for example while developing the emulator alongside the
firmware), point the test runner at it in the untracked `platformio.local.ini` next to
`platformio.ini`; it is then used instead of the download:

```ini
[env:emu-test]
custom_x3emu_dir = ../x3-vcu-emu
custom_x3emu_python = /path/to/venv/bin/python   # a venv with the emulator's dependencies
```

To rerun one image by hand, with a trace:

```sh
<python> <x3-vcu-emu> test .pio/build/emu-test/firmware.elf --trace out/test.jsonl
```

## How it works

| Piece | Where | What it does |
|---|---|---|
| Image | `[env:emu-test]` in `platformio.ini` | each `test/test_<module>/` is its own image: `src/` without `main.c` + the `test/*.c` support files + the suite, so every suite starts from a fresh boot |
| Linker script | `test/emu-test.ld` | vectors at 0x08001000 as on the device; 120 KB FLASH up to the identity pages (0x0801F000); product `.data`/`.bss` grouped so they can be reset |
| Output | `test/unity_config.h`, `tst_support.c` | Unity prints through ITM stimulus port 0 (CMSIS `ITM_SendChar`); x3emu streams it to stdout |
| End of run | `tst_exit()` | semihosting `SYS_EXIT_EXTENDED` with the Unity failure count |
| Verdict | `x3emu test --pio` | PlatformIO parses the Unity lines; a timeout (10 s emulated) or a CPU fault makes the suite ERRORED, with PC, LR and symbols in the `-v` output |
| Fakes | `tst_bsp_fakes.c`, `-Wl,--wrap=...` | fff fakes at the BSP boundary that record and, by default, forward to the real BSP function |

The product sources are not changed for testing: there are no `#ifdef UNIT_TEST`, weak
symbols or test hooks in `src/`. The only seams are the BSP functions named in the
`-Wl,--wrap=` flags.

## Writing a suite

Create `test/test_<module>/test_<module>.c`:

```c
/**
 * \file            test_<module>.c
 * \brief           <module> behaviour
 */
#include <unity.h>
#include "<module>.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

/** \brief Fresh boot state for every test */
void setUp(void) {
    tst_reset_product_state(); /* every product global/static back to its boot value */
    tst_fakes_reset();         /* fakes record + forward, captures cleared */
    /* re-run the inits the code under test relies on, e.g. vcu_usart_init(); */
}

/** \brief Nothing to undo */
void tearDown(void) {}

/**
 * \brief           <the behaviour under test>
 * \note            <the rule it checks: protocol field, timing, edge case>.
 */
static void test_<behaviour>(void) {
    /* arrange: blackboard values, injected input */
    /* act: call the product function */
    /* assert: registers, captured frames, blackboard */
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_<behaviour>);
    tst_exit(UNITY_END());
    return 0;
}
```

Rules:

- **A test states required behaviour** (wire format, timing, values the other nodes rely
  on), not incidental implementation details. Say in the test's `\note` which rule it checks.
- Follow the C style of the project (Doxygen at the definition, `(void)`, at most two
  `return`s, `prv_` statics) and run clang-format with the project's `.clang-format`.
- Keep `main` to `tst_init()`, `UNITY_BEGIN()`, the `RUN_TEST`s and `tst_exit(UNITY_END())`.
- Use `test/test_harness/` (helper self-tests), `test/test_ninebot/` and `test/test_identity/`
  (flash writes checked in the fake log and in the emulated flash) as worked examples.

## Helpers

`tst_support.h`:

| Helper | Use |
|---|---|
| `tst_init()` | first call in `main`: app vector table (VTOR), `clock_config()` (144 MHz), ITM on |
| `tst_exit(failures)` | last call in `main`: semihosting exit with the failure count |
| `tst_reset_product_state()` | every product global and static (`g_vs`, `g_in1`, `g_in2`, `g_scr`, `g_uartctx`, module statics) back to its boot value: `.data` re-copied from flash, `.bss` zeroed. Peripheral registers, the NVIC and run-time init values (e.g. `vcu_usart_init`'s free-slot count) are not restored: re-run those inits |
| `tst_sched_run_ticks(n)` | advance `g_sched_tick` by n and `sched_dispatch()` after each tick: registered tasks run when they would on the device |
| `tst_wait_set(reg, mask, loops)` | busy-wait for a register bit (`TST_WAIT_LOOPS_DEFAULT` is about 10 ms) |
| `tst_delay_us(us)` | busy-wait about `us` of core time (for bytes or frames in flight) |
| `tst_stack_paint(pattern)` | fill the free stack, so a read of an uninitialised local shows up in the result |
| `tst_emu_usart_rx(data, len)` | bytes arrive on USART2 RX from the bus side (x3emu semihosting extension); wait, then call the receiver (`vcu_usart_poll`) |
| `tst_emu_can_rx(id, ext, data, dlc)` | a frame arrives from the CAN bus through the emulated CAN1 (FIFO0, RX0 interrupt); needs `vcu_can_lowlevel_init()` |
| `tst_emu_adc_set(channel, raw)` | ADC1 input `channel` (`ADC_CHANNEL_*`) converts to the 12-bit `raw` from now on (x3emu op 0x102); the running scan + DMA deliver it within one scan (~0.1 ms). The level outlives `tst_reset_product_state()`. For the control-loop inputs use `tst_control.h` |
| `tst_emu_imu_set(roll, pitch, roll_rate, pitch_rate)` | the emulated LSM6DS3 presents the board at that attitude (mdeg) turning at those rates (mdeg/s), as a scenario's `set_attitude` (op 0x103). The estimator has no seed snap: the Mahony filter turns over in about 100 passes and the tilt is a 100-sample mean. It also skips every update while `(ax * ay) * az` is 0, so give a non-zero pitch (an exact 0 on any accel axis freezes it). Put it back level (`0, 0, 0, 0`) at the end of the test |
| `tst_emu_imu_sample_set(accel_ug, gyro_mdps, temp_mdegc)` | the LSM6DS3's physical sample in chip axes (op 0x104, a scenario's `set_imu`) |

`tst_bsp_fakes.h` (all fakes record and forward to the real BSP after `tst_fakes_reset()`):

| Helper | Use |
|---|---|
| `tst_can_tx_count()` / `tst_can_tx_get(i)` | frames handed to `can_message_transmit` (id, ext, rtr, dlc, data). Pump the TX ring with `vcu_can_poll()` first |
| `tst_can_tx_forward(0)` | capture only, report mailbox 0 (no emulated controller needed) |
| `tst_can_rx_inject(id, ext, data, dlc)` + `tst_can_rx_deliver()` | a received frame without the controller: runs `CAN1_RX0_IRQHandler` (the fakes make it read the injected frame) and `vcu_can_rx_dispatch()` |
| `tst_usart_tx_count()` / `tst_usart_tx_get(i)` | Ninebot frames sent on USART2: captured when the reply sequencer arms DMA1 channel 7 (`len`, the first 96 bytes) |
| `tst_usart_drain()` | send what is queued in the reply ring (`vcu_usart_tx_sequencer()` + wait for each DMA burst); needs `vcu_usart_init()` |
| `tst_flash_forward(0)` | flash erase/program fakes report `FLASH_OPERATE_DONE` without touching the flash |
| `tst_flash_op_count()` / `tst_flash_op_get(i)` | flash operation log: erases (`addr`, 1 KB) and program runs (`addr`, `len`, `data`, `status`); consecutive halfword/word programs coalesce, so one identity save is one 512-byte run. 16 operations / 1 KB of data kept |
| `__wrap_<fn>_fake` | the fff record: `call_count`, `arg0_val`, `argN_history[]`, `return_val`, `custom_fake` (history length 8, set in `platformio.ini`) |

`tst_control.h` (control-loop ADC inputs, used by the control, control_fsm, faultmon, charging,
vehicle and passcode suites): `tst_control_adc_freeze()` runs the product bring-up
(`vcu_control_init`) from a stopped converter with every scanned input at 0;
`tst_control_adc_set(slot, raw)` / `tst_control_adc_set_codes(thr, full, half)` apply the input
level of a `VCU_ADC_SLOT_*` through `tst_emu_adc_set` and return once the scan has written it to
the product's buffer; `tst_control_adc_release()` puts all five inputs back to 0 (call it in
`setUp`, `tst_fm_release_all()` does). Don't write the buffer (`tst_control_adc_buf()`): the
DMA overwrites it.

`tst_ninebot.h`: `tst_nb_build` / `tst_nb_parse` / `tst_nb_checksum` (written from the wire
format, independent of `usart.c`) and `tst_nb_request(transport, src, dst, cmd, arg, body,
len)`, which hands a request to `vcu_ninebot_process` as the receivers do.

To fake another BSP function: add `-Wl,--wrap=<fn>` to `[env:emu-test] build_flags`, a
`DECLARE_FAKE_*` line in `tst_bsp_fakes.h`, the `DEFINE_FAKE_*` line, the `__real_<fn>`
prototype and a forwarding `custom_fake` in `tst_bsp_fakes.c`, and reset it in
`tst_fakes_reset()`. Only calls between object files are wrapped: a call inside the BSP's
own `.c` file to a function of that file is not.

## Golden values

`test_sensor/sensor_gold.h` holds reference outputs of the IMU attitude path for fixed chip
samples, as raw IEEE-754 bits; `test_sensor` compares the tilt within 0.001 deg and the rest bit for bit. Don't edit them by hand:
a change that moves one of them is a change of the attitude filter's behaviour.

## Host check of the run-time helpers

`src/runtime.c` (`__udivmoddi4`) replaces libgcc's 64-bit divide to fit the 60 KB slot.
`src/softfloat.c` is an opt-in compact replacement for libgcc's `__aeabi_*` float and double
helpers, built only with `-D VCU_SOFTFLOAT_COMPACT` (see its file header); by default the
firmware uses libgcc. `test/host/runtime_check.c` compiles both with that define and compares
them with the host's IEEE-754 hardware:
- all 2^32 inputs of every unary helper, and of `x * 3`, `1 / x`, `x + 1` and `x * 2^-148`;
- tens of millions of random pairs for the binary helpers.

The host must round to nearest-even with subnormals, and its float-to-integer conversions must
saturate with NaN -> 0; arm64 (Apple silicon) does all of this. It is not a PlatformIO suite. It
runs in about 3 minutes:

```sh
cc -std=c11 -O2 -ffp-contract=off -Wall -D VCU_SOFTFLOAT_COMPACT -o /tmp/runtime_check \
    test/host/runtime_check.c src/softfloat.c src/runtime.c -lm && /tmp/runtime_check
```

## What a test can observe

- **Peripheral registers** the emulator models (x3-vcu-emu README, "Peripherals"): GPIO
  output data, CAN mailboxes/FIFO, USART, DMA, timers, ADC, SPI2 + IMU, flash contents,
  ERTC, WDT, NVIC/SCB/SysTick. Read them through the BSP structs (`GPIOB->odt`, ...).
- **Captured BSP calls**: CAN TX frames, USART2 TX frames, flash erase/program arguments.
- **Product RAM**: the blackboards in `vcu_state.h` and any non-static global.
- Not observable from inside the image: the emulator's trace (pass `--trace` and read the
  JSONL afterwards), timing finer than about 0.1 ms, CAN error handling (ideal bus).
- CAN1 `msts.cuss` (bit 8) reads 1 while a frame from a real mailbox is on the bus, and the
  TX pump (`vcu_can_poll`) sends nothing then. A suite that forwards frames to
  the controller and pumps once per tick sees the next frame one poll later; use
  `tst_can_tx_forward(0)` when only the captured frames matter.

## Known limitations

- The image covers slot 1 (0x08010000-0x0801EFFF). Code under test that erases or programs
  slot 1 for real overwrites the test image: use `tst_flash_forward(0)` for OTA staging
  tests. Identity pages and the boot descriptor (0x0801F000-0x0801FFFF) are provided by
  x3emu and may be erased/programmed (in emulator memory only).
- A CPU fault (unmapped access, undefined instruction) ends the run instead of entering
  `HardFault_Handler`; a store to flash without unlock (including a NULL store to 0x0) is
  refused and printed as an `x3emu: warning:` line, not faulted.
- `tst_reset_product_state()` resets RAM only; peripherals keep their state between tests
  of one suite, and an interrupt left enabled by one test still fires in the next.
- Interrupts run as on the device: a test that enables a peripheral interrupt gets the
  product ISR, with the emulator's interrupt latency of one quantum (10 us).
- USART2 TX captures keep the first 96 bytes of a frame (`TST_USART_FRAME_MAX`), 8 frames
  and 16 CAN frames per test (`TST_*_MAX`, overridable with `-D`).
- The test image runs `clock_config()`; the 8 MHz HICK start-up path is not what the tests
  see.
