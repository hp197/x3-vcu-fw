/**
 * \file            unity_config.h
 * \brief           Unity configuration of the on-target test images (pio test -e emu-test)
 * \note            PlatformIO uses this file instead of generating its own because it sits in
 *                      the test root. Unity prints through the Cortex-M ITM stimulus port 0
 *                      (tst_support.c), which x3emu streams to stdout; the run ends with
 *                      tst_exit(UNITY_END()), a semihosting exit carrying the failure count.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#include <stdint.h>

void tst_output_start(void);
void tst_output_char(int32_t chr);
void tst_output_flush(void);

#define UNITY_OUTPUT_START()    tst_output_start()
#define UNITY_OUTPUT_CHAR(a)    tst_output_char((int32_t)(a))
#define UNITY_OUTPUT_FLUSH()    tst_output_flush()
#define UNITY_OUTPUT_COMPLETE() tst_output_flush()

#endif /* UNITY_CONFIG_H */
