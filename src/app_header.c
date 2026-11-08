/**
 * \file            app_header.c
 * \brief           The image banner (0x08001400)
 * \note            The image carries the 16-byte ASCII token `SCOOTER_VCU_xGT3` at 0x08001400
 *                      (no terminator); no firmware code reads it. Tools identify a GT3 VCU image
 *                      by it (x3utils looks for `SCOOTER_<VCU|MCU>_<code>` at payload offset 0x400).
 *                      x3-vcu-fw.ld links it at 0x08001400, then the key (0x08001420) and the build
 *                      id (0x08001430).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "config.h"
#include "model.h"

#ifdef VCU_MODEL_UNVERIFIED
#warning "this model's feature list is not verified against the scooter yet (model.h)"
#endif

#define APP_BANNER_LEN 16u /*!< banner length: SCOOTER_ (8) + VCU (3) + _ (1) + model code (4) */

/** \brief Image banner of the model, e.g. `SCOOTER_VCU_xGT3` (0x08001400, no terminator) */
const char k_app_banner[APP_BANNER_LEN] APP_HEADER_SECTION("banner") = VCU_MODEL_BANNER;
