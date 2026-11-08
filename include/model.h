/**
 * \file            model.h
 * \brief           The scooter model the image is built for, and its feature switches
 * \note            Each PlatformIO environment builds one model, passed as
 *                      `-D VCU_MODEL=VCU_MODEL_<name>` (`[env:gt3]`, `[env:zt3]`, `[env:f3]`; the
 *                      unit tests build GT3). An unknown model stops the build here.
 *
 *                      Model-specific code tests a feature, not the model, with VCU_HAS_FEATURE -
 *                      in `#if` or in a C `if` (the dead branch is dropped). Every model block
 *                      defines every feature as 1 or 0, so a new model is one new block and
 *                      toggling a feature is one line; a feature a model omits is a build error
 *                      (undeclared identifier in C, `-Werror=undef` in `#if`).
 *
 *                      A model block also sets the identity: the serial is a 3-char family prefix,
 *                      a 1-char region code and "0000000001" for a factory part, and the region
 *                      code picks the drive row (config.c CONFIG_REGION_CODES).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef MODEL_H
#define MODEL_H

/* Model ids (values of VCU_MODEL). */
#define VCU_MODEL_GT3         1 /*!< Segway-Ninebot GT3 */
#define VCU_MODEL_ZT3         2 /*!< Segway-Ninebot ZT3 */
#define VCU_MODEL_F3          3 /*!< Segway-Ninebot F3  */

/** \brief 1 if the model this image is built for has feature `name` (VCU_FEATURE_<name>), else 0 */
#define VCU_HAS_FEATURE(name) (VCU_FEATURE_##name)

#ifndef VCU_MODEL
#error "VCU_MODEL is not set: build a model environment, e.g. pio run -e gt3"
#endif

#if VCU_MODEL == VCU_MODEL_GT3
#define VCU_MODEL_NAME              "GT3"              /*!< model name                                          */
#define VCU_MODEL_BANNER            "SCOOTER_VCU_xGT3" /*!< image banner: SCOOTER_VCU_ + the 4-char model code */
/* Identity: serial = prefix (3) + region code (1) + "0000000001". */
#define VCU_MODEL_SERIAL_PREFIX     "03S"  /*!< serial family prefix                         */
#define VCU_MODEL_DEFAULT_REGION    "P"    /*!< region of the factory-default serial         */
#define VCU_MODEL_BOARD_PREFIX      "Z03Q" /*!< board id prefix of the default model string   */
/* Features (each one 1 or 0, the same list in every model block). */
#define VCU_FEATURE_IMU             1 /*!< LSM6DS3 IMU: attitude, tilt, gyro rates              */
#define VCU_FEATURE_CLIMB_ASSIST    1 /*!< climb boost: more current / energy on a slope         */
#define VCU_FEATURE_OVERSPEED_REGEN 1 /*!< regen braking above the ride-mode ceiling             */
#define VCU_FEATURE_WOBBLE_ALERT    1 /*!< speed-wobble warning from the roll rate (needs IMU)   */
#define VCU_FEATURE_TURN_TRACKING   1 /*!< turn-signal auto-cancel from the yaw rate (needs IMU) */

#elif VCU_MODEL == VCU_MODEL_ZT3
#define VCU_MODEL_NAME              "ZT3"              /*!< model name                                          */
#define VCU_MODEL_BANNER            "SCOOTER_VCU_xxU2" /*!< image banner: SCOOTER_VCU_ + the 4-char model code */
/* Identity: serial = prefix (3) + region code (1) + "0000000001". */
#define VCU_MODEL_SERIAL_PREFIX     "1K1"              /*!< serial family prefix                         */
#define VCU_MODEL_DEFAULT_REGION    "P"                /*!< region of the factory-default serial         */
#define VCU_MODEL_BOARD_PREFIX      "Z03X"             /*!< board id prefix of the default model string   */
/* Features (each one 1 or 0, the same list in every model block). */
#define VCU_FEATURE_IMU             0                  /*!< no IMU on the ZT3 hardware seen so far               */
#define VCU_FEATURE_CLIMB_ASSIST    0                  /*!< off until verified                                   */
#define VCU_FEATURE_OVERSPEED_REGEN 0                  /*!< off until verified                                   */
#define VCU_FEATURE_WOBBLE_ALERT    0                  /*!< needs the IMU                                        */
#define VCU_FEATURE_TURN_TRACKING   0                  /*!< needs the IMU                                        */
/* Not verified yet: this feature list is provisional, and the rest of the image keeps the GT3
   behaviour (app_header.c warns once per build). */
#define VCU_MODEL_UNVERIFIED        1                  /*!< the feature list is not checked against the scooter yet */

#elif VCU_MODEL == VCU_MODEL_F3
#define VCU_MODEL_NAME              "F3"               /*!< model name                                          */
#define VCU_MODEL_BANNER            "SCOOTER_VCU_xxF3" /*!< image banner: SCOOTER_VCU_ + the 4-char model code */
/* Identity: serial = prefix (3) + region code (1) + "0000000001". */
#define VCU_MODEL_SERIAL_PREFIX     "1EF"              /*!< serial family prefix                         */
#define VCU_MODEL_DEFAULT_REGION    "P"                /*!< region of the factory-default serial         */
#define VCU_MODEL_BOARD_PREFIX      "Z03B"             /*!< board id prefix of the default model string   */
/* Features (each one 1 or 0, the same list in every model block). */
#define VCU_FEATURE_IMU             0                  /*!< no IMU in the F3 hardware seen (old build)            */
#define VCU_FEATURE_CLIMB_ASSIST    0
#define VCU_FEATURE_OVERSPEED_REGEN 0
#define VCU_FEATURE_WOBBLE_ALERT    0
#define VCU_FEATURE_TURN_TRACKING   0
/* Not verified yet: the F3 list is a cautious default, and the rest of the image keeps the GT3
   behaviour (app_header.c warns once per build). */
#define VCU_MODEL_UNVERIFIED        1 /*!< the feature list is not checked against the scooter yet */

#else
#error "unknown VCU_MODEL"
#endif

/* Features that need another one. */
#if (VCU_HAS_FEATURE(WOBBLE_ALERT) || VCU_HAS_FEATURE(TURN_TRACKING)) && !VCU_HAS_FEATURE(IMU)
#error "WOBBLE_ALERT and TURN_TRACKING need the IMU"
#endif

#endif /* MODEL_H */
