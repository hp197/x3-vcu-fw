/**
 * \file            control.c
 * \brief           Control loop: ADC analog front-end + input->drive-command pipeline
 * \note            Control front-end of the x3 VCU on the Artery AT32F415 BSP:
 *                        vcu_control_init        (ADC1 configure + bring-up)
 *                        control_adc_gpio_init   (PA0/1/4/5/7 = analog)
 *                        control_adc_dma_init    (DMA1_CH1 -> scan buffer)
 *                        control_build_drive_cmd (per-mode drive command)
 *                        control_scale_current   (voltage/SoC derate)
 *                        control_map / control_map_dec (clamped linear maps)
 *                        control_event_set / control_event_clear (event latch)
 *
 *                      The five analog inputs (throttle, two brake levers, two current senses) run as a
 *                      5-channel ADC1 ordinary scan in continuous mode; DMA1 channel 1 copies each
 *                      conversion into a 5-halfword circular RAM buffer owned here. No ADC ISR runs, so
 *                      the super-loop reads the newest buffer snapshot at task rate. The drive command
 *                      staged in g_mode is pulled by the CAN scheduler (can.c vcu_can_payload_get) and
 *                      sent as StdId 0x100, DLC 8.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "control.h"
#include "board.h" /* BOARD_CLOCK_ON */
#include "can.h"
#include "gpio.h"        /* vcu_gpio_config */
#include "ride_assist.h" /* ride-assist overrides read by the drive command / input scan */
#include "sensor.h"      /* IMU accessors */
#include "vcu_state.h"

/* ------------------------------------------------------------------------- */
/* ADC configuration constants.                                              */
/* ------------------------------------------------------------------------- */

/** \brief ADC clock = APB2 / 6. */
#define VCU_ADC_CLOCK_DIV  CRM_ADC_DIV_6

/** \brief Per-channel sample time, 239.5 cycles. */
#define VCU_ADC_SAMPLETIME ADC_SAMPLETIME_239_5

/* Ordinary trigger source: software. With continuous (repeat) mode, a single
   software start keeps the scan free-running into the DMA buffer. */
#define VCU_ADC_TRIG       ADC12_ORDINARY_TRIG_SOFTWARE

/* The ADC1 fields adc_base_config / adc_ordinary_channel_set / adc_ordinary_conversion_trigger_set
   write for this scan:
     ctrl2: rpen [1] = 1 (repeat), dtalign [11] = 0 (right), octesel_l [19:17] = trigger,
            octen [20] = 1, octesel_h [25] = 0 (trigger <= 7);
     spt2 : 3-bit sample time of channels 0, 1, 4, 5, 7;
     osq3 : 5-bit channel of sequence slots 1..5 [24:0]. */
#define VCU_ADC_CTRL2_MASK 0x021E0802u
#define VCU_ADC_CTRL2_CFG  (0x2u | ((uint32_t)VCU_ADC_TRIG << 17) | (1u << 20))
#define VCU_ADC_SPT(ch)    ((uint32_t)VCU_ADC_SAMPLETIME << (3u * (uint32_t)(ch)))
#define VCU_ADC_SPT2_MASK  (0x7u | (0x7u << 3) | (0x7u << 12) | (0x7u << 15) | (0x7u << 21))
#define VCU_ADC_SPT2_CFG                                                                                               \
    (VCU_ADC_SPT(VCU_ADC_CH_THROTTLE) | VCU_ADC_SPT(VCU_ADC_CH_BRAKE_HALF) | VCU_ADC_SPT(VCU_ADC_CH_BRAKE_FULL)        \
     | VCU_ADC_SPT(VCU_ADC_CH_ISENSE_PB10) | VCU_ADC_SPT(VCU_ADC_CH_ISENSE_PB0))
#define VCU_ADC_OSQ3_MASK 0x01FFFFFFu
#define VCU_ADC_OSQ3_CFG                                                                                               \
    ((uint32_t)VCU_ADC_CH_THROTTLE | ((uint32_t)VCU_ADC_CH_BRAKE_HALF << 5) | ((uint32_t)VCU_ADC_CH_BRAKE_FULL << 10)  \
     | ((uint32_t)VCU_ADC_CH_ISENSE_PB10 << 15) | ((uint32_t)VCU_ADC_CH_ISENSE_PB0 << 20))

/* DMA1 channel 1 ctrl (what dma_reset + dma_init leave for the settings below): peripheral ->
   memory, peripheral 16-bit fixed, memory 16-bit incremented, circular, priority medium. */
#define VCU_ADC_DMA_CTRL                                                                                               \
    ((uint32_t)DMA_DIR_PERIPHERAL_TO_MEMORY | (1u << 5) | (1u << 7)                                                    \
     | ((uint32_t)DMA_PERIPHERAL_DATA_WIDTH_HALFWORD << 8) | ((uint32_t)DMA_MEMORY_DATA_WIDTH_HALFWORD << 10)          \
     | ((uint32_t)DMA_PRIORITY_MEDIUM << 12))
/** \brief DMA1 clr flags of channel 1 (global, full, half, error) */
#define VCU_ADC_DMA_FLAGS 0xFu

/* Scan buffer: 5 halfwords, circular, memory-increment, 16-bit both ends. DMA writes it in the
   background, the super-loop reads it. One hardware writer, one reader, so `volatile` is enough. */
static volatile uint16_t g_adc_dma_buf[VCU_ADC_SCAN_LEN];

/* The only accessor for the scan buffer: every analog leaf reader indexes it here (byte offset
   0/2/4/6/8). DMA keeps it fresh; no live single conversion is ever issued. */
uint16_t control_adc_raw(uint8_t slot) {
    if (slot >= VCU_ADC_SCAN_LEN) {
        return 0u;
    }
    return g_adc_dma_buf[slot];
}

/* Shared blackboards: the drive pipeline reaches several SRAM blocks, each typed by an overlay
   struct in vcu_state.h (g_vs, g_in1, g_in2, g_scr, ...). They alias the one shared SRAM store, so a
   write here is seen by every other module at the same address, and vice versa. */

/* Sensor-delta channels: current samples are the IMU accel norm (IMU state +0x68) and pitch mdeg
   (state +0x3C) via the sensor accessors; previous samples live in the IMU output block (+0x10/+0x14). */

/* ------------------------------------------------------------------------- */
/* GPIO analog front-end                                                     */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Configure PA0, PA1, PA4, PA5, PA7 as analog inputs
 * \note            Enables the GPIOA clock and sets the five pins analog (no pull) in one mask.
 *                      They are ADC1 channels 0, 1, 4, 5, 7: throttle PA0, half/full brake levers
 *                      PA1/PA4, current senses PA5/PA7 (see control.h).
 */
static void control_adc_gpio_init(void) {
    BOARD_CLOCK_ON(CRM_GPIOA_PERIPH_CLOCK);
    vcu_gpio_config(GPIOA, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_7, VCU_GPIO_CFG_ANALOG);
}

/* ------------------------------------------------------------------------- */
/* ADC DMA                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Configure DMA1 channel 1 to stream the ADC1 scan into RAM
 * \note            Enable the DMA1 clock; ctrl = \ref VCU_ADC_DMA_CTRL (peripheral -> memory,
 *                      16-bit both ends, memory increment, circular, medium priority); clear the
 *                      channel-1 flags; dtcnt = 5, paddr = &ADC1->odt (0x4001244C), maddr = scan
 *                      buffer; then enable the channel. DMA1_CH1 is the ADC1 request line on the
 *                      AT32F415, so no mux selection is needed.
 */
static void control_adc_dma_init(void) {
    BOARD_CLOCK_ON(CRM_DMA1_PERIPH_CLOCK);

    DMA1_CHANNEL1->ctrl = VCU_ADC_DMA_CTRL; /* circular: buffer reloads each full scan */
    DMA1->clr = VCU_ADC_DMA_FLAGS;
    DMA1_CHANNEL1->dtcnt = VCU_ADC_SCAN_LEN;
    DMA1_CHANNEL1->paddr = (uint32_t)&ADC1->odt; /* ADC1 ordinary data register, 0x4001244C */
    DMA1_CHANNEL1->maddr = (uint32_t)&g_adc_dma_buf[0];

    dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

/* ------------------------------------------------------------------------- */
/* ADC1 configure + bring-up                                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Initialise the control-loop analog front-end (ADC1 + DMA)
 * \note            Called once from vcu_periph_init (config.c). Order matters:
 *                        1. GPIO analog (control_adc_gpio_init)
 *                        2. ADC DMA (control_adc_dma_init)
 *                        3. enable ADC1 clock (CRM_ADC1_PERIPH_CLOCK)
 *                        4. adc_clock_div = APB2/6
 *                        5. base config: scan on, repeat on, right align, ordinary length = 5
 *                        6. ordinary channels ch0,ch1,ch4,ch5,ch7 in seq 1..5, 239.5 cycles each
 *                        7. ordinary trigger = software, enabled
 *                        8. CTRL2.ocdmaen = 1
 *                        9. CTRL2.adcen = 1
 *                       10. calibration: init -> wait -> start -> wait
 *                       11. CTRL2.ocswtrg = 1 -> scan starts
 */
void vcu_control_init(void) {
    control_adc_gpio_init();
    control_adc_dma_init();

    BOARD_CLOCK_ON(CRM_ADC1_PERIPH_CLOCK);
    CRM->cfg_bit.adcdiv_l = (uint32_t)VCU_ADC_CLOCK_DIV & 0x3u; /* the crm_adc_clock_div_set writes */
    CRM->cfg_bit.adcdiv_h = ((uint32_t)VCU_ADC_CLOCK_DIV >> 2) & 0x1u;

    /* base config: sequence mode on, repeat on, right align, length 5; ordinary sequence: slots
       1..5 over channels 0,1,4,5,7 at 239.5 cycles; software trigger enabled. */
    ADC1->ctrl1_bit.sqen = TRUE;
    ADC1->ctrl2 = (ADC1->ctrl2 & ~VCU_ADC_CTRL2_MASK) | VCU_ADC_CTRL2_CFG;
    ADC1->osq1_bit.oclen = VCU_ADC_SCAN_LEN - 1u;
    ADC1->spt2 = (ADC1->spt2 & ~VCU_ADC_SPT2_MASK) | VCU_ADC_SPT2_CFG;
    ADC1->osq3 = (ADC1->osq3 & ~VCU_ADC_OSQ3_MASK) | VCU_ADC_OSQ3_CFG;

    ADC1->ctrl2_bit.ocdmaen = TRUE; /* CTRL2.ocdmaen = 1 */
    ADC1->ctrl2_bit.adcen = TRUE;   /* CTRL2.adcen   = 1 */

    ADC1->ctrl2_bit.adcalinit = TRUE;
    while (ADC1->ctrl2_bit.adcalinit != 0u) {}
    ADC1->ctrl2_bit.adcal = TRUE;
    while (ADC1->ctrl2_bit.adcal != 0u) {}

    /* fire the (continuous) ordinary scan once; DMA keeps the buffer fresh. */
    ADC1->ctrl2_bit.ocswtrg = TRUE;
}

/* Drive-mode parameter table (ROM): 3 rows (region 0/1/2) x 9 uint16_t (0x12 bytes/row), read as
   `base + region*0x12 + col*2`. Columns are named by the producer variable they feed. */
enum {
    MP_CEIL2 = 0, /* col +0x00 -> mode-2 output ceiling seed + neutral-override cmd */
    MP_WE2 = 1,   /* col +0x02 -> mode-2 staging +0x0E low byte                     */
    MP_WC2 = 2,   /* col +0x04 -> mode-2 staging +0x0C current arg + override low   */
    MP_CEIL3 = 3, /* col +0x06 -> mode-3 output ceiling seed                        */
    MP_WE3 = 4,   /* col +0x08 -> mode-3 staging +0x0E low byte                     */
    MP_WC3 = 5,   /* col +0x0A -> mode-3 staging +0x0C current arg                  */
    MP_WC4 = 6,   /* col +0x0C -> (unused by this producer; reserved)               */
    MP_WE4 = 7,   /* col +0x0E -> mode-4 staging +0x0E low byte                     */
    MP_ACC = 8    /* col +0x10 -> accel-limit base (x1.3 / x0.8) + mode-4 curr arg  */
};

static const uint16_t g_mode_table[VCU_MODE_ROWS][VCU_MODE_COLS] = {
    /* row 0 GLOBAL  */ {0x0019, 0x0032, 0x0014, 0x0032, 0x005a, 0x001e, 0x0032, 0x008c, 0x0028},
    /* row 1 EUROPE  */ {0x000f, 0x0032, 0x0014, 0x0019, 0x005a, 0x0019, 0x0019, 0x008c, 0x0028},
    /* row 2 GERMANY */ {0x000f, 0x0032, 0x0014, 0x0014, 0x005a, 0x0019, 0x0016, 0x008c, 0x0028},
};

/**
 * \brief           One cell of the drive-mode parameter table (see control.h)
 * \note            The row wraps (% 3).
 */
uint16_t control_mode_param(uint8_t row, uint8_t col) { return g_mode_table[row % VCU_MODE_ROWS][col % VCU_MODE_COLS]; }

/** \brief Gear / drive-profile selector (blackboard IN2->gear). */
typedef enum {
    VCU_GEAR_NONE = 0,    /*!< not driving: command fields left untouched */
    VCU_GEAR_IDLE = 1,    /*!< idle / standby (regen hold)                */
    VCU_GEAR_DRIVE_A = 2, /*!< drive profile A                           */
    VCU_GEAR_DRIVE_B = 3, /*!< drive profile B                           */
    VCU_GEAR_DRIVE_C = 4  /*!< drive profile C                           */
} vcu_gear_t;

/** \brief Drive over-temp fault FSM state (blackboard VS->drive_fault_state). */
enum {
    DRIVE_FAULT_NONE = 0,    /*!< no fault                             */
    DRIVE_FAULT_TRIPPED = 1, /*!< over-temp tripped, holding           */
    DRIVE_FAULT_RECOVER = 2  /*!< recovery pending (request cleared)   */
};

/* Current/torque conditioning constants: these scale a base command byte. */
#define CURR_SCALE_SHIFT     7u        /*!< Q7 fixed-point: 0x80 == unity gain          */
#define CURR_SCALE_MASK      0x7FFFFFu /*!< 23-bit product mask (overflow guard)        */
#define CURR_CEIL_DIVISOR    100       /*!< STAT->curr_limit is in 1/100 units          */
#define CURR_SCALE_UNITY     0x80      /*!< full-scale (no derate)                       */

/* Battery-voltage derate curve breakpoints (voltage CODE = A3C->vbus_code >> 8). */
#define VBUS_SENTINEL_A      0xFDu /*!< sentinel code -> treat as nominal */
#define VBUS_SENTINEL_B      0xFEu /*!< sentinel code -> treat as nominal */
#define VBUS_NOMINAL         155u  /*!< nominal code used for the sentinels */
#define VBUS_BP_FULL         116u  /*!< below this: full scale (0x80)       */
#define VBUS_BP_MID          146u  /*!< below this: fixed 0x74 scale        */
#define VBUS_BP_HI           156u  /*!< below this: ramp 0x74 -> 0x40        */
#define VBUS_BP_CUT          191u  /*!< below this: ramp 0x40 -> 0x0C; else 0 */

/* Battery-voltage derate applied to the staging +0x0E byte (second curve). */
#define VBUS_WE_BP_FULL      165u /*!< code <= this: full scale            */
#define VBUS_WE_BP_CUT       190u /*!< code <= this: ramp 0x80 -> 0x0C; else 0 */

/* Accel-limit gains applied to the per-mode accel base (mp[MP_ACC]). */
#define ACCEL_LIMIT_FWD_GAIN 13u /*!< forward accel limit scale, tenths (x 1.3) */
#define ACCEL_LIMIT_REV_GAIN 8u  /*!< reverse accel limit scale, tenths (x 0.8) */
#define ACCEL_LIMIT_GAIN_DIV 10u /*!< divisor of the two gains                   */

/* Clamped linear maps for SoC/voltage scaling: control_map rises, control_map_dec falls. */

/** \brief Rising linear map, clamped high. */
static int control_map(int x, int in_lo, int in_hi, int out_lo, int out_hi) {
    if (in_hi < x) {
        return out_hi;
    }
    return ((out_hi - out_lo) * (x - in_lo)) / (in_hi - in_lo) + out_lo;
}

/** \brief Falling linear map, clamped low. */
static int control_map_dec(int x, int in_lo, int in_hi, int out_lo, int out_hi) {
    if (in_hi < x) {
        return out_lo;
    }
    return out_hi - ((out_hi - out_lo) * (x - in_lo)) / (in_hi - in_lo);
}

/* Event / fault latch (control_event_set raises, control_event_clear clears). State:
     - 32-bit pending bitmask g_scr.event_mask_lo/hi (SCR+0x2A/+0x2C): event 1..16 -> mask_lo
       bit (e-1), 17..32 -> mask_hi bit (e-17);
     - active-event byte g_lowglob.active_event_id = the lowest-numbered pending event;
     - reset halfword g_in2.f.event_ack_reset, zeroed when the active event is acked. */

/** \brief Latch the lowest-id active event. */
static void control_event_latch(uint8_t ev) {
    uint8_t cur;

    cur = g_lowglob.active_event_id;
    if (cur != 0u) {
        if (cur < ev) {
            ev = cur; /* keep the lower (higher-priority) id */
        }
        g_lowglob.active_event_id = ev;
    } else {
        g_lowglob.active_event_id = ev;
    }
}

/** \brief Drop the active-event latch on ack. */
static void control_event_unlatch(uint8_t ev) {
    if (g_lowglob.active_event_id == ev) {
        g_in2.f.event_ack_reset = 0u;
        g_lowglob.active_event_id = (uint8_t)g_in2.f.event_ack_reset; /* := 0 */
    }
}

/** \brief Raise event `ev` (latch + set mask bit). */
void control_event_set(uint8_t ev) {
    uint8_t n;

    control_event_latch(ev);
    if ((ev != 0u) && (ev <= 0x20u)) {
        n = (uint8_t)(ev - 1u);
        if (n < 0x10u) {
            g_scr.event_mask_lo |= (uint16_t)(1u << n);
        } else if (n < 0x20u) {
            g_scr.event_mask_hi |= (uint16_t)(1u << (n & 0x0Fu));
        }
    }
}

/** \brief Clear event `ev` (unlatch + clear mask bit). */
void control_event_clear(uint8_t ev) {
    uint8_t n;

    control_event_unlatch(ev);
    if ((ev != 0u) && (ev <= 0x20u)) {
        n = (uint8_t)(ev - 1u);
        if (n < 0x10u) {
            g_scr.event_mask_lo &= (uint16_t)~(1u << n);
        } else if (n < 0x20u) {
            g_scr.event_mask_hi &= (uint16_t)~(1u << (n & 0x0Fu));
        }
    }
}

/* Throttle/current conditioning: apply the battery-voltage derate, a low-SoC scale and the
   configured current ceiling to a base command byte (torque/current, not a speed cap). */
/** \brief Q7 scale with a 23-bit product guard: (base * scale) / 128. */
static uint32_t scale_q7(uint32_t base, uint32_t scale) {
    return ((base * scale) & CURR_SCALE_MASK) >> CURR_SCALE_SHIFT;
}

/** \brief Map the battery-voltage code to a Q7 current-derate scale. */
static int control_vbus_derate(uint32_t vcode) {
    int scale;

    if ((vcode == VBUS_SENTINEL_A) || (vcode == VBUS_SENTINEL_B)) {
        vcode = VBUS_NOMINAL; /* sentinel codes -> nominal */
    }
    if (vcode < VBUS_BP_FULL) {
        scale = CURR_SCALE_UNITY;
    } else if (vcode < VBUS_BP_MID) {
        scale = 0x74;
    } else if (vcode < VBUS_BP_HI) {
        scale = control_map_dec((int)vcode, 0x91, 0x9b, 0x40, 0x74);
    } else if (vcode < VBUS_BP_CUT) {
        scale = control_map_dec((int)vcode, 0x9b, 0xaa, 0x0c, 0x40);
    } else {
        scale = 0;
    }
    return scale;
}

static uint8_t control_scale_current(uint8_t base) {
    int curr_ceiling;
    int soc_scale;
    int voltage_scale;
    uint32_t volt_limited;
    uint32_t soc_limited;
    uint32_t result;

    /* Configured current ceiling (STAT+0x122, stored in 1/100 units). */
    curr_ceiling = (int)g_stat.curr_limit / CURR_CEIL_DIVISOR;

    /* Low-SoC scale from the battery state of charge (STAT+0x11E, 0..30 %). */
    soc_scale = control_map((int)g_stat.soc, 0, 0x1e, 0x1e, CURR_SCALE_UNITY);

    /* Battery-voltage derate from the voltage code (g_fault_codes.vbus_code, high byte). */
    voltage_scale = control_vbus_derate((uint32_t)(g_fault_codes.vbus_code >> 8));

    /* Take the most restrictive of voltage, raw base, current ceiling, SoC. */
    volt_limited = scale_q7((uint32_t)base, (uint32_t)voltage_scale);
    if ((uint32_t)base < volt_limited) {
        volt_limited = base;
    }
    result = volt_limited & 0xffu;
    if ((uint32_t)curr_ceiling <= result) {
        result = (uint32_t)curr_ceiling;
    }
    soc_limited = scale_q7((uint32_t)base, (uint32_t)soc_scale);
    if (soc_limited <= result) {
        result = soc_limited;
    }
    return (uint8_t)(result & 0xffu);
}

/* ------------------------------------------------------------------------- */
/* Per-tick input -> drive-command pipeline.                                 */
/* ------------------------------------------------------------------------- */

/* Drive-command constants. */
#define DRIVE_IDLE_ENERGY_REST     10u   /*!< gear 1 energy byte with the throttle at 0      */
#define DRIVE_IDLE_ENERGY_THROTTLE 25u   /*!< gear 1 energy byte with throttle               */
#define DRIVE_REGEN_OVERSPEED      3u    /*!< regen byte while the over-speed regen runs     */
#define DRIVE_REGEN_MEDIUM         22u   /*!< regen byte for reg 0x70 == 1                   */
#define DRIVE_REGEN_STRONG         45u   /*!< regen byte for reg 0x70 == 2                   */
#define DRIVE_WORD06               0x0Eu /*!< staging +0x06 in rows 0 and 1                  */
#define DRIVE_WORD06_ECO           0x0Au /*!< staging +0x06 in row 2                         */

/* Stored output ceilings below this value are replaced by the per-mode fallback. */
#define DRIVE_CEIL_MIN_VALID       5u
/* Output ceiling below which the per-mode seed is passed through unchanged (no ramp). */
#define DRIVE_CEIL_PASSTHRU        11u
/* Staging frame +0x10 bit13: idle/standby marker (set in idle, cleared while driving). */
#define FRAME_W10_IDLE             0x2000u

/**
 * \brief           Map the output ceiling through the low-SoC ramp
 * \param[in]       ceiling: the selected per-mode output ceiling
 * \return          SoC-adjusted lead byte
 * \note            Shared by drive profiles B and C (gears 3 and 4). A ceiling below
 *                      DRIVE_CEIL_PASSTHRU passes straight through; region 0 with a high ceiling
 *                      uses the 0..10 / 10..20 % SoC ramp; otherwise the 0..15 % ramp of the
 *                      battery SoC (STAT+0x11E).
 */
static uint8_t drive_ceiling_soc(uint32_t ceiling) {
    uint8_t lead;

    if (ceiling < DRIVE_CEIL_PASSTHRU) {
        lead = (uint8_t)ceiling;
    } else if ((g_vs.region == VCU_REGION_GLOBAL) && (ceiling > 0x1du)) {
        if (g_stat.soc < 0x0a) {
            lead = (uint8_t)control_map((int)g_stat.soc, 0, 0x0a, 0x0a, 0x1e);
        } else {
            lead = (uint8_t)control_map((int)g_stat.soc, 0x0a, 0x14, 0x1e, (int)ceiling);
        }
    } else {
        lead = (uint8_t)control_map((int)g_stat.soc, 0, 0x0f, 0x0a, (int)ceiling);
    }
    return lead;
}

/**
 * \brief           Build the ESC drive command into the staging frame
 * \note            Per-tick drive-command producer, run each control cycle from the active-state
 *                      task handlers. Reads the per-mode ROM table and vehicle state and packs the
 *                      command into g_mode. The stored per-mode ceiling is kept when it is between
 *                      DRIVE_CEIL_MIN_VALID and the mode's fallback; a lower or higher value selects
 *                      the fallback and is corrected in IN2+0x8E/+0x90 before the low-SoC ramp.
 */
void control_build_drive_cmd(void) {
    const uint16_t* params;
    uint8_t row;
    uint8_t base_ceiling;
    uint16_t curr_word; /* staging +0x0C : scaled current command */
    uint16_t we_word;   /* staging +0x0E : energy/limit word       */
    uint16_t word04;    /* staging +0x04                          */
    uint16_t word06;    /* staging +0x06                          */
    uint16_t word0a;    /* staging +0x0A : flags word              */
    uint16_t word10;    /* staging +0x10 : idle/relay word         */
    uint8_t cmd_lead;   /* staging +0x00 : output lead/ceiling byte */
    uint32_t ceiling;
    int volt_scale;
    uint32_t volt_code;
    uint8_t regen; /* staging +0x0E : regen strength          */

    /* Row-select by region (VS->region, vcu_region_t), wrapped % 3 to stay in the 3-row table.
       Not the app-facing vcu_ride_mode_t (WALK=1..POWER=4) stored elsewhere. */
    row = (uint8_t)(g_vs.region % VCU_MODE_ROWS);
    params = g_mode_table[row];
    base_ceiling = (uint8_t)g_in2.f.speed_limit;

    /* Load the staging-frame RMW mirrors (fields whose high bits are preserved). */
    curr_word = g_mode.curr_word;
    we_word = g_mode.we_word;
    word04 = g_mode.word04;
    word06 = g_mode.word06;
    word0a = g_mode.mode_flags;
    word10 = g_mode.enable_flags;
    cmd_lead = g_mode.cmd_lead;

    /* Accel-limit derivation: VS+9 = (u8)(mode[+0x10] * 1.3),
       VS+10 = (u8)(mode[+0x10] * 0.8), truncated. */
    g_vs.accel_limit_fwd = (uint8_t)(((uint32_t)params[MP_ACC] * ACCEL_LIMIT_FWD_GAIN) / ACCEL_LIMIT_GAIN_DIV);
    g_vs.accel_limit_rev = (uint8_t)(((uint32_t)params[MP_ACC] * ACCEL_LIMIT_REV_GAIN) / ACCEL_LIMIT_GAIN_DIV);

    g_in1.regen_active = 0; /* reset each tick */

    /* Gear / drive-state dispatch (IN2+0xB4). */
    switch ((vcu_gear_t)g_in2.f.gear) {
        case VCU_GEAR_IDLE:
            /* Energy 10 with the throttle at 0, else 25. */
            if ((g_mode.drive_output_word & 0xffu) == 0u) {
                we_word = (uint16_t)((we_word & 0xff00u) + DRIVE_IDLE_ENERGY_REST);
            } else {
                we_word = (uint16_t)((we_word & 0xff00u) + DRIVE_IDLE_ENERGY_THROTTLE);
            }
            curr_word = (uint16_t)((curr_word & 0xff00u) | control_scale_current(0x0a));
            cmd_lead = 0x05;
            word10 = (uint16_t)(word10 | FRAME_W10_IDLE);
            g_in1.regen_active = 1;
            break;

        case VCU_GEAR_DRIVE_A:
            curr_word = (uint16_t)((curr_word & 0xff00u) | control_scale_current((uint8_t)params[MP_WC2]));
            we_word = (uint16_t)((we_word & 0xff00u) | (uint8_t)params[MP_WE2]);

            ceiling = (uint8_t)g_in2.f.gear_a_ceiling;
            if ((ceiling < DRIVE_CEIL_MIN_VALID) || (ceiling > (uint8_t)params[MP_CEIL2])) {
                ceiling = (uint8_t)params[MP_CEIL2];
                g_in2.f.gear_a_ceiling = (uint16_t)((g_in2.f.gear_a_ceiling & 0xff00u) | (uint16_t)ceiling);
            }
            if (ceiling < DRIVE_CEIL_PASSTHRU) {
                cmd_lead = (uint8_t)ceiling;
            } else {
                cmd_lead = (uint8_t)control_map((int)g_stat.soc, 0, 0x0f, 0x0a, (int)ceiling);
            }
            word04 = 0x0062u;
            word10 = (uint16_t)(word10 & ~FRAME_W10_IDLE);
            break;

        case VCU_GEAR_DRIVE_B:
            curr_word = (uint16_t)((curr_word & 0xff00u) | control_scale_current((uint8_t)params[MP_WC3]));
            we_word = (uint16_t)((we_word & 0xff00u) | (uint8_t)params[MP_WE3]);

            ceiling = (uint8_t)(g_in2.f.speed_limit_2 >> 8);
            if ((ceiling < DRIVE_CEIL_MIN_VALID) || (ceiling > (uint8_t)params[MP_CEIL3])) {
                ceiling = (uint8_t)params[MP_CEIL3];
                g_in2.f.speed_limit_2 = (uint16_t)((g_in2.f.speed_limit_2 & 0x00ffu) | ((uint16_t)ceiling << 8));
            }
            cmd_lead = drive_ceiling_soc(ceiling);
            word04 = 0x0147u; /* every row */
            word10 = (uint16_t)(word10 & ~FRAME_W10_IDLE);
            break;

        case VCU_GEAR_DRIVE_C:
            curr_word = (uint16_t)((curr_word & 0xff00u) | control_scale_current((uint8_t)params[MP_ACC]));
            we_word = (uint16_t)((we_word & 0xff00u) | (uint8_t)params[MP_WE4]);

            ceiling = (uint8_t)g_in2.f.speed_limit_2;
            if ((ceiling < DRIVE_CEIL_MIN_VALID) || (ceiling > base_ceiling)) {
                ceiling = base_ceiling;
                g_in2.f.speed_limit_2 = (uint16_t)((g_in2.f.speed_limit_2 & 0xff00u) | (uint16_t)ceiling);
            }
            cmd_lead = drive_ceiling_soc(ceiling);
            word04 = 0x01ebu; /* every row */
            word10 = (uint16_t)(word10 & ~FRAME_W10_IDLE);
            break;

        default: /* gear not in 1..4: leave command fields untouched */ break;
    }

    /* --- common tail ------------------------------------------------------- */

    /* Coast preset: throttle at 0 with MODE+0x17 bit1 takes the energy byte from the ride-assist `coast_energy`.
       Neither has a writer in this firmware, so this never fires in practice. */
    if (((g_mode.drive_output_word & 0xffu) == 0u) && ((g_mode.output_cfg & OCFG_COAST_PRESET) != 0u)) {
        we_word = (uint16_t)((we_word & 0xff00u) | ride_assist_state()->coast_energy);
    }

    /* Battery-voltage derate of the energy byte (second curve), unless the climb boost overrides
       the energy and current bytes. */
    volt_code = (uint32_t)(g_fault_codes.vbus_code >> 8);
    if (volt_code <= VBUS_WE_BP_FULL) {
        volt_scale = CURR_SCALE_UNITY;
    } else if (volt_code <= VBUS_WE_BP_CUT) {
        volt_scale = control_map_dec((int)volt_code, 0xa5, 0xaa, 0x0c, 0x80);
    } else {
        volt_scale = 0;
    }
    if (((g_in2.f.cfg_flags2 & CFG2_CLIMB_BOOST) != 0u) && (ride_assist_state()->climb_factor > 1.0f)
        && (ride_assist_state()->climb_allowed != 0u)) {
        we_word = (uint16_t)((we_word & 0xff00u) | (uint8_t)ride_assist_state()->climb_energy);
        curr_word = (uint16_t)((curr_word & 0xff00u) | ride_assist_state()->climb_current);
    } else {
        we_word = (uint16_t)((we_word & 0xff00u)
                             | (uint8_t)(((uint32_t)(uint8_t)we_word * (uint32_t)volt_scale) >> CURR_SCALE_SHIFT));
    }

    /* Regen byte (staging +0x0E): reg 0x70 (IN2+0xE0) 0 / 1 / 2; other values
       leave it. 3 while the over-speed regen runs; the over-speed level lifts it. */
    if (g_in2.f.dir_bits == 0u) {
        regen = 0u;
        if (((g_mode.output_cfg & OCFG_OVERSPEED_REGEN) != 0u) && ((g_in2.f.cfg_flags2 & CFG2_OVERSPEED_REGEN) != 0u)) {
            regen = DRIVE_REGEN_OVERSPEED;
        }
    } else if (g_in2.f.dir_bits == 1u) {
        regen = DRIVE_REGEN_MEDIUM;
    } else if (g_in2.f.dir_bits == 2u) {
        regen = DRIVE_REGEN_STRONG;
    } else {
        regen = g_mode.regen_level;
    }
    if (((g_in2.f.cfg_flags2 & CFG2_OVERSPEED_REGEN) != 0u) && (ride_assist_state()->overspeed_regen > regen)) {
        regen = ride_assist_state()->overspeed_regen;
    }

    we_word = (uint16_t)((we_word & 0x00ffu) | 0x5a00u);
    curr_word = (uint16_t)((curr_word & 0xc0ffu) + 0x1400u);
    curr_word = (uint16_t)((curr_word & 0x3fffu) | (uint16_t)((g_in2.f.dir_bits & 0x3u) << 14));
    if (word0a & 0x0080u) {
        word04 = 0x3fffu;
    }

    g_mode.cmd_param = 0xc8u;
    cmd_lead = (uint8_t)(cmd_lead << 1);
    if (g_vs.region == VCU_REGION_GERMANY) {
        word06 = DRIVE_WORD06_ECO; /* 0x0A in row 2 */
        if (cmd_lead == 0x2cu) {
            cmd_lead = 0x2bu;
        }
    } else {
        word06 = DRIVE_WORD06;
    }

    if (((g_vs.region == 0) && (g_stat.soc < 0x14)) || (g_stat.soc < 0x0f)) {
        g_mode.cfg_17 |= CFG17_LOW_SOC;
    } else {
        g_mode.cfg_17 &= (uint8_t)~CFG17_LOW_SOC;
    }

    /* Over-speed event: when the speed is below base ceiling + 3, event 9 is
       cleared; otherwise it is raised. */
    if ((uint32_t)g_vs.speed_kmh < (uint32_t)base_ceiling + 3u) {
        control_event_clear(9);
    } else {
        control_event_set(9);
    }

    /* Output-enable gate (status_flags bit11, SF_SESSION): when CLEAR, overwrite the
       command with the mode's raw neutral params (failsafe); when SET, keep the
       computed command. */
    if ((g_in2.f.status_flags & SF_SESSION) == 0u) {
        curr_word = (uint16_t)((curr_word & 0xff00u) | (uint8_t)params[MP_WC2]);
        we_word = (uint16_t)((we_word & 0xff00u) | (uint8_t)params[MP_WE2]);
        cmd_lead = (uint8_t)((int16_t)params[MP_CEIL2] << 1);
    }

    /* Flush the RMW mirrors back into the staging frame. */
    g_mode.cmd_lead = cmd_lead;
    g_mode.word04 = word04;
    g_mode.word06 = word06;
    g_mode.mode_flags = word0a;
    g_mode.curr_word = curr_word;
    g_mode.we_word = we_word;
    g_mode.enable_flags = word10;
    g_mode.regen_level = regen;
}

/* ========================================================================= */
/* Drive/charge/lock fault FSMs + LED/DTC helpers.                           */
/* ========================================================================= */

/* Cross-module APIs the FSMs below call, through their owners' headers:
     usart.h   : vcu_usart_report_send
     report.h  : report_ninebot, report_subsys_enable
     buzzer.h  : buzzer_beep, buzzer_on, buzzer_off
     vehicle.h : vehicle_event9_prep, vehicle_event10_prep, vehicle_lock_antitheft_latch
     ninebot.h : nb_commit_mode (drive-mode commit) */
#include "buzzer.h"
#include "config.h" /* vcu_outputs_reset */
#include "ninebot.h"
#include "report.h"
#include "usart.h"
#include "vehicle.h"

/* Indicator / subsystem-enable GPIO leaves. */

/* Indicator primitive: set/clear GPIOB pin 0x2000 (PB13). */
void control_out_pb13(int32_t a) {
    if (a != 0) {
        gpio_bits_set(GPIOB, GPIO_PINS_13);
    } else {
        gpio_bits_reset(GPIOB, GPIO_PINS_13);
    }
}

/* Indicator primitive: set/clear GPIOA pin 0x40 (PA6). */
void control_out_pa6(int32_t a) {
    if (a != 0) {
        gpio_bits_set(GPIOA, GPIO_PINS_6);
    } else {
        gpio_bits_reset(GPIOA, GPIO_PINS_6);
    }
}

/* Indicator primitive: set/clear GPIOB pin 0x4000 (PB14). */
void control_out_pb14(int32_t a) {
    if (a != 0) {
        gpio_bits_set(GPIOB, GPIO_PINS_14);
    } else {
        gpio_bits_reset(GPIOB, GPIO_PINS_14);
    }
}

/* Subsystem-enable GPIO port + pins. */
#define CTRL_GPIO_SUBSYS   GPIOB        /* GPIOB, 0x40010C00                                  */
#define CTRL_GPIO_SUBSYS_A GPIO_PINS_10 /* GPIOB bit10 subsystem-A enable (active-low)           */
#define CTRL_GPIO_SUBSYS_B GPIO_PINS_0  /* GPIOB bit0  subsystem-B enable (active-low)           */

/* Subsystem-enable GPIO leaves, both active-low: en!=0 clears the pin (asserts), en==0 sets it
   (releases). Subsystem A = GPIOB pin 10 (mask 0x400), subsystem B = GPIOB pin 0 (mask 0x0001). */
void control_subsys_a_enable(int32_t en) { /* subsystem A enable (GPIOB pin10, active-low) */
    if (en != 0) {
        gpio_bits_reset(CTRL_GPIO_SUBSYS, CTRL_GPIO_SUBSYS_A); /* assert (low) */
    } else {
        gpio_bits_set(CTRL_GPIO_SUBSYS, CTRL_GPIO_SUBSYS_A); /* release (high) */
    }
}

void control_subsys_b_enable(int32_t en) { /* subsystem B enable (GPIOB pin0, active-low) */
    if (en != 0) {
        gpio_bits_reset(CTRL_GPIO_SUBSYS, CTRL_GPIO_SUBSYS_B); /* assert (low) */
    } else {
        gpio_bits_set(CTRL_GPIO_SUBSYS, CTRL_GPIO_SUBSYS_B); /* release (high) */
    }
}

/* DTC / active-error-code subsystem (separate from the event latch above). Active code byte
   g_lowglob.active_error_code; DTC mask halfwords g_scr +0x24 (10..25), +0x26 (26..41), +0x28 (42..57). */

/** \brief Latch the lowest active error code. */
static void control_err_latch(uint32_t code) {
    if (g_lowglob.active_error_code != 0u) {
        if ((uint32_t)g_lowglob.active_error_code < code) {
            code = g_lowglob.active_error_code; /* keep the lower (higher-priority) code */
        }
        g_lowglob.active_error_code = (uint8_t)code;
    } else {
        g_lowglob.active_error_code = (uint8_t)code;
    }
}

/**
 * \brief           Raise DTC `code`
 * \note            Latches the active code, then sets the DTC mask bit. Codes 10..25 ->
 *                      mask0 bit (code-10); 26..41 -> mask1 bit ((code+6)&0xF); 42..57 ->
 *                      mask2 bit; the high ranges share the `1 << ((code+6)&0xF)` index.
 */
void control_dtc_set(uint32_t code) {
    uint32_t n;
    uint16_t bit;

    control_err_latch(code);
    if ((9u < code) && (code < 0x3au)) {
        n = (code - 10u) & 0xffu;
        if (0xfu < n) {
            bit = (uint16_t)(1u << ((code + 6u) & 0xfu));
            if (n < 0x20u) {
                g_scr.dtc_mask1 |= bit;
            } else if (n < 0x30u) {
                g_scr.dtc_mask2 |= bit;
            }
        } else {
            g_scr.dtc_mask0 |= (uint16_t)(1u << n);
        }
    }
}

/** \brief Drop the active error code on ack. */
static void control_err_clear_active(uint32_t code) {
    if ((uint32_t)g_lowglob.active_error_code == code) {
        g_lowglob.active_error_code = 0u;
    }
}

/** \brief Clear DTC `code` (ack + clear mask bit). */
void control_dtc_clear(uint32_t code) {
    uint32_t n;
    uint16_t bit;

    control_err_clear_active(code);
    if ((9u < code) && (code < 0x3au)) {
        n = (code - 10u) & 0xffu;
        if (0xfu < n) {
            bit = (uint16_t)(1u << ((code + 6u) & 0xfu));
            if (n < 0x20u) {
                g_scr.dtc_mask1 &= (uint16_t)~bit;
            } else if (n < 0x30u) {
                g_scr.dtc_mask2 &= (uint16_t)~bit;
            }
        } else {
            g_scr.dtc_mask0 &= (uint16_t)~(1u << n);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Countdown mirror + periodic beep.                                         */
/* ------------------------------------------------------------------------- */

/** \brief Mirror VS+0x3C/100 and beep on multiples. */
void control_countdown_beep(void) {
    uint32_t cd;

    cd = g_vs.auto_off_ticks; /* VS+0x3C */
    g_scr.countdown_mirror = (uint16_t)(cd / 100u);
    if ((cd != 0u) && (cd < 0x1f5u)) {
        if ((cd % 100u) == 0u) {
            buzzer_beep(BEEP_SHORT);
        }
    }
}

/* Mode / lighting status reporter. `arg` is a lighting/status code (0,1,2,4,8,0x10), not a ride
   mode. scr+0x1AA holds a reported indicator level (0/1/2) in bits 0..1 and light-state flags
   (0x04/0x08/0x10) in bits 2..4. Each change emits report 0xD5 (dst 0x23); a bit-3/4 change also
   reloads VS+0x36 = 300. Returns the published status halfword on an emit, else `arg` (one caller,
   the charge timer, uses control_mode_report_fsm(4) as its chirp value). */

/** \brief Emit the mode/lighting status halfword (scr+0x1AA) as report 0xD5; return it. */
static uint32_t control_mode_emit(void) {
    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3, 0xd5,
                          &g_scr.mode_status);
    return (uint32_t)g_scr.mode_status;
}

/** \brief Update + report the mode/lighting status. */
uint32_t control_mode_report_fsm(uint32_t arg) {
    if (arg == 4u) {
        g_scr.mode_status |= 4u;
        (void)control_mode_emit();
        g_scr.mode_status &= 0xfffbu;
    } else {
        if ((int32_t)arg < 5) {
            if ((arg != 0u) && (arg != 1u) && (arg != 2u)) {
                return arg; /* early reject: return arg */
            }
            g_scr.mode_status = (uint16_t)arg;
            return control_mode_emit();
        }
        if ((arg != 8u) && (arg != 0x10u)) {
            return arg; /* early reject: return arg */
        }
    }
    if ((uint32_t)(g_scr.mode_status & 0x18u) != arg) {
        g_scr.mode_status = (uint16_t)((g_scr.mode_status & 3u) | arg);
        g_vs.mode_timeout = 300; /* VS+0x36 reload */
        return control_mode_emit();
    }
    return arg; /* unchanged flag: (scr+0x1AA & 0x18) == arg */
}

/* Conditional buzzer chirp: when the buzzer is idle (TMR1 cctrl.c1en bit0 clear) and IN2+0x3A
   bit11 (0x800) is set, chirp it (BUZZER_TONE). */

/** \brief Chirp only if the buzzer is idle. */
static void control_trigger_out(void) {
    if ((TMR1->cctrl_bit.c1en == 0u) && ((g_in2.f.cfg_flags & 0x0800u) != 0u)) {
        buzzer_on(BUZZER_TONE);
    }
}

/* ------------------------------------------------------------------------- */
/* Indicator output from the mode flag.                                      */
/* ------------------------------------------------------------------------- */

/** \brief Drive the indicator primitives from VS+0x00. */
void control_output_light(void) {
    control_out_pb13(0);
    control_out_pa6(0);
    if (g_vs.region != 0u) {
        control_out_pb14(0);
    } else {
        control_out_pb14(1);
    }
}

/* Drive over-temp fault FSM + mode-report dispatch. Gated by the brake-lever faults IN1+0x1F /
   IN1+0x2C. With IN2+0x3A bit5 set, it debounces a guard counter (drv+0x1A) against standstill
   (VS+0x02 == 0), then on a latched over-temp (drv+0x0A) runs a dwell timer (drv+0x18, limit 300).
   At expiry: SoC (STAT+0x11E) < 6 % reports lighting-8; otherwise it trips the fault (requests
   state 3, sets drv+9, scr+0x56, frame+0x0A bits 0x4|0x8, VS+4=1). Recovery clears them and
   reports lighting-0. */

/* Drive-fault state in staging +0x19 bits 2..3 (MODE cfg_17, sent in CAN 0x20C byte 6). */
#define CFG17_FAULT_NONE      0u   /*!< no fault                                     */
#define CFG17_FAULT_TRIPPED   1u   /*!< tripped                                      */
#define DRIVE_FAULT_VBUS_HIGH 150u /*!< tripped + brake + vbus code above: state 3   */

/**
 * \brief           Write the drive-fault state into staging +0x19 bits 2..3
 * \param[in]       state: 0 none, 1 tripped
 * \note            Tripped: `(b & ~0xC) + 4`; none: `b & ~0xC`.
 */
static void control_drive_fault_report(uint8_t state) {
    g_mode.cfg_17 = (uint8_t)((g_mode.cfg_17 & (uint8_t)~CFG17_DRIVE_FAULT)
                              + (uint8_t)(state << CFG17_DRIVE_FAULT_SHIFT));
}

/** \brief Drive over-temp fault supervisor. */
void control_fault_drive_fsm(void) {
    uint16_t c;

    if ((g_in1.brake_full_range_fault != 0u) || (g_in1.brake_half_range_fault != 0u)) {
        if (g_vs.drive_fault_state != DRIVE_FAULT_NONE) { /* outer else of the lock guard */
            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & 0xfff7u);
            g_scr.drive_fault_req = 0;
            g_vs.drive_fault_state = DRIVE_FAULT_NONE;
            control_mode_report_fsm(0);
        }
        return;
    }

    if ((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x1a) < 0) { /* IN2+0x3A bit5 */
        if (g_vs.speed_kmh == 0u) {                           /* VS+0x02 */
            if (g_in1.brake_any != 0u) {                      /* IN1+0x0B */
                g_tmr.guard_timer = 0;
            }
            c = g_tmr.guard_timer;
            g_tmr.guard_timer = (uint16_t)(c + 1);
            if (100u < c) {
                g_tmr.ovt_active = 1;
                g_tmr.guard_timer = 0;
            }
        } else if (g_vs.drive_fault_state == DRIVE_FAULT_NONE) { /* else-if: VS+4 == 0 -> clear over-temp latch */
            g_tmr.ovt_active = 0;
            g_tmr.guard_timer = 0;
        }

        if (g_tmr.ovt_active != 0u) {
            if (g_in1.brake_any == 1u) {
                if (g_vs.drive_fault_state == DRIVE_FAULT_NONE) {
                    if (g_tmr.ovt_timer < 300u) {
                        g_tmr.ovt_timer = (uint16_t)(g_tmr.ovt_timer + 1);
                    } else {
                        g_tmr.ovt_timer = 0;
                        if (g_stat.soc < 6) {
                            control_mode_report_fsm(8);
                        } else {
                            buzzer_beep(BEEP_DOUBLE);
                            g_tmr.ovt_flag = 1;
                            g_scr.drive_fault_req = 1;
                            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 4u);
                            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 8u);
                            control_drive_fault_report(CFG17_FAULT_TRIPPED);
                            g_vs.drive_fault_state = DRIVE_FAULT_TRIPPED;
                        }
                    }
                } else if (g_vs.drive_fault_state == DRIVE_FAULT_TRIPPED) {
                    if ((uint32_t)(g_fault_codes.vbus_code >> 8) > DRIVE_FAULT_VBUS_HIGH) {
                        g_mode.cfg_17 |= CFG17_DRIVE_FAULT;
                    }
                    if (g_tmr.ovt_flag == 0u) {
                        g_vs.drive_fault_state = DRIVE_FAULT_RECOVER;
                    }
                } else {
                    g_tmr.ovt_timer = 0;
                }
            } else {
                g_tmr.ovt_timer = 0;
                g_tmr.ovt_flag = 0;
                if ((g_vs.drive_fault_state == DRIVE_FAULT_RECOVER)
                    || ((g_stat.soc < 6) && (g_vs.drive_fault_state == DRIVE_FAULT_TRIPPED))) {
                    buzzer_beep(BEEP_LONG);
                    g_scr.drive_fault_req = 0;
                    g_vs.drive_fault_state = DRIVE_FAULT_NONE;
                    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & 0xfff7u);
                    control_drive_fault_report(CFG17_FAULT_NONE);
                    control_mode_report_fsm(0);
                }
            }
        }
    } else if (g_vs.drive_fault_state != DRIVE_FAULT_NONE) { /* IN2+0x3A bit5 clear */
        g_scr.drive_fault_req = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & 0xfff7u);
        g_vs.drive_fault_state = DRIVE_FAULT_NONE;
        control_mode_report_fsm(0);
    }

    if ((g_in1.throttle_flag != 0u) && (g_vs.drive_fault_state == DRIVE_FAULT_TRIPPED) && (g_scr.mode_status != 2u)) {
        control_mode_report_fsm(2);
    }
}

/* Lock / standby arming FSM. Gated by VS+4 / VS+0x41. When unlocked (VS+0x00 == 0) and the arm
   conditions hold, it counts VS+0x30 up to 600 (raises the lock request scr+0x3E at 600). The
   request branch acks the lock (reports lighting-8 or 0) or holds it (reports lighting-1/2). A
   subdivider (drv+0x07/+0x08, limit 0x32) gates staging frame+0x0A bit 2. */

/** \brief Standby-arm timer value that raises the lock request (VS lock_timer). */
#define LOCK_TIMER_FULL 600u

/**
 * \brief           Lock / standby arming FSM
 * \note            MODE+0x17 bit1 clears the timer and holds the lock-out gate; a brake lever
 *                      holds the timer at 0; the request branch clears TMR lock_out first and
 *                      reports 0 once the request drops; the lock-out releases only at a
 *                      standstill; the throttle clears MODE+0xA bit2 without the 0x32-tick divider;
 *                      a pending power-button short press is consumed at the end.
 */
void control_lock_fsm(void) {
    uint8_t cancel;

    if ((g_vs.drive_fault_state != DRIVE_FAULT_NONE) || (g_vs.passcode_set_pending != 0u)) {
        return;
    }
    if ((g_mode.output_cfg & OCFG_COAST_PRESET) != 0u) {
        g_vs.lock_timer = 0;
    }

    if (g_vs.region == 0u) { /* VS+0x00 == 0 : unlocked/idle */
        if (((int16_t)g_in2.f.fault_code >= 3) || (g_vs.speed_kmh >= VCU_STANDSTILL_KMH)
            || (g_in1.regen_active != 0u)) {
            g_vs.lock_timer = 0;
        } else if (g_vs.lock_timer >= LOCK_TIMER_FULL) {
            g_scr.lock_req = 1;
        } else {
            /* a brake lever holds the timer at 0 */
            if (g_in1.brake_any != 0u) {
                g_vs.lock_timer = 0;
            } else {
                g_vs.lock_timer = (uint16_t)(g_vs.lock_timer + 1);
            }
            if (g_vs.lock_timer == LOCK_TIMER_FULL) {
                buzzer_beep(BEEP_LONG);
            }
        }
    } else {
        if ((int16_t)g_in2.f.fault_code < 3) {
            g_in2.f.fault_code = 3;
        }
        g_scr.lock_req = 0;
    }

    if (g_scr.lock_req == 1u) {
        g_tmr.lock_out = 0;
        if ((g_in1.throttle_flag != 0u) && (g_scr.mode_status != 1u)) {
            if (g_in1.charger_connected != 0u) {
                control_mode_report_fsm(8);
                return;
            }
            control_mode_report_fsm(1);
        } else {
            if (g_in1.regen_active != 0u) {
                g_vs.lock_timer = LOCK_TIMER_FULL; /* VS+0x30..0x31 = 0x0258 */
                g_scr.lock_req = 0;
            } else {
                cancel = ((g_in1.brake_any != 0u) || (g_evt.mon[FM_MON_POWER_BTN].code == 1u)
                          || (g_vs.speed_kmh >= VCU_STANDSTILL_KMH) || ((int16_t)g_in2.f.fault_code >= 3))
                             ? 1u
                             : 0u;
                if (cancel != 0u) {
                    if (g_in1.charger_connected != 0u) {
                        control_mode_report_fsm(8);
                    } else {
                        buzzer_beep(BEEP_DOUBLE);
                        g_vs.lock_timer = 0;
                        g_scr.lock_req = 0;
                    }
                }
            }
            if (g_scr.lock_req == 0u) {
                control_mode_report_fsm(0);
            }
        }
    } else if (g_in2.f.lock_state != 0u) {
        g_tmr.lock_out = 0;
    } else if (((int16_t)(uint16_t)(uint8_t)g_vs.speed_kmh >= (int16_t)g_in2.f.fault_code)
               || (g_in1.regen_active != 0u)) {
        g_tmr.lock_out = 1;
    } else if ((g_in1.brake_any == 0u) && (g_vs.speed_kmh == 0u)) { /* at a standstill only */
        g_tmr.lock_out = 0;
    }

    /* MODE+0x17 bit1 holds the gate like a lock-out; the throttle drops it at once. */
    if ((g_tmr.lock_out != 0u) || ((g_mode.output_cfg & OCFG_COAST_PRESET) != 0u)) {
        g_tmr.lock_div = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 4u);
    } else if ((g_in1.throttle_flag != 0u) || (g_tmr.lock_div >= 0x32u)) {
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & 0xfffbu);
    } else {
        g_tmr.lock_div = (uint8_t)(g_tmr.lock_div + 1);
    }
    if (g_evt.mon[FM_MON_POWER_BTN].code == 1u) {
        g_evt.mon[FM_MON_POWER_BTN].code = 0;
    }
}

/* Scheduled-charging window. Runs while scheduled charging is enabled (reg 0x1F b4) and armed
   (IN1+0x22). Every 0x32 ticks it reads the hour from the BMS clock (packed status word STAT+0x1FC
   bits 12..16) and compares it with the window start/end hours (regs 0x4B / 0x4C, high byte). At the
   window end it stops charging (VS+0x19), requests the BMS charge-control push and a screen refresh
   and clears the enable; at the window start a single-shot schedule (b5) clears b4/b5/b7. */

/** \brief Scheduled-charging window tick. */
void charge_schedule_tick(void) {
    uint16_t c;
    uint32_t code;
    uint32_t t_lo;
    uint32_t t_hi;

    if (((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1b) < 0) && (g_in1.chg_sched_armed != 0u)) {
        c = g_oneshot.window_timer;
        g_oneshot.window_timer = (uint16_t)(c + 1);
        if (0x32u < c) {
            g_oneshot.window_timer = 0;
            code = (g_stat.status_packed & 0x1ffffu) >> 0xc; /* bits 12..16 */
            t_lo = (uint32_t)(g_in2.f.chg_sched_start >> 8);
            t_hi = (uint32_t)(g_in2.f.chg_sched_end >> 8);
            if (t_lo < t_hi) {
                if (t_hi <= code) {
                    g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
                    g_in1.chg_sched_armed = 0;
                    g_vs.chg_disp_refresh = 1;
                    g_vs.chg_sched_stop = 1;
                    g_in2.f.chg_cfg &= 0xffefu;
                }
            } else if ((t_hi <= code) && (code < t_lo)) {
                g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
                g_in1.chg_sched_armed = 0;
                g_vs.chg_disp_refresh = 1;
                g_vs.chg_sched_stop = 1;
                g_in2.f.chg_cfg &= 0xffefu;
            }
            if ((t_lo <= code) && ((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1a) < 0)) {
                g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
                g_in1.chg_sched_armed = 0;
                g_vs.chg_disp_refresh = 1;
                g_in2.f.chg_cfg &= 0xffefu;
                g_in2.f.chg_cfg &= 0xffdfu;
                g_in2.f.chg_cfg &= 0xff7fu;
            }
        }
        return;
    }
    g_oneshot.window_timer = 0;
    g_in1.chg_sched_armed = 0;
}

/* Charge detect + relay FSM.
   First half (flash_seq == 0): while a charger is present (chgstate+2/+3) and the debounce timer
   (chgstate+8) is below 0x2B, it averages the two charge-current senses (scan slots 3/4, each gated
   on its own PB10 / PB0 output-latch read-back) into chgbuf and, after enough stable samples, sets
   the charge-detected latch (chgstate+5) and raises event 0x18.
   Second half (flash_seq != 0): a relay dwell counter (chgbuf+0x0C) sequences the charge outputs
   (subsystem A/B enables) and the staging frame+0x10 bits. */

/** \brief Charge detection + relay FSM. */
void control_charge_fsm(void) {
    uint16_t v;
    uint8_t n;
    int16_t s;
    uint32_t cnt;
    uint16_t w10;

    if (g_vs.flash_seq == FLASH_SEQ_IDLE) {
        if ((g_chg.f2 == 1u) || (g_chg.f3 == 1u)) {
            if (g_chg.timer < 0x2bu) {
                if ((g_chg.f2 == 1u) && (g_chg.f3 == 1u)) {
                    control_trigger_out();
                    report_subsys_enable();
                } else if ((g_chg.f2 == 1u) && (g_chg.f1 == 0u)) {
                    control_trigger_out();
                    vehicle_event9_prep();
                    g_chg.f7 = 0;
                } else if ((g_chg.f3 == 1u) && (g_chg.f0 == 0u)) {
                    control_trigger_out();
                    vehicle_event10_prep();
                    g_chg.f6 = 0;
                }
                if (g_chg.f6 == 0u) {
                    g_chgbuf.acc[2] = 0;
                }
                if (g_chg.f7 == 0u) {
                    g_chgbuf.acc[3] = 0;
                }
                /* Read back the PB10 / PB0 subsystem-enable output latches (GPIOB->odt
                   @0x40010C0C), each with its own live read. */
                if (gpio_output_data_bit_read(GPIOB, GPIO_PINS_10) == RESET) { /* PB10 latch low */
                    s = (int16_t)control_adc_raw(VCU_ADC_SLOT_ISENSE_PB10); /* charge-current sense A (slot 3, PA5) */
                    v = g_chgbuf.acc[2];
                    g_chgbuf.acc[2] = (uint16_t)(s + v);
                    n = g_chg.f6;
                    g_chg.f6 = (uint8_t)(n + 1);
                    if (3u < n) {
                        v = (uint16_t)((uint16_t)(s + v) >> 2);
                        g_chgbuf.acc[2] = v;
                        if (v < 400u) {
                            v = g_chgbuf.acc[0];
                            g_chgbuf.acc[0] = (uint16_t)(v + 1);
                            if (4u < v) {
                                g_chgbuf.acc[0] = 0;
                                g_chg.f5 = 1;
                            }
                        } else {
                            g_chgbuf.acc[0] = 0;
                        }
                        g_chgbuf.acc[2] = 0;
                        g_chg.f6 = 0;
                    }
                }
                if (gpio_output_data_bit_read(GPIOB, GPIO_PINS_0) == RESET) { /* PB0 latch low, re-read */
                    s = (int16_t)control_adc_raw(VCU_ADC_SLOT_ISENSE_PB0);    /* charge-current sense B (slot 4, PA7) */
                    v = g_chgbuf.acc[3];
                    g_chgbuf.acc[3] = (uint16_t)(s + v);
                    n = g_chg.f7;
                    g_chg.f7 = (uint8_t)(n + 1);
                    if (3u < n) {
                        v = (uint16_t)((uint16_t)(s + v) >> 2);
                        g_chgbuf.acc[3] = v;
                        if (v < 400u) {
                            v = g_chgbuf.acc[1];
                            g_chgbuf.acc[1] = (uint16_t)(v + 1);
                            if (4u < v) {
                                g_chgbuf.acc[1] = 0;
                                g_chg.f5 = 1;
                            }
                        } else {
                            g_chgbuf.acc[1] = 0;
                        }
                        g_chgbuf.acc[3] = 0;
                        g_chg.f7 = 0;
                    }
                }
            } else {
                if (g_chg.f5 == 0u) {
                    buzzer_off();
                }
                control_mode_sync();
                g_chg.f6 = 0;
                g_chg.f7 = 0;
                if ((g_chg.f2 == 1u) && (g_chg.f1 == 1u)) {
                    g_chg.f1 = 0;
                    g_chg.timer = 0;
                    g_chg.f5 = 0;
                }
                if ((g_chg.f3 == 1u) && (g_chg.f0 == 1u)) {
                    g_chg.f0 = 0;
                    g_chg.timer = 0;
                    g_chg.f5 = 0;
                }
            }
            if (g_chg.f5 == 0u) {
                v = g_chg.timer;
                g_chg.timer = (uint16_t)(v + 1);
                if (0x55u < v) {
                    g_chg.timer = 0;
                }
            } else {
                control_event_set(0x18);
                v = g_chg.timer;
                g_chg.timer = (uint16_t)(v + 1);
                if (0x3bu < v) {
                    g_chg.timer = 0;
                }
            }
        } else if (g_chg.timer == 0u) {
            control_mode_sync();
            control_event_clear(0x18);
            g_chgbuf.acc[1] = 0;
            g_chgbuf.acc[0] = 0;
            g_chg.f1 = 0;
            g_chg.f0 = 0;
            g_chg.f5 = 0;
            g_chg.f6 = 0;
            g_chg.f7 = 0;
        } else if (g_chg.f5 == 0u) {
            v = g_chg.timer;
            g_chg.timer = (uint16_t)(v + 1);
            if (0x2au < v) {
                g_chg.timer = 0;
            }
        } else {
            v = g_chg.timer;
            g_chg.timer = (uint16_t)(v + 1);
            if (0x1du < v) {
                g_chg.timer = 0;
            }
        }
        return;
    }

    /* flash_seq != 0 : relay dwell state machine */
    if ((-1 < (int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x11)) && (g_vs.flash_seq != FLASH_SEQ_LOCATE)) {
        g_vs.flash_seq = FLASH_SEQ_IDLE; /* IN2+0x3C bit14 clear & not 0xEE -> abort */
        return;
    }
    if ((g_chg.f2 == 1u) || (g_chg.f3 == 1u)) {
        g_chgbuf.count = 0;
        g_vs.flash_seq = FLASH_SEQ_IDLE;
        return;
    }
    if (g_vs.flash_seq == FLASH_SEQ_START) {
        g_chgbuf.count = 0xfa; /* 250 */
        g_vs.flash_seq = FLASH_SEQ_RUN;
    } else if (g_vs.flash_seq == FLASH_SEQ_RUN) {
        cnt = g_chgbuf.count - 1u;
        g_chgbuf.count = cnt;
        if (cnt < 0x65u) {
            if (0x3cu < cnt) {
                control_subsys_a_enable(1);
                control_subsys_b_enable(1);
                w10 = (uint16_t)(g_mode.enable_flags | 3u);
                g_mode.enable_flags = w10;
                return;
            }
            if (cnt < 0x29u) {
                if (cnt == 0u) {
                    control_subsys_a_enable(0);
                    control_subsys_b_enable(0);
                    g_vs.flash_seq = FLASH_SEQ_IDLE;
                    g_mode.enable_flags = (uint16_t)(g_mode.enable_flags & 0xfffcu);
                    return;
                }
                control_subsys_a_enable(1);
                control_subsys_b_enable(1);
                g_mode.enable_flags = (uint16_t)(g_mode.enable_flags | 3u);
                return;
            }
            control_subsys_a_enable(0);
            control_subsys_b_enable(0);
            g_mode.enable_flags = (uint16_t)(g_mode.enable_flags & 0xfffcu);
        }
    } else if (g_vs.flash_seq != FLASH_SEQ_LOCATE) {
        g_chgbuf.count = 0;
        control_subsys_a_enable(0);
        control_subsys_b_enable(0);
        g_vs.flash_seq = FLASH_SEQ_IDLE;
        g_mode.enable_flags = (uint16_t)(g_mode.enable_flags & 0xfffcu);
    } else {
        cnt = g_chgbuf.count + 1u;
        g_chgbuf.count = cnt;
        g_vs.keepalive_ticks = 200; /* VS+0x34 */
        if (0x13u < cnt) {
            if (0x27u < cnt) {
                g_chgbuf.count = 0;
                return;
            }
            control_subsys_a_enable(0);
            control_subsys_b_enable(0);
            g_mode.enable_flags = (uint16_t)(g_mode.enable_flags & 0xfffcu);
            return;
        }
        control_subsys_a_enable(1);
        control_subsys_b_enable(1);
        g_mode.enable_flags = (uint16_t)(g_mode.enable_flags | 3u);
    }
}

/* ========================================================================= */
/* Anti-theft / lock-arm / gesture FSMs (control domain).                    */
/* ========================================================================= */

/* Over-speed / lock-arm supervisor. Gated on the top-level FSM state and the secondary-lock ack
   (FM_MON_PARK code, EVT+0x50). While acking the lock it checks a cluster of guards (active error,
   bus flag, IN2+0xD6 range, the VS+0x0C arm counter, SCR+0x50, and the current ceiling vs. the
   per-mode accel base at table col MP_ACC); if any trips it HOLDS (reloads the TMR+0x14 dwell,
   requests state 1), else it raises the alarm (MODE+0x0A bit7, requests state 3). The tail runs the
   dwell countdown, packs the VS+0x0C counter into TMR+0x12 (/100, /6), steps the counter against the
   speed/accel threshold and mirrors it to SCR+0x44. */

/* Counter cap 1200, step every 3rd tick (div >= 2), packed count / 12; the SoC guards read
   STAT+0x11E. */
#define SPEED_LOCK_CTR_MAX  1200u
#define SPEED_LOCK_STEP_DIV 2u
#define SPEED_LOCK_PACK_DIV 12u
#define SPEED_LOCK_SOC_MIN  20

/** \brief Over-speed / lock-arm supervisor. */
void control_speed_lock_fsm(void) {
    uint8_t row;
    uint16_t dwell;
    int32_t accel_lim;

    if (g_state_block.fsm_state == VCU_FSM_POWER_DOWN) { /* power-down: clear and leave */
        g_tmr.lock_subdiv = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~0x80u);
        g_vs.lock_arm_ctr = 0;
        g_tmr.lock_packed_ctr = 0;
        return;
    }

    /* A live mode >= 3 is wrapped (% 3) so the index stays inside the 3-row table. */
    row = (uint8_t)(g_vs.region % VCU_MODE_ROWS);

    /* Lock-ack processing only while MODE+0x0A bit7 (alarm latch) is CLEAR. */
    if ((g_evt.mon[FM_MON_PARK].code == 1u) && ((int32_t)((uint32_t)g_mode.mode_flags << 0x18) >= 0)) {
        g_evt.mon[FM_MON_PARK].code = 0;
        if (g_vs.gear_sel != VCU_MODE_POWER) {
            control_mode_report_fsm(8); /* lighting/status 8 */
            return;
        }
        accel_lim = (int32_t)(int16_t)g_mode_table[row][MP_ACC];
        if ((g_lowglob.active_error_code != 0u)                                  /* active error code    */
            || ((int32_t)((uint32_t)g_fault_codes.comm_fault_word << 0x17) >= 0) /* A3C+0 bit8 clear     */
            || ((int32_t)(int16_t)g_in2.f.pack_temp > 0x1cb)                     /* IN2+0xD6 over range   */
            || (g_vs.lock_arm_ctr < 100u)                                        /* arm counter too low   */
            || (g_stat.soc < SPEED_LOCK_SOC_MIN)                                 /* SoC below 20 % (STAT+0x11E) */
            || (((int32_t)g_stat.curr_limit / CURR_CEIL_DIVISOR) < accel_lim)) {
            /* HOLD: reload the dwell, mark TMR+0x12, request state 1. */
            if (g_tmr.lock_ack_dwell != 0u) {
                return;
            }
            g_tmr.lock_ack_dwell = 200u;
            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~0x80u);
            g_tmr.lock_packed_ctr = (uint16_t)((g_tmr.lock_packed_ctr & 0x8001u) | 1u);
            buzzer_beep(BEEP_LONG);
            return;
        }
        /* ARM the alarm (MODE+0x0A bit7), request state 3; fall into the tail. */
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 0x80u);
        buzzer_beep(BEEP_DOUBLE);
    }

    /* --- common tail ------------------------------------------------------- */
    dwell = g_tmr.lock_ack_dwell;
    if (dwell != 0u) {
        dwell = (uint16_t)(dwell - 1u);
        g_tmr.lock_ack_dwell = dwell;
        if (dwell == 0u) {
            g_tmr.lock_packed_ctr &= 0x8000u;
        }
    } else if ((int32_t)((uint32_t)g_mode.mode_flags << 0x18) < 0) { /* MODE+0x0A bit7 set */
        uint16_t packed = g_tmr.lock_packed_ctr;
        uint16_t cnt = g_vs.lock_arm_ctr;

        packed = (uint16_t)((packed & (uint16_t)~(0x7fu << 8)) | (uint16_t)(((cnt / 100u) & 0x7fu) << 8));
        packed = (uint16_t)((packed & (uint16_t)~(0x7fu << 1))
                            | (uint16_t)(((cnt / SPEED_LOCK_PACK_DIV) & 0x7fu) << 1));
        g_tmr.lock_packed_ctr = packed;

        accel_lim = (int32_t)(int16_t)g_mode_table[row][MP_ACC];
        if (((-(int32_t)g_stat.volt_sample) / 100) >= accel_lim) {
            g_vs.lock_arm_ctr = (uint16_t)(cnt - 1u);
        }
    }

    /* Speed/accel arm-counter stepping. */
    if (((-(int32_t)g_stat.volt_sample) / 100) < (int32_t)(uint8_t)g_mode.spd_cmp) { /* strict */
        uint8_t div = g_tmr.lock_subdiv;
        g_tmr.lock_subdiv = (uint8_t)(div + 1u);
        if (div >= SPEED_LOCK_STEP_DIV) {
            uint16_t cnt = g_vs.lock_arm_ctr;
            g_tmr.lock_subdiv = 0;
            g_vs.lock_arm_ctr = (uint16_t)(cnt + 1u);
            if (cnt >= SPEED_LOCK_CTR_MAX) {
                g_vs.lock_arm_ctr = SPEED_LOCK_CTR_MAX;
            }
        }
    }

    /* Lock-release / ack reset. */
    if ((((g_vs.gear_sel != VCU_MODE_POWER) || (g_vs.lock_arm_ctr == 0u)) || (g_evt.mon[FM_MON_PARK].code == 1u)
         || (g_stat.soc < SPEED_LOCK_SOC_MIN))
        && ((int32_t)((uint32_t)g_mode.mode_flags << 0x18) < 0)) {
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~0x80u);
        g_tmr.lock_packed_ctr &= 0x8000u;
        buzzer_beep(BEEP_LONG);
        g_evt.mon[FM_MON_PARK].code = 0;
    }
    g_scr.lock_status_mirror = g_tmr.lock_packed_ctr;
}

/* Charge-complete check + beep sequence. While the SoC (STAT+0x11E) is below both 100 and the
   charge limit (low byte of STAT+0x104) and the schedule has not stopped charging (VS+0x19), it
   resets and returns. Otherwise charging is complete: SCR+0x10 = 3, VS+0x1A (charge full), one-shot
   VS+0x20 (screen refresh), clears the scheduled-charging enable (reg 0x1F b4), then runs a 6-step
   beep counter (ONESHOT+5) gated by the beep sequencer: step 0 beeps (BEEP_SHORT), steps 1..4
   advance, step 5 beeps (BEEP_LONG) then latches step 6. */

/** \brief Charge-complete check + 6-step beep counter. */
void charge_complete_tick(void) {
    char gest;

    /* the limit only completes the charge once the charger reports done
       (IN1+0x25 clear); a schedule stop completes it regardless. */
    if ((((g_stat.soc < 100) && (g_stat.soc < (int16_t)(g_stat.chg_limit & 0xffu))) || (g_in1.charger_connected != 0u))
        && (g_vs.chg_sched_stop == 0u)) {
        g_oneshot.beep_counter = 0;
        g_oneshot.report_latch6 = 0;
        g_vs.chg_full = 0;
        return;
    }

    g_scr.drive_oneshot = 3;
    g_vs.chg_full = 1;
    if (g_oneshot.report_latch6 == 0u) {
        g_oneshot.report_latch6 = 1;
        g_vs.chg_disp_refresh_req = 1;
    }
    if ((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1b) < 0) { /* IN2+0x3E bit4 */
        g_in2.f.chg_cfg &= 0xffefu;
    }

    gest = (char)g_lowglob.beep_seq_id;
    switch (g_oneshot.beep_counter) {
        case 0:
            if (gest == BEEP_NONE) {
                buzzer_beep(BEEP_SHORT);
                g_oneshot.beep_counter = 1;
            }
            break;
        case 1:
        case 2:
        case 3:
        case 4:
            if (gest == BEEP_NONE) {
                g_oneshot.beep_counter = (uint8_t)(g_oneshot.beep_counter + 1u);
                return;
            }
            break;
        case 5:
            if (gest != BEEP_NONE) {
                return;
            }
            buzzer_beep(BEEP_LONG);
            g_oneshot.beep_counter = 6;
            return;
        default: break;
    }
}

/* ------------------------------------------------------------------------- */
/* Packed charge-control report word.                                        */
/* Bits 0..5 = IN2+0x96 low6, 6..10 = IN2+0x96 high5, 11..16 = IN2+0x98 low6,   */
/* 17..21 = IN2+0x98 high5, bit22 = reg 0x1F bit4, bit23 = bit5, bit24 = bit7.  */
/* Upper bits are don't-care.                                                */
/* ------------------------------------------------------------------------- */
static uint32_t control_pack_lock_report(void) {
    /* The schedule window regs 0x4B / 0x4C and reg 0x1F, all in IN2. */
    uint16_t a = g_in2.f.chg_sched_start;
    uint16_t b = g_in2.f.chg_sched_end;
    uint16_t f = g_in2.f.chg_cfg;
    uint32_t w = 0;

    w |= (uint32_t)(a & 0x3fu);
    w |= (uint32_t)((a >> 8) & 0x1fu) << 6;
    w |= (uint32_t)(b & 0x3fu) << 11;
    w |= (uint32_t)((b >> 8) & 0x1fu) << 17;
    if ((f & 0x80u) != 0u) {
        w |= 0x1000000u;
    }
    if ((f & 0x10u) != 0u) {
        w |= 0x400000u;
    }
    if ((f & 0x20u) != 0u) {
        w |= 0x800000u;
    }
    return w;
}

/* Charger detect + charge-control report.
   Detect half: reads the BMS status words (STAT+0x124 bit6, STAT+0x126 bit3 = charger, CAN 0x422).
   A charger sets charger_connected (IN1+0x25) and SF_CHARGING, beeps once per session and arms a
   scheduled charge (reg 0x1F b4); it also maintains the charger-seen bit (b6) and cancels the
   schedule when the charger goes away.
   Periodic half (10-tick divider, ONESHOT+1): pushes the BMS charge-control register (op 4 arg 0x87
   to node 7) on request, shows the schedule-cancel notice (screen 0x13 + beep), keeps the VCU awake
   for a scheduled charge below the limit, and runs the schedule window. */

#define CHG_SCREEN_CHARGING        18u /*!< SCR+0x4A dashboard screen id: charging              */
#define CHG_NOTICE_PERIODS         60u /*!< schedule-cancel notice length, report periods       */
#define CHG_NOTICE_PERIODS_CHARGER 30u /*!< ... with a charger connected                         */

/**
 * \brief           Charger detect + charge-control report
 * \note            The schedule stop is dropped on every call without the BMS charger bit;
 *                      the schedule-cancel notice lasts 60 report periods, 30 with a charger
 *                      connected. The report word and the keyhold / schedule bits are IN2
 *                      (regs 0x4B / 0x4C / 0x1F), the screen id is SCR+0x4A.
 */
void charger_detect_tick(void) {
    uint32_t report;
    uint8_t div;

    /* ---- gating half ---------------------------------------------------- */
    if (((int32_t)((uint32_t)g_stat.status_word_a << 0x19) < 0)    /* STAT+0x124 bit6 */
        && ((int32_t)((uint32_t)g_stat.status_word_b << 0x1c) < 0) /* STAT+0x126 bit3 */
        && (g_vs.chg_sched_stop == 0u)) {
        g_in1.charger_connected = 1;
        g_in2.f.status_flags |= SF_CHARGING;
        if ((g_in2.f.status_flags & SF_SESSION) != 0u) {
            if (g_oneshot.report_latch4 == 0u) {
                buzzer_beep(BEEP_DOUBLE);
                g_oneshot.report_latch4 = 1;
                if ((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1b) < 0) { /* chg_cfg bit4 */
                    g_in1.chg_sched_armed = 1;
                    g_vs.chirp_req = 1;
                    g_vs.chg_disp_refresh_req = 1;
                }
            }
            g_scr.drive_oneshot = 1;
        }
    } else if ((int32_t)((uint32_t)g_stat.status_word_a << 0x19) >= 0) { /* STAT+0x124 bit6 clear */
        g_oneshot.report_latch4 = 0;
        g_in1.charger_connected = 0;
        g_scr.drive_oneshot = 0;
        g_in2.f.status_flags &= 0xfffbu;
    }

    if ((int32_t)((uint32_t)g_stat.status_word_b << 0x1c) < 0) { /* STAT+0x126 bit3 set */
        g_in2.f.chg_cfg |= 0x40u;
    } else {
        if (((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x19) < 0)       /* chg_cfg bit6 */
            && ((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1b) < 0)) { /* chg_cfg bit4 */
            g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
            g_vs.chg_sched_cancel_notice = 1;
            g_in2.f.chg_cfg &= 0xffefu;
        }
        g_vs.chg_sched_stop = 0; /* on every call without the charger bit */
        g_in2.f.chg_cfg &= 0xffbfu;
    }

    /* ---- periodic report half (10-tick divider) ------------------------- */
    div = g_oneshot.report_divider;
    g_oneshot.report_divider = (uint8_t)(div + 1u);
    if (div < 10u) {
        return;
    }
    g_oneshot.report_divider = 0;

    if (g_in1.drive_active == 0u) {
        if (g_in1.bms_chg_cmd_req == BMS_CHG_CMD_SEND) {
            g_oneshot.mode_hold_countdown = 0x1e;
            report = control_pack_lock_report();
            report_ninebot(NB_ADDR_VCU, 7, 4, 2, 0x87, &report);
            if ((g_scr.ui_screen_u == CHG_SCREEN_CHARGING) && (g_vs.chg_sched_cancel_notice == 0u)) { /* SCR+0x4A */
                g_vs.chg_disp_refresh_req = 1;
            }
        } else if (g_in1.bms_chg_cmd_req == BMS_CHG_CMD_ACKED) {
            g_oneshot.mode_hold_countdown = (uint8_t)(g_oneshot.mode_hold_countdown - 1u);
            if (g_oneshot.mode_hold_countdown == 0u) {
                g_in1.bms_chg_cmd_req = BMS_CHG_CMD_IDLE;
            }
        }

        if ((g_in2.f.chg_cfg & CHG_KEYHOLD) != 0u) { /* IN2+0x3E bit7 */
            g_oneshot.mode_apply_cnt = 0;
            report = control_pack_lock_report();
            report_ninebot(0x3e, 7, 4, NB_CMD_WRITE3, 0x87, &report);
            g_in2.f.chg_cfg &= 0xff7fu;
            g_vs.chg_sched_cancel_notice = 1;
            g_vs.chg_sched_stop = 0;
        }

        if (g_vs.chg_sched_cancel_notice != 0u) {
            if (g_oneshot.mode_apply_cnt == 0u) {
                g_oneshot.stored_mode_shadow = g_scr.ui_screen_u; /* SCR+0x4A */
                nb_commit_mode(0x13);
                buzzer_beep(BEEP_LONG);
            }
            g_oneshot.mode_apply_cnt = (uint8_t)(g_oneshot.mode_apply_cnt + 1u);
            /* 60 periods, 30 with a charger connected */
            if ((g_oneshot.mode_apply_cnt >= CHG_NOTICE_PERIODS)
                || ((g_in1.charger_connected == 1u) && (g_oneshot.mode_apply_cnt >= CHG_NOTICE_PERIODS_CHARGER))) {
                g_oneshot.mode_apply_cnt = 0;
                g_vs.chg_sched_cancel_notice = 0;
                g_in2.f.chg_cfg &= 0xffefu;
                if ((int32_t)((uint32_t)g_stat.status_word_b << 0x1c) < 0) { /* STAT+0x126 bit3 */
                    g_vs.chg_disp_refresh_req = 1;
                } else if ((g_oneshot.stored_mode_shadow != 3u)
                           && (g_oneshot.stored_mode_shadow != CHG_SCREEN_CHARGING)) {
                    nb_commit_mode((int16_t)g_oneshot.stored_mode_shadow);
                }
            }
        }

        if (((g_in2.f.chg_cfg & CHG_SCHED_EN) != 0u)                      /* IN2+0x3E bit4 */
            && ((int32_t)((uint32_t)g_stat.status_word_b << 0x1c) < 0)) { /* STAT+0x126 bit3 */
            g_vs.chg_keep_awake = 1;
            if ((g_stat.soc > 99) || ((int16_t)(g_stat.chg_limit & 0xffu) <= g_stat.soc)) {
                g_vs.chg_keep_awake = 0;
            }
        } else {
            g_vs.chg_keep_awake = 0;
        }
        charge_schedule_tick();
    } else if (g_vs.chg_sched_cancel_notice != 0u) {
        uint8_t s2 = (uint8_t)(g_oneshot.mode_apply_cnt + 1u);
        g_oneshot.mode_apply_cnt = s2;
        if (s2 > 0x13u) {
            g_oneshot.mode_apply_cnt = 0;
            nb_commit_mode((int16_t)g_oneshot.stored_mode_shadow);
        }
    }
}

/* Charge-detect state reset: turn the charge indicator off (control_mode_sync) and clear the
   charge-detect flags (CHG timer / present A / present B), the scratch block and the relay FSM
   state (VS->flash_seq). */

/** \brief Charge-detect / relay state reset. */
void control_charge_state_reset(void) {
    control_mode_sync();
    g_chg.timer = 0;
    g_chg.f4 = 0;
    g_chg.f3 = 0;
    g_chg.f2 = 0;
    g_evt.mon[FM_MON_LIGHT_HI].code = 0;
    g_evt.mon[FM_MON_LIGHT_LO].code = 0;
    g_vs.flash_seq = FLASH_SEQ_IDLE;
}

/* Timed button / gesture beep FSM. While a charger is present (CHG present A/B) and the trigger bit
   is set (IN2->cfg_flags bit11) it is held reset. Otherwise g_lowglob.beep_seq_id selects a timed
   beep pattern; each switches the buzzer on/off at fixed dwell marks, then resets to BEEP_NONE. */

/** \brief Timed button/gesture beep FSM. */
void control_gesture_fsm(void) {
    uint16_t t;

    if (((g_chg.f3 == 1u) || (g_chg.f2 == 1u)) && ((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x14) < 0)) { /* cfg_flags
                                                                                                             bit11 */
        g_lowglob.beep_seq_id = BEEP_NONE;
        g_lowglob.beep_seq_tick = 0;
        return;
    }

    t = g_lowglob.beep_seq_tick;
    switch (g_lowglob.beep_seq_id) {
        case BEEP_NONE: break;
        case BEEP_LONG:
            if (t == 0u) {
                buzzer_on(BUZZER_TONE);
            } else if (t > 0x32u) {
                g_lowglob.beep_seq_tick = 0;
                g_lowglob.beep_seq_id = BEEP_NONE;
                buzzer_off();
            }
            g_lowglob.beep_seq_tick = (uint16_t)(g_lowglob.beep_seq_tick + 1u);
            return;
        case BEEP_SHORT:
            if (t == 0u) {
                buzzer_on(BUZZER_TONE);
            } else if (t > 9u) {
                g_lowglob.beep_seq_id = BEEP_NONE;
                buzzer_off();
            }
            g_lowglob.beep_seq_tick = (uint16_t)(g_lowglob.beep_seq_tick + 1u);
            return;
        case BEEP_DOUBLE:
            if (t == 0u) {
                buzzer_on(BUZZER_TONE);
            } else if (t == 10u) {
                buzzer_off();
            } else if (t == 0x14u) {
                buzzer_on(BUZZER_TONE);
            } else if (t > 0x1du) {
                g_lowglob.beep_seq_id = BEEP_NONE;
                buzzer_off();
            }
            g_lowglob.beep_seq_tick = (uint16_t)(g_lowglob.beep_seq_tick + 1u);
            return;
        case BEEP_TRIPLE:
            if (t == 0u) {
                buzzer_on(BUZZER_TONE);
            } else if (t == 10u) {
                buzzer_off();
            } else if (t == 0x14u) {
                buzzer_on(BUZZER_TONE);
            } else if (t == 0x1eu) {
                buzzer_off();
            } else if (t == 0x28u) {
                buzzer_on(BUZZER_TONE);
            } else if (t > 0x31u) {
                g_lowglob.beep_seq_id = BEEP_NONE;
                buzzer_off();
            }
            g_lowglob.beep_seq_tick = (uint16_t)(g_lowglob.beep_seq_tick + 1u);
            return;
        case BEEP_DOUBLE_LONG:
            if (t == 0u) {
                buzzer_on(BUZZER_TONE);
            } else if (t == 0x32u) {
                buzzer_off();
            } else if (t == 100u) {
                buzzer_on(BUZZER_TONE);
            } else if (t > 0x95u) {
                g_lowglob.beep_seq_id = BEEP_NONE;
                buzzer_off();
            }
            g_lowglob.beep_seq_tick = (uint16_t)(g_lowglob.beep_seq_tick + 1u);
            return;
        case BEEP_STOP:
            g_lowglob.beep_seq_id = BEEP_NONE;
            buzzer_off();
            return;
        default: break;
    }
}

/* Deferred-action trigger: if the request flag (IN1+0x10) is set, clear it and run the dual-bank
   NVM commit (config.c identity_store). */

/** \brief Deferred-action trigger. */
void control_deferred_action(void) {
    if (g_in1.recompute_req != 0u) {
        g_in1.recompute_req = 0;
        identity_store((identity_rec_t*)config_identity()); /* dual-bank NVM commit */
    }
}

/* ========================================================================= */
/* Control-domain FSMs.                                                      */
/* ========================================================================= */

/* Raw 12-bit ADC snapshot (3 halfwords), module-local scratch like g_adc_dma_buf. */
static volatile uint16_t g_adc_snapshot[3];

/* Brake cap: speed_src above this (0.1 km/h) gets at least brake 15. */
#define SCAN_BRAKE_CAP_SPEED 150
#define SCAN_BRAKE_CAP       15u

/* 16-sample throttle average: byte ring + index. */
#define THROTTLE_AVG_LEN     16u
static volatile uint8_t g_throttle_avg_ring[THROTTLE_AVG_LEN];
static volatile uint8_t g_throttle_avg_idx;

/**
 * \brief           Push a throttle sample into the 16-sample ring and return the ring average
 * \param[in]       sample: throttle command byte
 * \return          (sum of the 16 entries) >> 4, low byte
 * \note            `ring[idx] = b; idx++; if (idx >= 16) idx = 0; sum (u16) of all 16;
 *                      return (sum >> 4) & 0xff`.
 */
static uint8_t control_throttle_avg16_push(uint8_t sample) {
    uint16_t sum = 0u;
    uint8_t idx;

    g_throttle_avg_ring[g_throttle_avg_idx] = sample;
    g_throttle_avg_idx = (uint8_t)(g_throttle_avg_idx + 1u);
    if (g_throttle_avg_idx >= THROTTLE_AVG_LEN) {
        g_throttle_avg_idx = 0u;
    }
    for (idx = 0u; idx < THROTTLE_AVG_LEN; ++idx) {
        sum = (uint16_t)(sum + g_throttle_avg_ring[idx]);
    }
    return (uint8_t)(sum >> 4);
}

/* Aux GPIO driven by the debounce FSM (GPIOF on the AT32F415, 0x40011C00; mask 0x40 = */
/* bit 6 / PF6). */
#define CTRL_GPIO_AUX     GPIOF /* 0x40011C00 */
#define CTRL_GPIO_AUX_BIT 0x40u

/* Conditioned-input scaling constants (raw 8-bit ADC codes). */
#define ADC_CMD_FULL      200u /* saturated command magnitude (0..200)        */
#define ADC_CMD_LO        50u  /* scale floor (below -> command 0)            */
#define ADC_CMD_HI        155u /* scale ceiling (at/above -> full)            */
#define ADC_CMD_SPAN      105u /* scaling denominator (ADC_CMD_HI-LO)         */

/* Conditioned analog front-end. Reads 3 ADC channels (throttle PA0, full-weight brake lever PA4,
   half-weight brake lever PA1), stores a raw snapshot, then runs per-channel range/fault latches
   (calibration branch when fsm_state == 1 (power-on) and SF_SESSION is set, else the normal in-range
   debounce) and scales each channel to a 0..200 command. The throttle lands in the MODE+8 low byte,
   max(PA4, PA1/2) in the high byte; config bits (MODE+0x15) and the charge gate (IN2+0x3C) then
   post-gate the word, and any latched channel fault forces the whole word to 0. */

/** \brief 3-channel ADC conditioning + output word. */
void control_throttle_brake_scan(uint8_t fsm_state) {
    uint32_t thr = control_adc_raw(VCU_ADC_SLOT_THROTTLE) >> 4;        /* PA0 throttle                */
    uint32_t brk_full = control_adc_raw(VCU_ADC_SLOT_BRAKE_FULL) >> 4; /* PA4 full-weight brake lever */
    uint32_t brk_half = control_adc_raw(VCU_ADC_SLOT_BRAKE_HALF) >> 4; /* PA1 half-weight brake lever */
    uint32_t out_full;
    uint32_t out_half;
    char prev_full;
    char prev_half;

    /* Full-resolution raw snapshot ([0] PA0, [1] PA4, [2] PA1). */
    g_adc_snapshot[0] = (uint16_t)control_adc_raw(VCU_ADC_SLOT_THROTTLE);
    g_adc_snapshot[1] = (uint16_t)control_adc_raw(VCU_ADC_SLOT_BRAKE_FULL);
    g_adc_snapshot[2] = (uint16_t)control_adc_raw(VCU_ADC_SLOT_BRAKE_HALF);

    if ((fsm_state == VCU_FSM_POWER_ON) && ((g_in2.f.status_flags & SF_SESSION) != 0u)) {
        /* Calibration branch: latch a channel calibration guard when out of the
           dead-band, else clear its out-of-band latch. */
        if ((thr - 0x19u) < 0x1au) {
            g_in1.throttle_fault_latch = 0;
        } else {
            g_in1.throttle_stuck_guard = 1;
        }
        if ((brk_full - 0x19u) < 0x1au) {
            g_in1.brake_full_range_fault = 0;
        } else {
            g_in1.brake_full_cal_guard = 1;
        }
        if ((brk_half - 0x19u) < 0x1au) {
            g_in1.brake_half_range_fault = 0;
        } else {
            g_in1.brake_half_cal_guard = 1;
        }
    } else {
        /* Throttle (PA0) out-of-band latch, IN1+0x1E. */
        if (g_in1.throttle_stuck_guard == 0u) {
            if (g_in2.f.throttle_cal_sel == 1) {
                g_in1.throttle_fault_latch = (uint8_t)(((thr - 0x19u) < 0xa1u) ? 0u : 1u);
            } else {
                g_in1.throttle_fault_latch = (uint8_t)(((thr - 0x19u) < 0xc4u) ? 0u : 1u);
            }
        } else {
            g_in1.throttle_fault_latch = 1;
            if ((thr - 0x1au) < 0x18u) {
                g_in1.throttle_stuck_guard = 0;
            }
        }
        /* Full-weight brake lever (PA4) out-of-band latch, IN1+0x1F. */
        if (g_in1.brake_full_cal_guard == 0u) {
            g_in1.brake_full_range_fault = (uint8_t)(((brk_full - 0x19u) < 0xc4u) ? 0u : 1u);
        } else {
            g_in1.brake_full_range_fault = 1;
            if ((brk_full - 0x1au) < 0x18u) {
                g_in1.brake_full_cal_guard = 0;
            }
        }
        /* Half-weight brake lever (PA1) out-of-band latch, IN1+0x2C. */
        if (g_in1.brake_half_cal_guard == 0u) {
            g_in1.brake_half_range_fault = (uint8_t)(((brk_half - 0x19u) < 0xc4u) ? 0u : 1u);
        } else {
            g_in1.brake_half_range_fault = 1;
            if ((brk_half - 0x1au) < 0x18u) {
                g_in1.brake_half_cal_guard = 0;
            }
        }
    }

    /* Throttle (PA0) -> MODE+8 low byte, active flag at g_tmr+3. */
    if (thr < ADC_CMD_HI) {
        if ((thr - ADC_CMD_LO) < ADC_CMD_SPAN) {
            g_tmr.throttle_active = 1;
            g_mode.drive_output_word = (uint16_t)((g_mode.drive_output_word & 0xff00u)
                                                  | ((uint16_t)((int32_t)((thr - ADC_CMD_LO) * ADC_CMD_FULL)
                                                                / (int32_t)ADC_CMD_SPAN)
                                                     & 0xffu));
        } else {
            g_tmr.throttle_active = 0;
            g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xff00u);
        }
    } else {
        g_tmr.throttle_active = 1;
        g_mode.drive_output_word = (uint16_t)((g_mode.drive_output_word & 0xff00u) + ADC_CMD_FULL);
    }

    /* Full-weight brake lever (PA4) command, active flag at g_tmr+1. */
    if (brk_full < ADC_CMD_HI) {
        if ((brk_full - ADC_CMD_LO) < ADC_CMD_SPAN) {
            g_tmr.brake_full_active = 1;
            out_full = (uint32_t)((int32_t)((brk_full - ADC_CMD_LO) * ADC_CMD_FULL) / (int32_t)ADC_CMD_SPAN) & 0xffffu;
        } else {
            g_tmr.brake_full_active = 0;
            out_full = 0u;
        }
    } else {
        g_tmr.brake_full_active = 1;
        out_full = ADC_CMD_FULL;
    }

    /* Half-weight brake lever (PA1) command, active flag at g_tmr+2. */
    if (brk_half < ADC_CMD_HI) {
        if ((brk_half - ADC_CMD_LO) < ADC_CMD_SPAN) {
            g_tmr.brake_half_active = 1;
            out_half = (uint32_t)((int32_t)((brk_half - ADC_CMD_LO) * ADC_CMD_FULL) / (int32_t)ADC_CMD_SPAN) & 0xffffu;
        } else {
            g_tmr.brake_half_active = 0;
            out_half = 0u;
        }
    } else {
        g_tmr.brake_half_active = 1;
        out_half = ADC_CMD_FULL;
    }

    /* MODE+8 high byte (brake) = max(PA4, PA1/2). */
    if (out_full <= (out_half >> 1)) {
        out_full = out_half >> 1;
    }
    g_mode.drive_output_word = (uint16_t)((g_mode.drive_output_word & 0xffu) | ((uint16_t)(out_full & 0xffu) << 8));

    /* Edge-detect brake lever / throttle activity; refresh IN1+0x0B (any brake lever)
       and IN1+0x0C (throttle active) + the CAN drive latch. */
    prev_full = (char)g_tmr.brake_full_active;
    if ((prev_full != (char)g_tmr.edge_prev_brake_full)
        || ((char)g_tmr.brake_half_active != (char)g_tmr.edge_prev_brake_half)
        || ((char)g_in1.throttle_flag != (char)g_tmr.throttle_active)) {
        g_in1.throttle_flag = g_tmr.throttle_active;
        g_tmr.edge_prev_brake_full = (uint8_t)prev_full;
        prev_half = (char)g_tmr.brake_half_active;
        g_tmr.edge_prev_brake_half = (uint8_t)prev_half;
        g_vs.can_drive_active = 1;
        g_in1.brake_any = (uint8_t)((prev_full == 0 && prev_half == 0) ? 0u : 1u);
    }

    /* Post-gate the output word on the config byte (MODE+0x17) and charge gate. */
    if ((g_mode.output_cfg & OCFG_BRAKE_CAP) != 0u) {
        /* The ride assists reset, throttle off, and at least brake 15 above 15.0 km/h. */
        ride_assist_reset_all();
        g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xff00u);
        if (((int16_t)g_in2.f.speed_src > SCAN_BRAKE_CAP_SPEED) && ((g_mode.drive_output_word >> 8) < SCAN_BRAKE_CAP)) {
            g_mode.drive_output_word = (uint16_t)((g_mode.drive_output_word & 0xffu) + (SCAN_BRAKE_CAP << 8));
        }
    } else {
        /* Every scan feeds the 16-sample throttle average; with the wobble warning up the
           throttle takes a second sample's average. */
        (void)control_throttle_avg16_push((uint8_t)g_mode.drive_output_word);
        if ((g_mode.output_cfg & OCFG_WOBBLE_ALERT) != 0u) {
            g_mode.drive_output_word = (uint16_t)((g_mode.drive_output_word & 0xff00u)
                                                  | control_throttle_avg16_push((uint8_t)g_mode.drive_output_word));
        }
        if (-1 < (int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x14)) { /* IN2+0x3C bit11 clear */
            g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xffu);
        }
        /* Throttle re-arm latch (ride-assist `throttle_rearm`, no writer in this firmware): hold the throttle
           off until it is released once. */
        if (ride_assist_state()->throttle_rearm != 0u) {
            if ((g_mode.drive_output_word & 0xffu) == 0u) {
                ride_assist_throttle_rearm_clear();
            } else {
                g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xff00u);
            }
        }
    }
    if (fsm_state == VCU_FSM_CHARGING) { /* charger on: throttle off */
        g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xff00u);
    }
    if ((g_in1.throttle_fault_latch != 0u) || (g_in1.brake_half_range_fault != 0u)
        || (g_in1.brake_full_range_fault != 0u)) {
        g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xff00u);
        g_mode.drive_output_word = (uint16_t)(g_mode.drive_output_word & 0xffu);
    }
}

/* Current/charge power monitor + comm-timeout FSM. Computes instantaneous power (I*V/100), stores
   max(SoC, 1) into g_scr+0x50 (mirrored to IN2+0xAA), and -- only in ride (FSM state 3) -- runs the
   600-tick comm-fault dwell: raise/clear event 1, chirp + status report when the dwell opens, and
   set the VS+0x1C power-off request at expiry. */

/** \brief Power calc + comm-timeout state machine. */
void control_power_monitor_fsm(void) {
    int32_t power = ((int32_t)g_stat.curr_sample * (int32_t)g_stat.volt_sample) / 100;
    uint16_t t;

    vcu_can_battery2_presence_tick(); /* first call of the power monitor */

    /* SCR+0x50 = max(STAT+0x11E, 1), no second-pack average. */
    if (g_stat.soc < 1) {
        g_scr.soc_combined = 1u;
    } else {
        g_scr.soc_combined = (uint16_t)g_stat.soc;
    }
    g_in2.f.soc = g_scr.soc_combined;
    g_scr.soc_report = g_in2.f.soc;

    if (g_state_block.fsm_state == VCU_FSM_RIDE) {
        if (power < 1) {
            power = -power;
            g_scr.power_sign = 0;
        } else {
            g_scr.power_sign = 1;
        }
        g_scr.power_half = (int16_t)(power / 2000);
        if (100u < g_scr.power_half) {
            g_scr.power_half = 100;
        }
    } else {
        g_scr.power_half = 0;
    }

    if (g_state_block.fsm_state != VCU_FSM_RIDE) {
        g_oneshot.commto_tick = 0;
        control_event_clear(1);
        return;
    }
    if (g_oneshot.commto_tick < 0x14u) {
        g_oneshot.commto_tick = (uint16_t)(g_oneshot.commto_tick + 1u);
        g_oneshot.chg_power_shadow = (uint8_t)g_stat.soc; /* STAT+0x11E */
    } else {
        if ((g_stat.soc < 6) && (5u < g_oneshot.chg_power_shadow)) {
            g_oneshot.commto_dwell = 0;
            buzzer_beep(BEEP_DOUBLE_LONG);
            control_mode_report_fsm(0x10);
        }
        g_oneshot.chg_power_shadow = (uint8_t)g_stat.soc;
        if ((int32_t)((uint32_t)g_fault_codes.comm_fault << 0x1e) < 0) { /* comm_fault bit1 */
            if ((int16_t)g_oneshot.commto_dwell == 0) {
                g_oneshot.commto_dwell = 600;
            }
            control_event_set(1);
        } else {
            g_oneshot.commto_dwell = 0;
            g_scr.commto_mirror = 0;
            control_event_clear(1);
        }
        if ((int16_t)g_oneshot.commto_dwell != 0) {
            t = (uint16_t)((int16_t)g_oneshot.commto_dwell - 1);
            g_oneshot.commto_dwell = t;
            g_scr.commto_mirror = (uint16_t)(t / 100u);
            if (t < 100u) {
                g_oneshot.commto_dwell = 0;
                g_vs.shutdown_req = 1;
                return;
            }
        }
    }
}

/* Auto-off countdown. At standstill with no re-arm pending: a power-button long press
   (FM_MON_POWER_BTN code == 3) or a tripped tilt latch (VS+6) requests shutdown (VS+0x1C) at once;
   otherwise a VS+0x3C countdown runs from `reload`, chirps near expiry (while a drive-fault request
   is active and the count is small), and requests shutdown at zero. The guarded-out path re-arms the
   countdown and consumes a pending long press. */

/** \brief Standstill auto-off countdown (shutdown request at 0). */
void control_auto_off_tick(uint32_t reload) {
    if ((g_vs.speed_kmh < VCU_STANDSTILL_KMH) && (g_vs.can_drive_active == 0u)) {
        if (g_evt.mon[FM_MON_POWER_BTN].code == 3u) { /* power-button long press */
            g_evt.mon[FM_MON_POWER_BTN].code = 0;
            g_vs.shutdown_req = 1;
            return;
        }
        if (g_vs.tilt_latch == 1u) {
            g_vs.creep_enable = 0;
            g_vs.tilt_latch = 0;
            g_vs.shutdown_req = 1;
            return;
        }
        if (g_vs.auto_off_ticks == 0u) {
            g_vs.auto_off_ticks = reload;
            return;
        }
        g_vs.auto_off_ticks = g_vs.auto_off_ticks - 1u;
        if ((g_scr.drive_fault_req != 0) && (g_vs.auto_off_ticks < 0x1771u) && (g_vs.auto_off_ticks < 100u)) {
            buzzer_beep(BEEP_SHORT);
        }
        if (g_vs.auto_off_ticks == 0u) {
            g_vs.shutdown_req = 1;
            return;
        }
    } else {
        g_vs.can_drive_active = 0;
        g_vs.auto_off_ticks = reload;
        if (g_evt.mon[FM_MON_POWER_BTN].code == 3u) {
            g_evt.mon[FM_MON_POWER_BTN].code = 0;
        }
    }
}

/* ESC staging-frame boot defaults (g_mode). Net effect on the staging words:
     +0x0A mode_flags   : clear bits 0..9 and 11, set bit 14
     +0x0C curr_word    : -> 0x1400
     +0x0E we_word      : -> 0x5A00
     +0x10 enable_flags : set bits 14 and 12, clear bits 0..11 */
#define STG_MODE_FLAGS_CLR   0x0BFFu /* mode_flags bits 0..9 + 11 cleared     */
#define STG_MODE_FLAGS_SET   0x4000u /* mode_flags bit 14 set                 */
#define STG_CURR_WORD_DEF    0x1400u /* curr_word final value                */
#define STG_WE_WORD_DEF      0x5A00u /* we_word final value                   */
#define STG_ENABLE_FLAGS_CLR 0x0FFFu /* enable_flags bits 0..11 cleared       */
#define STG_ENABLE_FLAGS_SET 0x5000u /* enable_flags bits 14 + 12 set         */
#define STG_REGEN_DEF        45u     /* regen byte (staging +0x0E)            */

/** \brief Seed the ESC staging frame (g_mode) defaults at boot. */
void control_esc_staging_defaults(void) {
    g_mode.mode_flags = (uint16_t)((g_mode.mode_flags & ~STG_MODE_FLAGS_CLR) | STG_MODE_FLAGS_SET);
    g_mode.enable_flags = (uint16_t)((g_mode.enable_flags | STG_ENABLE_FLAGS_SET) & ~STG_ENABLE_FLAGS_CLR);
    g_mode.curr_word = STG_CURR_WORD_DEF;
    g_mode.we_word = STG_WE_WORD_DEF;
    g_mode.cfg_17 = (uint8_t)(g_mode.cfg_17 & (uint8_t)~CFG17_DRIVE_FAULT);
    g_mode.regen_level = STG_REGEN_DEF;
}

/* Mirror IN2+0x3A (cfg_flags) bit0 into MODE+0x0A (mode_flags) bit0. */

/** \brief Mirror a peripheral bit into the mode word. */
void control_status_bit_mirror(void) {
    /* not while the climb boost is enabled (reg 0x1E bit2) */
    if (((g_in2.f.cfg_flags & 1u) != 0u) && ((g_in2.f.cfg_flags2 & CFG2_CLIMB_BOOST) == 0u)) {
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 1u);
    } else {
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & 0xfffeu);
    }
}

/* One-shot shutdown countdown: count VS+0x3C down from `reload`, drive the FSM to state 4 at expiry. */

/** \brief One-shot down-counter -> FSM state 4 (power-down). */
void control_shutdown_countdown(uint32_t reload) {
    if (g_vs.auto_off_ticks != 0u) {
        g_vs.auto_off_ticks = g_vs.auto_off_ticks - 1u;
        if (g_vs.auto_off_ticks == 0u) {
            g_state_block.fsm_state = (uint8_t)VCU_FSM_POWER_DOWN;
        }
        return;
    }
    g_vs.auto_off_ticks = reload;
}

/* Event-mask / DTC-mask resets. Two siblings at different offsets (not an alias): the first clears
   the active-event byte + IN2+0xB2 + both event masks; the second clears the active-error byte +
   IN2 lock_state + all three DTC masks. */

/** \brief Clear the active-event + event masks. */
void control_event_mask_reset(void) {
    g_lowglob.active_event_id = 0; /* active-event byte */
    g_in2.f.event_ack_reset = 0;
    g_scr.event_mask_lo = 0;
    g_scr.event_mask_hi = 0;
}

/** \brief Clear the active-error + DTC masks. */
void control_dtc_mask_reset(void) {
    g_lowglob.active_error_code = 0; /* active-error byte */
    g_in2.f.lock_state = 0;          /* IN2+0xB0 */
    g_scr.dtc_mask0 = 0;
    g_scr.dtc_mask1 = 0;
    g_scr.dtc_mask2 = 0;
}

/* Diagnostic/session reset: run vcu_outputs_reset, then clear VS+5 unless it is pinned to 3. */

/** \brief Reset wrapper. */
void control_diag_reset(void) {
    vcu_outputs_reset();
    if (g_vs.substate != 3u) {
        g_vs.substate = 0;
    }
}

/* Mode sync: drop both subsystem outputs (PB10 / PB0), then publish subsystem mask 0 via
   report_subsys_mask_set (MODE+0x10 low bits, g_scr.subsys_enable; reports only on a change). */

/** \brief Subsystems A/B off + mask-0 report on change. */
void control_mode_sync(void) {
    control_subsys_a_enable(0);
    control_subsys_b_enable(0);
    report_subsys_mask_set(0u);
}

/* Debounced aux-GPIO drive: while armed (VS+0x29), hold the aux GPIO bit set for the first 10 ticks
   (counter VS+0x2E), then clear it and disarm. */

/** \brief 10-tick debounce then aux-GPIO drive. */
void control_debounced_gpio_drive(void) {
    uint16_t n;

    if (g_vs.chirp_req != 0u) {
        n = g_vs.chirp_tick;
        g_vs.chirp_tick = (uint16_t)(n + 1u);
        if (n < 10u) {
            gpio_bits_set(CTRL_GPIO_AUX, CTRL_GPIO_AUX_BIT);
            return;
        }
        gpio_bits_reset(CTRL_GPIO_AUX, CTRL_GPIO_AUX_BIT);
        g_vs.chirp_tick = 0;
        g_vs.chirp_req = 0;
    }
}

/* ========================================================================= */
/* Drive-parameter initializer + anti-theft motion check FSM.                */
/* ========================================================================= */

/* control_drive_param_init (called from ninebot.c nb_drive_recompute_b) seeds the per-mode
   speed/limit profile registers (+0x84..+0x90, incl. +0x8E/+0x90); control_build_drive_cmd then
   selects and corrects the active ceiling from them. It walks g_in2 as three ctrl-status bitfield
   regs (cfg_flags/cfg_flags2/chg_cfg @ +0x3a/+0x3c/+0x3e), a cleared work area, the alarm-sensitivity
   selector (+0xE8, read by the motion alarm FSM), and a 7-halfword profile table (+0x84..+0x90)
   chosen by the region (VS->region). In that table +0x84 aliases in2.fault_code (ceiling seed),
   +0x8C aliases in2.speed_limit, and +0x8E/+0x90 hold the stored per-mode ceilings. */

/**
 * \brief           Seed the drive-parameter / ctrl-status block + profile table
 * \note            Called from ninebot.c `nb_drive_recompute_b`. The long per-bit AND/OR
 *                      runs on the three ctrl-status registers are a bitfield-struct init
 *                      written field-by-field (reg A nets bit4 + bit11 set, all else clear).
 *                      Calls `report_ninebot` to publish the seeded report cell.
 */
void control_drive_param_init(void) {
    volatile uint16_t* report; /* ninebot report payload cell g_stat.chg_limit */

    /* ctrl-status reg A (+0x3a): per-bit clear/set from a bitfield init. */
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfffeu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfffdu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfffbu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfff7u);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags | 0x0010u);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xffdfu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xffbfu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xff7fu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfeffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfdffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xfbffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags | 0x0800u);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xefffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xdfffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0xbfffu);
    g_in2.f.cfg_flags = (uint16_t)(g_in2.f.cfg_flags & 0x7fffu);

    /* ctrl-status reg B (+0x3c). */
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 | 0x0001u);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xfffdu);
    g_in2.f.wobble_sens = 0u; /* reg 0x75 */
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xfffbu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xfff7u);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xffefu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xffdfu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xffbfu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xff7fu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 | 0x0100u);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 | 0x0200u);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xfbffu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 | 0x0800u);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xefffu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xdfffu);
    g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & 0xbfffu);

    /* scratch byte @ g_in2-0x95 (== g_vs+0x05). */
    g_vs.substate = 3u;

    /* ctrl-status reg C (+0x3e). */
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg | 0x0001u);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg | 0x0002u);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xf7ffu);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xfff7u);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xffefu);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xffdfu);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xffbfu);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xff7fu);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg | 0x0100u);
    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg & 0xfdffu);
    g_in2.f.in2_5e = (uint16_t)(g_in2.f.in2_5e & ~REG2F_BIT0); /* reg 0x2F bit0 */
    g_in2.f.reg_2e = 0u;                                       /* reg 0x2E   */

    /* clear the work area; seed the ramp-mode selector + dir / window thresholds. */
    g_in2.f.passcode_num = 0;
    g_in2.f.passcode_key[0] = 0;
    g_in2.f.passcode_key[1] = 0;
    g_in2.f.auto_off_min = 5;
    g_in2.f.dp_b6 = 0;
    g_in2.f.dp_b8 = 0;
    g_in2.f.brake_light_flash = 0;
    g_in2.f.derived_speed = 0;
    g_in2.f.dir_bits = 1;            /* +0xE0 */
    g_in2.f.alarm_sensitivity = 1;   /* +0xE8 ramp-mode selector */
    g_in2.f.chg_sched_start = 0;     /* +0x96 */
    g_in2.f.chg_sched_end = 0x0100u; /* +0x98 */

    /* BMS charge-control push byte @ g_in2-0x0A (== g_in1+0x28 = in1.bms_chg_cmd_req). */
    g_in1.bms_chg_cmd_req = 1u;

    /* seed the ninebot report payload cell (g_stat.chg_limit), low byte := 100, then report. */
    report = &g_stat.chg_limit;
    *report = (uint16_t)(*report & 0xff00u);
    *report = (uint16_t)(*report | 100u);
    report_ninebot(0x3e, 7, 2, NB_CMD_WRITE3, 0x82, (const void*)report);

    /* per-region speed/limit profile table, selected by VS->region (g_in2-0x9A). */
    if (g_vs.region == VCU_REGION_GLOBAL) {
        g_in2.f.in2_5e = (uint16_t)(g_in2.f.in2_5e & ~REG2F_TURN_CANCEL_OFF);
        g_in2.f.fault_code = 0x0000u;
        g_in2.f.gear_a_ceiling = 0x0019u;
        g_in2.f.speed_limit_2 = 0x3232u;
        g_in2.f.prof_86 = 0x0505u;
        g_in2.f.prof_88 = 0x0505u;
        g_in2.f.prof_8a = 0x0019u;
        g_in2.f.speed_limit = 0x3232u;
    } else if (g_vs.region == VCU_REGION_EUROPE) {
        g_in2.f.fault_code = 0x0003u;
        g_in2.f.gear_a_ceiling = 0x000fu;
        g_in2.f.speed_limit_2 = 0x1919u;
        g_in2.f.prof_86 = 0x0505u;
        g_in2.f.prof_88 = 0x0505u;
        g_in2.f.prof_8a = 0x000fu;
        g_in2.f.speed_limit = 0x1919u;
    } else {
        g_in2.f.fault_code = 0x0003u;
        g_in2.f.gear_a_ceiling = 0x0f0fu;
        g_in2.f.speed_limit_2 = 0x1414u;
        g_in2.f.prof_86 = 0x0505u;
        g_in2.f.prof_88 = 0x0505u;
        g_in2.f.prof_8a = 0x0f0fu;
        g_in2.f.speed_limit = 0x1414u;
    }
}

/* Anti-theft motion check / siren FSM (vehicle state 6). The step state, ramp accumulator and tick
   live in g_tmr (step_state / step_ramp / step_tick). The state advances 1->2->3->4 while the
   motion ramp (IMU sensor-delta gated) crosses alarm-sensitivity-keyed thresholds; each stage
   chirps the buzzer or blinks the lights. The sensitivity selector and alarm-enable bit come from
   g_in2; the mode-flag word is g_mode. */

enum {                      /* alarm step state byte (g_tmr.step_state) */
       ALARM_STEP_ARM = 1,  /*!< initial ramp-arm (alarm-enable + sensor-delta gated) */
       ALARM_STEP_RUN2 = 2, /*!< ramp window A, buzzer / headlight chirp phase        */
       ALARM_STEP_RUN3 = 3, /*!< ramp window A, lock-latch / enable-report phase      */
       ALARM_STEP_RUN4 = 4  /*!< extended window, lock-latch / enable-report phase    */
};

#define ALARM_RAMP_TH_DEFAULT 119u /* ramp advance threshold, mode != 1,2         */
#define ALARM_RAMP_TH_MODE1   69u  /* ramp advance threshold, mode == 1           */
#define ALARM_RAMP_TH_MODE2   14u  /* ramp advance threshold, mode == 2           */
#define ALARM_RAMP_CLAMP      500u /* ramp accumulator ceiling                    */
#define ALARM_ARM_HOLD        500u /* state-1 give-up tick (499 < tick)           */

/* sensor-delta gate: raw bits of VS->sensor_delta_f (@g_vs+0x10) taken as int   */
/* and compared against this reinterpreted float threshold (20.0f).              */
#define ALARM_DELTA_MIN       0x41A00000 /* 20.0f */

#define ALARM_WIN_A           550u  /* ramp-eval window for states 2 and 3        */
#define ALARM_WIN_B           1050u /* ramp-eval window for state 4               */

/** \brief Ramp accumulator reached the mode-keyed advance threshold? (g_in2+0xE8). */
static int alarm_ramp_reached(uint16_t ramp) {
    int16_t mode;

    mode = (int16_t)g_in2.f.alarm_sensitivity;
    if (ramp > ALARM_RAMP_TH_DEFAULT) {
        return 1;
    }
    if ((mode == 1) && (ramp > ALARM_RAMP_TH_MODE1)) {
        return 1;
    }
    if ((mode == 2) && (ramp > ALARM_RAMP_TH_MODE2)) {
        return 1;
    }
    return 0;
}

/** \brief Advance the ramp accumulator while the sensor-delta gate is open. */
static void alarm_ramp_advance(void) {
    if ((g_vs.sensor_delta_bits > ALARM_DELTA_MIN) && (g_tmr.step_ramp < ALARM_RAMP_CLAMP)) {
        g_tmr.step_ramp = (uint16_t)(g_tmr.step_ramp + 1u);
    }
}

/** \brief Buzzer chirp phase (step state 2 and the lock-out siren). */
static void alarm_chirp_phase(uint32_t tick) {
    if ((tick == 1u) || (tick == 0x96u) || (tick == 300u) || (tick == 0x1c2u)) {
        buzzer_on(BUZZER_TONE);
    } else if ((tick == 100u) || (tick == 0xfau) || (tick == 400u) || (tick == 0x226u)) {
        buzzer_off();
    }
}

/** \brief Lock-latch + subsystem-enable report, with the drive mode-flag SET. */
static void alarm_latch_enable(void) {
    vehicle_lock_antitheft_latch();
    report_subsys_enable();
    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | MF_LIGHT_EN);
    mode_light_set(MODE_LIGHT_BLINK);
}

/** \brief Outputs-reset + mode-sync, with the drive mode-flag CLEARED. */
static void alarm_latch_disable(void) {
    vcu_outputs_reset();
    control_mode_sync();
    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~MF_LIGHT_EN);
}

/** \brief At/after the window: advance to \p next_state when the ramp is reached,
 *         otherwise raise the hold flag (g_vs+0x1c). */
static void alarm_window_eval(uint8_t next_state) {
    if (alarm_ramp_reached(g_tmr.step_ramp)) {
        g_tmr.step_state = next_state;
        g_tmr.step_ramp = 0;
        g_tmr.step_tick = 0;
    } else {
        g_vs.shutdown_req = 1u;
    }
}

/**
 * \brief           Anti-theft motion check / siren FSM (vehicle state 6)
 * \param[in]       run: 0 tears the ramp down and syncs outputs off; otherwise one
 *                      FSM step is run against the current state byte (g_tmr.step_state)
 * \return          1 when state 1 completes (ramp reached -> advance to state 2), else 0
 * \note            The tick phases and the sensitivity-keyed ramp thresholds (0x77 / 0x45 / 0x0E)
 *                      are fixed. During the passcode lock-out alarm (VS lockout_phase 2) it runs
 *                      only the siren phase.
 */
uint32_t control_alarm_motion_tick(int32_t run) {
    uint32_t tick_prev;
    uint32_t tick;
    uint8_t state;

    if (run == 0) {
        /* teardown: reset the ramp, clear the drive mode-flags, sync outputs off. */
        g_tmr.step_state = 1u;
        g_tmr.step_ramp = 0;
        g_tmr.step_tick = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~MF_LIGHT_EN);
        mode_light_set(MODE_LIGHT_OFF);
        g_vs.creep_enable = 1u;
        buzzer_off();
        vcu_outputs_reset();
        control_mode_sync();
        return 0;
    }

    tick_prev = g_tmr.step_tick;
    tick = tick_prev + 1u;

    if (g_vs.lockout_phase == LOCKOUT_ALARM) {
        /* passcode lock-out alarm: siren phase, then shutdown with the lock-out still armed. */
        g_tmr.step_tick = tick;
        alarm_chirp_phase(tick);
        if (g_tmr.step_tick > 0x225u) {
            g_vs.shutdown_req = 1u;
            g_vs.lockout_phase = LOCKOUT_ARMED;
        }
        return 0;
    }

    state = g_tmr.step_state;
    if (state == ALARM_STEP_ARM) {
        g_tmr.step_tick = tick;
        if (((g_in2.f.cfg_flags & CFG_ALARM_EN) != 0u) && (g_vs.sensor_delta_bits > ALARM_DELTA_MIN)
            && (g_tmr.step_ramp < ALARM_RAMP_CLAMP)) {
            g_tmr.step_ramp = (uint16_t)(g_tmr.step_ramp + 1u);
        }
        if (alarm_ramp_reached(g_tmr.step_ramp)) {
            g_tmr.step_state = ALARM_STEP_RUN2;
            g_tmr.step_ramp = 0;
            g_tmr.step_tick = 0;
            return 1;
        }
        if (tick >= ALARM_ARM_HOLD) { /* 499 < tick */
            g_vs.shutdown_req = 1u;
            g_vs.poweroff_beep = 1u;
        }
    } else if (state == ALARM_STEP_RUN2) {
        g_tmr.step_tick = tick;
        alarm_chirp_phase(tick);
        if (g_tmr.step_tick < ALARM_WIN_A) {
            alarm_ramp_advance();
        } else {
            alarm_window_eval(ALARM_STEP_RUN3);
        }
    } else if (state == ALARM_STEP_RUN3) {
        g_tmr.step_tick = tick;
        if ((tick == 1u) || (tick == 0x96u) || (tick == 300u) || (tick == 0x1c2u)) {
            alarm_latch_enable();
        } else if ((tick == 100u) || (tick == 0xfau) || (tick == 400u) || (tick == 0x226u)) {
            alarm_latch_disable();
        }
        if (g_tmr.step_tick < ALARM_WIN_A) {
            alarm_ramp_advance();
        } else {
            alarm_window_eval(ALARM_STEP_RUN4);
        }
    } else if (state == ALARM_STEP_RUN4) {
        g_tmr.step_tick = tick;
        if ((tick == 0x32u) || (tick == 200u) || (tick == 0x15eu) || (tick == 500u) || (tick_prev == 0x289u)
            || (tick == 800u) || (tick_prev == 0x3b5u)) {
            alarm_latch_enable();
        } else if ((tick == 0x96u) || (tick == 300u) || (tick == 0x1c2u) || (tick == 600u) || (tick_prev == 0x2edu)
                   || (tick == 900u) || (tick_prev == 0x419u)) {
            alarm_latch_disable();
        }
        if (g_tmr.step_tick < ALARM_WIN_B) {
            alarm_ramp_advance();
        } else {
            alarm_window_eval(ALARM_STEP_RUN4);
        }
    }

    return 0;
}
