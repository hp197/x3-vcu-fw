/**
 * \file            ninebot.h
 * \brief           Shared Ninebot command dispatcher for the x3 VCU
 * \note            The shared command processor of the x3 VCU on the Artery AT32F415 BSP.
 *
 *                      This is the SINGLE dispatcher that BOTH transports feed:
 *                        - USART2 / BLE Ninebot 5A A5 parser  (usart.c)
 *                        - CAN1 29-bit extended-id tunnel     (can.c)
 *                      The decoded frame is handed in starting at the Ninebot `len`
 *                      byte; the transport selector sends local replies back on the bus
 *                      the request came from and forwards frames for other nodes to the
 *                      other bus.
 *
 *                      Functions:
 *                        vcu_ninebot_process   (the shared dispatcher)
 *                        vcu_ninebot_dispatch  (USART parser entry, transport 0)
 *                        nb_crc32              (CRC-32, poly 0xEDB88320)
 *                        nb_keycmp             (byte compare, memcmp-like)
 *                        nb_reply_can   -> report_ninebot        (CAN ext-id reply)
 *                        nb_reply_usart -> vcu_usart_report_send (USART ring reply)
 *                        vcu_ota_dispatch      (dst 0x16 OTA cmds, see ota.c)
 *
 *                      Constants:
 *                        0xEDB88320  reflected CRC-32 poly
 *                        0x000F4240  pairing checksum modulus (1000000)
 *                        "0000000001" serial template (10 bytes, flash)
 *                        0x1FFFFFFF  29-bit ext-id mask
 *                      The 16-byte pairing key is held in RAM (\ref vcu_ninebot_ctx_t), copied
 *                      from the flash auth-key cell at boot.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef NINEBOT_H
#define NINEBOT_H

#include <stddef.h>
#include <stdint.h>

#include "usart.h" /* NB_OFF_* / NB_ADDR_VCU / NB_ADDR_APP / NB_CMD_WRITE3 (shared frame header) */

/* ------------------------------------------------------------------------- */
/* Transport selector: the `transport` argument of vcu_ninebot_process, passed */
/* on to the OTA dispatcher. Shared by ninebot.c and ota.c. Callers:            */
/*   USART2 RX: vcu_ninebot_dispatch -> vcu_ninebot_process(.., 0)              */
/*   CAN PF 0xEF: nb_reply_fc -> vcu_ninebot_process(.., 1)                     */
/* The code tests `== 1` (CAN) everywhere except the OTA busy ACK (`!= 0`).     */
/* ------------------------------------------------------------------------- */

/** \brief Bus a Ninebot request arrived on. */
typedef enum {
    VCU_TRANSPORT_USART = 0, /*!< USART2: replies via vcu_usart_report_send into the g_uartctx ring */
    VCU_TRANSPORT_CAN = 1    /*!< CAN 0x18EF ext-id: replies via report_ninebot                 */
} vcu_transport_t;

/* ------------------------------------------------------------------------- */
/* Frame field offsets: the shared NB_OFF_* table (src / dst / cmd / arg) and */
/* NB_ADDR_VCU / NB_ADDR_APP / NB_CMD_WRITE3 live in usart.h. The stored frame: */
/* len = frame[0], src = frame[1], dst = frame[2] (the dispatcher handles       */
/* dst == NB_ADDR_VCU or 0 locally), cmd = frame[3], arg = frame[4],           */
/* body = frame + 5.                                                           */
/* ------------------------------------------------------------------------- */

/* Further node addresses (the dispatcher routes local frames on `src`). */
#define NB_ADDR_ESC_3D         0x3DU /* ESC register/pairing node (also 0x3E / 0x3F)     */
#define NB_ADDR_ESC_3E         0x3EU
#define NB_ADDR_ESC_3F         0x3FU

/* ------------------------------------------------------------------------- */
/* Register commands in the `cmd` field.                                     */
/* cmd 3 is NB_CMD_WRITE3 (usart.h, shared with every report sender).         */
/* ------------------------------------------------------------------------- */

#define NB_CMD_READ            0x01U /* read  register block -> reply                     */
#define NB_CMD_WRITE2          0x02U /* guarded write (ride-mode / 0x46 pairing gate)     */
#define NB_CMD_WRITE4          0x04U /* raw register write                                */

/* OTA sub-commands handed to vcu_ota_dispatch (cases 7..10). */
#define NB_CMD_OTA_PREPARE     0x07U
#define NB_CMD_OTA_DATA        0x08U
#define NB_CMD_OTA_COMMIT      0x09U
#define NB_CMD_OTA_APPLY       0x0AU
#define NB_CMD_OTA_REPLY       0x0BU /* OTA sub-protocol status reply (ota.c; also the OTA-busy reply) */
#define NB_OTA_BUSY_CODE       0x09U /* reply code when the VCU's own OTA is busy                   */

/* Pairing / auth sub-commands. */
#define NB_CMD_PAIR_CHALLENGE  0x18U /* CRC-32 challenge over key + serial         */
#define NB_REG_SPEED_LIMIT     0x46U /* CRC-gated speed-limit register (IN2+0x8C)   */

/* Custom (non-Ninebot) command range: 0xF0..0xFE, picked high and clear of the official command
   space (0x00..0x1F, 0x70..0x7F) so a future stock firmware that adds a command does not collide. */
#define NB_CMD_BACKUP          0xF0U /* read-only ESC backup relay (app <-> MCU)   */

/* ------------------------------------------------------------------------- */
/* Crypto / pairing constants.                                               */
/* ------------------------------------------------------------------------- */

#define NB_CRC32_POLY          0xEDB88320U /* reflected IEEE CRC-32 poly                */
#define NB_PAIR_MOD            1000000U    /* pairing checksum modulus                  */
#define NB_KEY_LEN             16U         /* pairing key length (bytes)                */
#define NB_SERIAL_LEN          10U         /* serial tail compared with "0000000001"     */
#define NB_SERIAL_PROV_LEN     14U         /* provisioned serial length (IN2+0x20, op 0x18) */
#define NB_CHALLENGE_LEN       31U         /* CRC-32 input length, cmd 0x18             */
#define NB_RIDEMODE_LEN        18U         /* CRC-32 input length, reg 0x46             */

/* Register count of one page of the 16-bit register file (g_in2.reg[]: the write
   path range-checks arg+len against 0x100, registers are `reg[arg] : uint16`). */
#define NB_REG_COUNT           0x100U

/* ------------------------------------------------------------------------- */
/* Node address selectors routed by the `src` byte (frame + 1) of the         */
/* dst==0x16 local path.                                                      */
/*   0x23      BLE app / dashboard node (config register R/W + 0xFC/0xFD OTA) */
/*   0x04      vehicle control node     (remote lock/unlock, buzzer, screen)  */
/*   0x3D..3F  ESC / info node          (pairing + info reads + state writes) */
/* NB_ADDR_APP is in usart.h; NB_ADDR_ESC_3D..3F are defined above.         */
/* ------------------------------------------------------------------------- */

#define NB_ADDR_CTRL           0x04U /* vehicle control node */

/* ------------------------------------------------------------------------- */
/* Control-node commands in the `cmd` field (src==0x04).                     */
/* ------------------------------------------------------------------------- */

#define NB_CTRL_REG_READ       0x01U /* read IN2 register block -> reply                    */
#define NB_CTRL_CMD3           0x03U /* cmd 3; arg 0x86 -> buzzer/horn                      */
#define NB_CTRL_CMD4           0x04U /* cmd 4; arg 0x20 -> display/ride-mode                */
#define NB_CTRL_OP7_SESSION    0x07U /* bus-active retrigger (IN1+0x03) + clear the remote lock */
#define NB_CTRL_OP0A           0x0AU /* IN1+0x05 long UV grace + CAN-active word 0 (report_can_toggle) */
#define NB_CTRL_LOCK_STATE     0x7AU /* remote lock value -> VS+0x22 (0 -> non-zero edge beeps) */
#define NB_CTRL_REMOTE_LOCK    0x7BU /* remote lock / unlock (arg 1 unlock, 4 lock, 0/2/3 clear) */

#define NB_CTRL_ARG_BUZZER     0x86U /* cmd 3 arg selecting the buzzer/horn tone           */
#define NB_CTRL_ARG_MODE       0x20U /* cmd 4 arg selecting the display/ride-mode register */
#define NB_CTRL_ARG_ECHO       0x7EU /* cmd 2/4 arg: beep + echo body[0] back as cmd 2 arg 0x7E */
#define NB_CTRL_ARG_E0         0xE0U /* cmd 2/4 arg: body[0] in {1,3,5} -> same beep + 0x7E echo */

/* ------------------------------------------------------------------------- */
/* Info-node commands in the `cmd` field (src==0x3D/0x3E/0x3F).              */
/* ------------------------------------------------------------------------- */

#define NB_INFO_REBOOT         0x11U /* arg==1 -> request reboot / clear scratch           */
#define NB_INFO_CFG_READ       0x13U /* read config block (telemetry frame, REPORT_TELEM_CFG_BASE) -> USART reply */
#define NB_INFO_CFG_READ_2     0x17U /* alias of 0x13                                      */
#define NB_INFO_VIN_WRITE      0x19U /* CRC-gated VIN/serial write                         */
#define NB_INFO_IDENT_READ     0x41U /* identity/version block read -> reply               */
#define NB_INFO_INPUT_TEST     0x56U /* input-test mode on/off (VS+0x64), ADC-gated            */
#define NB_INFO_RIDE_START     0x57U /* token-gated ride start (shares 0x59 handler)       */
#define NB_INFO_RIDE_STOP      0x58U /* token-gated ride/session stop                      */
#define NB_INFO_UNLOCK         0x59U /* token-gated unlock / session open                  */
#define NB_INFO_FLAG_5E        0x5EU /* set IN1+0x23 (app_flag_5e, no reader), ack        */
#define NB_INFO_LIMIT_SET      0x5FU /* PIN-gated write of the session distance (IN2+0xC4) */
#define NB_INFO_STANDBY        0x79U /* app start (+unlock) / shutdown requests (VS+0x28/+0x2C/+0x1C) */
#define NB_INFO_LOCK_COMMIT    0x80U /* commit lock / recompute, ack                       */

/* ------------------------------------------------------------------------- */
/* VCU state blackboard model. The dispatcher works on these state blocks:    */
/*   main vehicle-state (VS), input/latch + drive regs (IN2),                  */
/*   handle/session flags (IN1), scratch / display-mode (SCR),                 */
/*   FSM state byte (STATE), OTA-in-progress gate, app-auth retry counter,     */
/*   speed-limit PIN compare word, serial / VIN base (IN2+0x2E).               */
/* These blocks are the named globals of vcu_state.h (g_vs, g_in2, g_in1, g_scr,   */
/* g_state_block, ...); the field OFFSETS are load-bearing. The info-node config   */
/* block is report.c's g_telem_frame.                                           */
/* See ninebot.c for the per-field accessors.                                   */
/* ------------------------------------------------------------------------- */

#define NB_BB_SERIAL           (((volatile uint8_t*)&g_in2) + 0x2Eu) /* serial / VIN base (cmd 0x18 / 0x19) == IN2+0x2E */

/* ------------------------------------------------------------------------- */
/* Additional bases for the APP node (addr 0x23), the extended sub-nodes      */
/* (addr 0x02 / 0x06 / 0x07) and the generic-path specials, as offsets into   */
/* the vcu_state.h blocks; the OFFSETS are load-bearing.                      */
/* ------------------------------------------------------------------------- */

#define NB_BB_APPCFG           (((volatile uint8_t*)&g_in2) + 0x40u) /* APP-node config register base == IN2+0x40 */
/* The BLE app-session flag is the shared fm_uv_cnt cell; ninebot clears it as the full u16
   via LOW->fm_uv_cnt (vcu_lowglob_t +0x0A). */
#define NB_BB_SCR2             (((volatile uint8_t*)&g_stat)) /* report/scratch region (sub06/07 + generic) == g_stat */
#define NB_BB_SUB02            (((volatile uint8_t*)&g_in2) + 0x180u) /* addr-0x02 op4/arg0xDA write base == IN2+0x180 */
/* The sub-node 0x06/0x07 flag is the BMS comm-loss counter (g_bms_commloss in
   vcu_state.h); ninebot clears that halfword. */
#define NB_BB_PROV_DST         (((volatile uint8_t*)&g_in2) + 0x1B4u) /* serial-provision 3-word dst == IN2+0x1B4 */
#define NB_BB_PROV_SRC         ((volatile uint8_t*)g_mcu_uid) /* serial-provision 3-word src == factory UID snapshot */

/* Extended sub-node addresses + APP-node registration ops. */
#define NB_ADDR_SUB02          0x02U /* ext sub-node: op4/arg0xDA bounded write        */
#define NB_REG_SUB02_COMPONENT 0xDAU /* sub-node 0x02 component record register (-> IN2+0x180) */
#define NB_REG_SUB02_PUSH      0xD0U /* node 0x02 reg 0xD0 parameter push (can.c sends it)  */
#define NB_CMD_SUB02_ACK       0x05U /* node 0x02 cmd 5 arg 0xD0 = push acknowledged       */
#define NB_SUB02_COMPONENT_LEN 20U   /* component record length requested at boot (bytes)   */
#define NB_ADDR_SUB06          0x06U /* ext sub-node: bounded reg-array write          */
#define NB_ADDR_SUB07          0x07U /* ext sub-node: bounded reg-array write          */
#define NB_APP_CMD_FC          0xFCU /* APP OTA/registration prepare (bp_handle_fc)     */
#define NB_APP_CMD_FD          0xFDU /* APP registration commit      (bp_node_register) */
#define NB_REG_PASSCODE        0x71U /* key passcode: app write is staged in VS+0x56, VS+0x41=1 */

/* ------------------------------------------------------------------------- */
/* Registers with write side effects (the reg-write side-effect switch).      */
/* Register n is IN2 + 2n; the bit names (CFG_*, CFG2_*, CHG_*) are   */
/* in vcu_state.h next to the IN2 register view.                               */
/* ------------------------------------------------------------------------- */

#define NB_REG_CFG_FLAGS       0x1DU /* settings: walk-bit change nudges gear 1 -> 2           */
#define NB_REG_CFG_FLAGS2      0x1EU /* settings 2: gear-enable change re-maps gears 3/4       */
#define NB_REG_CHG_CFG         0x1FU /* charge settings: scheduled-charging enable / single-shot */
#define NB_REG_CHG_START       0x4BU /* scheduled-charging window start (hh:mm)                */
#define NB_REG_CHG_END         0x4CU /* scheduled-charging window end (hh:mm)                  */
#define NB_REG_RIDE_ASSIST     0x75U /* writable reg whose cmd 2/3 write toggles reg 0x1E bit1 */
#define NB_REG_SESSION_DIST    0x62U /* regs 0x62..0x67 = IN2+0xC4..+0xCF session counters, zeroed by 0x58/0x59 */

/* ------------------------------------------------------------------------- */
/* Reply status codes and local event/beep signals.                          */
/* ------------------------------------------------------------------------- */

#define NB_ST_OK               0U /* request accepted / write committed                */
#define NB_ST_DENIED           1U /* permission-table rejects the register range       */
#define NB_ST_REJECT           4U /* gate failed (CRC / length / wrong ride state)      */

/* Beep feedback uses the shared buzzer_pattern_t ids (buzzer.h): BEEP_DOUBLE = accept,
   BEEP_TRIPLE = reject, BEEP_LONG = token reject, BEEP_SHORT = notice. */

/* ------------------------------------------------------------------------- */
/* Housekeeping / auth events handed to \ref nb_vcu_event. When bit         */
/* NB_EVT_CTRL_CMD is set, the low byte carries a control/info `cmd` and the  */
/* dispatcher runs the matching live-state side effects; otherwise the value   */
/* is one of the small housekeeping ids below.                               */
/* ------------------------------------------------------------------------- */

#define NB_EVT_CTRL_CMD        0x0100U /* flag: low byte is a control/info cmd        */
#define NB_EVT_AUTH_OK         0x0003U /* auth OK  -> beep(3)                          */
#define NB_EVT_AUTH_FAIL       0x0004U /* auth reject -> beep(4)                       */
#define NB_EVT_OTA_FLAG        0x0007U /* raise IN1 can_active_req (IN1+3)             */
#define NB_EVT_REG_WRITTEN     0x0010U /* register write committed -> side effects    */
#define NB_EVT_SERIAL_COMMIT   0x0018U /* provisioned serial committed -> refresh      */

/* The dispatcher reaches the blackboard through the shared vcu_state.h structs   */
/* (g_vs, g_in1, g_in2, g_scr): every cell it touches has one canonical name there. */

/**
 * \brief           Ninebot RAM context: the 16-byte pairing key
 * \note            The register file is NOT here: it is the shared IN2 block
 *                      (`IN2 + arg*2`, \ref vcu_in2_block_t.reg), so app reads return the
 *                      live state and app writes reach the cells the vehicle reads. The
 *                      serial number is IN2+0x20 (\ref vcu_in2_t.serial).
 */
typedef struct {
    uint8_t key[NB_KEY_LEN]; /*!< pairing key, copied from flash at boot */
} vcu_ninebot_ctx_t;

/**
 * \brief           Flash auth-key cell (image header 0x08001420, 16 B)
 * \note            Erased in this build, 16 x 0xFF. A per-model 16-char ASCII key can be
 *                      stored here (see the OTA_TEA_KEY_SEED note in ota.c).
 *                      Read by nb_key_load and by the config sync compare (config.c).
 */
extern const uint8_t NB_FLASH_AUTH_KEY[NB_KEY_LEN];

/** \brief Boot copy of the 16-byte auth key from flash into the Ninebot context. */
void nb_key_load(void);

/**
 * \brief           CRC-32 over a buffer
 * \note            Reflected IEEE CRC-32, init 0xFFFFFFFF, poly 0xEDB88320,
 *                      final XOR 0xFFFFFFFF. Used for the pairing challenge check.
 * \param[in]       data: input bytes
 * \param[in]       len: number of bytes
 * \return          the CRC-32 value
 */
uint32_t nb_crc32(const uint8_t* data, uint32_t len);

/**
 * \brief           Byte compare, memcmp-like
 * \note            Returns a[i] - b[i] for the first i where the
 *                      bytes differ, 0 when all \p len bytes are equal. The cmd-0x18 reply
 *                      status is this value's low byte.
 * \param[in]       a: first buffer
 * \param[in]       b: second buffer
 * \param[in]       len: byte count
 * \return          difference of the first mismatching bytes, 0 on a full match
 */
int nb_keycmp(const uint8_t* a, const uint8_t* b, uint32_t len);

/**
 * \brief           The shared Ninebot command dispatcher
 * \note            Parse a decoded frame, route by \ref NB_OFF_DST / \ref NB_OFF_SRC,
 *                      run the register read/write, pairing/auth and OTA routing, and
 *                      send the reply on the selected transport. Both the USART seam and
 *                      the CAN tunnel call into this.
 * \param[in,out]   frame: frame-assembly buffer, starting at the Ninebot `len` byte
 * \param[in]       transport: \ref vcu_transport_t (VCU_TRANSPORT_USART or VCU_TRANSPORT_CAN)
 * \param[in]       p3: reserved; forwarded to OTA
 * \param[in]       p4: reserved; forwarded to OTA
 */
void vcu_ninebot_process(uint8_t* frame, int transport, uint32_t p3, uint32_t p4);

/**
 * \brief           USART/BLE entry into the Ninebot dispatcher
 * \note            Called by usart.c's frame parser with a single argument; this enters
 *                      \ref vcu_ninebot_process with
 *                      \ref VCU_TRANSPORT_USART.
 * \param[in]       frame: the frame-assembly buffer, starting at the `len` byte
 */
void vcu_ninebot_dispatch(const uint8_t* frame);

/**
 * \brief           Commit the drive / indicator mode
 * \note            Latches the mode halfword at g_scr+0x4a, reports it (dst 0x23,
 *                      arg 0x25) and clears the registered report-node record g_bp_node on mode 0.
 *                      Called by the vehicle FSM (vehicle.c) and the config path
 *                      (busproto.c).
 * \param[in]       mode: drive/indicator mode code (high byte carries the mode param)
 */
void nb_commit_mode(int16_t mode);

/**
 * \brief           Build+send a 0xFC Ninebot reply
 * \note            Candev handler +0x38 and the 0xEF route target of can.c's ext-id RX
 *                      handler. Frame: [total_len-3][src][dst][body1][body2][body3..]
 *                      dispatched via the shared Ninebot handler (vcu_ninebot_process,
 *                      VCU_TRANSPORT_CAN). The leading \p x arg is unused.
 */
void nb_reply_fc(void* x, uint8_t src, uint8_t dst, const uint8_t* body, char total_len);

/**
 * \brief           Send a solicited reply from the VCU to a node (public reply wrapper)
 * \param[in]       transport: the bus to reply on (\ref vcu_transport_t)
 * \param[in]       dst: destination node address
 * \param[in]       cmd: reply command code
 * \param[in]       data: reply body bytes (may be NULL when \p len is 0)
 * \param[in]       len: body length in bytes
 */
void nb_reply_pub(int transport, uint8_t dst, uint8_t cmd, const uint8_t* data, uint8_t len);

#endif /* NINEBOT_H */
