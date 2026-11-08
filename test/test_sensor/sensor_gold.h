/**
 * \file            sensor_gold.h
 * \brief           Reference IMU outputs for the sensor tests (do not edit)
 * \note            Fixed reference outputs of vcu_sensor_init + vcu_sensor_attitude_update
 *                      for a constant chip sample per scenario, checked at selected passes.
 *                      The sensor tests compare the tilt within 0.001 deg and the rest bit for
 *                      bit. Floats are raw IEEE-754 bits.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef SENSOR_GOLD_H
#define SENSOR_GOLD_H

#include <stdint.h>

/** \brief One checkpoint: the outputs after `pass` calls of vcu_sensor_attitude_update */
typedef struct {
    uint16_t pass;      /*!< pass number (1 = first update after init)          */
    uint32_t tilt;      /*!< attitude +0x0C tilt of this pass (deg, bits)        */
    uint32_t tilt_mean; /*!< attitude +0x14 averaged tilt (deg, bits)            */
    uint32_t norm;      /*!< |a| (g, bits)                                       */
    uint32_t metric;    /*!< g_vs.sensor_delta_f motion metric (bits)            */
    uint32_t quat[4];   /*!< Mahony quaternion w, x, y, z (bits)                 */
    uint8_t cls;        /*!< g_vs.tilt_latch tilt class                          */
} gold_point_t;

/** \brief A speed step: g_in2.f.speed_src_s = `speed` from pass `from_pass` on */
typedef struct {
    uint16_t from_pass; /*!< first pass with this speed */
    int16_t speed;      /*!< g_in2.f.speed_src_s        */
} gold_speed_t;

/** \brief One scenario: fixed inputs, its checkpoints */
typedef struct {
    const char* name;           /*!< scenario name                               */
    int16_t sample[7];          /*!< chip block TEMP, GX, GY, GZ, AX, AY, AZ      */
    uint8_t fsm_state;          /*!< g_state_block.fsm_state                     */
    uint8_t tilt_cfg;           /*!< tilt-class enable (CHG_TILT_LATCH_EN) set   */
    const gold_speed_t* speed;  /*!< speed schedule                              */
    uint8_t speed_count;        /*!< entries in `speed`                          */
    const gold_point_t* points; /*!< checkpoints, ascending passes               */
    uint8_t point_count;        /*!< entries in `points`                         */
} gold_scenario_t;

/* tilt_pos */
static const gold_speed_t gold_tilt_pos_speed[] = {{1u, 0}};
static const gold_point_t gold_tilt_pos_points[] = {
    {1u, 0x43340000u, 0x00000000u, 0x3EDD0F2Au, 0x45C28CB0u, {0x3F7F910Fu, 0xBA178A0Fu, 0xBC6CC7B8u, 0x00000000u}, 2u},
    {2u, 0xC332575Bu, 0xC332575Bu, 0x3EBBC6C1u, 0x45A5591Au, {0x3F7F7C31u, 0xBA9D9112u, 0xBCF1DD2Fu, 0x34A48111u}, 2u},
    {3u, 0xC3309C65u, 0xC33179E0u, 0x3E999134u, 0x45875870u, {0x3F7F5677u, 0xBAF868E1u, 0xBD38B201u, 0x360B2BF1u}, 2u},
    {5u, 0xC32D01B6u, 0xC32FB20Au, 0x3E2DC8A3u, 0x451882F8u, {0x3F7ED91Eu, 0xBB6DEE12u, 0xBD9C676Fu, 0x37CB2F82u}, 2u},
    {20u, 0x43260F71u, 0xC21B98AAu, 0x3F046734u, 0x44535590u, {0x3F7D1C0Bu, 0xBC0AEDD5u, 0x3E0E4D96u, 0x3B5D1EF7u}, 2u},
    {50u, 0x42EFA17Bu, 0x4290CE76u, 0x3F69078Du, 0x4308B800u, {0x3F5C12ADu, 0xBAFD4DD7u, 0x3F01E7C9u, 0x3C42F669u}, 2u},
    {94u, 0x42D438A6u, 0x42B3DA4Au, 0x3F7BF944u, 0x41B98C00u, {0x3F4C508Fu, 0xBB3BD8C1u, 0x3F196CBCu, 0x3CB1AF18u}, 2u},
    {95u, 0x42D46E9Au, 0x42B433FBu, 0x3F7C140Bu, 0x41E57E00u, {0x3F4C77A1u, 0xBB44B374u, 0x3F193825u, 0x3CB34715u}, 1u},
    {96u, 0x42D4A983u, 0x42B48C63u, 0x3F7C2ED2u, 0x4208B800u, {0x3F4CA1C9u, 0xBB4DBC4Fu, 0x3F18FF51u, 0x3CB4DDA0u}, 1u},
    {100u, 0x42D5C18Du, 0x42B5E2DCu, 0x3F7C840Du, 0x4188B800u, {0x3F4D6598u, 0xBB737DB6u, 0x3F17F5F0u, 0x3CBB2A6Fu}, 1u},
    {150u, 0x42E99193u, 0x42DCB5E6u, 0x3F7FD8EEu, 0x406A6000u, {0x3F59AEC7u, 0xBC40B2EFu, 0x3F059D33u, 0x3D0373D8u}, 1u},
    {300u, 0x42F00C67u, 0x42F1315Au, 0x3F7FDC65u, 0x00000000u, {0x3F5CC8F9u, 0xBCEAE5E1u, 0x3EFF1603u, 0x3D7BE4E4u}, 1u},
};

/* tilt_neg */
static const gold_speed_t gold_tilt_neg_speed[] = {{1u, 0}};
static const gold_point_t gold_tilt_neg_points[] = {
    {1u, 0x43340000u, 0x00000000u, 0x3EDD0F2Au, 0x45C28CB0u, {0x3F7F910Fu, 0xBA178A0Fu, 0x3C6CC7B8u, 0x00000000u}, 2u},
    {2u, 0x4332575Bu, 0x4332575Bu, 0x3EBBC6C1u, 0x45A5591Au, {0x3F7F7C31u, 0xBA9D9112u, 0x3CF1DD2Fu, 0xB4A48111u}, 1u},
    {3u, 0x43309C65u, 0x433179E0u, 0x3E999134u, 0x45875870u, {0x3F7F5677u, 0xBAF868E1u, 0x3D38B201u, 0xB60B2BF1u}, 1u},
    {20u, 0xC3260FE9u, 0x421B97CDu, 0x3F0463B2u, 0x44535590u, {0x3F7D1C32u, 0xBC0AF421u, 0xBE0E4945u, 0xBB5D284Fu}, 1u},
    {21u, 0xC323FF08u, 0x41E1C1B5u, 0x3F0B6EFDu, 0x4499F610u, {0x3F7C6F47u, 0xBC075345u, 0xBE20502Au, 0xBB6F5A51u}, 2u},
    {22u, 0xC321F3F6u, 0x4195B00Au, 0x3F120EC2u, 0x44C75B28u, {0x3F7BB06Au, 0xBC03882Cu, 0xBE320E9Bu, 0xBB80E62Au}, 2u},
    {50u, 0xC2EFA2E4u, 0xC290CF6Eu, 0x3F690418u, 0x4308B800u, {0x3F5C137Au, 0xBAFD63EFu, 0xBF01E66Du, 0xBC42FDA6u}, 2u},
    {100u, 0xC2D5C2A6u, 0xC2B5E3FBu, 0x3F7C8098u, 0x4188B800u, {0x3F4D664Eu, 0xBB738983u, 0xBF17F4F6u, 0xBCBB30F0u}, 2u},
    {300u, 0xC2F00D4Eu, 0xC2F1323Bu, 0x3F7FD8EEu, 0x00000000u, {0x3F5CC971u, 0xBCEAEC59u, 0xBEFF1442u, 0xBD7BED50u}, 2u},
};

/* gyro */
static const gold_speed_t gold_gyro_speed[] = {{1u, 0}};
static const gold_point_t gold_gyro_points[] = {
    {1u, 0x43340000u, 0x00000000u, 0x3EF0092Au, 0x45F7F490u, {0x3F7F910Fu, 0xBBC9E8B7u, 0xBC286183u, 0xBA9FDD9Du}, 2u},
    {2u, 0xC332D1C8u, 0xC332D1C8u, 0x3ECC54ADu, 0x45D399ECu, {0x3F7F830Bu, 0xBC4D6598u, 0xBCA8176Bu, 0xBB1FA6C0u}, 2u},
    {3u, 0xC331A403u, 0xC3323AE6u, 0x3EA7F725u, 0x45AEB690u, {0x3F7F6A7Au, 0xBC9EBB82u, 0xBCFCB91Cu, 0xBB6EC4D3u}, 2u},
    {10u, 0xC32AF7CDu, 0xC32E37EFu, 0x3E05C4A7u, 0x45693144u, {0x3F7DB235u, 0xBDC74EB6u, 0xBD9202E0u, 0xBC0BE715u}, 2u},
    {50u, 0x42ABAA40u, 0x42070450u, 0x3F65F2C0u, 0x43189680u, {0x3F271F83u, 0xBE2BA223u, 0x3F3B7CEBu, 0x3D9D7649u}, 2u},
    {100u, 0xBF06B200u, 0x41F36DD4u, 0x3F7892FDu, 0x41975E00u, {0xBD03F32Cu, 0xBE1EFD8Du, 0x3F78CA57u, 0x3E284A46u}, 2u},
    {200u, 0x420BBE22u, 0x41898AAEu, 0x3F7B49ACu, 0x00000000u, {0x3E767150u, 0xBEC06F4Fu, 0x3F5F230Du, 0x3E46D78Cu}, 2u},
    {300u, 0x420CFDE7u, 0x421110DEu, 0x3F7B49ACu, 0x00000000u, {0x3E486ED2u, 0xBF053FCBu, 0x3F4AC893u, 0x3E7AAE77u}, 2u},
};

/* speed */
static const gold_speed_t gold_speed_speed[] = {{1u, 0}, {5u, 25}, {16u, 40}, {27u, -10}};
static const gold_point_t gold_speed_points[] = {
    {1u, 0x43340000u, 0x00000000u, 0x3EA01BCEu, 0x4577F490u, {0x3F7F910Fu, 0xBB02AF80u, 0xBB82AF80u, 0x00000000u}, 0u},
    {2u, 0xC3338ACEu, 0x00000000u, 0x3E8757E1u, 0x45530770u, {0x3F7F8F6Cu, 0xBB83E9DFu, 0xBC03D61Fu, 0x32219967u}, 0u},
    {3u, 0xC333138Bu, 0x00000000u, 0x3E59E906u, 0x452C62DCu, {0x3F7F8BE6u, 0xBBCA6AB2u, 0xBC4A5740u, 0xB020A693u}, 0u},
    {4u, 0xC3329512u, 0xC3329512u, 0x3E23E135u, 0x45053590u, {0x3F7F863Bu, 0xBC0CDAD8u, 0xBC8CD146u, 0xB253A361u}, 0u},
    {5u, 0xC33206CFu, 0xC3324DF0u, 0x3DDBC1BDu, 0x44BC3798u, {0x3F7F7CB9u, 0xBC3E90B1u, 0xBCBE6EF8u, 0x339B80C1u}, 0u},
    {6u, 0xC33154B9u, 0xC331FADEu, 0x3D68DB40u, 0x445E7920u, {0x3F7F6900u, 0xBC86826Du, 0xBD0643DBu, 0x350803C5u}, 0u},
    {7u, 0xC3303C43u, 0xC3318B37u, 0x3CCA120Eu, 0x44158940u, {0x3F7F3254u, 0xBCD46B15u, 0xBD538C71u, 0x362601B5u}, 0u},
    {8u, 0xC32E1076u, 0xC330D910u, 0x3D67E519u, 0x4492CA20u, {0x3F7EFCDBu, 0xBCFA2078u, 0xBD78D818u, 0x368DC200u}, 0u},
    {9u, 0xC32D0342u, 0xC330356Eu, 0x3DD17D7Bu, 0x44D7E970u, {0x3F7EDAACu, 0xBD09C7D8u, 0xBD88FFCFu, 0x36B96C31u}, 0u},
    {10u, 0xC32C4D8Au, 0xC32FA697u, 0x3E1729A1u, 0x450CF3FCu, {0x3F7EBC6Fu, 0xBD148B80u, 0xBD93A0B3u, 0x36EA8369u}, 0u},
    {11u, 0xC32BB3D0u, 0xC32F283Du, 0x3E41D0A6u, 0x4529FBA0u, {0x3F7E9DB4u, 0xBD1E8A16u, 0xBD9DE2C1u, 0x35C6C32Fu}, 0u},
    {12u, 0xC32B1F4Bu, 0xC32EB578u, 0x3E68C8A2u, 0x45438A98u, {0x3F7E7C7Fu, 0xBD284135u, 0xBDA86BA4u, 0xB778BA38u}, 0u},
    {13u, 0xC32A869Bu, 0xC32E4A62u, 0x3E863E95u, 0x43BB60C0u, {0x3F7E5786u, 0xBD31ED4Bu, 0xBDB3910Au, 0xB83CEC1Eu}, 0u},
    {14u, 0xC329E4E2u, 0xC32DE411u, 0x3E969597u, 0x4436F3F0u, {0x3F7E2DB3u, 0xBD3BB33Bu, 0xBDBF8A87u, 0xB8C0F8B5u}, 0u},
    {15u, 0xC32936EDu, 0xC32D804Du, 0x3EA5923Fu, 0x4485F8E0u, {0x3F7DFDF4u, 0xBD45AC11u, 0xBDCC7E01u, 0xB925F8A8u}, 0u},
    {16u, 0xC3287A88u, 0xC32D1D66u, 0x3EB35EC2u, 0x44AEA308u, {0x3F7DC71Bu, 0xBD4FED5Au, 0xBDDA90D4u, 0xB98161EDu}, 0u},
    {17u, 0xC327AD81u, 0xC32CB9FAu, 0x3EC0151Fu, 0x44D58BF8u, {0x3F7D87FEu, 0xBD5A7F85u, 0xBDE9D906u, 0xB9BCF892u}, 0u},
    {18u, 0xC326CE7Eu, 0xC32C54F1u, 0x3ECBCEEEu, 0x44FAB3B0u, {0x3F7D3F45u, 0xBD656F34u, 0xBDFA6DF7u, 0xBA0399D4u}, 0u},
    {19u, 0xC325DC0Cu, 0xC32BED63u, 0x3ED6A5ADu, 0x438E3640u, {0x3F7CEB6Du, 0xBD70C59Du, 0xBE063389u, 0xBA30E4E3u}, 0u},
    {20u, 0xC324D4B3u, 0xC32B8287u, 0x3EE0B2D1u, 0x440B5010u, {0x3F7C8ABFu, 0xBD7C8A8Bu, 0xBE0FED3Bu, 0xBA674525u}, 0u},
    {21u, 0xC323B701u, 0xC32B13AAu, 0x3EEA068Au, 0x444C77C0u, {0x3F7C1B62u, 0xBD84633Bu, 0xBE1A6CC0u, 0xBA93C097u}, 0u},
    {22u, 0xC32281C2u, 0xC32AA031u, 0x3EF2E05Fu, 0x44853590u, {0x3F7B9BA1u, 0xBD8ABED2u, 0xBE25B174u, 0xBAB8F80Cu}, 0u},
    {23u, 0xC32134CBu, 0xC32A27A1u, 0x3EFB3EC3u, 0x44A29518u, {0x3F7B0993u, 0xBD915A74u, 0xBE31BC53u, 0xBAE37B0Cu}, 0u},
    {24u, 0xC31FCFB5u, 0xC329A988u, 0x3F019827u, 0x44BE8188u, {0x3F7A62F2u, 0xBD983A58u, 0xBE3E9309u, 0xBB09C730u}, 0u},
    {25u, 0xC31E5183u, 0xC3292588u, 0x3F05596Au, 0x43532E80u, {0x3F79A555u, 0xBD9F6141u, 0xBE4C3868u, 0xBB24B503u}, 0u},
    {26u, 0xC31CB97Fu, 0xC3289B46u, 0x3F08E72Bu, 0x43CDB040u, {0x3F78CE39u, 0xBDA6D0B7u, 0xBE5AAD9Du, 0xBB42A0D9u}, 0u},
    {27u, 0xC31B0715u, 0xC3280A6Eu, 0x3F0C45C0u, 0x44169AB0u, {0x3F77DAE4u, 0xBDAE88C2u, 0xBE69F40Eu, 0xBB63AE63u}, 0u},
    {28u, 0xC3193999u, 0xC32772B6u, 0x3F0F7421u, 0x4443C530u, {0x3F76C86Du, 0xBDB68A32u, 0xBE7A0C38u, 0xBB83FB1Du}, 0u},
    {29u, 0xC3175069u, 0xC326D3DBu, 0x3F1275F8u, 0x446E7EB0u, {0x3F7593B9u, 0xBDBED562u, 0xBE857AEDu, 0xBB97C726u}, 0u},
    {30u, 0xC3154AE7u, 0xC3262D9Bu, 0x3F154EF1u, 0x448B7720u, {0x3F743986u, 0xBDC76A29u, 0xBE8E57E1u, 0xBBAD447Du}, 0u},
    {31u, 0xC3132879u, 0xC3257FB3u, 0x3F17FFADu, 0x4319CF00u, {0x3F72B660u, 0xBDD04692u, 0xBE979BFBu, 0xBBC480EBu}, 0u},
    {32u, 0xC310E88Bu, 0xC324C9F0u, 0x3F1A8B17u, 0x43958940u, {0x3F7106B9u, 0xBDD9690Eu, 0xBEA14552u, 0xBBDD81B0u}, 0u},
    {33u, 0xC30E8AA7u, 0xC3240C18u, 0x3F1D0CD1u, 0x43D99720u, {0x3F6F273Eu, 0xBDE2D096u, 0xBEAB4F3Fu, 0xBBF83419u}, 0u},
    {34u, 0xC30C0EEAu, 0xC32345FFu, 0x3F1F852Au, 0x440CD6B0u, {0x3F6D1497u, 0xBDEC7A7Du, 0xBEB5B4B5u, 0xBC0A4399u}, 0u},
    {35u, 0xC3097574u, 0xC3227778u, 0x3F21EFB7u, 0x442A97E0u, {0x3F6ACB6Bu, 0xBDF66448u, 0xBEC07000u, 0xBC192FE3u}, 0u},
    {36u, 0xC306BE7Eu, 0xC321A06Bu, 0x3F2451EDu, 0x4446AB60u, {0x3F684869u, 0xBE0044DAu, 0xBECB7AF7u, 0xBC28D2C7u}, 0u},
    {37u, 0xC303EA48u, 0xC320C0B4u, 0x3F26A8A7u, 0x42D32E80u, {0x3F658853u, 0xBE0572C1u, 0xBED6CEC6u, 0xBC391EB0u}, 0u},
    {38u, 0xC300F930u, 0xC31FD842u, 0x3F28F49Cu, 0x434C77C0u, {0x3F628813u, 0xBE0AB92Fu, 0xBEE263D2u, 0xBC4A00FBu}, 0u},
    {39u, 0xC2FBD76Au, 0xC31EE707u, 0x3F2B3236u, 0x439450C0u, {0x3F5F44CBu, 0xBE101491u, 0xBEEE319Cu, 0xBC5B67ABu}, 0u},
};

/* level */
static const gold_speed_t gold_level_speed[] = {{1u, 0}};
static const gold_point_t gold_level_points[] = {
    {1u, 0x00000000u, 0x00000000u, 0x3E9C4000u, 0x453EBC20u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {2u, 0x00000000u, 0x00000000u, 0x3E83D800u, 0x4520F12Cu, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {3u, 0x00000000u, 0x00000000u, 0x3E536000u, 0x45010358u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {6u, 0x00000000u, 0x00000000u, 0x3D434000u, 0x43EE57A0u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {7u, 0x00000000u, 0x00000000u, 0x3B8C0000u, 0x4401DA30u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {10u, 0x00000000u, 0x00000000u, 0x3E197000u, 0x44F6E320u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {11u, 0x00000000u, 0x00000000u, 0x3E470000u, 0x451740B4u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {12u, 0x00000000u, 0x00000000u, 0x3E728000u, 0x4531CD94u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {13u, 0x00000000u, 0x00000000u, 0x3E8DF000u, 0x43CA06C0u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {50u, 0x00000000u, 0x00000000u, 0x3F651C00u, 0x42CD1400u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {100u, 0x00000000u, 0x00000000u, 0x3F7A9000u, 0x4188B800u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
    {200u, 0x00000000u, 0x00000000u, 0x3F7FE400u, 0x00000000u, {0x3F800000u, 0x00000000u, 0x00000000u, 0x00000000u}, 2u},
};

/** \brief Every scenario */
static const gold_scenario_t gold_scenarios[] = {
    {"tilt_pos", {0, 0, 0, 0, 14189, 200, 8192}, 3u, 1u, gold_tilt_pos_speed, 1u, gold_tilt_pos_points, 12u},
    {"tilt_neg", {0, 0, 0, 0, -14189, 200, 8192}, 3u, 1u, gold_tilt_neg_speed, 1u, gold_tilt_neg_points, 9u},
    {"gyro", {0, 300, -500, 200, 9000, 3000, -13000}, 7u, 1u, gold_gyro_speed, 1u, gold_gyro_points, 8u},
    {"speed", {0, 0, 0, 0, 1000, 500, -16000}, 2u, 0u, gold_speed_speed, 4u, gold_speed_points, 39u},
    {"level", {0, 0, 0, 0, 0, 0, -16384}, 3u, 1u, gold_level_speed, 1u, gold_level_points, 12u},
};

#endif /* SENSOR_GOLD_H */
