////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//            Copyright (C) 2003-2007, Coolsand Technologies, Inc.            //
//                            All Rights Reserved                             //
//                                                                            //
//      This source code is the property of Coolsand Technologies and is      //
//      confidential.  Any  modification, distribution,  reproduction or      //
//      exploitation  of  any content of this file is totally forbidden,      //
//      except  with the  written permission  of  Coolsand Technologies.      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
/// @file tgt_board_config.h
/// That file describes the configuration of the board drivers for the specific
/// gallite g33 target.
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#ifndef _TGT_BOARD_CFG_H_
#define _TGT_BOARD_CFG_H_



// #############################################################################
// #                                                                           #
// #                      CHIP AND MANDATORY DRIVERS                           #
// #                                                                           #
// #############################################################################




// =============================================================================
// TGT_HAL_CONFIG
// =============================================================================
#include "tgt_gpio_setting.h"

// =============================================================================
// Chip version
// =============================================================================
#define TGT_HAL_CHIP_VERSION 1

// =============================================================================
// RF CLK FREQUENCY
// =============================================================================
#define TGT_HAL_RF_CLK_FREQ     HAL_SYS_FREQ_26M

// =============================================================================
// TGT_HAL_CAM_CFG
// -----------------------------------------------------------------------------
// This fills the structure HAL_CFG_CAM_T
// =============================================================================
#ifndef TGT_HAL_CAM_CFG
#define TGT_HAL_CAM_CFG                                                 \
{                                                                       \
    .camUsed            = TRUE,                                         \
    .camRstActiveH      = FALSE,                                        \
    .camPdnActiveH      = TRUE,                                         \
    .camPdnRemap        = GPIO_NONE,                                    \
    .camRstRemap        = GPIO_NONE,                                    \
    .camCsiId           = HAL_CAM_CSI_NONE,                             \
    .cam1Used           = FALSE,                                        \
    .cam1RstActiveH     = FALSE,                                        \
    .cam1PdnActiveH     = TRUE,                                         \
    .cam1PdnRemap       = GPIO_NONE,                                    \
    .cam1RstRemap       = GPIO_NONE,                                    \
    .cam1CsiId          = HAL_CAM_CSI_NONE,                             \
    .camMode            = HAL_CAM_MODE_PARALLEL,                        \
}
#endif

// =============================================================================
// TGT_HAL_PWM_CFG
// -----------------------------------------------------------------------------
/// This structure describes the PWM configuration for a given target.
/// The first field identify which PWL is used for GLOW (if any).
/// The lasts fields tell wether the pin corresponding to PWM output
/// is actually used as PWM output and not as something else (for
/// instance as a GPIO).
// =============================================================================
#ifndef TGT_HAL_PWM_CFG
#define TGT_HAL_PWM_CFG                                                 \
{                                                                       \
    .pwl0Used           = TRUE,                                         \
    .pwl1Used           = FALSE,                                        \
    .pwtUsed            = FALSE,                                        \
    .lpgUsed            = FALSE                                         \
}
#endif

// =============================================================================
// HAL_CFG_I2C_T
// -----------------------------------------------------------------------------
/// This structure describes the I2C configuration for a given target. The
/// fields tell wether the corresponding I2C pins are actually used
/// for I2C and not as something else (for instance as a GPIO).
// =============================================================================
#ifndef TGT_HAL_I2C_CFG
#define TGT_HAL_I2C_CFG                                                 \
{                                                                       \
    .i2cUsed             = TRUE,                                        \
    .i2c2Used            = TRUE,                                       \
    .i2c2PinsCam         = FALSE,                                       \
    .i2c3Used            = TRUE,                                        \
}
#endif

// =============================================================================
// TGT_HAL_I2S_CFG
// -----------------------------------------------------------------------------
/// This structure describes the I2S configuration for a given target. The
/// fields tell wether the corresponding I2S pin is actually used
/// for I2S and not as something else (for instance as a GPIO).
// =============================================================================
#ifndef TGT_HAL_I2S_CFG
#define TGT_HAL_I2S_CFG                                                 \
{                                                                       \
    .doUsed             = TRUE,                                         \
    .di0Used            = TRUE,                                         \
    .di1Used            = FALSE,                                        \
    .di2Used            = FALSE,                                        \
}
#endif

// =============================================================================
// TGT_HAL_UART_CFG
// -----------------------------------------------------------------------------
/// Used to describes a configuration for used pin by an UART for a given target.
// =============================================================================
#ifndef TGT_HAL_UART_CFG
#define TGT_HAL_UART_CFG                                                \
{                                                                       \
    HAL_UART_CONFIG_FLOWCONTROL,                                        \
    HAL_UART_CONFIG_DATA,                                               \
    HAL_UART_CONFIG_FLOWCONTROL,                                        \
}
#endif

// =============================================================================
// TGT_HAL_MODEM_SPI_CFG
// -----------------------------------------------------------------------------
/// This structure describes the SPI configuration for a given target. The first
/// fields tell wether the pin corresponding to chip select is actually used
/// as a chip select and not as something else (for instance as a GPIO).
/// Then, the polarity of the Chip Select is given. It is only relevant
/// if the corresponding Chip Select is used as a Chip Select.
/// Finally which pin is used as input, Can be none, one or the other.
/// On most chip configuration the input 0 (di0) is on the output pin: SPI_DIO
// =============================================================================
#ifndef TGT_HAL_MODEM_SPI_CFG
#define TGT_HAL_MODEM_SPI_CFG                                           \
{                                                                       \
    {                                                                   \
    .cs0Used            = FALSE,                                        \
    .cs1Used            = FALSE,                                        \
    .cs2Used            = FALSE,                                        \
    .cs3Used            = FALSE,                                        \
    .cs0ActiveLow       = TRUE,                                         \
    .cs1ActiveLow       = TRUE,                                         \
    .cs2ActiveLow       = TRUE,                                         \
    .cs3ActiveLow       = TRUE,                                         \
    .di0Used            = TRUE,                                         \
    .di1Used            = FALSE,                                        \
    },                                                                  \
}
#endif

// =============================================================================
// TGT_HAL_SPI_CFG
// -----------------------------------------------------------------------------
/// This structure describes the SPI configuration for a given target. The first
/// fields tell wether the pin corresponding to chip select is actually used
/// as a chip select and not as something else (for instance as a GPIO).
/// Then, the polarity of the Chip Select is given. It is only relevant
/// if the corresponding Chip Select is used as a Chip Select.
/// Finally which pin is used as input, Can be none, one or the other.
/// On most chip configuration the input 0 (di0) is on the output pin: SPI_DIO
// =============================================================================
#ifndef TGT_HAL_SPI_CFG
#define TGT_HAL_SPI_CFG                                                 \
{                                                                       \
    {                                                                   \
    .cs0Used            = FALSE,                                         \
    .cs1Used            = FALSE,                                        \
    .cs2Used            = FALSE,                                        \
    .cs3Used            = FALSE,                                        \
    .cs0ActiveLow       = TRUE,                                         \
    .cs1ActiveLow       = TRUE,                                         \
    .cs2ActiveLow       = TRUE,                                         \
    .cs3ActiveLow       = TRUE,                                         \
    .di0Used            = TRUE,                                         \
    .di1Used            = FALSE,                                        \
    },                                                                  \
    {                                                                   \
    .cs0Used            = TRUE,                                        \
    .cs1Used            = TRUE,                                        \
    .cs2Used            = FALSE,                                        \
    .cs3Used            = FALSE,                                        \
    .cs0ActiveLow       = TRUE,                                         \
    .cs1ActiveLow       = TRUE,                                         \
    .cs2ActiveLow       = TRUE,                                         \
    .cs3ActiveLow       = TRUE,                                         \
    .di0Used            = TRUE,                                         \
    .di1Used            = FALSE,                                        \
    },                                                                  \
}
#endif

// =============================================================================
// TGT_HAL_SDMMC_CFG
// -----------------------------------------------------------------------------
/// This structure describes the SDMMC configuration for a given target. The
/// fields tell wether the corresponding I2S pin is actually used
/// for I2S and not as something else (for instance as a GPIO).
// =============================================================================
#ifndef TGT_HAL_SDMMC_CFG
#define TGT_HAL_SDMMC_CFG                                               \
{                                                                       \
    .sdmmcUsed          = TRUE,                                         \
    .sdmmc2Used         = TRUE,                                         \
    .sdmmc3Used         = TRUE,                                        \
}
#endif

// =============================================================================
// TGT_HAL_GOUDA_CFG
// -----------------------------------------------------------------------------
/// This structure describes the GOUDA configuration for a given target.
/// The first fields tell wether the pin corresponding to chip select is
/// actually used as a chip select and not as something else (for instance
/// as a GPIO). If none are used, the GOUDA is considered unused.
// =============================================================================
#ifndef TGT_HAL_GOUDA_CFG
#define TGT_HAL_GOUDA_CFG                                               \
{                                                                       \
    .cs0Used            = TRUE,                                        \
    .cs1Used            = FALSE,                                        \
    .lcd16_23Cam        = FALSE,                                        \
    .lcdMode            = HAL_LCD_MODE_PARALLEL_16BIT,                  \
}
#endif

// =============================================================================
// TGT_HAL_IO_DRIVE
// -----------------------------------------------------------------------------
/// This structure describes the IO Drive configuration for a given target.
// =============================================================================
#ifndef TGT_HAL_IO_DRIVE
#define TGT_HAL_IO_DRIVE                                                \
{                                                                       \
    {                                                                   \
        .vDdrDomain     = 4,                                            \
        .vPsram1Domain  = 4,                                            \
        .vPsram2Domain  = 4,                                            \
        .vNFlashDomain  = 2,                                            \
        .vLcd1Domain    = 4,                                            \
        .vLcd2Domain    = 4,                                            \
        .vSDat1Domain   = 3,                                            \
        .vSDat2Domain   = 3,                                            \
        .vCamDomain     = 0,                                            \
        .vSim1Domain    = 0,                                            \
        .vSim2Domain    = 0,                                            \
        .vSim3Domain    = 0,                                            \
        .vGpioDomain    = 0,                                            \
    }                                                                   \
}
#endif

// =============================================================================
// TGT_HAL_CONFIG
// =============================================================================
#ifndef TGT_HAL_CONFIG
#define TGT_HAL_CONFIG                                                  \
{                                                                       \
    .chipVersion        = TGT_HAL_CHIP_VERSION,                         \
    .rfClkFreq          = TGT_HAL_RF_CLK_FREQ,                          \
    .useLpsCo1          = FALSE,                                        \
    .keyInMask          = 0x00,                                         \
    .keyOutMask         = 0x00,                                         \
    .pwmCfg             = TGT_HAL_PWM_CFG,                              \
    .i2cCfg             = TGT_HAL_I2C_CFG,                              \
    .i2sCfg             = TGT_HAL_I2S_CFG,                              \
    .uartCfg            = TGT_HAL_UART_CFG,                             \
    .modemSpiCfg        = TGT_HAL_MODEM_SPI_CFG,                        \
    .spiCfg             = TGT_HAL_SPI_CFG,                              \
    .sdmmcCfg           = TGT_HAL_SDMMC_CFG,                            \
    .camCfg             = TGT_HAL_CAM_CFG,                              \
    .goudaCfg           = TGT_HAL_GOUDA_CFG,                            \
    .parallelNandUsed   = -1,                                         \
    .hostUartUsed       = FALSE,                                         \
    .hostClkUsed        = FALSE,                                        \
    .clkOutUsed         = TRUE,                                         \
    .useClk32k          = FALSE,                                        \
    .noConnectGpio_C    = TGT_HAL_NO_CONNECT_GPIO_C,                    \
    .usedGpio_C         = TGT_HAL_USED_GPIO_C,                          \
    .usedTco            = TGT_HAL_USED_TCO,                             \
    .noConnectGpio_A    = TGT_AP_HAL_NO_CONNECT_GPIO_A,                 \
    .usedGpio_A         = TGT_AP_HAL_USED_GPIO_A,                       \
    .usedGpo_A          = TGT_AP_HAL_USED_GPO_A,                        \
    .noConnectGpio_B    = TGT_AP_HAL_NO_CONNECT_GPIO_B,                 \
    .usedGpio_B         = TGT_AP_HAL_USED_GPIO_B,                       \
    .noConnectGpio_D    = TGT_AP_HAL_NO_CONNECT_GPIO_D,                 \
    .usedGpio_D         = TGT_AP_HAL_USED_GPIO_D,                       \
    .ioDrive            = TGT_HAL_IO_DRIVE                              \
}
#endif


// =============================================================================
// KEY Mapping
// =============================================================================
#ifndef KEY_MAP

#define KEY_ROW_NUM 8
#define KEY_COL_NUM 8

#define TGT_KEY_NB (KEY_ROW_NUM * KEY_COL_NUM)

#define KEY_MAP                                                           \
{                                                                         \
    KP_STAR, KP_7, KP_4, KP_1, KP_UP , KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_0, KP_8, KP_5, KP_2, KP_DW, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_POUND,KP_9, KP_6, KP_3, KP_RT , KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_DEL, KP_SR, KP_BACK ,KP_OK ,KP_LT , KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_FM, KP_SL, KP_UNMAPPED, KP_UNMAPPED, KP_VD, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
    KP_SND, KP_END, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, KP_UNMAPPED, \
}
#endif // KEY_MAP


#ifndef KEY_BOOT_DOWNLOAD

#define KEY_BOOT_DOWNLOAD { KP_0, KP_OK, }

#endif // KEY_BOOT_DOWNLOAD


// =============================================================================
// RFD config
// =============================================================================
#ifndef TGT_RFD_CONFIG
#define TGT_RFD_CONFIG

#include "hal_tcu.h"
#define TGT_XCV_CONFIG    {.RST = TCO_UNUSED, .PDN  = TCO(11)              }
#define TGT_PA_CONFIG     {.ENA = TCO_UNUSED, .TXEN = TCO_UNUSED, .BS  = TCO_UNUSED }
#define TGT_SW_CONFIG     {.SW1 = TCO(2), .SW2  = TCO(3), .SW3 = TCO(4) }

// Note: Some XCV maybe have different control pin names, so someone who develop
//       the target configuration should explain the pin usage as below.
//
// FIXME Fix that with proper knowledge !
// PA->ENA is VLOGIC pin for SKY77518, MODEN for TQM4002, MOD for RDA6216
// PA-TXEN is BIAS for RDA6216 ?
//
#endif // TGT_RFD_CONFIG


// =============================================================================
// PMD config
// -----------------------------------------------------------------------------
/// This fills the structure PMD_CONFIG_T
// =============================================================================
#ifndef TGT_PMD_CONFIG

#define TGT_PMD_CONFIG                                                  \
    {                                                                   \
        .power =                                                        \
        {                                                               \
            { /* PMD_POWER_MIC          : Micro bias enable */          \
                .ldo            = {  .opal = PMD_LDO_MIC},              \
                .polarity       = TRUE,                                 \
                .shared         = TRUE, /* with PMD_POWER_EARPIECE */   \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_CAMERA       : Camera LDO enable */          \
                .ldo            = { .opal = PMD_LDO_CAM|PMD_LDO_RF},    \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_AUDIO        : Audio LDO enable */           \
                .ldo            = { .opal = PMD_LDO_ABB},               \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_STEREO_DAC   : Stereo DAC LDO enable */      \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_LOUD_SPEAKER : Loud Speaker enable */        \
                .ldo            = { .pin = { .gpioId = HAL_GPIO_NONE/*1*/ } },  \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_PA           : RF Power Amplifier */         \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_USB          : USB LDOs enable */            \
                .ldo            = { .opal = PMD_LDO_USB},               \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_SDMMC        : SD/MMC LDO enable */          \
                .ldo            = { .opal = PMD_LDO_MMC},               \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_FM           : FM LDO enable */              \
                .ldo            = { .opal = PMD_LDO_ABB},               \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_EARPIECE     : Ear Piece Micro bias enable */\
                .ldo            = { .opal = PMD_LDO_MIC},               \
                .polarity       = TRUE,                                 \
                .shared         = TRUE, /* with PMD_POWER_MIC */        \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_BT           : BlueTooth LDOs enable */      \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_CAMERA_FLASH : Camera Flash Light enable */  \
                .ldo            = { .pin = {.gpoId = HAL_GPO_0}},       \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_LCD          : (main) LCD LDO enable */      \
                .ldo            = { .opal = PMD_LDO_LCD|PMD_LDO_RF},               \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
            { /* PMD_POWER_I2C          : I2C LDO enable */             \
                /* Inside this chip, if I2C2 is used, and it is     */  \
                /*   multiplexed on camera pins, PMD_LDO_CAM must   */  \
                /*   be specified here to supply power to I2C2 I/O. */  \
                /* On this board, if any LDO (except for always-on  */  \
                /*   LDOs like vPad) supplies power to I2C pull-up  */  \
                /*   resistor, it must be specified here too.       */  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .polarity       = TRUE,                                 \
                .shared         = FALSE,                                \
                .powerOnState   = FALSE,                                \
            },                                                          \
        },                                                              \
        .level =                                                        \
        {                                                               \
            { /* PMD_LEVEL_SIM          : Sim class voltage */          \
                .type           = PMD_LEVEL_TYPE_NONE,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_KEYPAD       : KeyPad Back Light level */    \
                .type           = PMD_LEVEL_TYPE_OPAL,                  \
                .ldo            = { .pin = { .gpoId  = HAL_GPO_NONE}},  \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LCD          : (main) LCD Back Light level*/ \
                .type           = PMD_LEVEL_TYPE_BACKLIGHT,             \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_SUB_LCD      : Sub LCD Back Light level */   \
                .type           = PMD_LEVEL_TYPE_NONE,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LED0         : LED0 Light level */           \
                .type           = PMD_LEVEL_TYPE_OPAL,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LED1         : LED1 Light level */           \
                .type           = PMD_LEVEL_TYPE_OPAL,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LED2         : LED2 Light level */           \
                .type           = PMD_LEVEL_TYPE_OPAL,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LED3         : LED3 Light level */           \
                .type           = PMD_LEVEL_TYPE_OPAL,                  \
                .ldo            = { .pin = { .gpioId  = HAL_GPIO_NONE}}, \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_VIBRATOR     : Vibrator control level */     \
                .type           = PMD_LEVEL_TYPE_LDO,                   \
                .ldo            = { .opal = PMD_LDO_VIBR},              \
                .powerOnState   = 0,                                    \
            },                                                          \
            { /* PMD_LEVEL_LOUD_SPEAKER : loudspeaker gain */           \
                .type           = PMD_LEVEL_TYPE_NONE,                  \
                .ldo            = { .opal = PMD_LDO_NONE},              \
                .powerOnState   = 0,                                    \
            },                                                          \
        },                                                              \
        /* If any LDO is configured in ldoEnableNormal, it cannot   */  \
        /*   be controlled as a POWER or LEVEL LDO any more.        */  \
        .ldoEnableNormal             = 0,                               \
        .ldoEnableLowPower           = 0,                               \
        .ldoCamIs2_8                 = TRUE,                            \
        .ldoLcdIs2_8                 = TRUE,                            \
        .ldoMMCIsHigh                = TRUE,                            \
        .ldoIbrIsHigh                = FALSE,                           \
        .ldoRfIs2_8                  = TRUE,                            \
        .ldoPadIs2_8                 = TRUE,                            \
        .lowerVPadLowPower           = FALSE,                           \
        .vBuck1LowPower              = 6,                               \
        .vBuck3LowPower              = 12,                              \
        .vBuck4Usage                 = PMD_VBUCK4_USAGE_2P8V,           \
        .vBuck4LowPower              = 13,                              \
        .vSim3Usage                  = PMD_VSIM3_USAGE_2P8V_ALWAYS_ON,             \
        .batteryGpadcChannel         = HAL_ANA_GPADC_CHAN_7,            \
        .batteryLevelChargeTermMV    = 4200,                            \
        .batteryLevelRechargeMV      = 4160,                            \
        .batteryLevelFullMV          = 4160,                            \
        .batteryChargeTimeout        = 6 HOURS,                         \
        .batteryOffsetHighActivityMV = 45,                              \
        .batteryOffsetPowerDownChargeMV = -150,                         \
        .powerOnVoltageMV            = 3400,                            \
        .powerDownVoltageMV          = 3400,                            \
        .batteryChargeCurrent        = PMD_CHARGER_1200MA,              \
        .batteryChargeCurrentOnUsb   = PMD_CHARGER_1200MA,              \
        .batteryTsDetect             = FALSE,                           \
        .chargerGpadcChannel         = HAL_ANA_GPADC_CHAN_6,            \
        .chargerLevelUpperLimit      = 6500,                            \
        .chargerOffsetBackToNormal   = -200,                            \
        .temperatureGpadcChannel     = HAL_ANA_GPADC_CHAN_1,            \
        .temperatureUpperLimit       = 0,                              \
        .temperatureOffsetBackToNormal = -7,                           \
        .batteryMVScreenAntiFlicker  = 3600,                            \
        .batteryOffsetScreenNormal   = 100,                             \
        .earpieceDetectGpio          = HAL_GPIO_NONE,                   \
        .earpieceGpadcChannel        = HAL_ANA_GPADC_CHAN_0,            \
    }

#define TGT_PMD_BATT_CAP_CURVE_STEP_MV  20

#define TGT_PMD_BATT_CAP_CURVE_ARRAY                                    \
 { 0,  0,  0,  0,  0,  0,  1,  3,  6,  8,                             \
   9, 10, 11, 12, 14, 18, 24, 30, 36, 42,                               \
  48, 53, 58, 63, 66, 71, 75, 78, 80, 83,                               \
  85, 87, 89, 91, 93, 95, 97, 99, 100 }

#define TGT_PMD_TEMPERATURE_CURVE_ARRAY                                 \
    { { 50, 33220 }, { 57, 25060 } }

#endif // TGT_PMD_CONFIG


// =============================================================================
// TGT_AUD_CONFIG
// -----------------------------------------------------------------------------
/// Audio interface configuration
// =============================================================================
#ifndef TGT_AUD_CONFIG
#define TGT_AUD_CONFIG

#define TGT_AUD_CONFIG_RECEIVER_DRIVER             CodecGallite // Ti
#define TGT_AUD_CONFIG_RECEIVER_PARAM              0
#define TGT_AUD_CONFIG_RECEIVER_OUTPUT_PATH        AUD_SPK_RECEIVER
#define TGT_AUD_CONFIG_RECEIVER_OUTPUT_TYPE        AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_RECEIVER_INPUT_PATH         AUD_MIC_RECEIVER

#define TGT_AUD_CONFIG_EAR_PIECE_DRIVER            CodecGallite // Ti
#define TGT_AUD_CONFIG_EAR_PIECE_PARAM             0
#define TGT_AUD_CONFIG_EAR_PIECE_OUTPUT_PATH       AUD_SPK_EAR_PIECE
#define TGT_AUD_CONFIG_EAR_PIECE_OUTPUT_TYPE       AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_EAR_PIECE_INPUT_PATH        AUD_MIC_EAR_PIECE

#define TGT_AUD_CONFIG_LOUD_SPEAKER_DRIVER         CodecGallite // Ti
#define TGT_AUD_CONFIG_LOUD_SPEAKER_PARAM          0
#define TGT_AUD_CONFIG_LOUD_SPEAKER_OUTPUT_PATH    AUD_SPK_LOUD_SPEAKER //AUD_SPK_EAR_PIECE
#define TGT_AUD_CONFIG_LOUD_SPEAKER_OUTPUT_TYPE    AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_LOUD_SPEAKER_INPUT_PATH     AUD_MIC_LOUD_SPEAKER

#define TGT_AUD_CONFIG_BT_DRIVER                   Bt
#define TGT_AUD_CONFIG_BT_PARAM                    0
#define TGT_AUD_CONFIG_BT_OUTPUT_PATH              AUD_SPK_EAR_PIECE
#define TGT_AUD_CONFIG_BT_OUTPUT_TYPE              AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_BT_INPUT_PATH               AUD_MIC_RECEIVER

#define TGT_AUD_CONFIG_FM_DRIVER                   Fm
#define TGT_AUD_CONFIG_FM_PARAM                    0
#define TGT_AUD_CONFIG_FM_OUTPUT_PATH              AUD_SPK_EAR_PIECE
#define TGT_AUD_CONFIG_FM_OUTPUT_TYPE              AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_FM_INPUT_PATH               AUD_MIC_RECEIVER

//atv aud config
#define TGT_AUD_CONFIG_TV_DRIVER                   Tv
#define TGT_AUD_CONFIG_TV_PARAM                    0
#define TGT_AUD_CONFIG_TV_OUTPUT_PATH              AUD_SPK_EAR_PIECE
#define TGT_AUD_CONFIG_TV_OUTPUT_TYPE              AUD_SPEAKER_STEREO
#define TGT_AUD_CONFIG_TV_INPUT_PATH               AUD_MIC_RECEIVER

#endif // TGT_AUD_CONFIG


// #############################################################################
// #                                                                           #
// #                           OPTIONNAL DRIVERS                               #
// #                                                                           #
// #############################################################################

#ifdef TGT_WITH_TS_MODEL_rda1203_gallite
// =============================================================================
// TSD config
// -----------------------------------------------------------------------------
/// This fills the TSD_CONFIG_T structure
// =============================================================================
#ifndef TGT_TSD_CONFIG
#define TGT_TSD_CONFIG                                                  \
    {                                                                   \
        .penGpio        = HAL_GPIO_NONE,                                \
        .debounceTime   = 5*HAL_TICK1S/1000,                            \
        .downPeriod     = 3,                                            \
        .upPeriod       = 3,                                            \
        .maxError       = 0x50                                          \
    }

#endif // TGT_TSD_CONFIG
#endif // TGT_WITH_TS_MODEL_rda1203_gallite

#ifdef TGT_WITH_TS_MODEL_rda8810_ap
#ifndef TGT_TSD_CONFIG
#define TGT_TSD_CONFIG                                                  \
    {                                                                   \
        .penGpio        = HAL_GPIO_2,                                \
        .debounceTime   = 1*HAL_TICK1S/1000,                            \
        .downPeriod     = 3,                                            \
        .upPeriod       = 3,                                            \
        .maxError       = 0x50                                          \
    }

#endif // TGT_TSD_CONFIG
#endif // TGT_WITH_TS_MODEL_rda8810_ap

#ifdef TGT_WITH_TS_MODEL_rda8810_ap_gtp868
#ifndef TGT_TSD_CONFIG
#define TGT_TSD_CONFIG                                                  \
    {                                                                   \
        .penGpio        = HAL_GPIO_2,                                \
        .debounceTime   = 1*HAL_TICK1S/1000,                            \
        .downPeriod     = 3,                                            \
        .upPeriod       = 3,                                            \
        .maxError       = 0x50                                          \
    }

#endif // TGT_TSD_CONFIG
#endif // TGT_WITH_TS_MODEL_rda8810_ap_gtp868

#ifdef TGT_WITH_TS_MODEL_ap_msg2133
#ifndef TGT_TSD_CONFIG
#define TGT_TSD_CONFIG                                                  \
    {                                                                   \
        .ts_Gpio_int       = HAL_GPIO_1,                                          \
        .ts_Gpio_rst      = HAL_GPIO_24,                                         \
        .rst_pd_level      = 1,                                          \
        .int_pd_level      = 1,                                          \
        .i2c_id      = 0x26,                                          \
        .ts_int_rise      = FALSE,                                          \
    }

#endif // TGT_TSD_CONFIG
#endif // TGT_WITH_TS_MODEL_rda8810_ap_gtp868


#ifdef TGT_WITH_GPIOI2C_MODEL_i2c_gpio
// =============================================================================
//  GPIO I2C config
// -----------------------------------------------------------------------------
/// This fills the GPIOI2C CONFIG_T structure
// =============================================================================
#ifndef TGT_GPIOI2C_CONFIG
#define TGT_GPIOI2C_CONFIG                                              \
    {                                                                   \
	.i2c_gpio_Bps       = GPIO_I2C_BPS_80K ,                        \
        .scl_i2c_gpio       = HAL_GPIO_4 ,                             \
        .scl_i2c_gpo        = HAL_GPO_NONE ,                            \
        .sda_i2c            = HAL_GPIO_5                               \
    }
#endif // TGT_GPIOI2C_CONFIG
#endif // TGT_WITH_GPIOI2C_MODEL_i2c_gpio

// ?d config
// -----------------------------------------------------------------------------
/// @todo add other driver configuration here if needed
// =============================================================================


// #############################################################################
// #                                                                           #
// #                                 SERVICES                                  #
// #                                                                           #
// #############################################################################


// =============================================================================
// ?s config
// -----------------------------------------------------------------------------
/// @todo add other service configuration here if needed
// =============================================================================


#endif //_TGT_BOARD_CFG_H_

