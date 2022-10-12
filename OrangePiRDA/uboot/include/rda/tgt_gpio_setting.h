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
/// @file tgt_ap_gpio_setting.h
/// That file describes the configuration of the AP board gpio setting for the specific
/// 8810 base target.
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#ifndef __TGT_GPIO_SETTING_H__
#define __TGT_GPIO_SETTING_H__

//Compare to BB_GPIO_mapping
#define AS_ALT_FUNC     0
/// pin used as it's GPIO function
#define AS_GPIO         1
/// GPIO not connected, set to driving 0 for low power
#define NOT_CONNECTED   2

// -----------------------------------------------------------------------------
// GPIO(0)
#define TGT_HAL_GPIO_C_0_USED AS_GPIO
// GPIO(1)  // GPIO_C1 INT:nil:nil:nil
#define TGT_HAL_GPIO_C_1_USED AS_GPIO
// GPIO(2)  // GPIO_C2 INT:nil:nil:nil
#define TGT_HAL_GPIO_C_2_USED NOT_CONNECTED
// GPIO(3)  // GPIO_C3 INT:nil:nil:nil
#define TGT_HAL_GPIO_C_3_USED NOT_CONNECTED
// GPIO(4)  // GPIO_C4 INT:nil:nil:nil
#define TGT_HAL_GPIO_C_4_USED AS_GPIO
// GPIO(5)  // GPIO_C5 INT:nil:nil:nil
#define TGT_HAL_GPIO_C_5_USED AS_GPIO
// GPIO(6)  // UART1 Receive Data:nil:nil:nil
#define TGT_HAL_GPIO_C_6_USED AS_ALT_FUNC
// GPIO(7)  // HST_RXD:GPIO_7:UART2_RXD-UART1_DTR
#define TGT_HAL_GPIO_C_7_USED AS_ALT_FUNC
// GPIO(8)  // HST_TXD:GPIO_8:UART2_TXD-UART1 Data Carrier Detect
#define TGT_HAL_GPIO_C_8_USED AS_ALT_FUNC
// GPIO(9)  // SD CLK:nil:nil:nil
#define TGT_HAL_GPIO_C_9_USED AS_ALT_FUNC
// GPIO(10) // SD CMD:nil:nil:nil
#define TGT_HAL_GPIO_C_10_USED AS_ALT_FUNC
// GPIO(11) // SD data0:nil:nil:nil
#define TGT_HAL_GPIO_C_11_USED AS_ALT_FUNC
// GPIO(12) // SD data1:nil:nil:nil
#define TGT_HAL_GPIO_C_12_USED AS_ALT_FUNC
// GPIO(13) // SD data2:nil:nil:nil
#define TGT_HAL_GPIO_C_13_USED AS_ALT_FUNC
// GPIO(14) // SD data3:nil:nil:nil
#define TGT_HAL_GPIO_C_14_USED AS_ALT_FUNC
// GPIO(15) // SD CLK:nil:nil:nil
#define TGT_HAL_GPIO_C_15_USED AS_ALT_FUNC
// GPIO(16) // SD CMD:nil:nil:nil
#define TGT_HAL_GPIO_C_16_USED AS_ALT_FUNC
// GPIO(17) // SD data0:nil:nil:nil
#define TGT_HAL_GPIO_C_17_USED AS_ALT_FUNC
// GPIO(18) // SD data1:nil:nil:nil
#define TGT_HAL_GPIO_C_18_USED AS_ALT_FUNC
// GPIO(19) // SD data2:nil:nil:nil
#define TGT_HAL_GPIO_C_19_USED AS_ALT_FUNC
// GPIO(20) // SD data3:nil:nil:nil
#define TGT_HAL_GPIO_C_20_USED AS_ALT_FUNC
// GPIO(21) // SPI1_CLK:SPI_BB_CLK:nil:nil
#define TGT_HAL_GPIO_C_21_USED AS_ALT_FUNC
// GPIO(22) // SPI1_CS_0:SPI_BB_CS_0:nil:nil
#define TGT_HAL_GPIO_C_22_USED AS_ALT_FUNC
// GPIO(23) // SPI1_DIO:SPI_BB_DIO:nil:nil
#define TGT_HAL_GPIO_C_23_USED AS_ALT_FUNC
// GPIO(24) // SPI1_DI:SPI_BB_DI:nil:nil
#define TGT_HAL_GPIO_C_24_USED AS_GPIO
// GPIO(25) // GPIO_C25:SIM2_RST:nil:nil
#define TGT_HAL_GPIO_C_25_USED AS_ALT_FUNC
// GPIO(26) // GPIO_C26:SIM2_CLK:nil:nil
#define TGT_HAL_GPIO_C_26_USED AS_ALT_FUNC
// GPIO(27) // GPIO_C27:SIM2_DIO:nil:nil
#define TGT_HAL_GPIO_C_27_USED AS_ALT_FUNC
// GPIO(28) // GPIO_C28:SIM3_RST:nil:nil
#define TGT_HAL_GPIO_C_28_USED AS_ALT_FUNC
// GPIO(29) // GPIO_C29:SIM3_CLK:nil:nil
#define TGT_HAL_GPIO_C_29_USED AS_ALT_FUNC
// GPIO(30) // GPIO_C30:SIM3_DIO:nil:nil
#define TGT_HAL_GPIO_C_30_USED AS_ALT_FUNC
// GPIO(31) // LCD DATA
#define TGT_HAL_GPIO_C_31_USED AS_ALT_FUNC

// -----------------------------------------------------------------------------
// Each TCO can be assigned one of the following values:
// 0 : unused
// 1 : used
// -----------------------------------------------------------------------------
// TCO(0)   // GPIO_B3 INT--KEYOUT_0--TCO_0
#define TGT_HAL_TCO_0_USED 0
// TCO(1)   // GPIO_B4 INT--KEYOUT_1--TCO_1
#define TGT_HAL_TCO_1_USED 0
// TCO(2)   // GPIO_B5 INT--KEYOUT_2--TCO_2
#define TGT_HAL_TCO_2_USED 0
// TCO(3)
#define TGT_HAL_TCO_3_USED 0
// TCO(4)
#define TGT_HAL_TCO_4_USED 0
// TCO(5)
#define TGT_HAL_TCO_5_USED 0
// TCO(6)
#define TGT_HAL_TCO_6_USED 0
// TCO(7)
#define TGT_HAL_TCO_7_USED 0
// TCO(8)
#define TGT_HAL_TCO_8_USED 0
// TCO(9)
#define TGT_HAL_TCO_9_USED 0
// TCO(10)
#define TGT_HAL_TCO_10_USED 0
// TCO(11)
#define TGT_HAL_TCO_11_USED 0

// -----------------------------------------------------------------------------
// GPIO(0)  // GPIO_A0 INT:I2C2_SCL:nil:Monitor_0
#define TGT_AP_HAL_GPIO_A_0_USED AS_GPIO
// GPIO(1)  // GPIO_A1 INT:I2C2_SDA:nil:Monitor_1
#define TGT_AP_HAL_GPIO_A_1_USED AS_GPIO
// GPIO(2)  // GPIO_A2 INT:SPI2_CLK:nil:Monitor_2
#define TGT_AP_HAL_GPIO_A_2_USED AS_ALT_FUNC
// GPIO(3)  // GPIO_A3 INT:SPI2_DIO:nil:Monitor_3
#define TGT_AP_HAL_GPIO_A_3_USED AS_ALT_FUNC
// GPIO(4)  // GPIO_A4 INT:SPI2_DI:nil:Monitor_4
#define TGT_AP_HAL_GPIO_A_4_USED AS_ALT_FUNC
// GPIO(5)  // GPIO_A5 INT:SPI2_CS_0:nil:Monitor_5
#define TGT_AP_HAL_GPIO_A_5_USED AS_ALT_FUNC
// GPIO(6)  // GPIO_A6 INT:SPI2_CS_1:KEYIN_3:Monitor_6
#define TGT_AP_HAL_GPIO_A_6_USED AS_ALT_FUNC
// GPIO(7)  // GPIO_A7 INT:KEYIN_4:LPSCO_1:Monitor_7
#define TGT_AP_HAL_GPIO_A_7_USED AS_GPIO
// GPIO(8)  // GPIO_A8:Clock Output
#define TGT_AP_HAL_GPIO_A_8_USED AS_ALT_FUNC
// GPIO(9)  // Audio BCK:nil:nil:nil
#define TGT_AP_HAL_GPIO_A_9_USED AS_ALT_FUNC
// GPIO(10) // Audio LRCK:DAI_CLK:DAI_SIMPLE_CLK:nil
#define TGT_AP_HAL_GPIO_A_10_USED AS_ALT_FUNC
// GPIO(11) // Audio Serial Data In 0:DAI_DI:DAI_SIMPLE_DI:nil
#define TGT_AP_HAL_GPIO_A_11_USED AS_ALT_FUNC
// GPIO(12) // Audio Serial Data In 1:DAI_RST:DAI_SIMPLE_RST:nil
#define TGT_AP_HAL_GPIO_A_12_USED NOT_CONNECTED
// GPIO(13) // Audio Serial Data Out:DAI_DO:DAI_SIMPLE_DO:nil
#define TGT_AP_HAL_GPIO_A_13_USED AS_ALT_FUNC
// GPIO(14) // UART1 Transmit Data:nil:nil:nil
#define TGT_AP_HAL_GPIO_A_14_USED AS_ALT_FUNC
// GPIO(15) // UART1 Clear To Send:KEYIN_7:nil:nil
#define TGT_AP_HAL_GPIO_A_15_USED AS_GPIO
// GPIO(16) // UART1 Request To Send:KEYOUT_7:nil:nil
#define TGT_AP_HAL_GPIO_A_16_USED AS_ALT_FUNC
// GPIO(17) // SPI1_CS_1:SPI_BB_CS_1:nil:nil
#define TGT_AP_HAL_GPIO_A_17_USED AS_GPIO
// GPIO(18) // LCD DATA6:DSI_D2_P:nil:nil
#define TGT_AP_HAL_GPIO_A_18_USED AS_ALT_FUNC
// GPIO(19) // LCD DATA7:DSI_D2_N:nil:nil
#define TGT_AP_HAL_GPIO_A_19_USED AS_ALT_FUNC
// GPIO(20) // LCD_WR:DSI_D3_P:lcd_rgb_de:M_SPI_D3
#define TGT_AP_HAL_GPIO_A_20_USED AS_ALT_FUNC
// GPIO(21) // LCD_RS:DSI_D3_N:lcd_rgb_hsync:M_SPI_D2
#define TGT_AP_HAL_GPIO_A_21_USED AS_ALT_FUNC
// GPIO(22) // LCD_RD:SPI_LCD_SDC:lcd_rgb_vsync:M_SPI_D1
#define TGT_AP_HAL_GPIO_A_22_USED AS_ALT_FUNC
// GPIO(22) // LCD_FMARK:SPI_LCD_CLK:lcd_rgb_clk:M_SPI_CLK
#define TGT_AP_HAL_GPIO_A_23_USED AS_ALT_FUNC
// GPIO(24) // LCD_DATA_8:NFD_8:nil:nil
#define TGT_AP_HAL_GPIO_A_24_USED AS_ALT_FUNC
// GPIO(25) // LCD_DATA_9:NFD_9:nil:nil
#define TGT_AP_HAL_GPIO_A_25_USED AS_ALT_FUNC
// GPIO(26) // LCD_DATA_10:NFD_10:nil:nil
#define TGT_AP_HAL_GPIO_A_26_USED AS_ALT_FUNC
// GPIO(27) // LCD_DATA_11:NFD_11:nil:nil
#define TGT_AP_HAL_GPIO_A_27_USED AS_ALT_FUNC
// GPIO(28) // LCD_DATA_12:NFD_12:nil:nil
#define TGT_AP_HAL_GPIO_A_28_USED AS_ALT_FUNC
// GPIO(29) // LCD_DATA_13:NFD_13:nil:nil
#define TGT_AP_HAL_GPIO_A_29_USED AS_ALT_FUNC
// GPIO(30) // LCD_DATA_14:NFD_14:nil:nil
#define TGT_AP_HAL_GPIO_A_30_USED AS_ALT_FUNC
// GPIO(31) // LCD_DATA_15:NFD_15:nil:nil
#define TGT_AP_HAL_GPIO_A_31_USED AS_ALT_FUNC

// ::::::::::::::::::::::::::::::::::::::-
// GPIO(0)  // GPIO_B0 INT:KEYIN_0:KEYIN_0:Monitor_8
#define TGT_AP_HAL_GPIO_B_0_USED NOT_CONNECTED
// GPIO(1)  // GPIO_B1 INT:KEYIN_1:SPI1_CS_2:Monitor_9
#define TGT_AP_HAL_GPIO_B_1_USED AS_GPIO
// GPIO(2)  // GPIO_B2 INT:KEYIN_2:I2S_DI_2:Monitor_10
#define TGT_AP_HAL_GPIO_B_2_USED AS_GPIO
// GPIO(3)  // GPIO_B3 INT:KEYOUT_0:TCO_0:Monitor_11
#define TGT_AP_HAL_GPIO_B_3_USED AS_GPIO
// GPIO(4)  // GPIO_B4 INT:KEYOUT_1:TCO_1:Monitor_12
#define TGT_AP_HAL_GPIO_B_4_USED NOT_CONNECTED
// GPIO(5)  // GPIO_B5 INT:KEYOUT_2:TCO_2:Monitor_13
#define TGT_AP_HAL_GPIO_B_5_USED AS_GPIO
// GPIO(6)  // GPIO_B6 INT:I2C3_SCL:KEYOUT_3:Monitor_14
#define TGT_AP_HAL_GPIO_B_6_USED AS_ALT_FUNC
// GPIO(7)  // GPIO_B7 INT:I2C3_SDA:KEYOUT_4:Monitor_15
#define TGT_AP_HAL_GPIO_B_7_USED AS_ALT_FUNC
// GPIO(8)  // UART2_CTS:KEYIN_6:UART1 Data Set Ready:nil
#define TGT_AP_HAL_GPIO_B_8_USED NOT_CONNECTED
// GPIO(9)  // UART2_RTS:KEYOUT_6:UART1 Ring Indicator:nil
#define TGT_AP_HAL_GPIO_B_9_USED NOT_CONNECTED
// GPIO(10) // CAM_RST:I2C2_SCL:nil:nil
#define TGT_AP_HAL_GPIO_B_10_USED AS_ALT_FUNC
// GPIO(11) // CAM_PDN:I2C2_SDA:nil:nil
#define TGT_AP_HAL_GPIO_B_11_USED AS_ALT_FUNC
// GPIO(12) // CAM_CLK:nil:nil:nil
#define TGT_AP_HAL_GPIO_B_12_USED AS_ALT_FUNC
// GPIO(13) // CAM_VSYNC:CSI1_CLK_P:M_SPI_CLK:nil
#define TGT_AP_HAL_GPIO_B_13_USED AS_ALT_FUNC
// GPIO(14) // CAM_HREF:CSI2_CLK_P:nil:nil
#define TGT_AP_HAL_GPIO_B_14_USED AS_ALT_FUNC
// GPIO(15) // CAM_PCLK:CSI2_CLK_N:SPI_CAM_SCK:nil
#define TGT_AP_HAL_GPIO_B_15_USED AS_ALT_FUNC
// GPIO(16) // CAM_Data 0:CSI2_D0_P:SPI_CAM_DI/d0:nil
#define TGT_AP_HAL_GPIO_B_16_USED AS_ALT_FUNC
// GPIO(17) // CAM_Data 1:CSI2_D0_N:SPI_CAM_OF/d1:nil
#define TGT_AP_HAL_GPIO_B_17_USED AS_ALT_FUNC
// GPIO(18) // CAM_Data 2:CSI2_D1_P:SPI_CAM_RD/d2:nil
#define TGT_AP_HAL_GPIO_B_18_USED AS_ALT_FUNC
// GPIO(19) // CAM_Data 3:CSI2_D1_N:SPI_CAM_SSN/d3:nil
#define TGT_AP_HAL_GPIO_B_19_USED AS_ALT_FUNC
// GPIO(20) // CAM_Data 4:CSI1_D0_P:M_SPI_D0:nil
#define TGT_AP_HAL_GPIO_B_20_USED AS_ALT_FUNC
// GPIO(21) // CAM_Data 5:CSI1_D0_N:M_SPI_D1:nil
#define TGT_AP_HAL_GPIO_B_21_USED AS_ALT_FUNC
// GPIO(22) // CAM_Data 6:CSI1_D1_P:M_SPI_D2:nil
#define TGT_AP_HAL_GPIO_B_22_USED AS_ALT_FUNC
// GPIO(23) // CAM_Data 7:CSI1_D1_N:M_SPI_D3:nil
#define TGT_AP_HAL_GPIO_B_23_USED AS_ALT_FUNC
// GPIO(24) // SPI Flash CS0:CSI1_CLK_N:nil:nil
#define TGT_AP_HAL_GPIO_B_24_USED AS_GPIO
// GPIO(25) // NFCLE:GPIO_B25
#define TGT_AP_HAL_GPIO_B_25_USED AS_ALT_FUNC
// GPIO(26) // NFWEN:GPIO_B26
#define TGT_AP_HAL_GPIO_B_26_USED AS_ALT_FUNC
// GPIO(27) // NFWPN:GPIO_B27
#define TGT_AP_HAL_GPIO_B_27_USED AS_ALT_FUNC
// GPIO(28) // NFREN:GPIO_B28
#define TGT_AP_HAL_GPIO_B_28_USED AS_ALT_FUNC
// GPIO(29) // NFRB:GPIO_B29
#define TGT_AP_HAL_GPIO_B_29_USED AS_ALT_FUNC
// GPIO(30) // I2C1_SCL:nil-nil-nil
#define TGT_AP_HAL_GPIO_B_30_USED AS_ALT_FUNC
// GPIO(31) // I2C1_SDA:nil-nil-nil
#define TGT_AP_HAL_GPIO_B_31_USED AS_ALT_FUNC

// ::::::::::::::::::::::::::::::::::::::-
// GPIO(0)  // UART3_RXD:GPIO_D0 INT:nil:nil
#define TGT_AP_HAL_GPIO_D_0_USED AS_ALT_FUNC
// GPIO(1)  // UART3_TXD:GPIO_D1 INT:nil:nil
#define TGT_AP_HAL_GPIO_D_1_USED AS_ALT_FUNC
// GPIO(2)  // UART3_CTS:GPIO_D2 INT:nil:nil
#define TGT_AP_HAL_GPIO_D_2_USED AS_GPIO
// GPIO(3)  // UART3_RTS:GPIO_D3 INT:nil:nil
#define TGT_AP_HAL_GPIO_D_3_USED AS_ALT_FUNC
// GPIO(4)  // NFDQS:GPIO_D4 INT:nil:nil
#define TGT_AP_HAL_GPIO_D_4_USED AS_ALT_FUNC
// GPIO(5)  // Volume down
#define TGT_AP_HAL_GPIO_D_5_USED AS_GPIO
// GPIO(6)  // Volume up
#define TGT_AP_HAL_GPIO_D_6_USED AS_GPIO
// GPIO(7)
#define TGT_AP_HAL_GPIO_D_7_USED AS_ALT_FUNC
// GPIO(8)
#define TGT_AP_HAL_GPIO_D_8_USED AS_ALT_FUNC
// GPIO(9)
#define TGT_AP_HAL_GPIO_D_9_USED AS_ALT_FUNC
// GPIO(10)
#define TGT_AP_HAL_GPIO_D_10_USED AS_ALT_FUNC
// GPIO(11)
#define TGT_AP_HAL_GPIO_D_11_USED AS_ALT_FUNC
// GPIO(12)
#define TGT_AP_HAL_GPIO_D_12_USED AS_ALT_FUNC
// GPIO(13)
#define TGT_AP_HAL_GPIO_D_13_USED AS_ALT_FUNC
// GPIO(14)
#define TGT_AP_HAL_GPIO_D_14_USED AS_ALT_FUNC
// GPIO(15)
#define TGT_AP_HAL_GPIO_D_15_USED AS_ALT_FUNC
// GPIO(16)
#define TGT_AP_HAL_GPIO_D_16_USED AS_ALT_FUNC
// GPIO(17)
#define TGT_AP_HAL_GPIO_D_17_USED AS_ALT_FUNC
// GPIO(18)
#define TGT_AP_HAL_GPIO_D_18_USED AS_ALT_FUNC
// GPIO(19)
#define TGT_AP_HAL_GPIO_D_19_USED AS_ALT_FUNC
// GPIO(20)
#define TGT_AP_HAL_GPIO_D_20_USED AS_ALT_FUNC
// GPIO(21)
#define TGT_AP_HAL_GPIO_D_21_USED AS_ALT_FUNC
// GPIO(22)
#define TGT_AP_HAL_GPIO_D_22_USED AS_ALT_FUNC
// GPIO(23)
#define TGT_AP_HAL_GPIO_D_23_USED AS_ALT_FUNC
// GPIO(24)
#define TGT_AP_HAL_GPIO_D_24_USED AS_ALT_FUNC
// GPIO(25)
#define TGT_AP_HAL_GPIO_D_25_USED AS_ALT_FUNC
// GPIO(26)
#define TGT_AP_HAL_GPIO_D_26_USED AS_ALT_FUNC
// GPIO(27)
#define TGT_AP_HAL_GPIO_D_27_USED AS_ALT_FUNC
// GPIO(28)
#define TGT_AP_HAL_GPIO_D_28_USED AS_ALT_FUNC
// GPIO(29)
#define TGT_AP_HAL_GPIO_D_29_USED AS_ALT_FUNC
// GPIO(30)
#define TGT_AP_HAL_GPIO_D_30_USED AS_ALT_FUNC
// GPIO(31)
#define TGT_AP_HAL_GPIO_D_31_USED AS_ALT_FUNC

// ::::::::::::::::::::::::::::::::::::::-
// Each GPO can be assigned one of the following values:
// 0 : unused
// 1 : used
// ::::::::::::::::::::::::::::::::::::::-
// GPO(0)   // GPO 0:PWT:KEYIN_5
#define TGT_AP_HAL_GPO_A_0_USED 1
// GPO(1)   // GPO 1:LPG:KEYOUT_5
#define TGT_AP_HAL_GPO_A_1_USED 1
// GPO(2)   // GPO 2:PWL_1:CLK_32K
#define TGT_AP_HAL_GPO_A_2_USED 1
// GPO(3)   // LCD CS_1:SPI_LCD_CS:GPO_3
#define TGT_AP_HAL_GPO_A_3_USED 1
// GPO(4)   // LCD CS_0:SPI_LCD_DIO:GPO_4
#define TGT_AP_HAL_GPO_A_4_USED 0
// GPO(5)
#define TGT_AP_HAL_GPO_A_5_USED 0
// GPO(6)
#define TGT_AP_HAL_GPO_A_6_USED 0
// GPO(7)
#define TGT_AP_HAL_GPO_A_7_USED 0
// GPO(8)
#define TGT_AP_HAL_GPO_A_8_USED 0
// GPO(9)
#define TGT_AP_HAL_GPO_A_9_USED 0

// =============================================================================
//  GPIO pins that can be GPIO_C but are not connected on the board
//  any pin described here will be driven low all the time for power optimization
//  It's actually computed from the TGT_HAL_GPIO_C_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_HAL_NO_CONNECT_GPIO_C         ( \
    ((TGT_HAL_GPIO_C_0_USED & 2) >> 1)    | \
    (TGT_HAL_GPIO_C_1_USED & 2)           | \
    ((TGT_HAL_GPIO_C_2_USED & 2) << 1)    | \
    ((TGT_HAL_GPIO_C_3_USED & 2) << 2)    | \
    ((TGT_HAL_GPIO_C_4_USED & 2) << 3)    | \
    ((TGT_HAL_GPIO_C_5_USED & 2) << 4)    | \
    ((TGT_HAL_GPIO_C_6_USED & 2) << 5)    | \
    ((TGT_HAL_GPIO_C_7_USED & 2) << 6)    | \
    ((TGT_HAL_GPIO_C_8_USED & 2) << 7)    | \
    ((TGT_HAL_GPIO_C_9_USED & 2) << 8)    | \
    ((TGT_HAL_GPIO_C_10_USED & 2) << 9)   | \
    ((TGT_HAL_GPIO_C_11_USED & 2) << 10)  | \
    ((TGT_HAL_GPIO_C_12_USED & 2) << 11)  | \
    ((TGT_HAL_GPIO_C_13_USED & 2) << 12)  | \
    ((TGT_HAL_GPIO_C_14_USED & 2) << 13)  | \
    ((TGT_HAL_GPIO_C_15_USED & 2) << 14)  | \
    ((TGT_HAL_GPIO_C_16_USED & 2) << 15)  | \
    ((TGT_HAL_GPIO_C_17_USED & 2) << 16)  | \
    ((TGT_HAL_GPIO_C_18_USED & 2) << 17)  | \
    ((TGT_HAL_GPIO_C_19_USED & 2) << 18)  | \
    ((TGT_HAL_GPIO_C_20_USED & 2) << 19)  | \
    ((TGT_HAL_GPIO_C_21_USED & 2) << 20)  | \
    ((TGT_HAL_GPIO_C_22_USED & 2) << 21)  | \
    ((TGT_HAL_GPIO_C_23_USED & 2) << 22)  | \
    ((TGT_HAL_GPIO_C_24_USED & 2) << 23)  | \
    ((TGT_HAL_GPIO_C_25_USED & 2) << 24)  | \
    ((TGT_HAL_GPIO_C_26_USED & 2) << 25)  | \
    ((TGT_HAL_GPIO_C_27_USED & 2) << 26)  | \
    ((TGT_HAL_GPIO_C_28_USED & 2) << 27)  | \
    ((TGT_HAL_GPIO_C_29_USED & 2) << 28)  | \
    ((TGT_HAL_GPIO_C_30_USED & 2) << 29)  | \
    ((TGT_HAL_GPIO_C_31_USED & 2) << 30)  )


// =============================================================================
//  GPIO pins that are actually used as GPIO_C
//  It's actually computed from the TGT_HAL_GPIO_C_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_HAL_USED_GPIO_C               ( \
    (TGT_HAL_GPIO_C_0_USED & 1)           | \
    ((TGT_HAL_GPIO_C_1_USED & 1) << 1)    | \
    ((TGT_HAL_GPIO_C_2_USED & 1) << 2)    | \
    ((TGT_HAL_GPIO_C_3_USED & 1) << 3)    | \
    ((TGT_HAL_GPIO_C_4_USED & 1) << 4)    | \
    ((TGT_HAL_GPIO_C_5_USED & 1) << 5)    | \
    ((TGT_HAL_GPIO_C_6_USED & 1) << 6)    | \
    ((TGT_HAL_GPIO_C_7_USED & 1) << 7)    | \
    ((TGT_HAL_GPIO_C_8_USED & 1) << 8)    | \
    ((TGT_HAL_GPIO_C_9_USED & 1) << 9)    | \
    ((TGT_HAL_GPIO_C_10_USED & 1) << 10)  | \
    ((TGT_HAL_GPIO_C_11_USED & 1) << 11)  | \
    ((TGT_HAL_GPIO_C_12_USED & 1) << 12)  | \
    ((TGT_HAL_GPIO_C_13_USED & 1) << 13)  | \
    ((TGT_HAL_GPIO_C_14_USED & 1) << 14)  | \
    ((TGT_HAL_GPIO_C_15_USED & 1) << 15)  | \
    ((TGT_HAL_GPIO_C_16_USED & 1) << 16)  | \
    ((TGT_HAL_GPIO_C_17_USED & 1) << 17)  | \
    ((TGT_HAL_GPIO_C_18_USED & 1) << 18)  | \
    ((TGT_HAL_GPIO_C_19_USED & 1) << 19)  | \
    ((TGT_HAL_GPIO_C_20_USED & 1) << 20)  | \
    ((TGT_HAL_GPIO_C_21_USED & 1) << 21)  | \
    ((TGT_HAL_GPIO_C_22_USED & 1) << 22)  | \
    ((TGT_HAL_GPIO_C_23_USED & 1) << 23)  | \
    ((TGT_HAL_GPIO_C_24_USED & 1) << 24)  | \
    ((TGT_HAL_GPIO_C_25_USED & 1) << 25)  | \
    ((TGT_HAL_GPIO_C_26_USED & 1) << 26)  | \
    ((TGT_HAL_GPIO_C_27_USED & 1) << 27)  | \
    ((TGT_HAL_GPIO_C_28_USED & 1) << 28)  | \
    ((TGT_HAL_GPIO_C_29_USED & 1) << 29)  | \
    ((TGT_HAL_GPIO_C_30_USED & 1) << 30)  | \
    ((TGT_HAL_GPIO_C_31_USED & 1) << 31)  )

// =============================================================================
//  TCO pins that are actually used as TCO
//  It's actually computed from the TGT_HAL_TCO_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_HAL_USED_TCO                ( \
    (TGT_HAL_TCO_0_USED & 1)            | \
    ((TGT_HAL_TCO_1_USED & 1) << 1)     | \
    ((TGT_HAL_TCO_2_USED & 1) << 2)     | \
    ((TGT_HAL_TCO_3_USED & 1) << 3)     | \
    ((TGT_HAL_TCO_4_USED & 1) << 4)     | \
    ((TGT_HAL_TCO_5_USED & 1) << 5)     | \
    ((TGT_HAL_TCO_6_USED & 1) << 6)     | \
    ((TGT_HAL_TCO_7_USED & 1) << 7)     | \
    ((TGT_HAL_TCO_8_USED & 1) << 8)     | \
    ((TGT_HAL_TCO_9_USED & 1) << 9)     | \
    ((TGT_HAL_TCO_10_USED & 1) << 10)   | \
    ((TGT_HAL_TCO_11_USED & 1) << 11)   )

// =============================================================================
//  GPIO pins that can be GPIO but are not connected on the board
//  any pin described here will be driven low all the time for power optimization
//  It's actually computed from the TGT_AP_HAL_GPIO_A_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_NO_CONNECT_GPIO_A         ( \
    ((TGT_AP_HAL_GPIO_A_0_USED & 2) >> 1)    | \
    (TGT_AP_HAL_GPIO_A_1_USED & 2)           | \
    ((TGT_AP_HAL_GPIO_A_2_USED & 2) << 1)    | \
    ((TGT_AP_HAL_GPIO_A_3_USED & 2) << 2)    | \
    ((TGT_AP_HAL_GPIO_A_4_USED & 2) << 3)    | \
    ((TGT_AP_HAL_GPIO_A_5_USED & 2) << 4)    | \
    ((TGT_AP_HAL_GPIO_A_6_USED & 2) << 5)    | \
    ((TGT_AP_HAL_GPIO_A_7_USED & 2) << 6)    | \
    ((TGT_AP_HAL_GPIO_A_8_USED & 2) << 7)    | \
    ((TGT_AP_HAL_GPIO_A_9_USED & 2) << 8)    | \
    ((TGT_AP_HAL_GPIO_A_10_USED & 2) << 9)   | \
    ((TGT_AP_HAL_GPIO_A_11_USED & 2) << 10)  | \
    ((TGT_AP_HAL_GPIO_A_12_USED & 2) << 11)  | \
    ((TGT_AP_HAL_GPIO_A_13_USED & 2) << 12)  | \
    ((TGT_AP_HAL_GPIO_A_14_USED & 2) << 13)  | \
    ((TGT_AP_HAL_GPIO_A_15_USED & 2) << 14)  | \
    ((TGT_AP_HAL_GPIO_A_16_USED & 2) << 15)  | \
    ((TGT_AP_HAL_GPIO_A_17_USED & 2) << 16)  | \
    ((TGT_AP_HAL_GPIO_A_18_USED & 2) << 17)  | \
    ((TGT_AP_HAL_GPIO_A_19_USED & 2) << 18)  | \
    ((TGT_AP_HAL_GPIO_A_20_USED & 2) << 19)  | \
    ((TGT_AP_HAL_GPIO_A_21_USED & 2) << 20)  | \
    ((TGT_AP_HAL_GPIO_A_22_USED & 2) << 21)  | \
    ((TGT_AP_HAL_GPIO_A_23_USED & 2) << 22)  | \
    ((TGT_AP_HAL_GPIO_A_24_USED & 2) << 23)  | \
    ((TGT_AP_HAL_GPIO_A_25_USED & 2) << 24)  | \
    ((TGT_AP_HAL_GPIO_A_26_USED & 2) << 25)  | \
    ((TGT_AP_HAL_GPIO_A_27_USED & 2) << 26)  | \
    ((TGT_AP_HAL_GPIO_A_28_USED & 2) << 27)  | \
    ((TGT_AP_HAL_GPIO_A_29_USED & 2) << 28)  | \
    ((TGT_AP_HAL_GPIO_A_30_USED & 2) << 29)  | \
    ((TGT_AP_HAL_GPIO_A_31_USED & 2) << 30)  )


// =============================================================================
//  GPIO pins that are actually used as GPIO
//  It's actually computed from the TGT_AP_HAL_GPIO_A_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_USED_GPIO_A               ( \
    (TGT_AP_HAL_GPIO_A_0_USED & 1)           | \
    ((TGT_AP_HAL_GPIO_A_1_USED & 1) << 1)    | \
    ((TGT_AP_HAL_GPIO_A_2_USED & 1) << 2)    | \
    ((TGT_AP_HAL_GPIO_A_3_USED & 1) << 3)    | \
    ((TGT_AP_HAL_GPIO_A_4_USED & 1) << 4)    | \
    ((TGT_AP_HAL_GPIO_A_5_USED & 1) << 5)    | \
    ((TGT_AP_HAL_GPIO_A_6_USED & 1) << 6)    | \
    ((TGT_AP_HAL_GPIO_A_7_USED & 1) << 7)    | \
    ((TGT_AP_HAL_GPIO_A_8_USED & 1) << 8)    | \
    ((TGT_AP_HAL_GPIO_A_9_USED & 1) << 9)    | \
    ((TGT_AP_HAL_GPIO_A_10_USED & 1) << 10)  | \
    ((TGT_AP_HAL_GPIO_A_11_USED & 1) << 11)  | \
    ((TGT_AP_HAL_GPIO_A_12_USED & 1) << 12)  | \
    ((TGT_AP_HAL_GPIO_A_13_USED & 1) << 13)  | \
    ((TGT_AP_HAL_GPIO_A_14_USED & 1) << 14)  | \
    ((TGT_AP_HAL_GPIO_A_15_USED & 1) << 15)  | \
    ((TGT_AP_HAL_GPIO_A_16_USED & 1) << 16)  | \
    ((TGT_AP_HAL_GPIO_A_17_USED & 1) << 17)  | \
    ((TGT_AP_HAL_GPIO_A_18_USED & 1) << 18)  | \
    ((TGT_AP_HAL_GPIO_A_19_USED & 1) << 19)  | \
    ((TGT_AP_HAL_GPIO_A_20_USED & 1) << 20)  | \
    ((TGT_AP_HAL_GPIO_A_21_USED & 1) << 21)  | \
    ((TGT_AP_HAL_GPIO_A_22_USED & 1) << 22)  | \
    ((TGT_AP_HAL_GPIO_A_23_USED & 1) << 23)  | \
    ((TGT_AP_HAL_GPIO_A_24_USED & 1) << 24)  | \
    ((TGT_AP_HAL_GPIO_A_25_USED & 1) << 25)  | \
    ((TGT_AP_HAL_GPIO_A_26_USED & 1) << 26)  | \
    ((TGT_AP_HAL_GPIO_A_27_USED & 1) << 27)  | \
    ((TGT_AP_HAL_GPIO_A_28_USED & 1) << 28)  | \
    ((TGT_AP_HAL_GPIO_A_29_USED & 1) << 29)  | \
    ((TGT_AP_HAL_GPIO_A_30_USED & 1) << 30)  | \
    ((TGT_AP_HAL_GPIO_A_31_USED & 1) << 31)  )

// =============================================================================
//  GPIO pins that can be GPIO but are not connected on the board
//  any pin described here will be driven low all the time for power optimization
//  It's actually computed from the TGT_AP_HAL_GPIO_B_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_NO_CONNECT_GPIO_B         ( \
    ((TGT_AP_HAL_GPIO_B_0_USED & 2) >> 1)    | \
    (TGT_AP_HAL_GPIO_B_1_USED & 2)           | \
    ((TGT_AP_HAL_GPIO_B_2_USED & 2) << 1)    | \
    ((TGT_AP_HAL_GPIO_B_3_USED & 2) << 2)    | \
    ((TGT_AP_HAL_GPIO_B_4_USED & 2) << 3)    | \
    ((TGT_AP_HAL_GPIO_B_5_USED & 2) << 4)    | \
    ((TGT_AP_HAL_GPIO_B_6_USED & 2) << 5)    | \
    ((TGT_AP_HAL_GPIO_B_7_USED & 2) << 6)    | \
    ((TGT_AP_HAL_GPIO_B_8_USED & 2) << 7)    | \
    ((TGT_AP_HAL_GPIO_B_9_USED & 2) << 8)    | \
    ((TGT_AP_HAL_GPIO_B_10_USED & 2) << 9)   | \
    ((TGT_AP_HAL_GPIO_B_11_USED & 2) << 10)  | \
    ((TGT_AP_HAL_GPIO_B_12_USED & 2) << 11)  | \
    ((TGT_AP_HAL_GPIO_B_13_USED & 2) << 12)  | \
    ((TGT_AP_HAL_GPIO_B_14_USED & 2) << 13)  | \
    ((TGT_AP_HAL_GPIO_B_15_USED & 2) << 14)  | \
    ((TGT_AP_HAL_GPIO_B_16_USED & 2) << 15)  | \
    ((TGT_AP_HAL_GPIO_B_17_USED & 2) << 16)  | \
    ((TGT_AP_HAL_GPIO_B_18_USED & 2) << 17)  | \
    ((TGT_AP_HAL_GPIO_B_19_USED & 2) << 18)  | \
    ((TGT_AP_HAL_GPIO_B_20_USED & 2) << 19)  | \
    ((TGT_AP_HAL_GPIO_B_21_USED & 2) << 20)  | \
    ((TGT_AP_HAL_GPIO_B_22_USED & 2) << 21)  | \
    ((TGT_AP_HAL_GPIO_B_23_USED & 2) << 22)  | \
    ((TGT_AP_HAL_GPIO_B_24_USED & 2) << 23)  | \
    ((TGT_AP_HAL_GPIO_B_25_USED & 2) << 24)  | \
    ((TGT_AP_HAL_GPIO_B_26_USED & 2) << 25)  | \
    ((TGT_AP_HAL_GPIO_B_27_USED & 2) << 26)  | \
    ((TGT_AP_HAL_GPIO_B_28_USED & 2) << 27)  | \
    ((TGT_AP_HAL_GPIO_B_29_USED & 2) << 28)  | \
    ((TGT_AP_HAL_GPIO_B_30_USED & 2) << 29)  | \
    ((TGT_AP_HAL_GPIO_B_31_USED & 2) << 30)  )


// =============================================================================
//  GPIO pins that are actually used as GPIO
//  It's actually computed from the TGT_AP_HAL_GPIO_B_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_USED_GPIO_B               ( \
    (TGT_AP_HAL_GPIO_B_0_USED & 1)           | \
    ((TGT_AP_HAL_GPIO_B_1_USED & 1) << 1)    | \
    ((TGT_AP_HAL_GPIO_B_2_USED & 1) << 2)    | \
    ((TGT_AP_HAL_GPIO_B_3_USED & 1) << 3)    | \
    ((TGT_AP_HAL_GPIO_B_4_USED & 1) << 4)    | \
    ((TGT_AP_HAL_GPIO_B_5_USED & 1) << 5)    | \
    ((TGT_AP_HAL_GPIO_B_6_USED & 1) << 6)    | \
    ((TGT_AP_HAL_GPIO_B_7_USED & 1) << 7)    | \
    ((TGT_AP_HAL_GPIO_B_8_USED & 1) << 8)    | \
    ((TGT_AP_HAL_GPIO_B_9_USED & 1) << 9)    | \
    ((TGT_AP_HAL_GPIO_B_10_USED & 1) << 10)  | \
    ((TGT_AP_HAL_GPIO_B_11_USED & 1) << 11)  | \
    ((TGT_AP_HAL_GPIO_B_12_USED & 1) << 12)  | \
    ((TGT_AP_HAL_GPIO_B_13_USED & 1) << 13)  | \
    ((TGT_AP_HAL_GPIO_B_14_USED & 1) << 14)  | \
    ((TGT_AP_HAL_GPIO_B_15_USED & 1) << 15)  | \
    ((TGT_AP_HAL_GPIO_B_16_USED & 1) << 16)  | \
    ((TGT_AP_HAL_GPIO_B_17_USED & 1) << 17)  | \
    ((TGT_AP_HAL_GPIO_B_18_USED & 1) << 18)  | \
    ((TGT_AP_HAL_GPIO_B_19_USED & 1) << 19)  | \
    ((TGT_AP_HAL_GPIO_B_20_USED & 1) << 20)  | \
    ((TGT_AP_HAL_GPIO_B_21_USED & 1) << 21)  | \
    ((TGT_AP_HAL_GPIO_B_22_USED & 1) << 22)  | \
    ((TGT_AP_HAL_GPIO_B_23_USED & 1) << 23)  | \
    ((TGT_AP_HAL_GPIO_B_24_USED & 1) << 24)  | \
    ((TGT_AP_HAL_GPIO_B_25_USED & 1) << 25)  | \
    ((TGT_AP_HAL_GPIO_B_26_USED & 1) << 26)  | \
    ((TGT_AP_HAL_GPIO_B_27_USED & 1) << 27)  | \
    ((TGT_AP_HAL_GPIO_B_28_USED & 1) << 28)  | \
    ((TGT_AP_HAL_GPIO_B_29_USED & 1) << 29)  | \
    ((TGT_AP_HAL_GPIO_B_30_USED & 1) << 30)  | \
    ((TGT_AP_HAL_GPIO_B_31_USED & 1) << 31)  )



// =============================================================================
//  GPIO pins that can be GPIO but are not connected on the board
//  any pin described here will be driven low all the time for power optimization
//  It's actually computed from the TGT_AP_HAL_GPIO_D_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_NO_CONNECT_GPIO_D         ( \
    ((TGT_AP_HAL_GPIO_D_0_USED & 2) >> 1)    | \
    (TGT_AP_HAL_GPIO_D_1_USED & 2)           | \
    ((TGT_AP_HAL_GPIO_D_2_USED & 2) << 1)    | \
    ((TGT_AP_HAL_GPIO_D_3_USED & 2) << 2)    | \
    ((TGT_AP_HAL_GPIO_D_4_USED & 2) << 3)    | \
    ((TGT_AP_HAL_GPIO_D_5_USED & 2) << 4)    | \
    ((TGT_AP_HAL_GPIO_D_6_USED & 2) << 5)    | \
    ((TGT_AP_HAL_GPIO_D_7_USED & 2) << 6)    | \
    ((TGT_AP_HAL_GPIO_D_8_USED & 2) << 7)    | \
    ((TGT_AP_HAL_GPIO_D_9_USED & 2) << 8)    | \
    ((TGT_AP_HAL_GPIO_D_10_USED & 2) << 9)   | \
    ((TGT_AP_HAL_GPIO_D_11_USED & 2) << 10)  | \
    ((TGT_AP_HAL_GPIO_D_12_USED & 2) << 11)  | \
    ((TGT_AP_HAL_GPIO_D_13_USED & 2) << 12)  | \
    ((TGT_AP_HAL_GPIO_D_14_USED & 2) << 13)  | \
    ((TGT_AP_HAL_GPIO_D_15_USED & 2) << 14)  | \
    ((TGT_AP_HAL_GPIO_D_16_USED & 2) << 15)  | \
    ((TGT_AP_HAL_GPIO_D_17_USED & 2) << 16)  | \
    ((TGT_AP_HAL_GPIO_D_18_USED & 2) << 17)  | \
    ((TGT_AP_HAL_GPIO_D_19_USED & 2) << 18)  | \
    ((TGT_AP_HAL_GPIO_D_20_USED & 2) << 19)  | \
    ((TGT_AP_HAL_GPIO_D_21_USED & 2) << 20)  | \
    ((TGT_AP_HAL_GPIO_D_22_USED & 2) << 21)  | \
    ((TGT_AP_HAL_GPIO_D_23_USED & 2) << 22)  | \
    ((TGT_AP_HAL_GPIO_D_24_USED & 2) << 23)  | \
    ((TGT_AP_HAL_GPIO_D_25_USED & 2) << 24)  | \
    ((TGT_AP_HAL_GPIO_D_26_USED & 2) << 25)  | \
    ((TGT_AP_HAL_GPIO_D_27_USED & 2) << 26)  | \
    ((TGT_AP_HAL_GPIO_D_28_USED & 2) << 27)  | \
    ((TGT_AP_HAL_GPIO_D_29_USED & 2) << 28)  | \
    ((TGT_AP_HAL_GPIO_D_30_USED & 2) << 29)  | \
    ((TGT_AP_HAL_GPIO_D_31_USED & 2) << 30)  )


// =============================================================================
//  GPIO pins that are actually used as GPIO
//  It's actually computed from the TGT_AP_HAL_GPIO_D_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_USED_GPIO_D               ( \
    (TGT_AP_HAL_GPIO_D_0_USED & 1)           | \
    ((TGT_AP_HAL_GPIO_D_1_USED & 1) << 1)    | \
    ((TGT_AP_HAL_GPIO_D_2_USED & 1) << 2)    | \
    ((TGT_AP_HAL_GPIO_D_3_USED & 1) << 3)    | \
    ((TGT_AP_HAL_GPIO_D_4_USED & 1) << 4)    | \
    ((TGT_AP_HAL_GPIO_D_5_USED & 1) << 5)    | \
    ((TGT_AP_HAL_GPIO_D_6_USED & 1) << 6)    | \
    ((TGT_AP_HAL_GPIO_D_7_USED & 1) << 7)    | \
    ((TGT_AP_HAL_GPIO_D_8_USED & 1) << 8)    | \
    ((TGT_AP_HAL_GPIO_D_9_USED & 1) << 9)    | \
    ((TGT_AP_HAL_GPIO_D_10_USED & 1) << 10)  | \
    ((TGT_AP_HAL_GPIO_D_11_USED & 1) << 11)  | \
    ((TGT_AP_HAL_GPIO_D_12_USED & 1) << 12)  | \
    ((TGT_AP_HAL_GPIO_D_13_USED & 1) << 13)  | \
    ((TGT_AP_HAL_GPIO_D_14_USED & 1) << 14)  | \
    ((TGT_AP_HAL_GPIO_D_15_USED & 1) << 15)  | \
    ((TGT_AP_HAL_GPIO_D_16_USED & 1) << 16)  | \
    ((TGT_AP_HAL_GPIO_D_17_USED & 1) << 17)  | \
    ((TGT_AP_HAL_GPIO_D_18_USED & 1) << 18)  | \
    ((TGT_AP_HAL_GPIO_D_19_USED & 1) << 19)  | \
    ((TGT_AP_HAL_GPIO_D_20_USED & 1) << 20)  | \
    ((TGT_AP_HAL_GPIO_D_21_USED & 1) << 21)  | \
    ((TGT_AP_HAL_GPIO_D_22_USED & 1) << 22)  | \
    ((TGT_AP_HAL_GPIO_D_23_USED & 1) << 23)  | \
    ((TGT_AP_HAL_GPIO_D_24_USED & 1) << 24)  | \
    ((TGT_AP_HAL_GPIO_D_25_USED & 1) << 25)  | \
    ((TGT_AP_HAL_GPIO_D_26_USED & 1) << 26)  | \
    ((TGT_AP_HAL_GPIO_D_27_USED & 1) << 27)  | \
    ((TGT_AP_HAL_GPIO_D_28_USED & 1) << 28)  | \
    ((TGT_AP_HAL_GPIO_D_29_USED & 1) << 29)  | \
    ((TGT_AP_HAL_GPIO_D_30_USED & 1) << 30)  | \
    ((TGT_AP_HAL_GPIO_D_31_USED & 1) << 31)  )


// =============================================================================
//  GPO pins that are actually used as GPO_A
//  It's actually computed from the TGT_AP_HAL_GPO_A_xx_USED macros above.
//  DO NOT MODIFY !
// =============================================================================
#define TGT_AP_HAL_USED_GPO_A                ( \
    (TGT_AP_HAL_GPO_A_0_USED & 1)            | \
    ((TGT_AP_HAL_GPO_A_1_USED & 1) << 1)     | \
    ((TGT_AP_HAL_GPO_A_2_USED & 1) << 2)     | \
    ((TGT_AP_HAL_GPO_A_3_USED & 1) << 3)     | \
    ((TGT_AP_HAL_GPO_A_4_USED & 1) << 4)     | \
    ((TGT_AP_HAL_GPO_A_5_USED & 1) << 5)     | \
    ((TGT_AP_HAL_GPO_A_6_USED & 1) << 6)     | \
    ((TGT_AP_HAL_GPO_A_7_USED & 1) << 7)     | \
    ((TGT_AP_HAL_GPO_A_8_USED & 1) << 8)     | \
    ((TGT_AP_HAL_GPO_A_9_USED & 1) << 9)     )


#endif
