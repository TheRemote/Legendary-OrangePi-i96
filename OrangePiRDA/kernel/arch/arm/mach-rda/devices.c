/*
 * devices.c - RDA platform devices
 *
 * Copyright (C) 2013-2014 RDA Microelectronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/switch.h>
#include <media/soc_camera.h>

#include <asm/io.h>
#include <asm/pmu.h>
#include <linux/usb/gpio_vbus.h>
#include <linux/usb/musb.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/iomap.h>
#include <plat/devices.h>
#include <plat/ap_clk.h>
#include <mach/board.h>
#include <mach/ifc.h>
#include <mach/gpio_id.h>
#include <linux/mmc/host.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi_gpio.h>
#ifdef CONFIG_CAN_MCP251X
#include <linux/can/platform/mcp251x.h>
#endif
//#ifdef CONFIG_LEDS_RDA
#include <mach/regulator.h>
//#endif /* CONFIG_LEDS_RDA */
#include <linux/regulator/consumer.h>

#include "../drivers/staging/android/ion/ion.h"

#include <rda/tgt_ap_board_config.h>
#include <rda/tgt_ap_gpio_setting.h>
#include <linux/gpio.h>

#define _TGT_AP_USED_UART2

#if 0
#define RDA_CAREVOUT_PHYS 0x80000000 + (_TGT_AP_OS_MEM_SIZE << 20)
#define CAREVOUT_SIZE (_TGT_AP_CAM_MEM_SIZE << 20)
//#define USE_CARVEOUT
#endif
#define RDA_CARVEOUT_BASE (_TGT_AP_VPU_MEM_BASE)
#define RDA_CARVEOUT_PHYS 0x80000000 + (RDA_CARVEOUT_BASE << 20)
#define CARVEOUT_SIZE (_TGT_AP_VPU_MEM_SIZE << 20)

#if defined(CONFIG_RDA_RMNET) && defined(_TGT_SMD_MEM_BASE)
#define RDA_SM_PHYS 0x80000000 + (_TGT_SMD_MEM_BASE << 20)
#define RDA_SM_SIZE (_TGT_SMD_MEM_SIZE << 20)
#endif

/*
 * ION
 */

#if 0
static u64 ion_cma_mask = DMA_BIT_MASK(32);

static struct platform_device rda_ion_cma_device = {
	.name = "rda_ion_cma_device",
	.id  = -1,
	.dev = {
		.dma_mask = &ion_cma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};
#endif
static struct ion_platform_heap rda_ion_heaps[] = {
#if defined(CONFIG_ARCH_RDA8850E) || defined(CONFIG_ARCH_8810H) || defined(CONFIG_ARCH_RDA8810E)
	{
		.type = ION_HEAP_TYPE_SYSTEM,
		.id = ION_HEAP_TYPE_SYSTEM,
		.name = "rdaion-system",
	},
#endif
	{
		.type = ION_HEAP_TYPE_CARVEOUT,
		.id = ION_HEAP_TYPE_CARVEOUT,
		.name = "rdaion-carveout",
		.base = RDA_CARVEOUT_PHYS,
		.size = CARVEOUT_SIZE,
	},
#if 0
		{
		   .type = ION_HEAP_TYPE_DMA,
		   .id = ION_HEAP_TYPE_DMA,
		   .name = "rdaion-cma",
		   .priv = (void *)&rda_ion_cma_device.dev,
		}
#endif
};

static struct ion_platform_data rda_ion_data = {
	.nr = sizeof(rda_ion_heaps) / sizeof(struct ion_platform_heap),
	.heaps = rda_ion_heaps,
};

static struct platform_device rda_ion = {
	.name = RDA_ION_DRV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &rda_ion_data,
	},
};

/*
 * PMU device
 */
static struct resource rda_resource_pmu = {
	.start = RDA_IRQ_DMC,
	.end = RDA_IRQ_DMC,
	.flags = IORESOURCE_IRQ,
};

struct platform_device rda_pmu = {
	.name = "arm-pmu",
	.id = -1,
	.resource = &rda_resource_pmu,
	.num_resources = 1,
};

/*
 * RDA UART Controller
 */
/*
 * use UART3 as default console
 * use UART1 as uart2
 * UART2 is for host interface, AP never use
 */
static struct resource rda_uart1_resource[] = {
	{
		.start = RDA_UART3_PHYS,
		.end = RDA_UART3_PHYS + RDA_UART3_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = RDA_IRQ_UART3,
		.end = RDA_IRQ_UART3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_uart_device_data rda_uart1_data[] = {
	{
		.dev_id = 3,
		.uartclk = 26000000,
		.rxdmarequestid = HAL_IFC_UART3_RX,
		.txdmarequestid = HAL_IFC_UART3_TX,
		.wakeup = 0,
	},
};

static struct platform_device rda_uart1 = {
	.name = RDA_UART_DRV_NAME,
	.id = 0,
	.resource = rda_uart1_resource,
	.num_resources = ARRAY_SIZE(rda_uart1_resource),
	.dev = {
		.platform_data = rda_uart1_data,
	},
};

static struct resource rda_uart2_resource[] = {
	{
		.start = RDA_UART1_PHYS,
		.end = RDA_UART1_PHYS + RDA_UART1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = RDA_IRQ_UART1,
		.end = RDA_IRQ_UART1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_uart_device_data rda_uart2_data[] = {
	{
		.dev_id = 1,
		.uartclk = 26000000,
		.rxdmarequestid = HAL_IFC_UART_RX,
		.txdmarequestid = HAL_IFC_UART_TX,
		.wakeup = 1,
	},
};

static struct platform_device rda_uart2 = {
	.name = RDA_UART_DRV_NAME,
	.id = 1,
	.resource = rda_uart2_resource,
	.num_resources = ARRAY_SIZE(rda_uart2_resource),
	.dev = {
		.platform_data = rda_uart2_data,
	},
};

#ifdef _TGT_AP_USED_UART2

static struct resource rda_uart3_resource[] = {
	{
		.start = RDA_UART2_PHYS,
		.end = RDA_UART2_PHYS + RDA_UART2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = RDA_IRQ_UART2,
		.end = RDA_IRQ_UART2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_uart_device_data rda_uart3_data[] = {
	{
		.dev_id = 2,
		.uartclk = 26000000,
		.rxdmarequestid = HAL_IFC_UART2_RX,
		.txdmarequestid = HAL_IFC_UART2_TX,
		.wakeup = 0,
	},
};

static struct platform_device rda_uart3 = {
	.name = RDA_UART_DRV_NAME,
	.id = 2,
	.resource = rda_uart3_resource,
	.num_resources = ARRAY_SIZE(rda_uart3_resource),
	.dev = {
		.platform_data = rda_uart3_data,
	},
};
#endif /*_TGT_AP_USED_UART2*/

#ifndef CONFIG_RDA_FPGA
/*
 * Modem Communication Controller
 */
static struct resource rda_comreg0_resource[] = {
	[0] = {
		.start = RDA_COMREGS_PHYS,
		.end = RDA_COMREGS_PHYS + RDA_COMREGS_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_COMREG0,
		.end = RDA_IRQ_COMREG0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device rda_comreg0 = {
	.name = RDA_COMREG0_DRV_NAME,
	.id = 0,
	.resource = rda_comreg0_resource,
	.num_resources = ARRAY_SIZE(rda_comreg0_resource),
};

//static struct spi_gpio_platform_data spi2_gpio_data = {
//		.sck = 2, // spi pins on the comb
//		.mosi = 3,
//		.miso = 4,
//
//		.num_chipselect = 2, // this is the number of devices (we will only connect the screen, but can be extended for spidev)
//};
//struct platform_device rda_spi2_gpio = {
//		.name = "spi_gpio", // spi driver name
//		.id = 2,			// bus number spi2 available on GPIO header
//		.dev = {
//				.platform_data = &spi2_gpio_data,
//		}};
static struct resource rda_md_resource[] = {
	[0] = {
		.start = RDA_COMREGS_PHYS,
		.end = RDA_COMREGS_PHYS + RDA_COMREGS_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_MD_MAILBOX_PHYS,
		.end = RDA_MD_MAILBOX_PHYS + RDA_MD_MAILBOX_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = RDA_IRQ_COMREG1,
		.end = RDA_IRQ_COMREG1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device rda_md = {
	.name = RDA_MD_DRV_NAME,
	.id = 0,
	.resource = rda_md_resource,
	.num_resources = ARRAY_SIZE(rda_md_resource),
};

static struct platform_device rda_msys = {
	.name = RDA_MSYS_DRV_NAME,
	.id = -1,
};

#if defined(CONFIG_RDA_RMNET) && defined(_TGT_SMD_MEM_BASE)
static struct resource rda_smd_resource[] = {
	[0] = {
		.start = RDA_SM_PHYS,
		.end = RDA_SM_PHYS + RDA_SM_SIZE - 1,
		.flags = IORESOURCE_MEM,
		.name = "rda_sm",
	},
};

static struct platform_device rda_smd = {
	.name = RDA_SMD_DRV_NAME,
	.id = 0,
	.resource = rda_smd_resource,
	.num_resources = ARRAY_SIZE(rda_smd_resource),
};

static struct platform_device rda_rmnet = {
	.name = RDA_RMNET_DRV_NAME,
	.id = 0,
};
#endif

int smd_is_ready(void)
{
#if defined(CONFIG_RDA_RMNET) && defined(_TGT_SMD_MEM_BASE)
	return 1;
#else
	return 0;
#endif
}
/*
 * Rtc Controller
 */
static struct resource rda_rtc_resource[] = {
	[0] = {
		.start = RDA_RTC_PHYS,
		.end = RDA_RTC_PHYS + RDA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device rda_rtc = {
	.name = RDA_RTC_DRV_NAME,
	.id = -1,
	.resource = rda_rtc_resource,
	.num_resources = ARRAY_SIZE(rda_rtc_resource),
};
#endif /* CONFIG_RDA_FPGA */

/****************************************************************
 * RDA MMC Controller                                           *
\****************************************************************/
static struct resource rda_mmc0_resource[] = {
	[0] = {
		.start = RDA_SDMMC1_PHYS,
		.end = RDA_SDMMC1_PHYS + RDA_SDMMC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SDMMC1,
		.end = RDA_IRQ_SDMMC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_mmc_device_data rda_mmc0_data[] = {
	{
		.f_min = 1000000,
		.f_max = _TGT_AP_SDMMC1_MAX_FREQ,
		.mclk_adj = _TGT_AP_SDMMC1_MCLK_ADJ,
		.ocr_avail = MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34,

		.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_UHS_DDR50 | MMC_CAP_SD_HIGHSPEED | MMC_CAP_ERASE | MMC_CAP_1_8V_DDR,

#ifdef _TGT_AP_GPIO_MMC_HOTPLUG
		.det_pin = _TGT_AP_GPIO_MMC_HOTPLUG,
#else
		.det_pin = -1,
#endif /* _TGT_AP_GPIO_MMC_HOTPLUG */
		.eirq_enable = 0,
		.sys_suspend = 1,
		.pm_caps = 0,
#ifdef _TGT_AP_SDMMC1_MCLK_INV
		.clk_inv = 1,
#else
		.clk_inv = 0,
#endif /* _TGT_AP_SDMMC1_MCLK_INV */
	},
};

static struct platform_device rda_mmc0 = {
	.name = RDA_MMC_DRV_NAME,
	.id = 0,
	.resource = rda_mmc0_resource,
	.num_resources = ARRAY_SIZE(rda_mmc0_resource),
	.dev = {
		.platform_data = rda_mmc0_data,
	},
};

static struct resource rda_mmc1_resource[] = {
	[0] = {
		.start = RDA_SDMMC2_PHYS,
		.end = RDA_SDMMC2_PHYS + RDA_SDMMC2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SDMMC2,
		.end = RDA_IRQ_SDMMC2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_mmc_device_data rda_mmc1_data[] = {
	{
		.f_min = 20000000,
		.f_max = _TGT_AP_SDMMC2_MAX_FREQ,
		.mclk_adj = _TGT_AP_SDMMC2_MCLK_ADJ,
		.ocr_avail = MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34,
		.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ,
		.eirq_enable = 1,
		.eirq_gpio = GPIO_WIFI, // WF_INTN
		.debounce = -1,			// not care now
		.eirq_sense = IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND,
		.sys_suspend = 0,
		.pm_caps = MMC_PM_KEEP_POWER,
		.dev_label = "rda_wlan_irq",
#ifdef _TGT_AP_SDMMC2_MCLK_INV
		.clk_inv = 1,
#else
		.clk_inv = 0,
#endif /* _TGT_AP_SDMMC2_MCLK_INV */
		.det_pin = -1,
	},
};

static struct platform_device rda_mmc1 = {
	.name = RDA_MMC_DRV_NAME,
	.id = 1,
	.resource = rda_mmc1_resource,
	.num_resources = ARRAY_SIZE(rda_mmc1_resource),
	.dev = {
		.platform_data = rda_mmc1_data,
	},
};

static struct resource rda_mmc2_resource[] = {
	[0] = {
		.start = RDA_SDMMC3_PHYS,
		.end = RDA_SDMMC3_PHYS + RDA_SDMMC3_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SDMMC3,
		.end = RDA_IRQ_SDMMC3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_mmc_device_data rda_mmc2_data[] = {
	{
		.f_min = 1000000,
		.f_max = _TGT_AP_SDMMC3_MAX_FREQ,
		.mclk_adj = 0,
		.clk_inv = 0,
		.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
#ifdef _TGT_AP_SDMMC3_UHS_DDR50
		.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_NONREMOVABLE,
#elif defined _TGT_AP_SDMMC3_HIGH_SPEED
		.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_NONREMOVABLE | MMC_CAP_MMC_HIGHSPEED,
#else
		.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_NONREMOVABLE,
#endif
		.det_pin = -1,
		.eirq_enable = 0,
		.sys_suspend = 0,
		.pm_caps = 0,
	},
};

static struct platform_device rda_mmc2 = {
	.name = RDA_MMC_DRV_NAME,
	.id = 2,
	.resource = rda_mmc2_resource,
	.num_resources = ARRAY_SIZE(rda_mmc2_resource),
	.dev = {
		.platform_data = rda_mmc2_data,
	},
};

/*
 * RDA Framebuffer
 */
#ifdef CONFIG_FB_RDA_DBI
static struct platform_device rda_fb_dbi_panel = {
	.name = RDA_DBI_PANEL_DRV_NAME,
	.id = -1,
	.dev = {
		.platform_data = NULL,
	},
};
#endif

#ifdef CONFIG_FB_RDA_DSI
static struct platform_device rda_fb_dsi_panel = {
	.name = RDA_DSI_PANEL_DRV_NAME,
	.id = -1,
	.dev = {
		.platform_data = NULL,
	},
};

#endif /* CONFIG_FB_RDA_CTRL_DBI */

/*
 * RDA GOUDA
 */
static struct resource rda_gouda_resource[] = {
	[0] = {
		.start = RDA_GOUDA_PHYS,
		.end = RDA_GOUDA_PHYS + RDA_GOUDA_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_GOUDA,
		.end = RDA_IRQ_GOUDA,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 rda_gouda_dmamask = DMA_BIT_MASK(32);

static struct platform_device rda_gouda = {
	.name = RDA_GOUDA_DRV_NAME,
	.id = -1,
	.resource = rda_gouda_resource,
	.num_resources = ARRAY_SIZE(rda_gouda_resource),
	.dev = {
		.dma_mask = &rda_gouda_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = NULL,
	},
};

/*
 * RDA LCDC
 */
#if defined(CONFIG_RDA_LCDC)
static struct resource rda_lcdc_resource[] = {
	[0] = {
		.start = RDA_LCDC_PHYS,
		.end = RDA_LCDC_PHYS + RDA_LCDC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_LCDC,
		.end = RDA_IRQ_LCDC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device rda_lcdc = {
	.name = RDA_LCDC_DRV_NAME,
	.id = -1,
	.resource = rda_lcdc_resource,
	.num_resources = ARRAY_SIZE(rda_lcdc_resource),
	.dev = {
		.platform_data = NULL,
	},
};
#endif

/*
 * RDA I2C Master
 */
static struct resource rda_i2c0_resource[] = {
	[0] = {
		.start = RDA_I2C1_PHYS,
		.end = RDA_I2C1_PHYS + RDA_I2C1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_I2C,
		.end = RDA_IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_i2c_device_data rda_i2c0_data[] = {
	{
		.speed = _TGT_AP_I2C0_CLOCK / 1000, // in kHz
	},
};

struct platform_device rda_i2c0 = {
	.name = RDA_I2C_DRV_NAME,
	.id = 0,
	.num_resources = ARRAY_SIZE(rda_i2c0_resource),
	.resource = rda_i2c0_resource,
	.dev = {
		.platform_data = rda_i2c0_data,
	},
};

static struct resource rda_i2c1_resource[] = {
	[0] = {
		.start = RDA_I2C2_PHYS,
		.end = RDA_I2C2_PHYS + RDA_I2C2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_I2C,
		.end = RDA_IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_i2c_device_data rda_i2c1_data[] = {
	{
		.speed = _TGT_AP_I2C1_CLOCK / 1000, // in kHz
	},
};

struct platform_device rda_i2c1 = {
	.name = RDA_I2C_DRV_NAME,
	.id = 1,
	.num_resources = ARRAY_SIZE(rda_i2c1_resource),
	.resource = rda_i2c1_resource,
	.dev = {
		.platform_data = rda_i2c1_data,
	},
};

static struct resource rda_i2c2_resource[] = {
	[0] = {
		.start = RDA_I2C3_PHYS,
		.end = RDA_I2C3_PHYS + RDA_I2C3_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_I2C,
		.end = RDA_IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_i2c_device_data rda_i2c2_data[] = {
	{
		.speed = _TGT_AP_I2C2_CLOCK / 1000, // in kHz
	},
};

struct platform_device rda_i2c2 = {
	.name = RDA_I2C_DRV_NAME,
	.id = 2,
	.num_resources = ARRAY_SIZE(rda_i2c2_resource),
	.resource = rda_i2c2_resource,
	.dev = {
		.platform_data = rda_i2c2_data,
	},
};

#ifdef _TGT_AP_WIFI_USE_GPIO_I2C
static struct i2c_gpio_platform_data rda_i2c_gpio_bus_data = {
	.sda_pin = _TGT_AP_GPIO_GPIOI2C_5991_SDA,
	.scl_pin = _TGT_AP_GPIO_GPIOI2C_5991_SCL,
	.udelay = 5,
	.timeout = 100,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 1,
};

static struct platform_device rda_i2c_gpio_bus_device = {
	.name = "i2c-gpio",
	.id = _TGT_AP_I2C_BUS_ID_WIFI,
	.dev = {
		.platform_data = &rda_i2c_gpio_bus_data,
	},
};
#endif

/*
 * NAND Controller
 */
static struct resource rda_nand_resource[] = {
	[0] = {
		.start = RDA_NAND_PHYS,
		.end = RDA_NAND_PHYS + RDA_NAND_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_NAND_NFSC,
		.end = RDA_IRQ_NAND_NFSC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_nand_device_data rda_nand_data[] = {
	{
		.max_clock = _TGT_AP_NAND_CLOCK,
	},
};

#ifdef _TGT_AP_NAND_V1_HIGHDENSITY
static struct platform_device rda_nand = {
	.name = RDA_NAND_HD_DRV_NAME,
	.id = -1,
	.resource = rda_nand_resource,
	.num_resources = ARRAY_SIZE(rda_nand_resource),
	.dev = {
		.platform_data = rda_nand_data,
	},
};
#else
static struct platform_device rda_nand = {
	.name = RDA_NAND_DRV_NAME,
	.id = -1,
	.resource = rda_nand_resource,
	.num_resources = ARRAY_SIZE(rda_nand_resource),
	.dev = {
		.platform_data = rda_nand_data,
	},
};
#endif
/*
 * SPI NAND Controller
 */
static struct resource rda_spinand_resource[] = {
	[0] = {
		.start = RDA_SPIFLASH_PHYS,
		.end = RDA_SPIFLASH_PHYS + RDA_SPIFLASH_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}};

static struct rda_spinand_device_data rda_spinand_data[] = {
	{
		.max_clock = 50000000,
	},
};

static struct platform_device rda_spinand = {
	.name = RDA_SPI_NAND_DRV_NAME,
	.id = -1,
	.resource = rda_spinand_resource,
	.num_resources = ARRAY_SIZE(rda_spinand_resource),
	.dev = {
		.platform_data = rda_spinand_data,
	},
};

static struct platform_device rda_wdt_device = {
	.name = "rda_wdt",
	.id = -1,
};

static int rda_nand_spi = 0;
static int rda_emmc = 0;
static int rda_sdcard = 0;
static int __init flash_intf_setup(char *str)
{
	/* NOTE: SPI get higher priorities, this is HW define */
	if (!strcmp(str, "spi"))
	{
		rda_nand_spi = 1;
	}
	else if (!strcmp(str, "emmc"))
	{
		rda_emmc = 1;
	}
	else if (!strcmp(str, "sdcard"))
	{
		rda_sdcard = 1;
	}
	else
	{
		rda_nand_spi = 0;
	}

	return 0;
}
__setup("flash_if=", flash_intf_setup);

static int emmc_id = 0;
static int __init emmc_id_setup(char *str)
{
	sscanf(str, "%d", &emmc_id);
	printk("Parse emmc id = %x\n", emmc_id);

	return 0;
}
__setup("emmc_id=", emmc_id_setup);

static const struct emmc_mfr_mclk_adj_inv emmc_mclk_adj_inv[] = {
#if defined(_TGT_AP_EMMC_MCLK_ADJ_TOSHIBA) && defined(_TGT_AP_EMMC_CLK_INV_TOSHIBA)
	{MMC_MFR_TOSHIBA, _TGT_AP_EMMC_MCLK_ADJ_TOSHIBA, _TGT_AP_EMMC_CLK_INV_TOSHIBA, 0},
#else
	{MMC_MFR_TOSHIBA, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_GIGADEVICE) && defined(_TGT_AP_EMMC_CLK_INV_GIGADEVICE)
	{MMC_MFR_GIGADEVICE, _TGT_AP_EMMC_MCLK_ADJ_GIGADEVICE, _TGT_AP_EMMC_CLK_INV_GIGADEVICE, 0},
#else
	{MMC_MFR_GIGADEVICE, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_SAMSUNG) && defined(_TGT_AP_EMMC_CLK_INV_SAMSUNG)
	{MMC_MFR_SAMSUNG, _TGT_AP_EMMC_MCLK_ADJ_SAMSUNG, _TGT_AP_EMMC_CLK_INV_SAMSUNG, 0},
#else
	{MMC_MFR_SAMSUNG, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_SANDISK) && defined(_TGT_AP_EMMC_CLK_INV_SANDISK)
	{MMC_MFR_SANDISK, _TGT_AP_EMMC_MCLK_ADJ_SANDISK, _TGT_AP_EMMC_CLK_INV_SANDISK, 0},
#else
	{MMC_MFR_SANDISK, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_HYNIX) && defined(_TGT_AP_EMMC_CLK_INV_HYNIX)
	{MMC_MFR_HYNIX, _TGT_AP_EMMC_MCLK_ADJ_HYNIX, _TGT_AP_EMMC_CLK_INV_HYNIX, 0},
#else
	{MMC_MFR_HYNIX, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_MICRON) && defined(_TGT_AP_EMMC_CLK_INV_MICRON)
	{MMC_MFR_MICRON, _TGT_AP_EMMC_MCLK_ADJ_MICRON, _TGT_AP_EMMC_CLK_INV_MICRON, 0},
#else
	{MMC_MFR_MICRON, 0, 1, 0},
#endif

#if defined(_TGT_AP_EMMC_MCLK_ADJ_MICRON1) && defined(_TGT_AP_EMMC_CLK_INV_MICRON1)
	{MMC_MFR_MICRON1, _TGT_AP_EMMC_MCLK_ADJ_MICRON1, _TGT_AP_EMMC_CLK_INV_MICRON1, 0},
#else
	{MMC_MFR_MICRON1, 0, 1, 0},
#endif
	{0, 0, 0, 0},
};

void __init nand_device_init(void)
{
	if (rda_nand_spi)
	{
		platform_device_register(&rda_spinand);
	}
	else if (rda_emmc)
	{
		int i = 0;

		for (; emmc_mclk_adj_inv[i].mfr_id != 0; i++)
		{
			if (emmc_id == emmc_mclk_adj_inv[i].mfr_id)
				break;
		}

		if (emmc_mclk_adj_inv[i].mfr_id == 0)
			printk("Can't find suitable emmc mclk adj and clock inv config, use default one.");
		else
		{
			rda_mmc2_data[0].mclk_adj = emmc_mclk_adj_inv[i].mclk_adj;
			rda_mmc2_data[0].clk_inv = emmc_mclk_adj_inv[i].mclk_inv;
		}
		platform_device_register(&rda_mmc2);
	}
	else if (rda_sdcard)
	{
		platform_device_register(&rda_mmc0);
	}
	else
	{
		platform_device_register(&rda_nand);
	}
}

/*
 * DMA Controller
 */
static struct resource rda_dma_resource[] = {
	[0] = {
		.start = RDA_DMA_PHYS,
		.end = RDA_DMA_PHYS + RDA_DMA_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_DMA,
		.end = RDA_IRQ_DMA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_dma_device_data rda_dma_data[] = {
	{
		.ch_num = RDA_DMA_CHANS,
	},
};

static struct platform_device rda_dma = {
	.name = RDA_DMA_DRV_NAME,
	.id = -1,
	.resource = rda_dma_resource,
	.num_resources = ARRAY_SIZE(rda_dma_resource),
	.dev = {
		.platform_data = rda_dma_data,
	},
};

/*
 * AUIFC Controller
 */
static struct resource rda_auifc_resource[] = {
	[0] = {
		.start = RDA_AUIFC_PHYS,
		.end = RDA_AUIFC_PHYS + RDA_AUIFC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_AUIFC0,
		.end = RDA_IRQ_AUIFC0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = RDA_IRQ_AUIFC1,
		.end = RDA_IRQ_AUIFC1,
		.flags = IORESOURCE_IRQ,
	},

};

static struct rda_audifc_device_data rda_auifc_data[] = {
	{
		.ch_num = 2,
	},
};

static struct platform_device rda_auifc = {
	.name = RDA_AUIFC_DRV_NAME,
	.id = -1,
	.resource = rda_auifc_resource,
	.num_resources = ARRAY_SIZE(rda_auifc_resource),
	.dev = {
		.platform_data = rda_auifc_data,
	},
};

static struct platform_device rda_pcm = {
	.name = RDA_PCM_DRV_NAME,
	.id = -1,
};

static struct resource codec_resources[] = {
	[0] = {
		.start = GPIO_AUDIO_EXTPA,
		.end = GPIO_AUDIO_EXTPA,
		.name = "audio-extpa",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = GPIO_AUDIO_EXTPA_1,
		.end = GPIO_AUDIO_EXTPA_1,
		.name = "audio-extpa-1",
		.flags = IORESOURCE_MEM,
	},
};
static struct platform_device rda_codec = {
	.name = RDA_CODEC_DRV_NAME,
	.id = -1,
	.resource = codec_resources,
	.num_resources = ARRAY_SIZE(codec_resources),
};

static struct platform_device rda_gpioc = {
	.name = RDA_GPIOC_DRV_NAME,
	.id = -1,
};

/*
 * AIF Controller
 */
static struct resource rda_aif_resource[] = {
	[0] = {
		.start = RDA_AIF_PHYS,
		.end = RDA_AIF_PHYS + RDA_AIF_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},

};

static struct rda_aif_device_data rda_aif_data[] = {
	{
		.ch_num = 2,
	},
};

static struct platform_device rda_aif = {
	.name = RDA_AIF_DRV_NAME,
	.id = -1,
	.resource = rda_aif_resource,
	.num_resources = ARRAY_SIZE(rda_aif_resource),
	.dev = {
		.platform_data = rda_aif_data,
	},
};

/*
 * Keypad Controller
 */
static struct resource rda_keypad_resource[] = {
	[0] = {
		.start = RDA_KEYPAD_PHYS,
		.end = RDA_KEYPAD_PHYS + RDA_KEYPAD_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_KEYPAD,
		.end = RDA_IRQ_KEYPAD,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_keypad_device_data rda_keypad_data[] = {
	{
		.dbn_time = 240,
		.itv_time = 20,
		.rows = 0,
		.cols = 0,
		.gpio_volume_up = GPIO_VOLUME_UP,
		.gpio_volume_down = GPIO_VOLUME_DOWN,
	},
};

static struct platform_device rda_keypad = {
	.name = RDA_KEYPAD_DRV_NAME,
	.id = -1,
	.resource = rda_keypad_resource,
	.num_resources = ARRAY_SIZE(rda_keypad_resource),
	.dev = {
		.platform_data = rda_keypad_data,
	},
};

static struct platform_device rda_headset = {
	.name = RDA_HEADSET_DRV_NAME,
};

/*
 * Camera Controller
 */
static struct resource rda_camera_resource[] = {
	[0] = {
		.start = RDA_CAMERA_PHYS,
		.end = RDA_CAMERA_PHYS + RDA_CAMERA_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_CAMERA,
		.end = RDA_IRQ_CAMERA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_camera_device_data rda_camera_data[] = {
	{
		.hsync_act_low = 0,
		.vsync_act_low = 0,
		.pclk_act_falling = 0,
	},
};

static u64 rda_camera_dmamask = DMA_BIT_MASK(32);

static struct platform_device rda_camera = {
	.name = RDA_CAMERA_DRV_NAME,
	.id = 0,
	.resource = rda_camera_resource,
	.num_resources = ARRAY_SIZE(rda_camera_resource),
	.dev = {
		.dma_mask = &rda_camera_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = rda_camera_data,
	},
};

/*
 * Camera Sensor
 */
static struct i2c_board_info i2c_dev_camera[] = {
	{I2C_BOARD_INFO(RDA_SENSOR_DRV_NAME, (0x78 >> 1))},
	{I2C_BOARD_INFO(RDA_SENSOR_DRV_NAME, (0x62 >> 1))},
};

static struct regulator_bulk_data rda_sensor_regulator = {
	.supply = LDO_CAM,
};

static struct soc_camera_desc sdesc_sensor[] = {
	{
		.subdev_desc = {
			.flags = 0,
			.drv_priv = NULL,
			.regulators = &rda_sensor_regulator,
			.num_regulators = 1,
			.power = NULL,
			.reset = NULL,
		},
		.host_desc = {
			.bus_id = 0,
			.i2c_adapter_id = _TGT_AP_I2C_BUS_ID_CAM,
			.board_info = &i2c_dev_camera[0],
		},
	},
	{
		.subdev_desc = {
			.flags = 0,
			.drv_priv = NULL,
			.regulators = &rda_sensor_regulator,
			.num_regulators = 1,
			.power = NULL,
			.reset = NULL,
		},
		.host_desc = {
			.bus_id = 0,
			.i2c_adapter_id = _TGT_AP_I2C_BUS_ID_CAM,
			.board_info = &i2c_dev_camera[1],
		},
	},
};

static struct platform_device rda_camera_sensor[] = {
	{
		.name = "soc-camera-pdrv",
		.id = 0,
		.dev = {
			.platform_data = &sdesc_sensor[0],
		},
	},
	{
		.name = "soc-camera-pdrv",
		.id = 1,
		.dev = {
			.platform_data = &sdesc_sensor[1],
		},
	},
};

struct gpio_vbus_mach_info rda_udc_info = {
#ifdef _TGT_AP_USB_OTG_ENABLE
	.gpio_vbus = GPIO_OTG_DETECT,
#else
	.gpio_vbus = -1,
#endif
	.gpio_pullup = -1,
	.gpio_vbus_inverted = 1};

static struct platform_device rda_gpio_vbus = {
	.name = "gpio-vbus",
	.id = -1,
	.dev = {
		.platform_data = &rda_udc_info,
	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint = false,
	.dyn_fifo = false,
	.num_eps = 16,
	.ram_bits = 12,
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef _TGT_AP_USB_OTG_ENABLE
	.mode = MUSB_OTG,
#else
	.mode = MUSB_PERIPHERAL,
#endif

	/* .clock is set dynamically */
	.config = &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 500 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power = 250, /* up to 500 mA */
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

static struct resource usb_resources[] = {
	[0] = {
		.start = RDA_USB_PHYS,
		.end = RDA_USB_PHYS + RDA_USB_SIZE - 1,
		.name = "usb-mem",
		.flags = IORESOURCE_MEM,
	},

	[1] = {
		.start = RDA_IRQ_USB,
		.name = "mc", /* hard-coded in musb */
		.flags = IORESOURCE_IRQ,
	},

	[2] = {
		.start = RDA_IRQ_USB,
		.name = "dma", /* hard-coded in musbhsdma */
		.flags = IORESOURCE_IRQ,
	},
#ifdef _TGT_AP_USB_OTG_ENABLE
	[3] = {
		.start = GPIO_USB_VBUS_SWITCH,
		.end = GPIO_USB_VBUS_SWITCH,
		.name = "vbus-switch",
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = GPIO_USBID_CTRL,
		.end = GPIO_USBID_CTRL,
		.name = "usbid_ctrl",
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = GPIO_PLUGIN_CTRL,
		.end = GPIO_PLUGIN_CTRL,
		.name = "plugin_ctrl",
		.flags = IORESOURCE_IRQ,
	},
#endif
};

struct platform_device rda_musb_device = {
	.name = "musb-rda",
	.id = 0,
	.dev = {
		.platform_data = &musb_plat,
		.dma_mask = &musb_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources = ARRAY_SIZE(usb_resources),
	.resource = usb_resources,
};

void __init usb_musb_init(void)
{
	apsys_reset_usbc();
	platform_device_register(&rda_gpio_vbus);
	platform_device_register(&rda_musb_device);
}

/*
 * RDA SPI Master
 */
static u64 rda_spi_dmamask = DMA_BIT_MASK(32);
static struct resource rda_spi0_resource[] = {
	[0] = {
		.start = RDA_SPI1_PHYS,
		.end = RDA_SPI1_PHYS + RDA_SPI1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SPI1,
		.end = RDA_IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_spi_device_data rda_spi0_data[] = {
	{
		.ifc_rxchannel = HAL_IFC_SPI_RX,
		.ifc_txchannel = HAL_IFC_SPI_TX,
		.csnum = 3,
	},
};

struct platform_device rda_spi0 = {
	.name = RDA_SPI_DRV_NAME,
	.id = 0,
	.num_resources = ARRAY_SIZE(rda_spi0_resource),
	.resource = rda_spi0_resource,
	.dev = {
		.platform_data = rda_spi0_data,
		.dma_mask = &rda_spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource rda_spi1_resource[] = {
	[0] = {
		.start = RDA_SPI2_PHYS,
		.end = RDA_SPI2_PHYS + RDA_SPI2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SPI2,
		.end = RDA_IRQ_SPI2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_spi_device_data rda_spi1_data[] = {
	{
		.ifc_rxchannel = HAL_IFC_SPI2_RX,
		.ifc_txchannel = HAL_IFC_SPI2_TX,
		.csnum = 3,
	},
};

struct platform_device rda_spi1 = {
	.name = RDA_SPI_DRV_NAME,
	.id = 1,
	.num_resources = ARRAY_SIZE(rda_spi1_resource),
	.resource = rda_spi1_resource,
	.dev = {
		.platform_data = rda_spi1_data,
		.dma_mask = &rda_spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource rda_spi2_resource[] = {
	[0] = {
		.start = RDA_SPI3_PHYS,
		.end = RDA_SPI3_PHYS + RDA_SPI3_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = RDA_IRQ_SPI3,
		.end = RDA_IRQ_SPI3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct rda_spi_device_data rda_spi2_data[] = {
	{
		.ifc_rxchannel = HAL_IFC_SPI3_RX,
		.ifc_txchannel = HAL_IFC_SPI3_TX,
		.csnum = 2,
	},
};

struct platform_device rda_spi2 = {
	.name = RDA_SPI_DRV_NAME,
	.id = 2,
	.num_resources = ARRAY_SIZE(rda_spi2_resource),
	.resource = rda_spi2_resource,
	.dev = {
		.platform_data = rda_spi2_data,
		.dma_mask = &rda_spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct platform_device rda_power_supply = {
	.name = "rda-power",
	.id = -1,
};

static struct platform_device rda_vibrator = {
	.name = RDA_VIBRATOR_DRV_NAME,
	.id = -1,
};

#ifdef CONFIG_LEDS_RDA
#ifdef _TGT_AP_LED_RED
struct rda_led_device_data rda_ledr_data = {
#ifdef _TGT_AP_LED_RED_KB
	.led_name = "button-backlight",
	.is_keyboard = 1,
	.trigger = 0,
#endif /* _TGT_AP_LED_RED_KB */

#ifdef _TGT_AP_LED_RED_FLASH
	.led_name = "red-flash",
	.is_keyboard = 0,
	.trigger = 1,
#endif /* _TGT_AP_LED_RED_FLASH */
	.ldo_name = LDO_LEDR,
};

static struct platform_device rda_ledr = {
	.name = RDA_LEDS_DRV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &rda_ledr_data,
	},
};
#endif /* _TGT_AP_LED_RED */

#ifdef _TGT_AP_LED_GREEN
struct rda_led_device_data rda_ledg_data = {
#ifdef _TGT_AP_LED_GREEN_KB
	.led_name = "button-backlight",
	.is_keyboard = 1,
	.trigger = 0,
#endif /* _TGT_AP_LED_GREEN_KB */

#ifdef _TGT_AP_LED_GREEN_FLASH
	.led_name = "green-flash",
	.is_keyboard = 0,
	.trigger = 1,
#endif /* _TGT_AP_LED_GREEN_FLASH */
	.ldo_name = LDO_LEDG,
};

static struct platform_device rda_ledg = {
	.name = RDA_LEDS_DRV_NAME,
	.id = 1,
	.dev = {
		.platform_data = &rda_ledg_data,
	},
};
#endif /* _TGT_AP_LED_GREEN */

#ifdef _TGT_AP_LED_BLUE
struct rda_led_device_data rda_ledb_data = {
#ifdef _TGT_AP_LED_BLUE_KB
	.led_name = "button-backlight",
	.is_keyboard = 1,
	.trigger = 0,
#endif /* _TGT_AP_LED_BLUE_KB */

#ifdef _TGT_AP_LED_BLUE_FLASH
	.led_name = "blue-flash",
	.is_keyboard = 0,
	.trigger = 1,
#endif /* _TGT_AP_LED_BLUE_FLASH */
	.ldo_name = LDO_LEDB,
};

static struct platform_device rda_ledb = {
	.name = RDA_LEDS_DRV_NAME,
	.id = 2,
	.dev = {
		.platform_data = &rda_ledb_data,
	},
};
#endif /* _TGT_AP_LED_BLUE */

#endif /* CONFIG_LEDS_RDA */

#ifndef _TGT_AP_BL_EXT_PWM
#ifdef CONFIG_BACKLIGHT_RDA
static struct rda_bl_device_data rda_bl_data = {
	.min = 0,
	.max = 255,
};

static struct platform_device rda_backlight = {
	.name = RDA_BL_DRV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &rda_bl_data,
	},
};
#endif /* CONFIG_BACKLIGHT_RDA */
#endif /* !_TGT_AP_BL_EXT_PWM */

#ifdef _TGT_AP_BL_EXT_PWM
#ifdef CONFIG_BACKLIGHT_RDA_PWM
static struct resource rda_pwm_resource[] = {
	[0] = {
		.start = RDA_PWM_PHYS,
		.end = RDA_PWM_PHYS + RDA_PWM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct rda_bl_pwm_device_data rda_bl_pwm_data = {
	.min = 0,
	.max = 255,
};

static struct platform_device rda_pwm = {
	.name = RDA_PWM_DRV_NAME,
	.id = 0,
	.num_resources = ARRAY_SIZE(rda_pwm_resource),
	.resource = rda_pwm_resource,
	.dev = {
		.platform_data = &rda_bl_pwm_data,
	},
};
#endif /* CONFIG_BACKLIGHT_RDA_PWM */
#endif /* _TGT_AP_BL_EXT_PWM */

#ifdef CONFIG_BACKLIGHT_RDA
void __init bl_device_init(void)
{
#ifndef _TGT_AP_BL_EXT_PWM
	platform_device_register(&rda_backlight);
#endif /*!_TGT_AP_BL_EXT_PWM*/

#ifdef _TGT_AP_BL_EXT_PWM
#ifdef _TGT_AP_BL_EXT_GPIO_ENABLE
	rda_bl_pwm_data.gpio_bl_on = _TGT_AP_GPIO_EXT_BL_ON;
	rda_bl_pwm_data.gpio_bl_on_valid = 1;
#else
	rda_bl_pwm_data.gpio_bl_on = GPIO_MAX_NUM;
	rda_bl_pwm_data.gpio_bl_on_valid = 0;
#endif /*_TGT_AP_BL_EXT_GPIO_ENABLE*/

#ifdef _TGT_AP_BL_EXT_PWM_INVERT
	rda_bl_pwm_data.pwm_invert = 1;
#else
	rda_bl_pwm_data.pwm_invert = 0;
#endif /*_TGT_AP_BL_EXT_PWM_INVERT*/

#ifdef _TGT_AP_EXT_PWM_CLOCK
	rda_bl_pwm_data.pwm_clk = _TGT_AP_EXT_PWM_CLOCK;
#else
	rda_bl_pwm_data.pwm_clk = 1000000;
#endif /*_TGT_AP_EXT_PWM_CLOCK*/

#ifdef _TGT_AP_BL_EXT_PWT
	rda_bl_pwm_data.pwt_used = 1;
#else
	rda_bl_pwm_data.pwt_used = 0;
#endif /*_TGT_AP_BL_EXT_PWT*/

	platform_device_register(&rda_pwm);
#endif /*_TGT_AP_BL_EXT_PWM*/
}
#endif /* CONFIG_BACKLIGHT_RDA */

/*
 * tlv320aic23_spi , mainly used to test FPGA.
 */
RDA_SPI_PARAMETERS tlv320aic23_spi = {
	.inputEn = true,
	.clkDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.doDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.diDelay = RDA_SPI_HALF_CLK_PERIOD_1,
	.csDelay = RDA_SPI_HALF_CLK_PERIOD_1,
	.csPulse = RDA_SPI_HALF_CLK_PERIOD_0,
	.frameSize = 8,
	.oeRatio = 8,
	.rxTrigger = RDA_SPI_RX_TRIGGER_4_BYTE,
	.txTrigger = RDA_SPI_TX_TRIGGER_1_EMPTY,
};
RDA_SPI_PARAMETERS rda_spi0_params = {
	.inputEn = true,
	.clkDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.doDelay = RDA_SPI_HALF_CLK_PERIOD_1,
	.diDelay = RDA_SPI_HALF_CLK_PERIOD_2,
	.csDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.csPulse = RDA_SPI_HALF_CLK_PERIOD_0,
	.frameSize = 8,
	.oeRatio = 0,
	.rxTrigger = RDA_SPI_RX_TRIGGER_1_BYTE,
	.txTrigger = RDA_SPI_TX_TRIGGER_1_EMPTY,
};
RDA_SPI_PARAMETERS rda_spi1_params = {
	.inputEn = true,
	.clkDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.doDelay = RDA_SPI_HALF_CLK_PERIOD_1,
	.diDelay = RDA_SPI_HALF_CLK_PERIOD_2,
	.csDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.csPulse = RDA_SPI_HALF_CLK_PERIOD_0,
	.frameSize = 8,
	.oeRatio = 0,
	.rxTrigger = RDA_SPI_RX_TRIGGER_1_BYTE,
	.txTrigger = RDA_SPI_TX_TRIGGER_1_EMPTY,
};
#ifdef CONFIG_CAN_MCP251X
RDA_SPI_PARAMETERS mcp251x_spi = {
	.inputEn = true,
	.clkDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.doDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.diDelay = RDA_SPI_HALF_CLK_PERIOD_2,
	.csDelay = RDA_SPI_HALF_CLK_PERIOD_0,
	.csPulse = RDA_SPI_HALF_CLK_PERIOD_0,
	.frameSize = 8,
	.oeRatio = 0,
	.rxTrigger = RDA_SPI_RX_TRIGGER_1_BYTE,
	.txTrigger = RDA_SPI_TX_TRIGGER_1_EMPTY,
};
static struct mcp251x_platform_data mcp251x_info = {
        .oscillator_frequency = 16000000,
        .board_specific_setup = NULL,
        .power_enable = NULL,
        .transceiver_enable = NULL,
		.irq_flags = IRQF_TRIGGER_FALLING,
};
#endif
static struct spi_board_info rda_spi_board_info[] = {
#ifdef CONFIG_FB_RDA_DPI
	{
		.modalias = RDA_DPI_PANEL_DRV_NAME,
		.max_speed_hz = 500000,
		.bus_num = 0,
		.chip_select = 0,
		//.controller_data =
	},
#endif
	{
		.modalias = "tlv320aic23-spi",
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 1,
		.controller_data = (void *)&tlv320aic23_spi,
		.platform_data = (void *) NULL,
	},
	{
		.modalias = "spidev", // driver name
		.max_speed_hz = 20000000,
		.mode = SPI_MODE_0,
		.bus_num = 0,
		.chip_select = 2,
		.controller_data = (void *) &rda_spi0_params,
		.platform_data = (void *) &rda_spi0,
	},
	{
		.modalias = "spidev", // driver name
		.max_speed_hz = 20000000,
		.mode = SPI_MODE_0,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = (void *) &rda_spi1_params,
		.platform_data = (void *) &rda_spi1,
	},
#ifdef CONFIG_CAN_MCP251X
	{
		.modalias = "mcp2510", // driver name
		.max_speed_hz = 20000000,
		.mode = SPI_MODE_2,
		.bus_num = 1, // mcp251x on spi1 CS_1
		.chip_select = 1,
		.controller_data = (void *) &mcp251x_spi,
		.platform_data = (void *) &mcp251x_info,
		.irq = 50,	// mcp2515 Interrupt pin on D02 ( GPIO 66) ( H26) = RDA_IRQ_NB + RDA_GPIO_BANK_IRQ * (Bank -1) + pin offset
	},
#else
	{
		.modalias = "spidev", // driver name
		.max_speed_hz = 20000000,
		.mode = SPI_MODE_2,
		.bus_num = 1, // spi2 CS_1
		.chip_select = 1,
		.controller_data = (void *) &rda_spi1_params,
		.platform_data = (void *) &rda_spi1,
	},
#endif
};

/*
Rda Wireless Combo
*/
static struct i2c_board_info i2c_dev_rda_wlan_combo[] = {
	{I2C_BOARD_INFO("rda_wifi_rf_i2c", 0x14)},
	{I2C_BOARD_INFO("rda_wifi_core_i2c", 0x13)},
	{I2C_BOARD_INFO("rda_bt_rf_i2c", 0x16)},
	{I2C_BOARD_INFO("rda_bt_core_i2c", 0x15)},
	{I2C_BOARD_INFO("rda_fm_radio_i2c", 0x11)},
};

/*
RDA Analog TV I2C Board Info.
*/
#ifdef _TGT_AP_BOARD_HAS_ATV
static struct i2c_board_info i2c_dev_rda5888[] = {
	{I2C_BOARD_INFO("rda5888_i2c", 0x62)},
};
#endif

/*
  Device Table
 */
static struct platform_device *devices[] __initdata = {
	&rda_ion,
	&rda_pmu,
	&rda_uart1,
	&rda_uart2,
#ifdef _TGT_AP_USED_UART2
	&rda_uart3,
#endif
#ifndef CONFIG_RDA_FPGA
	&rda_comreg0,
	&rda_md,
#if defined(CONFIG_RDA_RMNET) && defined(_TGT_SMD_MEM_BASE)
	&rda_smd,
	&rda_rmnet,
#endif
	&rda_msys,
	&rda_rtc,
#endif
	&rda_mmc0,
	&rda_mmc1,
#ifdef CONFIG_FB_RDA_DBI
	&rda_fb_dbi_panel,
#endif
#ifdef CONFIG_FB_RDA_DSI
	&rda_fb_dsi_panel,
#endif
	&rda_i2c0,
	&rda_i2c1,
	&rda_i2c2,
#ifdef _TGT_AP_WIFI_USE_GPIO_I2C
	&rda_i2c_gpio_bus_device,
#endif
	&rda_keypad,
	&rda_dma,
	&rda_camera_sensor[0],
	//	&rda_camera_sensor[1],
	&rda_camera,
	&rda_gouda,
#ifdef CONFIG_RDA_LCDC
	&rda_lcdc,
#endif
	&rda_spi0,
	&rda_spi1,
//	&rda_spi2_gpio,
	&rda_vibrator,
	&rda_power_supply,
#ifdef CONFIG_LEDS_RDA
#ifdef _TGT_AP_LED_RED
	&rda_ledr,
#endif /* _TGT_AP_LED_RED */

#ifdef _TGT_AP_LED_GREEN
	&rda_ledg,
#endif /* _TGT_AP_LED_GREEN */

#ifdef _TGT_AP_LED_BLUE
	&rda_ledb,
#endif /* _TGT_AP_LED_BLUE */
#endif /* CONFIG_LEDS_RDA */
	&rda_auifc,
	&rda_aif,
	&rda_codec,
	&rda_gpioc,
	&rda_pcm,
	&rda_headset,
	&rda_wdt_device,
};
void __init rda_init_devices(void)
{
	i2c_register_board_info(_TGT_AP_I2C_BUS_ID_WIFI,
							i2c_dev_rda_wlan_combo, 5);
#ifdef _TGT_AP_BOARD_HAS_ATV
	i2c_register_board_info(_TGT_AP_I2C_BUS_ID_ATV,
							i2c_dev_rda5888, 1);
#endif
	/* leave soc_camera_probe() to register i2c board_info */
	// i2c_register_board_info(1, &i2c_dev_camera, 1);
	spi_register_board_info(rda_spi_board_info,
							ARRAY_SIZE(rda_spi_board_info));

	usb_musb_init();
	nand_device_init();
#ifdef CONFIG_BACKLIGHT_RDA
	bl_device_init();
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
}
