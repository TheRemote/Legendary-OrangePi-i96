#ifndef __RDA_CONFIG_H__
#define __RDA_CONFIG_H__

#include <rda/tgt_ap_board_config.h>
#include <rda/tgt_ap_clock_config.h>
#define MEM_SIZE			_TGT_AP_OS_MEM_SIZE

/*
 * Board
 */
#define CONFIG_RDA_MUX_CONFIG		1

/*
 * SoC Configuration
 */
#define CONFIG_ARMV7
#define CONFIG_SYS_HZ_CLOCK		2000000       /* 2M */
#define CONFIG_SYS_HZ			1000
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SYS_TEXT_BASE		0x80008000
/* #define CONFIG_SYS_ICACHE_OFF */
/* #define CONFIG_SYS_DCACHE_OFF */
/* #define CONFIG_SYS_NEON_OFF */

/* Display CPU and Board Info */
#define CONFIG_DISPLAY_CPUINFO 		1
#define CONFIG_DISPLAY_BOARDINFO	1
/*
 * Memory Info
 */
#define CONFIG_SYS_MALLOC_LEN		(0x10000 + 16*1024*1024) /* malloc() len */
#define PHYS_SDRAM_1			(0x80000000) /* DRAM Start */
#define PHYS_SDRAM_1_SIZE		((MEM_SIZE) << 20)

/* memtest start addr */
#define CONFIG_SYS_MEMTEST_START	(PHYS_SDRAM_1 + 0x00800000)

/* memtest will be run on 16MB */
#define CONFIG_SYS_MEMTEST_END 		(PHYS_SDRAM_1 + 0x00800000 + 16*1024*1024)

/* spl need 20k bytes for nand one page data load (max size is 16k page size) */
#define CONFIG_SPL_NAND_MIDDLE_DATA_BUFFER	(PHYS_SDRAM_1 + 0x01000000)

#define CONFIG_NR_DRAM_BANKS		1 /* we have 1 bank of DRAM */
#define CONFIG_STACKSIZE		(256*1024) /* regular stack */

/*
 * Serial Driver info
 */
#define CONFIG_BAUDRATE			921600 /* Default baud rate */
#define CONFIG_SYS_BAUDRATE_TABLE 	{ 9600, 38400, 115200, 921600 }

/*
 * SD/MMC
 */
#define CONFIG_MMC
#define CONFIG_EFI_PARTITION
#define CONFIG_DOS_PARTITION
/*#define CONFIG_RDA_MMC_LEGACY*/
#define CONFIG_GENERIC_MMC
#define CONFIG_RDA_MMC
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT2
/*
 * ROM in U0B chips cannot provide information on the booting media, whereas
 * SPL and U-BOOT must know the media type. The macro, CONFIG_SDMMC_BOOT,
 * is introduced to tell SPL and U-BOOT that this is a SDMMC boot.
 * The macro will be obsoleted for future chips, after ROM is updated to
 * provide such information.
 */
/*#define CONFIG_SDMMC_BOOT*/

#ifdef CONFIG_SDMMC_BOOT /* sdcard */
#define CONFIG_MMC_DEV_NUM	0
#define CONFIG_MMC_DEV_NAME	"mmc0"
#else /* emmc */
#define CONFIG_MMC_DEV_NUM	1
#define CONFIG_MMC_DEV_NAME	"mmc1"
#endif

/*
 * Misc drivers
 */
#define CONFIG_RDA_FACTORY	1
#define CONFIG_RDA_PRDINFO	1
#define CONFIG_CMD_PRDINFO	1

/*
 * I2C Configuration
 */

/*
 * Flash & Environment
 */
#define CONFIG_NAND_RDA
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#define CONFIG_SYS_NAND_PAGE_2K

#define CONFIG_SYS_NAND_LARGEPAGE
#define CONFIG_SYS_MAX_NAND_DEVICE	1

#define CONFIG_SYS_NAND_BASE		(PHYS_SDRAM_1 + 0x00800000)

/*
 * Console configuration
 */
#define CONFIG_SYS_PROMPT		"RDA > " /* Command Prompt */
#define CONFIG_SYS_CBSIZE		1024 /* Console I/O Buffer Size	*/
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS		16 /* max number of command args */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE /* Boot Arg Buf Sz */
#define CONFIG_SYS_LOAD_ADDR		(PHYS_SDRAM_1 + 0x02000000)
#define CONFIG_VERSION_VARIABLE
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_LONGHELP
#define CONFIG_CMDLINE_EDITING
#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC

/*
 * USB Configuration
 */
/*
#define CONFIG_MUSB_HCD	1
*/
#define CONFIG_MUSB_UDC	1
#define CONFIG_USB_RDA

#ifdef CONFIG_MUSB_HCD
#define MUSB_NO_MULTIPOINT
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#define CONFIG_FAT_WRITE
#endif
/*
 *USB Gadget Configuration
 */
#ifdef CONFIG_MUSB_UDC
#define CONFIG_RDA_USB_SERIAL
#define CONFIG_USB_DEVICE
#define CONFIG_USBD_HS
#endif

/*
 * U-BOOT only configurations
 */
#ifndef CONFIG_SPL_BUILD

#ifdef UBIFS_SYSTEM_IMAGE
#define CONFIG_UBIFS_SYSTEM_IMAGE	1
#endif

#ifdef UBIFS_USER_IMAGES
#define CONFIG_UBIFS_USER_IMAGES	1
#endif

#ifdef EXTFS_SYSTEM_IMAGE
#define CONFIG_EXTFS_SYSTEM_IMAGE	1
#endif

#ifdef EXTFS_USER_IMAGES
#define CONFIG_EXTFS_USER_IMAGES	1
#endif
/*#define CONFIG_SYS_NAND_BASE_LIST	{ (PHYS_SDRAM_1 + 0x00800000), } */

/*
 * U-boot USB Configuration
 */

#ifdef CONFIG_MUSB_UDC
#define CONFIG_USB_FASTBOOT
#define CONFIG_MUSB_DMA
#define CONFIG_CMD_FASTBOOT
#define CONFIG_CMD_PDL2
#define SCRATCH_ADDR			(PHYS_SDRAM_1 + 0x02000000)	/* leave 32M for boot and test */
#define FB_DOWNLOAD_BUF_SIZE		(PHYS_SDRAM_1_SIZE - 0x02000000 - 0x800000)	/* 216M */
#endif

/*
 * U-Boot commands
 */
#include <config_cmd_default.h>
#include <asm/arch/mtdparts_def.h>
#define CONFIG_CMD_ENV
#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_DIAG
#define CONFIG_CMD_SAVES
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_MMC
#define CONFIG_CMD_HWFLOW

/* modem commands */
#define CONFIG_CMD_MDCOM

#ifdef CONFIG_NAND_RDA
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
/*
#define CONFIG_MTD_DEBUG
#define CONFIG_MTD_DEBUG_VERBOSE 3
*/
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_YAFFS
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_SOURCE
#define CONFIG_CMD_BOOTZ
#define CONFIG_RBTREE

/*#define CONFIG_YAFFS2*/
#define MTDIDS_DEFAULT			"nand0=rda_nand"
#define MTDPARTS_DEFAULT		"mtdparts=rda_nand:"MTDPARTS_DEF
#define MTDPARTS_KERNEL_DEFAULT		"mtdparts=rda_nand:"MTDPARTS_KERNEL_DEF
#endif

/*
 * U-BOOT library
 */
#define CONFIG_USE_ARCH_MEMCPY		1
/*
 * communication code between modem and ap
 */
#define CONFIG_MDCOM

#undef CONFIG_CMD_NET
#undef CONFIG_CMD_DHCP
#undef CONFIG_CMD_MII
#undef CONFIG_CMD_PING
#undef CONFIG_CMD_DNS
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_RARP
#undef CONFIG_CMD_SNTP

#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_SPI
#define CONFIG_SPI_RDA
#undef CONFIG_CMD_SF
#undef CONFIG_CMD_SAVEENV
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_BDI
#undef CONFIG_CMD_CONSOLE

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_LZO

/*
 * Linux Information
 */
#define LINUX_BOOT_PARAM_ADDR		(PHYS_SDRAM_1 + 0x100)
#define CONFIG_CMDLINE_TAG
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SETUP_MEMORY_TAGS
// #define CONFIG_SERIAL_TAG

/* The delay to check whether to stop auto boot */
#define CONFIG_BOOTDELAY		1
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_EXTRA_ENV_SETTINGS	\
	"script_addr=81000000\0" \
	"modem_addr=82000000\0" \
	"factorydata_addr=83000000\0" \
	"kernel_addr=84000000\0" \
	"initrd_addr=85000000\0" \
	"boot_device=mmc\0" \
	""

#define CONFIG_BOOTARGS			\
	"mem="MK_STR(MEM_SIZE)"M "		\
	"console=ttyS0," MK_STR(CONFIG_BAUDRATE) " "	\
	"root=/dev/ram rw "		\
	"rdinit=/init"

#define CONFIG_BOOTCOMMAND		\
	"mux_config; "		\
	"mmc dev 0; "		\
	"ext2load mmc 0:1 ${script_addr} boot.scr && source ${script_addr};" \
	"echo Running boot script failed;"

#endif /* !CONFIG_SPL_BUILD */

#if !defined(CONFIG_USE_NAND) && \
	!defined(CONFIG_USE_NOR) && \
	!defined(CONFIG_USE_SPIFLASH)
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_SIZE			(16 << 10)
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_ENV
#endif

/* additions for new relocation code, must added to all boards */
#define CONFIG_SYS_SDRAM_BASE		(PHYS_SDRAM_1)
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x1000 - GENERATED_GBL_DATA_SIZE)

/*
 * Defines for SPL
 */
#define CONFIG_SPL
#define CONFIG_SYS_SRAM_BASE		0x00100000
#define CONFIG_SYS_SRAM_SIZE		0x00010000  /* 64k */
#define CONFIG_SPL_TEXT_BASE		0x00100100  /* first 4k leave for boot_rom bss */
/*
 * SPL Build for thumb code
 */
#define CONFIG_SPL_THUMB_BUILD		1
/*
 the spl code should be 48 KB,and must be aligned to page offset
*/
/* In nand driver V3,
 the first page of nand is nand parameter,
 it is invisible to uplayer except for nand dump command,
 so we also think that spl offset is 0.
 need add one page offset when read and write bootloader in nand driver v3.
 In nand driver V1,
 spl offset must be 0.
*/
#define NAND_START_ADDR  0
#define CONFIG_SPL_NAND_OFFS		(NAND_START_ADDR)
#define CONFIG_SPL_MAX_SIZE		49152

#define CONFIG_UIMAGEHDR_SIZE		0x40

#ifdef _TGT_AP_DDR_AUTO_CALI_ENABLE
#define CONFIG_DDR_CAL_VAL_SIZE	0x10
#define CONFIG_SPL_CODE_MAX_SIZE		48880	//CONFIG_SPL_MAX_SIZE -  CONFIG_DDR_CAL_VAL_SIZE - 0x100
#else
#define CONFIG_DDR_CAL_VAL_SIZE	0
#define CONFIG_SPL_CODE_MAX_SIZE		48896	//CONFIG_SPL_MAX_SIZE -  0x100
#endif

#define CONFIG_SPL_BOARD_INFO_SIZE		512
#define CONFIG_SPL_BOARD_INFO_ADDR		(CONFIG_SYS_SRAM_BASE + CONFIG_SYS_SRAM_SIZE - CONFIG_SPL_BOARD_INFO_SIZE)

#define CONFIG_TRAP_SIZE		0x00c0
#define CONFIG_SPL_LOAD_ADDRESS 	(CONFIG_SYS_SRAM_BASE + CONFIG_TRAP_SIZE)

#define CONFIG_SPL_STACK		CONFIG_SPL_BOARD_INFO_ADDR

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_YMODEM_SUPPORT
#define CONFIG_SPL_IMAGE_SUPPORT

#define CONFIG_MTD_PTBL_OFFS		(CONFIG_SPL_NAND_OFFS + CONFIG_SPL_MAX_SIZE)
#define CONFIG_MTD_PTBL_SIZE		((SPL_APPENDING_TO-CONFIG_SPL_MAX_SIZE/1024)*1024)
#define CONFIG_SYS_BOOTLOADER_MAX_SIZE	(1024*1024)

#ifndef CONFIG_RDA_PDL
/* Check uimage*/
#define CONFIG_SPL_CHECK_IMAGE

/* NAND boot */
#define CONFIG_SPL_NAND_LOAD
/* EMMC boot */
#define CONFIG_SPL_EMMC_LOAD
/* XMODME boot */
#define CONFIG_SPL_XMODEM_LOAD

/* XMODEM load configs */
#ifdef CONFIG_SPL_XMODEM_LOAD
#define CONFIG_SYS_XMODEM_U_BOOT_START	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_XMODEM_U_BOOT_DST 	(CONFIG_SYS_TEXT_BASE - 64)
#endif

/*
 * NAND load configs
*/
#ifdef CONFIG_SPL_NAND_LOAD
#define CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_SIMPLE

#define CONFIG_SYS_NAND_U_BOOT_START	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_NAND_U_BOOT_OFFS 	(CONFIG_SPL_NAND_OFFS + CONFIG_SPL_MAX_SIZE + CONFIG_MTD_PTBL_SIZE)
#define CONFIG_SYS_NAND_U_BOOT_SIZE 	(CONFIG_SYS_BOOTLOADER_MAX_SIZE - CONFIG_MTD_PTBL_SIZE - CONFIG_SPL_MAX_SIZE - CONFIG_SPL_NAND_OFFS)
#define CONFIG_SYS_NAND_U_BOOT_DST 	(CONFIG_SYS_TEXT_BASE - 64)
#endif /* CONFIG_SPL_NAND_LOAD */

/*
 * EMMC load configs
 */
#ifdef CONFIG_SPL_EMMC_LOAD
#define CONFIG_SPL_EMMC_SUPPORT

#define CONFIG_SPL_EMMC_OFFS		0x20000

#define CONFIG_SYS_EMMC_U_BOOT_START	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_EMMC_U_BOOT_OFFS 	(CONFIG_SPL_EMMC_OFFS + CONFIG_SPL_MAX_SIZE + CONFIG_MTD_PTBL_SIZE)
#define CONFIG_SYS_EMMC_U_BOOT_SIZE 	(CONFIG_SYS_BOOTLOADER_MAX_SIZE - CONFIG_MTD_PTBL_SIZE - CONFIG_SPL_MAX_SIZE - CONFIG_SPL_EMMC_OFFS)
#define CONFIG_SYS_EMMC_U_BOOT_DST 	(CONFIG_SYS_TEXT_BASE - 64)
#endif /* CONFIG_SPL_EMMC_LOAD */

#else /* CONFIG_RDA_PDL */

#define CONFIG_SPL_USB_SUPPORT
#define CONFIG_SPL_MUSB_UDC_SUPPORT
#define CONFIG_SPL_RDA_PDL

#endif /* CONFIG_RDA_PDL */

#define CONIFG_DDR_CAL_VAL_FLAG	0xA1B2
#define CONFIG_DDR_CAL_SEC_OFFS	CONFIG_SPL_MAX_SIZE - CONFIG_DDR_CAL_VAL_SIZE - 0x100
#endif /* __RDA_CONFIG_H__ */
