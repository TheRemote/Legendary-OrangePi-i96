#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/semaphore.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/io.h>
#include <asm/sizes.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/ifc.h>
#include <mach/timex.h>
#include <plat/devices.h>
#include <plat/reg_ifc.h>
#include <plat/rda_debug.h>
#include <plat/reg_mmc.h>
#include <linux/gpio.h>

#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <mach/rda_clk_name.h>
#include <mach/iomap.h>
#include <plat/reg_sysctrl.h>
#include <plat/cpu.h>

#include <asm/irq.h>
#include <linux/irq.h>

#include <rda/tgt_ap_board_config.h>

#define RDA_MMC_USE_INT

#ifdef _TGT_AP_SDMMC1_REQ_NUM_PAGE
#define RDA_MMC0_REQ_NUM_PAGE	_TGT_AP_SDMMC1_REQ_NUM_PAGE
#else
#define RDA_MMC0_REQ_NUM_PAGE	(16)
#endif /* _TGT_AP_SDMMC1_REQ_NUM_PAGE */

#define MCD_CMD_TIMEOUT_MS	( 100 )
#define MCD_DATA_TIMEOUT_MS 	( 5000 )
#define RDA_MMC_HOST_COUNT 	( 4 )

typedef struct
{
	u8* sysMemAddr;
	u32 blockNum;
	u32 blockSize;
	HAL_SDMMC_DIRECTION_T direction;
	HAL_IFC_REQUEST_ID_T ifcReq;
	u32 channel;
} HAL_SDMMC_TRANSFER_T;

struct rda_mmc_host {
	struct mmc_host *mmc;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data *data;

	spinlock_t		lock;
	void __iomem		*base;		/* virtual */
	int 		irq;

	HAL_SDMMC_TRANSFER_T data_transfer;
	int transfer_done;
	struct completion req_done;

	unsigned int dma_len;
	unsigned int dma_dir;
	int data_err;

	u8 suspend;
	u8 sys_suspend;
	u8 sdio_irq_enable;
	u8 eirq_enable;
	int eirq;
	int id;
	struct tasklet_struct	finish_tasklet;
	u8 mmc_pm;

	struct regulator *host_reg;
	struct clk *master_clk;
	unsigned long clock;
	unsigned int mclk_adj;
	unsigned long bus_width;

	int clk_inv;

	int det_pin;
	int present;

	struct delayed_work timeout_work;
	/*
	 * Note:
	 * Following definition is for fixing a bug of IFC dma before U06.
	 * If HW has fixed it, we will remove these definitions.
	 */
	struct platform_device *plat_dev;
	void *tmp_buf;
	dma_addr_t phys_tmp;

	unsigned int unaligned_count;
	unsigned int reset_count;
};

struct rda_mmc_host* mmc_host_p[RDA_MMC_HOST_COUNT] = {NULL, NULL, NULL, NULL};

static void rda_mmc_reset(u32 host_id);
static int hal_mmc_init(struct rda_mmc_host *host);
static void hal_set_bus_width(struct rda_mmc_host *host,
				unsigned char bus_width);
static int hal_mmc_init(struct rda_mmc_host *host);

static void hal_send_cmd(struct rda_mmc_host *host,
				struct mmc_command *cmd, struct mmc_data *data)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	u32 configReg = 0;
	HAL_SDMMC_TRANSFER_T *transfer = &host->data_transfer;

	hwp_sdmmc->SDMMC_CONFIG = 0x00000000;

	configReg = SDMMC_SDMMC_SENDCMD;
	if (cmd->flags & MMC_RSP_PRESENT) {
		configReg |= SDMMC_RSP_EN;
		if (cmd->flags & MMC_RSP_136)
			configReg |= SDMMC_RSP_SEL_R2;
		else if (cmd->flags & MMC_RSP_CRC)
			configReg |= SDMMC_RSP_SEL_OTHER;
		else
			configReg |= SDMMC_RSP_SEL_R3;
	}

	/* cases for data transfer */
	if (cmd->opcode == MMC_READ_SINGLE_BLOCK) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);
	} else if (cmd->opcode == MMC_READ_MULTIPLE_BLOCK) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ |
				SDMMC_S_M_SEL_MULTIPLE);

		if (host->id == 0 || host->id == 2) {
			/*
			 * Multiple block transfers for SD require CMD12 to stop the transactions.
			 * The Host Controller automatically issues CMD12 when the last block transfer
			 * is completed.
			 * For more detals, please refer to SD Host Controller Specification.
			 * */
			configReg |= SDMMC_AUTO_FLAG_EN;
		}
	} else if (cmd->opcode == MMC_WRITE_BLOCK) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_WRITE);
	} else if (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_WRITE |
				SDMMC_S_M_SEL_MULTIPLE);

		if (host->id == 0 || host->id == 2) {
			configReg |= SDMMC_AUTO_FLAG_EN;
		}
	} else if (cmd->opcode == SD_APP_SEND_SCR) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);
	} else if (cmd->opcode == MMC_SEND_STATUS && data) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);
	} else if (cmd->opcode == MMC_SWITCH && data) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);
	} else if(cmd->opcode == MMC_SEND_EXT_CSD && data) {
		configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);
	} else if(cmd->opcode == SD_IO_RW_EXTENDED) {
		if (cmd->data->flags & MMC_DATA_WRITE)
			configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_WRITE);
		else
			configReg |= (SDMMC_RD_WT_EN | SDMMC_RD_WT_SEL_READ);

		if(cmd->data->blocks > 1)
			configReg |= SDMMC_S_M_SEL_MULTIPLE;

		if(transfer->blockSize != data->blksz){
		   cmd->arg &= 0xfffffe00;
			cmd->arg |= transfer->blockSize;
		}
	}

	hwp_sdmmc->SDMMC_CMD_INDEX = SDMMC_COMMAND(cmd->opcode);
	hwp_sdmmc->SDMMC_CMD_ARG   = SDMMC_ARGUMENT(cmd->arg);
	hwp_sdmmc->SDMMC_CONFIG    = configReg ;
}

static int hal_cmd_done(struct rda_mmc_host *host)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	return (!(hwp_sdmmc->SDMMC_STATUS & SDMMC_NOT_SDMMC_OVER));
}

static int hal_wait_cmd_done(struct rda_mmc_host *host,
				struct mmc_command *cmd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MCD_CMD_TIMEOUT_MS);

	/* command done need be polled, no interrupt indicating cmd done */
	while (time_before(jiffies, timeout) && !hal_cmd_done(host));

	if (!hal_cmd_done(host)) {
		dev_info(mmc_dev(host->mmc), "cmd %d timeout\n", cmd->opcode);
		return -ETIMEDOUT;
	}

	return 0;
}

HAL_SDMMC_OP_STATUS_T hal_get_op_status(struct rda_mmc_host *host)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	return ((HAL_SDMMC_OP_STATUS_T)(u32)hwp_sdmmc->SDMMC_STATUS);
}

static int hal_wait_cmd_resp(struct rda_mmc_host *host)
{
	HAL_SDMMC_OP_STATUS_T status = hal_get_op_status(host);

	if (status.fields.noResponseReceived) {
		rda_dbg_mmc("rsp noResponseReceived\n");
		return -EIO;
	}

	if (status.fields.responseCrcError) {
		dev_err(mmc_dev(host->mmc), "rsp responseCrcError\n");
		return -EIO;
	}

	return 0;
}

static void hal_get_resp(struct rda_mmc_host *host, struct mmc_command *cmd)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[0] = hwp_sdmmc->SDMMC_RESP_ARG3;
			cmd->resp[1] = hwp_sdmmc->SDMMC_RESP_ARG2;
			cmd->resp[2] = hwp_sdmmc->SDMMC_RESP_ARG1;
			cmd->resp[3] = hwp_sdmmc->SDMMC_RESP_ARG0 << 1;
		}
		else {
			cmd->resp[0] = hwp_sdmmc->SDMMC_RESP_ARG3;
			cmd->resp[1] = 0;
			cmd->resp[2] = 0;
			cmd->resp[3] = 0;
		}
	}
}

static int hal_data_transfer_start(struct rda_mmc_host *host,
				HAL_SDMMC_TRANSFER_T* transfer)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	u32 length = 0;
	u32 lengthExp = 2;
	u32 blockSize = 4;

	length = transfer->blockSize;

	/* The block size of register must be 2 by order. */
	if (length > 4) {
		while (blockSize < length) {
			blockSize = blockSize << 1;
			lengthExp++;
		}
	}
	/* resize blockSize */
	transfer->blockSize = blockSize;

	// Configure amount of data
	hwp_sdmmc->SDMMC_BLOCK_CNT = SDMMC_SDMMC_BLOCK_CNT(transfer->blockNum);
	hwp_sdmmc->SDMMC_BLOCK_SIZE = SDMMC_SDMMC_BLOCK_SIZE(lengthExp);

	// Configure Bytes reordering
	hwp_sdmmc->SDMMC_CTRL = SDMMC_SOFT_RST_L | SDMMC_L_ENDIAN(1);

	switch (transfer->direction){
		case HAL_SDMMC_DIRECTION_READ:
			transfer->ifcReq = HAL_IFC_SDMMC_RX + host->id * 2;
			break;

		case HAL_SDMMC_DIRECTION_WRITE:
			transfer->ifcReq = HAL_IFC_SDMMC_TX + host->id *2;
			break;

		default:
			dev_err(mmc_dev(host->mmc),
				"hal_data_transfer_start, invalide direction %d\n",
				transfer->direction);
			return -EILSEQ;
	}

	transfer->channel = ifc_transfer_start(
			transfer->ifcReq, transfer->sysMemAddr,
			transfer->blockNum*transfer->blockSize,
			HAL_IFC_SIZE_32_MODE_MANUAL);
	if (transfer->channel == HAL_UNKNOWN_CHANNEL){
		dev_err(mmc_dev(host->mmc), "transfer start with invalide channel\n");
		return -EILSEQ;
	}
	else{
		rda_dbg_mmc("start channel %d\n", (int)transfer->channel);
		return 0;
	}
}

static void hal_data_transfer_stop(struct rda_mmc_host *host,
				HAL_SDMMC_TRANSFER_T* transfer)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	struct mmc_data *data = host->data;

	// Configure amount of data
	hwp_sdmmc->SDMMC_BLOCK_CNT	= SDMMC_SDMMC_BLOCK_CNT(0);
	hwp_sdmmc->SDMMC_BLOCK_SIZE = SDMMC_SDMMC_BLOCK_SIZE(0);

	rda_dbg_mmc("stop channel %d\n", (int)transfer->channel);

	if (transfer->channel == HAL_UNKNOWN_CHANNEL
		|| transfer->channel >= SYS_IFC_STD_CHAN_NB) {
		dev_err(mmc_dev(host->mmc),
			"hal_data_transfer_stop invalide channel %d \n",
			transfer->channel);
		return;
	}

	/* Check if there is sd-card as doing hot-plug. */
	if (host->present && data && (data->flags & MMC_DATA_READ)) {
		/* IFC supports only flush operation with read. */
		ifc_transfer_flush(transfer->ifcReq, transfer->channel);
	}

	if (data) {
		ifc_transfer_stop(transfer->ifcReq, transfer->channel);
	}
	transfer->channel = HAL_UNKNOWN_CHANNEL;
	transfer->ifcReq = HAL_IFC_NO_REQWEST;
}

static u32 hal_irq_get_status(struct rda_mmc_host *host)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	u32 int_status;

	int_status = hwp_sdmmc->SDMMC_INT_STATUS;
	return int_status;
}

#ifdef RDA_MMC_USE_INT
static void hal_irq_clear(struct rda_mmc_host *host, u32 int_status)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	hwp_sdmmc->SDMMC_INT_CLEAR = (int_status & 0xFF);
}
#endif

#ifndef RDA_MMC_USE_INT
static int hal_data_transfer_done(struct rda_mmc_host *host,
				HAL_SDMMC_TRANSFER_T* transfer)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	int data_cmpl = 0;
	u32 int_status;

	int_status = hal_irq_get_status(host);

	BUG_ON(transfer->channel == HAL_UNKNOWN_CHANNEL);

	if (transfer->direction == HAL_SDMMC_DIRECTION_READ
			&& int_status & SDMMC_RXDMA_DONE_INT) {
		data_cmpl = 1;
	}

	if (transfer->direction == HAL_SDMMC_DIRECTION_WRITE
			&& int_status & SDMMC_DAT_OVER_INT) {
		data_cmpl = 1;
	}

	if (data_cmpl) {
		// Transfer is over
		hwp_sdmmc->SDMMC_INT_CLEAR = SDMMC_DAT_OVER_CL;
		ifc_transfer_stop(transfer->ifcReq, transfer->channel);

		rda_dbg_mmc("release channel %d\n", (int)transfer->channel);

		// We finished a read
		transfer->channel = HAL_UNKNOWN_CHANNEL;

		//	Put the FIFO in reset state.
		//hwp_sdmmc->SDMMC_CTRL = 0 | SDMMC_L_ENDIAN(1);
		hwp_sdmmc->SDMMC_CTRL = SDMMC_SOFT_RST_L | SDMMC_L_ENDIAN(1);

		return 1;
	}
	else {
		return 0;
	}
}

static int hal_wait_data_transfer_done(struct rda_mmc_host *host,
				HAL_SDMMC_TRANSFER_T* transfer)
{
	//HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	u32 tran_time_out_ms = MCD_DATA_TIMEOUT_MS* transfer->blockNum;
	unsigned long timeout;

	// Wait (This could be done in interrupt */
	timeout = jiffies + msecs_to_jiffies(tran_time_out_ms);
	while(!hal_data_transfer_done(host, transfer)){
		if (time_after(jiffies, timeout)) {
			hal_data_transfer_stop(host, transfer);
			dev_err(mmc_dev(host->mmc),
				"wait transfer done timeout\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}
#endif

#if 0
static int hal_data_read_check_crc(struct rda_mmc_host *host)
{
	HAL_SDMMC_OP_STATUS_T operationStatus;
	operationStatus = hal_get_op_status(host);

	if (operationStatus.fields.dataError != 0){
		dev_err(mmc_dev(host->mmc),
			"data_read_check_crc fail, status:%08x\n",
			operationStatus.reg);
	}

	return 0;
}
#endif /* #if 0 */

static int hal_data_write_check_crc(struct rda_mmc_host *host)
{
	HAL_SDMMC_OP_STATUS_T operationStatus;
	operationStatus = hal_get_op_status(host);

	if (operationStatus.fields.crcStatus != 2){
		dev_err(mmc_dev(host->mmc),
			"data_write_check_crc fail, status:%08x\n",
			operationStatus.reg);
		return -EILSEQ;
	}
	else {
		return 0;
	}
}

static void hal_set_bus_width(struct rda_mmc_host *host,
				unsigned char bus_width)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	hwp_sdmmc->SDMMC_DATA_WIDTH = 1<<bus_width;
}


static int hal_mmc_init(struct rda_mmc_host *host)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;

	/* we only care DATA_OVER and DATA ERR Interrupt */
	hwp_sdmmc->SDMMC_INT_MASK = 0x5F;

	return 0;
}

static int hal_mmc_disable(struct rda_mmc_host *host)
{
	HWP_SDMMC_T *hwp_sdmmc = (HWP_SDMMC_T*)host->base;

	hwp_sdmmc->SDMMC_INT_MASK = 0;
	hwp_sdmmc->SDMMC_MCLK_ADJUST = SDMMC_CLK_DISA;

	return 0;
}

static int do_command(struct mmc_host *mmc,
				struct mmc_command *cmd,
				struct mmc_data *data);
static void finish_request(struct rda_mmc_host *host,
				struct mmc_request *mrq);

#ifdef RDA_MMC_USE_INT
static int do_data_abort(struct rda_mmc_host *host)
{
	int result = 0;
	HAL_SDMMC_TRANSFER_T *transfer = &host->data_transfer;
	struct mmc_data *data = host->data;

	rda_dbg_mmc("do_data_abort\n");

	hal_data_transfer_stop(host, transfer);

	if (host->mrq) {
		host->mrq->cmd->error = -EILSEQ;
	}

	if (data) {
		data->error = -EILSEQ;
	}

	return result;
}

static int do_data_complete(struct rda_mmc_host *host)
{
	int result = 0;
	HAL_SDMMC_TRANSFER_T *transfer = &host->data_transfer;
	struct mmc_data *data = host->data;

	rda_dbg_mmc("host id:%d do_data_complete \n", host->id);

	if (!data || transfer->channel == HAL_UNKNOWN_CHANNEL
		|| transfer->channel >= SYS_IFC_STD_CHAN_NB) {
		dev_err(mmc_dev(host->mmc), "do_data_complete invalid channel(%d), flags = 0x%x\n",
			transfer->channel, (data ? data->flags : 0));
		return -EREMOTEIO;
	}

	if (ifc_transfer_get_tc(transfer->ifcReq, transfer->channel) == 0) {
		ifc_transfer_stop(transfer->ifcReq, transfer->channel);
		rda_dbg_mmc("release channel %d\n", (int)transfer->channel);
		transfer->channel = HAL_UNKNOWN_CHANNEL;
	}
	else {
		hal_data_transfer_stop(host, transfer);
		dev_err(mmc_dev(host->mmc), "data complete but DMA not done\n");
		data->error = -ETIMEDOUT;
	}

	/* 
	 * Just check if there is a crc error of writing.
	 * Do not care reading, because our HW does not clear crc error automatically.
	 * In fact, there is always RD_ERR_INT before crc error. It will be processed firstly.
	 */
	if (data->flags & MMC_DATA_WRITE) {
		result = hal_data_write_check_crc(host);
		if (result) {
			dev_err(mmc_dev(host->mmc),
				"hal_data_write_check_crc fail, ret = %d\n",
				result);
			data->error = result;
		}
	}

	return result;
}

static void tasklet_worker(unsigned long param)
{
	struct rda_mmc_host *host = (struct rda_mmc_host*)param;

	if (host->data_err) {
		do_data_abort(host);
		host->data_err = 0;
	} else {
		do_data_complete(host);
	}

	complete(&host->req_done);

	return;
}

static irqreturn_t rda_mmc_irq(int irq, void *dev_id)
{
	struct rda_mmc_host *host = dev_id;
	struct mmc_host * mmc = NULL;
	int data_cmpl = 0, data_err = 0;
	u32 int_status;
	struct mmc_data *data;

	int_status = hal_irq_get_status(host);
	hal_irq_clear(host, int_status);

	rda_dbg_mmc("rda_mmc_irq, int_status = 0x%08x\n", int_status);

	/* If there isn't card, we return immediately. */
	if (!host->present) {
		return IRQ_HANDLED;
	}

	mmc = host->mmc;
	data = host->data;

	/* we only care DATA_OVER and DATA ERR Interrupt */
	if (int_status & (SDMMC_RD_ERR_INT | SDMMC_WR_ERR_INT)) {
		data_err = 1;
	}

	/*
	 * RXDMA_DONE might arrive before DAT_OVER setting as reading,
	 * or RXDMA_DONE has arrived, but there is DAT_OVER flag.
	 * So, we check only RXDMA_DONE flag to indicate that reading is over.
	 *
	 */
	if (data && (data->flags & MMC_DATA_READ)) {
		if (int_status & SDMMC_RXDMA_DONE_INT) {
			data_cmpl = 1;
		}
	}

	/* Ignore DAT_OVER flag as reading */
	if (int_status & SDMMC_DAT_OVER_INT) {
		if (data && (data->flags & MMC_DATA_WRITE)) {
			data_cmpl = 1;
		}
	}

	if ((int_status & SDMMC_SDIO_INT) && host->sdio_irq_enable
			&& !host->eirq_enable) {
		mmc_signal_sdio_irq(mmc);
	}

	if (data_err) {
		host->data_err = data_err;
		tasklet_schedule(&host->finish_tasklet);
	} else if (data_cmpl) {
		tasklet_schedule(&host->finish_tasklet);
	}

	return IRQ_HANDLED;
}
#endif

static inline void finish_request(struct rda_mmc_host *host, struct mmc_request *mrq)
{
	mmc_request_done(host->mmc, mrq);
}

/* send command to the mmc card and wait for results */
static int do_command(struct mmc_host *mmc,
				struct mmc_command *cmd,
				struct mmc_data *data)
{
	int result;
	struct rda_mmc_host *host = mmc_priv(mmc);

	rda_dbg_mmc("do_command, cmdidx = %d, cmdarg = 0x%08x, flags = %x\n",
		cmd->opcode, cmd->arg, cmd->flags);

	host->cmd = cmd;
	hal_send_cmd(host, cmd, data);
	result = hal_wait_cmd_done(host, cmd);
	if (result) {
		result = (host->present == 0) ? -ENOMEDIUM : result;
		cmd->error = result;

		rda_dbg_mmc("cmd %d, wait cmd fail, ret = %d\n",
				cmd->opcode, result);

		return result;
	}

	if (cmd->flags & MMC_RSP_PRESENT){
		result= hal_wait_cmd_resp(host);
		if (result) {
			result = (host->present == 0) ? -ENOMEDIUM : result;
			cmd->error = result;

			rda_dbg_mmc("cmd %d, wait resp fail, ifc = %d, ret = %d\n",
				cmd->opcode, (int)host->data_transfer.channel, result);

			return result;
		}
	}
	hal_get_resp(host, cmd);

	if (cmd->flags & MMC_RSP_PRESENT) {
		rda_dbg_mmc("  response: %08x %08x %08x %08x\n",
			cmd->resp[0], cmd->resp[1],
			cmd->resp[2], cmd->resp[3]);
	}

	return result;
}

static int __do_data_transfer(struct mmc_host *mmc,
	struct mmc_command *cmd,
	struct mmc_data *data,
	dma_addr_t dma_addr)
{
	int result = 0;
	struct rda_mmc_host *host = mmc_priv(mmc);
	HAL_SDMMC_TRANSFER_T *transfer = &host->data_transfer;
	unsigned int tsize;
#ifdef RDA_MMC_USE_INT
	unsigned long timeout;
#endif /* RDA_MMC_USE_INT */

	rda_dbg_mmc(
		"%s, id:%d cmdidx = %d, blks = %d, addr = 0x%08x\n",
		__func__, host->id,cmd->opcode, data->blocks,
		dma_addr);

	/* Check if address of dma is aligened with 4bytes. */
	if (!IS_ALIGNED(dma_addr, 4)) {
		return -EFAULT;
	}

	transfer->sysMemAddr = (u8 *)dma_addr;
	transfer->blockNum = data->blocks;
	transfer->blockSize = data->blksz;

	if (data->flags & MMC_DATA_READ) {
		transfer->direction  = HAL_SDMMC_DIRECTION_READ;
	} else if (data->flags & MMC_DATA_WRITE) {
		transfer->direction  = HAL_SDMMC_DIRECTION_WRITE;
	}

	tsize = data->blocks * data->blksz;
	if (rda_soc_is_older_metal10()) {
		if (!IS_ALIGNED(dma_addr, 16) &&
			(dma_addr >> PAGE_SHIFT != ((dma_addr + tsize -1) >> PAGE_SHIFT))) {
			transfer->sysMemAddr = (u8 *)host->phys_tmp;
			++host->unaligned_count;

			if (data->blocks * data->blksz > PAGE_SIZE) {
				dev_err(mmc_dev(host->mmc),
					"data is too large! size = %d\n",
					data->blocks * data->blksz);
				BUG_ON(1);
			}

			if (data->flags & MMC_DATA_WRITE) {
				memcpy(host->tmp_buf, (void*)phys_to_virt(dma_addr), tsize);
			}
		}
	}

	// Initiate data migration through Ifc.
	result = hal_data_transfer_start(host, transfer);
	if (result) {
		cmd->error = result;
		dev_err(mmc_dev(host->mmc),
			"data transfer start fail, ret = %d\n", result);
		goto exit;
	}

	/* Config completion of request */
	INIT_COMPLETION(host->req_done);

	result = do_command(mmc, cmd, data);
	if (result) {
		dev_err(mmc_dev(host->mmc),
			"do_command fail, ret = %d\n", result);
		hal_data_transfer_stop(host, transfer);
		goto exit;
	}

#ifndef RDA_MMC_USE_INT
	result = hal_wait_data_transfer_done(host, transfer);
	if (result) {
		dev_err(mmc_dev(host->mmc),
			"wait transfer done fail, ret = %d\n", result);
		hal_data_transfer_stop(host, transfer);
		goto exit;
	}

	if (data->flags & MMC_DATA_READ) {
		result = hal_data_read_check_crc(host);
		if (result) {
			dev_err(mmc_dev(host->mmc),
				"read check crc fail, ret = %d\n", result);
			goto exit;
		}
	} else if (data->flags & MMC_DATA_WRITE) {
		result = hal_data_write_check_crc(host);
		if (result) {
			dev_err(mmc_dev(host->mmc),
				"write check crc fail, ret = %d\n", result);
			goto exit;
		}
	}

	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

	if (data->stop) {
		result = do_command(mmc, data->stop, NULL);
	}
	if (result) {
		dev_err(mmc_dev(host->mmc),
			"stop command fail, ret = %d\n", result);
	}

	host->transfer_done = 1;
#else
	timeout = wait_for_completion_timeout(&host->req_done,
		msecs_to_jiffies(MCD_DATA_TIMEOUT_MS));
	if (timeout == 0) {
		u32 irq = hal_irq_get_status(host);
		u32 op_status = (hal_get_op_status(host)).reg;
		u32 ifc_tc = ifc_transfer_get_tc(transfer->ifcReq,
				transfer->channel);

		if (data->flags & MMC_DATA_READ) {
			dev_err(mmc_dev(host->mmc),
				"transfer (read) cmd timeout(%d ms): "
				"cmd(%d), blocks(%d), irq = 0x%x, "
				"op_sta = 0x%x, tc = 0x%x\n",
				MCD_DATA_TIMEOUT_MS, cmd->opcode,
				data->blocks, irq, op_status, ifc_tc);
		} else if (data->flags & MMC_DATA_WRITE) {
			dev_err(mmc_dev(host->mmc),
				"transfer (write) cmd timeout(%d ms): "
				"cmd(%d), blocks(%d), irq = 0x%x, "
				"op_sta = 0x%x, tc = 0x%x\n",
				MCD_DATA_TIMEOUT_MS, cmd->opcode,
				data->blocks, irq, op_status, ifc_tc);
		}

		/* Stop dma's action */
		hal_data_transfer_stop(host, transfer);

		/* Not for SDIO */
		if (host->id == 0 || host->id == 2) {
			struct mmc_command stop;

			/*
		 	 * As timeout in sending-data/receive-data state, HW doesn't automatically return to
		 	 * transfer state. So we have to send a CMD12 to HW to return to transfer state.
		 	 * Otherwise, HW will not process any data.
		 	 * For more details, please refer to SD' specification.
		 	 */
			stop.opcode = MMC_STOP_TRANSMISSION;
			stop.arg = 0;
			stop.flags = MMC_RSP_R1B | MMC_CMD_AC;

			if (!data->stop) {
				do_command(host->mmc, &stop, NULL);
			}
		}

		result = -ETIMEDOUT;
		/* Report error to upper layer */
		cmd->error = result;
		data->error = result;
	} else {
		host->transfer_done = 1;
	}

	/* Check if there is a card. */
	if (host->present == 0) {
		cmd->error = -ENOMEDIUM;
		data->error = -ENOMEDIUM;
	}

	if (!data->error) {
		data->bytes_xfered += data->blocks * (data->blksz);
		if (data->flags & MMC_DATA_READ) {
			if (rda_soc_is_older_metal10()) {
				if (!IS_ALIGNED(dma_addr, 16) &&
					(dma_addr >> PAGE_SHIFT != ((dma_addr + tsize -1) >> PAGE_SHIFT))) {
					memcpy((void*)phys_to_virt(dma_addr), host->tmp_buf,  tsize);
				}
			}
		}

	} else {
		hal_data_transfer_stop(host, transfer);
		dev_err(mmc_dev(host->mmc), "transfer data error %d\n",
			data->error);
		data->bytes_xfered = 0;
	}

#endif /* RDA_MMC_USE_INT */

exit:

	return result;
}

static int do_data_transfer(struct mmc_host *mmc,
				struct mmc_command *cmd,
				struct mmc_data *data)
{
	struct rda_mmc_host *host = mmc_priv(mmc);
	unsigned int i;
	int ret = 0;

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
		host->dma_len = dma_map_sg(mmc_dev(mmc), data->sg,
			data->sg_len, host->dma_dir);

	} else if (data->flags & MMC_DATA_WRITE) {
		host->dma_dir = DMA_TO_DEVICE;
		host->dma_len = dma_map_sg(mmc_dev(mmc), data->sg,
			data->sg_len, host->dma_dir);

	} else {
		/* Invalid flag */
		return -EINVAL;
	}

	host->data = data;

	for (i = 0; i < data->sg_len; i++) {
		rda_dbg_mmc("do_data_transfer, sg %d, size = %d\n",
			i, data->sg[i].length);

#if 0
		if ((rda_debug & RDA_DBG_MMC) && (data->flags & MMC_DATA_WRITE))
			rda_dump_buf((char *)sg_virt(&data->sg[i]),
				data->sg[i].length);
#endif

		/*
		 * For some card of emmc, when exec command 18 & 25,
		 * the start offset must be aligned to 2.
		 */
		if (host->id == 2 && (cmd->arg & 1) && data->blocks > 1 &&
			(cmd->opcode == MMC_READ_MULTIPLE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK))
		{
			u32 blksz = data->blksz;
			u32 blknum = data->blocks;
			u32 opcode = cmd->opcode;
			dma_addr_t dma_addr = sg_dma_address(&data->sg[i]);

			//dev_err(mmc_dev(host->mmc), "WARNING: unaligned block offset 0x%08x.\n", cmd->arg);

			/* read/write one block first by single r/w command */
			if(opcode == MMC_WRITE_MULTIPLE_BLOCK) {
				cmd->opcode = MMC_WRITE_BLOCK;
			}
			else {
				cmd->opcode = MMC_READ_SINGLE_BLOCK;
			}
			data->blocks = 1;
			ret = __do_data_transfer(mmc, cmd, data, dma_addr);
			if (ret) {
				break;
			}

			/* read/write other block(s) by aligned offset */
			if((blknum - 1) > 1) {
				cmd->opcode = opcode;
			}
			data->blocks = blknum - 1;
			cmd->arg += 1; /* aligned offset */
			dma_addr += blksz;
			ret = __do_data_transfer(mmc, cmd, data, dma_addr);
			if (ret) {
				break;
			}
		} else {
			dma_addr_t dma_addr = sg_dma_address(&data->sg[i]);
			ret = __do_data_transfer(mmc, cmd, data, dma_addr);
			if (ret) {
				break;
			}
		}

		/* Sync memory and cache */
		if (data->flags & MMC_DATA_READ) {
			dma_map_sg(mmc_dev(host->mmc), &data->sg[i], 1, DMA_FROM_DEVICE);
		}

#if 0
		if ((rda_debug & RDA_DBG_MMC) && (data->flags & MMC_DATA_READ))
			rda_dump_buf((char *)sg_virt(&data->sg[i]),
				data->sg[i].length);
#endif
	}

	host->data = NULL;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len, host->dma_dir);

	return ret;
}

static void rda_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct rda_mmc_host *host = mmc_priv(mmc);
	int ret;

	/*
	 * HW supports four controllers of sdmmc.
	 * Only do SDIO command for sdmmc1
	 */
	if ((host->id != 1 && mrq->cmd->opcode == SD_IO_SEND_OP_COND) ||
		(host->id != 1 && mrq->cmd->opcode == SD_IO_RW_DIRECT)) {
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	host->transfer_done = 0;
	if (mrq->data) {
		ret = do_data_transfer(mmc, mrq->cmd, mrq->data);
	} else {
		ret = do_command(mmc, mrq->cmd, mrq->data);
		mrq->cmd->error = ret;
	}

	host->mrq = NULL;

	if (ret) {
		rda_dbg_mmc("rda_mmc_request fail, ret = %d\n", ret);
		mmc_request_done(mmc, mrq);
		return;
	}

	if (!mrq->data || host->transfer_done) {
		finish_request(host, mrq);
	}
}

static int rda_mmc_get_ro(struct mmc_host *mmc)
{
	/*
	 * Board doesn't support read only detection; let the mmc core
	 * decide what to do.
	 */
	//return -ENOSYS;

	/* return not read-only for now */
	return 0;
}

static void rda_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct rda_mmc_host *host = mmc_priv(mmc);
	unsigned long mclk = clk_get_rate(host->master_clk);
	unsigned long clk_div;
	int ret;
	HWP_SDMMC_T *hwp_sdmmc = NULL;

	if (!host) {
		return;
	}

	hwp_sdmmc = (HWP_SDMMC_T*)host->base;

	/* Power control */
	switch (ios->power_mode) {
		case MMC_POWER_OFF:
			if (host->mmc_pm != MMC_POWER_OFF && host->id == 0) {
				ret = regulator_disable(host->host_reg);
				/* Waiting 40ms until power is completely power off. */
				mdelay(40);
			}
			host->mmc_pm = MMC_POWER_OFF;
			break;

		case MMC_POWER_UP:
			if (host->mmc_pm == MMC_POWER_OFF && host->id == 0) {
				ret = regulator_enable(host->host_reg);
			}
			host->mmc_pm = MMC_POWER_UP;
			break;

		case MMC_POWER_ON:
			host->mmc_pm = MMC_POWER_ON;
			break;

		default:
			break;
	}

	if (host->clock != ios->clock) {
		host->clock = ios->clock;
		if (host->clock) {
			/*
			 * Check if HW can provide the frequency we want or not.
			 * If no, we use the maximum frequency provide by HW.
			 * In general mode, sd card can work at 25MHz.
			 * In High-Speed mode, it can work at 50MHz.
			 * For more information, please refer to SD card spec.
			 */
			clk_div = mclk / (2 * host->clock);
			if (mclk % (2 * host->clock))
				clk_div ++;

			if (clk_div >= 1) {
				clk_div -= 1;
			}
			if (clk_div > 255) {
				/* clock too slow */
				clk_div = 255;
			}
			dev_info(mmc_dev(host->mmc),
				"set clk = %d, bus_clk = %d, divider = %d\n",
				(int)host->clock, (int)mclk, (int)clk_div);

			hwp_sdmmc->SDMMC_TRANS_SPEED =
				SDMMC_TRANS_SPEED(clk_div);
			hwp_sdmmc->SDMMC_MCLK_ADJUST =
				SDMMC_MCLK_ADJUST(host->mclk_adj);
			if (host->clk_inv) {
				hwp_sdmmc->SDMMC_MCLK_ADJUST |= SDMMC_CLK_INV;
			}
		} else {
			hwp_sdmmc->SDMMC_MCLK_ADJUST = SDMMC_CLK_DISA;
		}
	}

	if (host->bus_width != ios->bus_width) {
		host->bus_width = ios->bus_width;
		dev_info(mmc_dev(host->mmc), "set bus_width = %d\n",
			1<<host->bus_width);
		hal_set_bus_width(host, host->bus_width);
	}
	rda_dbg_mmc("host:(%d)set bus_width to %d pm_mode %d \n",
		host->id, ios->bus_width, ios->power_mode);
}

static void rda_mmc_mask_eirq(struct rda_mmc_host *host)
{
	struct irq_desc * desc = NULL;
	desc = irq_to_desc(host->eirq);

	if (!desc->depth) {
		disable_irq_nosync(host->eirq);
	}
}

static void rda_mmc_unmask_eirq(struct rda_mmc_host *host)
{
	struct irq_desc * desc = NULL;
	desc = irq_to_desc(host->eirq);

	while (desc->depth) {
		enable_irq(host->eirq);
	}
}
static void rda_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	unsigned long flags;
	struct rda_mmc_host *host = NULL;
	HWP_SDMMC_T *hwp_sdmmc = NULL;
	host = (struct rda_mmc_host *)mmc_priv(mmc);

	hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	spin_lock_irqsave(&host->lock, flags);

	if(enable && host->sdio_irq_enable){
		if(!host->eirq_enable)
			hwp_sdmmc->SDMMC_INT_MASK |= SDMMC_SDIO_INT_MK;
		else
			rda_mmc_unmask_eirq(host);
	} else {
		if(!host->eirq_enable) {
			hwp_sdmmc->SDMMC_INT_MASK &= ~SDMMC_SDIO_INT_MK;
		}
		else
			rda_mmc_mask_eirq(host);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int rda_mmc_get_cd(struct mmc_host *mmc)
{
	int present = -ENOSYS;
	struct rda_mmc_host *host = mmc_priv(mmc);

	if (gpio_is_valid(host->det_pin)) {
		present = !gpio_get_value(host->det_pin);
		host->present = present;
		dev_dbg(mmc_dev(host->mmc), "card is %s present\n", present ? "" : "not");

		if (!present) {
			hal_mmc_disable(host);
			/* Clear all pending irqs. */
			hal_irq_clear(host, 0xFF);
			/* Reset ifc to clear fifo. */
			rda_mmc_reset(host->id);
			hal_set_bus_width(host, 0);
			host->bus_width = 0;
		}
	} else {
		/* If no detection, we assume there is a card. */
		present = 1;
		host->present = 1;
	}

	return present;
}

static irqreturn_t rda_mmc_det_irq(int irq, void *data)
{
	struct rda_mmc_host *host = data;
	int present;

	/* entering this ISR means that we have configured det_pin:
	 * we can use its value in board structure */
	present = !gpio_get_value(host->det_pin);

	/*
	 * we expect this irq on both insert and remove,
	 * and use a short delay to debounce.
	 */
	if (present != host->present) {
		host->present = present;
		pr_info("%s: card %s\n", mmc_hostname(host->mmc),
			present ? "insert" : "remove");

		if (!present) {
			if (host->data) {
				/* Wake up requeset's waiting queue */
				complete(&host->req_done);
			}
		} else {
			hal_mmc_init(host);
		}

		mmc_detect_change(host->mmc, 0);
	}

	return IRQ_HANDLED;
}

static const struct mmc_host_ops rda_mmc_ops = {
	.request	= rda_mmc_request,
	.get_ro 	= rda_mmc_get_ro,
	.get_cd		= rda_mmc_get_cd,
	.set_ios	= rda_mmc_set_ios,
	.enable_sdio_irq = rda_mmc_enable_sdio_irq,
};

static void rda_sdio_timeout_work(struct work_struct *work)
{
	struct rda_mmc_host *host = container_of(work, struct rda_mmc_host, timeout_work.work);

	if (host) {
		mmc_signal_sdio_irq(host->mmc);
	}
}

static irqreturn_t rda_sdio_eirq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	struct rda_mmc_host *host = dev_id;
	struct mmc_host * mmc = NULL;
	u32 int_status;

	if(host){
		mmc = host->mmc;
	}else
		return IRQ_HANDLED;

	int_status = hal_irq_get_status(host);
	if (!(int_status & SDMMC_SDIO_INT)) {
		rda_dbg_mmc("host id : %d,int_status:%d\n", host->id, int_status);
		return IRQ_NONE;
	}

	if(mmc->sdio_irqs) {
		if (host->suspend) {
			/*
			 * The handler is invoked as soon as AP is waked up via WiFi,
			 * but resume function is not called by AP at this time.
			 * So,we disable interrupt at first, because the interrupt is triggered
			 * via low-level, then schedule a delayed work to wait for resume callback.
			 * */
			rda_mmc_enable_sdio_irq(mmc, 0);
			schedule_delayed_work(&host->timeout_work, msecs_to_jiffies(10));
		} else {
			mmc_signal_sdio_irq(mmc);
		}
	} else {
		spin_lock_irqsave(&host->lock, flags);
		rda_mmc_mask_eirq(host);
		spin_unlock_irqrestore(&host->lock, flags);
	}

	return IRQ_HANDLED;
}

static ssize_t rda_mmc_unaligned_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct rda_mmc_host *host = (struct rda_mmc_host *)mmc_priv(mmc);

	return sprintf(buf, "%d\n", host->unaligned_count);
}

static ssize_t rda_mmc_reset_num_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct rda_mmc_host *host = (struct rda_mmc_host *)mmc_priv(mmc);

	return sprintf(buf, "%d\n", host->reset_count);
}

static struct device_attribute rda_mmc_attributes[] = {
	__ATTR(mmc_unalign, S_IRUGO, rda_mmc_unaligned_show, NULL),
	__ATTR(reset_num, S_IRUGO, rda_mmc_reset_num_show, NULL),
};

static int rda_mmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct rda_mmc_host *host = NULL;
	struct resource *res;
	int ret = 0, irq;
	int i = pdev->id;
	struct rda_mmc_device_data * hw= NULL;
	unsigned long flags;
	int index;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0) {
		return -ENXIO;
	}

	rda_dbg_mmc("rda_mmc_probe, %d, base = %08x, irq = %d\n",
		i, res->start, irq);

	hw = (struct rda_mmc_device_data*)pdev->dev.platform_data;
	mmc = mmc_alloc_host(sizeof(struct rda_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}

	mmc->ops = &rda_mmc_ops;

	/*
	 * We do not have SG-DMA, use 1.
	 */
	mmc->max_segs = 1;
	/*
	 * Our hardware DMA can handle a maximum of one page per SG entry.
	 */
	if (i == 0) {
		mmc->max_req_size = RDA_MMC0_REQ_NUM_PAGE * PAGE_SIZE;
		mmc->max_seg_size = RDA_MMC0_REQ_NUM_PAGE * PAGE_SIZE;
	} else {
		mmc->max_req_size = 16 * PAGE_SIZE;
		mmc->max_seg_size = 16 * PAGE_SIZE;
	}

	/*
	 * Block length register is only 10 bits before PXA27x.
	 */
	mmc->max_blk_size = 4096;

	/*
	 * Block count register is 16 bits.
	 */
	mmc->max_blk_count = 65535;

	host = mmc_priv(mmc);
	host->id = i;
	host->mmc = mmc;
	host->irq = irq;
	host->sdio_irq_enable = 0;
	host->mmc_pm = MMC_POWER_OFF;
	host->plat_dev = pdev;

	init_completion(&host->req_done);

	host->base = ioremap(res->start, resource_size(res));
	if (!host->base) {
		dev_err(&pdev->dev, "ioremap fail\n");
		ret = -ENOMEM;
		goto err_free_host;
	}

	if (host->id == 0) {
		host->host_reg = regulator_get(NULL, LDO_SDMMC);
		if (IS_ERR(host->host_reg)) {
			dev_err(&pdev->dev, "could not find regulator devices\n");
			ret = PTR_ERR(host->host_reg);
			goto err_free_reg;
		}
	}

	if (rda_soc_is_older_metal10()) {
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
		/*
	 	* A 4KBytes memory is enough to save data
	 	* whose address is not aligned with page.
	 	*/
		host->tmp_buf = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->phys_tmp, GFP_KERNEL);
		if (!host->tmp_buf) {
			dev_err(&pdev->dev, "could not allocate a reserved memory of dma\n");
			ret = -ENOMEM;
			goto err_dma_alloc;
		}
	}

	/*
	 * Calculate minimum clock rate, rounding up.
	 */
	mmc->f_min = hw->f_min;
	mmc->f_max = hw->f_max;
	mmc->ocr_avail = hw->ocr_avail;
	mmc->caps = hw->caps;
	mmc->pm_caps = hw->pm_caps;

	host->eirq_enable = hw->eirq_enable;
	host->det_pin = hw->det_pin;
	host->sys_suspend = hw->sys_suspend;

	hal_mmc_init(host);
	host->id = i;
	host->clk_inv = hw->clk_inv;

	hal_set_bus_width(host, 0);
	host->bus_width = 0;

	spin_lock_init(&host->lock);

	host->master_clk = clk_get(NULL, RDA_CLK_APB2);
	if (!host->master_clk) {
		dev_err(&pdev->dev, "no handler of clock\n");
		ret = -EINVAL;
		goto err_free_reg;
	}
	host->clock = 0;
	host->mclk_adj = hw->mclk_adj;

	ret = clk_prepare_enable(host->master_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "do not enable the specified clock\n");
		goto err_clk;
	}

#ifdef RDA_MMC_USE_INT
	tasklet_init(&host->finish_tasklet, tasklet_worker, (unsigned long)host);

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, rda_mmc_irq, IRQF_DISABLED,
						mmc_hostname(mmc), host);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		goto out;
	}
#endif

	if (host->eirq_enable){
		ret = gpio_request(hw->eirq_gpio, hw->dev_label);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ\n");
			goto out;
		}
		gpio_direction_input(hw->eirq_gpio);
		host->eirq = gpio_to_irq(hw->eirq_gpio);

		ret =  request_irq(host->eirq, rda_sdio_eirq_handler,
				hw->eirq_sense,hw->dev_label, host);
		if (ret)
			dev_err(&pdev->dev, "request_irq sd eirq failed \n");
		else{
			spin_lock_irqsave(&host->lock, flags);
			rda_mmc_mask_eirq(host);
			spin_unlock_irqrestore(&host->lock, flags);
		}
	}

	if (gpio_is_valid(host->det_pin)) {
		ret = gpio_request(host->det_pin, "mmc_detect");
		if (ret < 0) {
			dev_warn(&pdev->dev, "couldn't claim card detect pin\n");
		} else {
			gpio_direction_input(host->det_pin);

			ret = request_irq(gpio_to_irq(host->det_pin),
				rda_mmc_det_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
				mmc_hostname(mmc),
				host);
			if (ret) {
				dev_warn(&pdev->dev, "request MMC detect irq failed\n");
			}
		}
	}

	/* Just for sdio host. */
	if (host->id == 1) {
		INIT_DELAYED_WORK(&host->timeout_work, rda_sdio_timeout_work);
	}

	mmc_add_host(mmc);

	platform_set_drvdata(pdev, (void *)mmc);

	if (host->eirq_enable) {
		device_init_wakeup(&pdev->dev, 1);
	}

	/* If success, we put host to a global array. */
	if (i < RDA_MMC_HOST_COUNT) {
		mmc_host_p[i] = host;
	}

	host->unaligned_count = 0;
	for (index = 0; index < ARRAY_SIZE(rda_mmc_attributes); index++) {
		device_create_file(&pdev->dev, &rda_mmc_attributes[index]);
	}

	dev_info(&pdev->dev, "rda_sdmmc %d initialized.\n", i);

	return 0;

out:

#ifdef RDA_MMC_USE_INT
	free_irq(host->irq, host);
	tasklet_kill(&host->finish_tasklet);
#endif

	if (host->master_clk) {
		clk_disable_unprepare(host->master_clk);
	}

err_clk:
	if (host->master_clk) {
		clk_put(host->master_clk);
	}

err_dma_alloc:
	if (host->tmp_buf) {
		dma_free_coherent(&pdev->dev, PAGE_SIZE, host->tmp_buf, host->phys_tmp);
	}

err_free_reg:

	if (host->id == 0) {
		if (!IS_ERR(host->host_reg)) {
			regulator_disable(host->host_reg);
			regulator_put(host->host_reg);
		}
	}

	if (host->base) {
		iounmap(host->base);
	}

err_free_host:

	if (mmc) {
		mmc_free_host(mmc);
	}

	return ret;
}

static int rda_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct rda_mmc_host *host = mmc_priv(mmc);

		if (host->master_clk) {
			clk_put(host->master_clk);
		}

		if (host->id == 0) {
			if (!IS_ERR(host->host_reg)) {
				regulator_disable(host->host_reg);
				regulator_put(host->host_reg);
			}
		}

		host->mmc_pm = MMC_POWER_OFF;

		mmc_remove_host(mmc);

#ifdef RDA_MMC_USE_INT
		free_irq(host->irq, host);
		tasklet_kill(&host->finish_tasklet);
#endif

		if(host->eirq_enable){
			free_irq(host->eirq, host);
		}

		if (gpio_is_valid(host->det_pin)) {
			free_irq(gpio_to_irq(host->det_pin), host);
			gpio_free(host->det_pin);
		}

		if (host->tmp_buf) {
			dma_free_coherent(&pdev->dev, PAGE_SIZE, host->tmp_buf, host->phys_tmp);
		}

		mmc_free_host(mmc);
	}
	return 0;
}

void rda_mmc_set_sdio_irq(u32 host_id, u8 enable)
{
	unsigned long flags;
	struct rda_mmc_host* host = NULL;
	HWP_SDMMC_T *hwp_sdmmc = NULL;

	host = mmc_host_p[host_id];
	if(!host)
		return;

	hwp_sdmmc = (HWP_SDMMC_T*)host->base;
	spin_lock_irqsave(&host->lock, flags);

	if(enable){
		if(!host->eirq_enable)
			hwp_sdmmc->SDMMC_INT_MASK |= SDMMC_SDIO_INT_MK;
		else
			rda_mmc_unmask_eirq(host);
	}else{
		if(!host->eirq_enable)
			hwp_sdmmc->SDMMC_INT_MASK &= ~SDMMC_SDIO_INT_MK;
		else
			rda_mmc_mask_eirq(host);
	}
	host->sdio_irq_enable = enable;

	spin_unlock_irqrestore(&host->lock, flags);
}

static void rda_mmc_reset(u32 host_id)
{
	static HWP_SYS_CTRL_AP_T *hwp_apSysCtrl = (HWP_SYS_CTRL_AP_T *)RDA_SYSCTRL_BASE;

	if (host_id > 2) {
		return;
	}

	hwp_apSysCtrl->APB2_Rst_Set=1<<(APB2_RST_SDMMC1+host_id);
	mdelay(1);
	hwp_apSysCtrl->APB2_Rst_Clr=1<<(APB2_RST_SDMMC1+host_id);
	mdelay(1);
}

void rda_mmc_bus_scan(u32 host_id)
{
	struct rda_mmc_host* host = NULL;
	host = mmc_host_p[host_id];

	if (!host) {
		return;
	}

	/* reset mmc host*/
	rda_mmc_reset(host_id);
	/* init mmc*/
	hal_mmc_init(host);

#ifdef CONFIG_PM
	if (host->suspend) {
		mmc_resume_host(host->mmc);
		host->suspend = 0;
	} else {
		mmc_detect_change(host->mmc, msecs_to_jiffies(50));
	}
#else
	mmc_detect_change(host->mmc, msecs_to_jiffies(50));
#endif
}

#ifdef CONFIG_PM
static int rda_mmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct rda_mmc_host *host;
	int ret = 0;

	if (mmc) {
		host = (struct rda_mmc_host*)mmc_priv(mmc);
		if (host) {

			rda_dbg_mmc("host:(%d) rda_mmc_suspend \n",host->id);

			if (!host->sys_suspend) {
				return ret;
			}

			if (!host->suspend) {
				ret = mmc_suspend_host(mmc);
				clk_disable(host->master_clk);
			}

			host->suspend = 1;
			if (!ret) {
				if (host->id == 0 && host->mmc_pm != MMC_POWER_OFF) {
					if (!IS_ERR(host->host_reg)) {
						ret = regulator_disable(host->host_reg);
					}
				}
				host->mmc_pm = MMC_POWER_OFF;
			}
		}
	}

	return ret;
}

static int rda_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct rda_mmc_host *host;
	int ret = 0;

	if (mmc) {
		host = (struct rda_mmc_host*)mmc_priv(mmc);
		if (host) {
			rda_dbg_mmc("host:(%d) rda_mmc_resume \n",host->id);

			if (!host->sys_suspend) {
				return ret;
			}

			if (host->id == 0 && host->mmc_pm == MMC_POWER_OFF) {
				if (!IS_ERR(host->host_reg)) {
					ret = regulator_enable(host->host_reg);
				}
				host->mmc_pm = MMC_POWER_UP;
			}

			if (host->suspend) {
				clk_enable(host->master_clk);
				ret = mmc_resume_host(mmc);
			} else {
				mmc_detect_change(host->mmc, msecs_to_jiffies(50));
			}
			host->suspend = 0;
		}
	}

	return ret;
}
#else
#define rda_mmc_suspend NULL
#define rda_mmc_resume	NULL
#endif

static struct platform_driver rda_mmc_driver = {
	.probe		= rda_mmc_probe,
	.remove 	= rda_mmc_remove,
	.suspend	= rda_mmc_suspend,
	.resume 	= rda_mmc_resume,
	.driver 	= {
		.name	= RDA_MMC_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init rda_mmc_init(void)
{
	int ret;
	ret = platform_driver_register(&rda_mmc_driver);
	return ret;
}

static void __exit rda_mmc_exit(void)
{
	platform_driver_unregister(&rda_mmc_driver);
}

module_init(rda_mmc_init);
module_exit(rda_mmc_exit);
EXPORT_SYMBOL(rda_mmc_set_sdio_irq);
EXPORT_SYMBOL(rda_mmc_bus_scan);

MODULE_DESCRIPTION("RDA Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rda-mmc");
