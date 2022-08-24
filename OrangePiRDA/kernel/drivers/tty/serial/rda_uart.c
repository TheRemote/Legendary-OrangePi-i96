/*******************************************************************************
 * FileName      :    rda_uart.c
 * Author          :   xiankuiwei
 * Description   :   The file is to achieve the rda's uart driver,including console driver
 * Version         :    rdadroid-4.0.4_r1.2  Date: 2012-10-26
 * Copyright      :   RDA Microelectronics Company Limited
 * Function List :
 *                 1.
 * History:
 *     <author>   <time>    <version >   <desc>
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/err.h>
#include <linux/tty_flip.h>
#include <linux/sysrq.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/wakelock.h>
#include <plat/devices.h>
#include <mach/irqs.h>
#include <mach/ifc.h>
#include <plat/reg_uart.h>
#include <plat/reg_sysctrl.h>
#include "plat/ap_clk.h"
#include "linux/delay.h"

#define RDA_UART_DEV_NAME "ttyS"
#define SERIAL_RDA_MAJOR 204
#define SERIAL_RDA_MINOR 64

/*
 * Local per-uart structure.
 */

#define RDA_UART_DMA_SIZE    512
#define RDA_ISR_PASS_LIMIT  256
#define RDA_SERIAL_RINGSIZE  1024
#define RDA_UART_TX_FIFO_SIZE 16
#define RDA_UART_RX_FIFO_SIZE 64
#define RDA_UART_DEFAULT_BAUD 921600
#define RDA_UART_DEFAULT_DATABITS 8
#define RDA_UART_DEFAULT_STOPBITS 1

#define RDA_UART_IOCTL_MAGIC 'u'
#define RDA_UART_ENABLE_RX_BREAK_IOCTL		_IO(RDA_UART_IOCTL_MAGIC ,0x01)

typedef struct rda_dma_buffer
{
	u8              *buf;
	dma_addr_t       dma_addr;
	u32              dma_size;
	u32              ofs;
} RDA_DMA_BUFFER;

struct rda_uart_char {
	u32  status;
	u16  ch;
};

struct rda_uart
{
	struct uart_port        port;
	struct timer_list       tmr;
	spinlock_t              lock;
	u16                     use_dma_rx;
	u16                     use_dma_tx;
	HAL_IFC_REQUEST_ID_T    rxdmarequestid;
	HAL_IFC_REQUEST_ID_T    txdmarequestid;
	RDA_DMA_BUFFER      dma_rx;
	RDA_DMA_BUFFER      dma_tx;
	struct tasklet_struct   tasklet;
	u32                     dmarxchannel;
	u32                     dmatxchannel;
	u32                     irqstatus;
	volatile u32            ori_status;
	//REG32                 * cfg_clk;
	struct circ_buf         rx_ring;
	u8                      dev_id;
	u8                      irq_free_flag;
	u8                      rxbreak_int_enable_flag;
	u8                      rxbreak_int_enable;
	u8                      rxbreak_int_detected;
	struct wake_lock        rxbreak_waklock;
	void			(*rxbreak_int_handle)(void);
	u32 			wakeup;
};

/* For the console init is before tty init, so we have to define and register the console device */
extern struct platform_device * rda_get_assigned_uartdevice(int uartid);
static void rda_uart_tx_dma(struct uart_port *port);

static inline bool rda_use_dma_rx(struct uart_port *port)
{
	struct rda_uart *rda_port =
			container_of(port, struct rda_uart, port);

	return rda_port->use_dma_rx;
}

static inline bool rda_use_dma_tx(struct uart_port *port)
{
	struct rda_uart *rda_port =
			container_of(port, struct rda_uart, port);

	return rda_port->use_dma_tx;
}

static void rda_enable_rx_break_int(struct uart_port *port, bool enable)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	if (rdaport->rxbreak_int_enable) {
		if (enable) {
        		/* clear error */
			spin_lock_irqsave(&rdaport->lock, flags);
        		hwp_uart->status |= (UART_RX_BREAK_INT | UART_RX_PARITY_ERR
                                 | UART_RX_FRAMING_ERR | UART_RX_OVERFLOW_ERR);
			hwp_uart->irq_mask |= UART_RX_LINE_ERR;
			spin_unlock_irqrestore(&rdaport->lock, flags);
			rdaport->rxbreak_int_enable_flag = true;
		}
		else {
			spin_lock_irqsave(&rdaport->lock, flags);
			hwp_uart->irq_mask &= ~(UART_RX_LINE_ERR);
			spin_unlock_irqrestore(&rdaport->lock, flags);
			rdaport->rxbreak_int_enable_flag = false;
		}
	}else
		return;
}

static unsigned int rda_uart_tx_empty(struct uart_port *port)
{
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	return ((GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE)) == RDA_UART_TX_FIFO_SIZE) ? TIOCSER_TEMT : 0;
}

static unsigned int rda_uart_get_mctrl(struct uart_port *port)
{
	unsigned int sigs = 0;
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	sigs = (hwp_uart->status & UART_CTS) ? 0 : TIOCM_CTS;

	return sigs;
}

static void rda_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct rda_uart * rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	spin_lock_irqsave(&rdaport->lock, flags);
	/* RTS set */
	if (sigs & TIOCM_RTS) {
		hwp_uart->CMD_Set |= UART_RTS;
	} else {
		/* Clear RTS to stop to receive. */
		hwp_uart->CMD_Clr |= UART_RTS;
	}

	if (sigs & TIOCM_LOOP) {
		hwp_uart->ctrl |= UART_LOOP_BACK_MODE;
	} else {
		hwp_uart->ctrl &= ~UART_LOOP_BACK_MODE;
	}
	spin_unlock_irqrestore(&rdaport->lock, flags);
}

static void rda_uart_start_tx(struct uart_port *port)
{
	struct rda_uart * rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	if (rda_use_dma_tx(port)) {
		/* Notice: Now the dma transmit may not over,so you should judge
		   if the transmit is over. If not over,please return, let
		   the interrupt handle the transmit */
		if (HAL_UNKNOWN_CHANNEL != rdaport->dmatxchannel) {
			if (ifc_transfer_get_tc(rdaport->txdmarequestid,
						rdaport->dmatxchannel)){
				return;
			}
		}

		/* Be careful, Don't enable dma uart interrupt here, because
		   it will runrda_uart_tx_dma function which will enable the
		   interrupt. */
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_cause |= UART_TX_DMA_DONE;
		hwp_uart->irq_mask &= ~UART_TX_DMA_DONE;
		spin_unlock_irqrestore(&rdaport->lock, flags);
		rda_uart_tx_dma(port);
	} else{
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_mask |= UART_TX_DATA_NEEDED;
		spin_unlock_irqrestore(&rdaport->lock, flags);
	}
}

static void rda_uart_stop_tx(struct uart_port *port)
{
	struct rda_uart * rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	if (rda_use_dma_tx(port)) {

		if (HAL_UNKNOWN_CHANNEL != rdaport->dmatxchannel) {
			ifc_transfer_stop(rdaport->txdmarequestid, rdaport->dmatxchannel);
			rdaport->dmatxchannel = HAL_UNKNOWN_CHANNEL;

			spin_lock_irqsave(&rdaport->lock, flags);
			hwp_uart->irq_mask &= ~UART_TX_DMA_DONE;
			hwp_uart->irq_cause |= UART_TX_DMA_DONE;
			spin_unlock_irqrestore(&rdaport->lock, flags);
		}
	} else {
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_mask &= ~UART_TX_DATA_NEEDED;
		spin_unlock_irqrestore(&rdaport->lock, flags);
	}

	spin_lock_irqsave(&rdaport->lock, flags);
	hwp_uart->CMD_Set |= UART_TX_FIFO_RESET;
	spin_unlock_irqrestore(&rdaport->lock, flags);
}

static void rda_uart_stop_rx(struct uart_port *port)
{
	struct rda_uart * rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	volatile unsigned int read_val;
	unsigned long flags;

	if (rda_use_dma_rx(port)) {
		/* disable dma uart interrupt */
		if (HAL_UNKNOWN_CHANNEL != rdaport->dmarxchannel) {
			ifc_transfer_stop(rdaport->rxdmarequestid, rdaport->dmarxchannel);
			rdaport->dmarxchannel = HAL_UNKNOWN_CHANNEL;

			/* disable dma uart interrupt */
			hwp_uart->irq_cause |= (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
			spin_lock_irqsave(&rdaport->lock, flags);
			hwp_uart->irq_mask &= ~(UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
			spin_unlock_irqrestore(&rdaport->lock, flags);
		}
	} else {
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_mask &= ~(UART_RX_DATA_AVAILABLE | UART_RX_TIMEOUT);
		spin_unlock_irqrestore(&rdaport->lock, flags);
	}

	/*
	 * Note:
	 * In some case, if there are any data in rx fifo,
	 * we reset only rx_fifo instead of reading rx_fifo before reset it.
	 * It will trigger rx_timeout irq that is not cleared by our driver.
	 */
	spin_lock_irqsave(&rdaport->lock, flags);
	read_val = hwp_uart->rxtx_buffer;
	hwp_uart->CMD_Set |= UART_RX_FIFO_RESET;
	spin_unlock_irqrestore(&rdaport->lock, flags);
}

static void rda_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct rda_uart * rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	spin_lock_irqsave(&rdaport->lock, flags);
	if (break_state != 0){
		hwp_uart->CMD_Set |= UART_TX_BREAK_CONTROL;
	} else{
		hwp_uart->CMD_Clr |= UART_TX_BREAK_CONTROL;
	}
	spin_unlock_irqrestore(&rdaport->lock, flags);
}

/**********************************************************************
 * The function is to enable modem we don't support now,so this function
   is empty.
**********************************************************************/
static void rda_uart_enable_ms(struct uart_port *port)
{
}

static void rda_uart_set_termios(struct uart_port *port, struct ktermios *termios,
					struct ktermios *old)
{
	unsigned long  flags;
	unsigned int   rdactrl      = 0;
	unsigned int   rdacmdset   = 0;
	//unsigned int   rdacmdclr    = 0;
	unsigned int   baud        = 0;
	unsigned int   divider     = 0;
	unsigned int   irq_mask   = 0;
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);

	/* get baud rate and quot from termios */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 4);
	divider = ((port->uartclk + 2 * baud) / ( 4 * baud)) - 2;

	/* byte size */
	switch (termios->c_cflag & CSIZE){
		case CS5:
		case CS6:
			dev_WARN(port->dev," Sorry,but the hardware don't support this ,"
                                 "and will set the data bit to 7 in a frame");
		case CS7:
			rdactrl &= ~UART_DATA_BITS_8_BITS;
			break;
		default:
			rdactrl |= UART_DATA_BITS_8_BITS;
			break;
	}

	/* stop bits */
	if (termios->c_cflag & CSTOPB){
		rdactrl |= UART_TX_STOP_BITS_2_BITS;
	} else{
		rdactrl &= ~UART_TX_STOP_BITS_2_BITS;
	}

	/* parity check */
	if (termios->c_cflag & PARENB){
		rdactrl    |= UART_PARITY_ENABLE;

		/* Mark or Space parity */
		if (termios->c_cflag & CMSPAR){
			if (termios->c_cflag & PARODD){
				rdactrl |= UART_PARITY_SELECT_MARK;
			} else {
				rdactrl |= UART_PARITY_SELECT_SPACE;
			}
		}
		else if (termios->c_cflag & PARODD){
			rdactrl |= UART_PARITY_SELECT_ODD;
		} else {
			rdactrl |= UART_PARITY_SELECT_EVEN;
		}
	} else{
		rdactrl &= ~UART_PARITY_ENABLE;
	}

	/* hardware handshake (RTS/CTS) */
	if (termios->c_cflag & CRTSCTS){
		rdactrl   |= UART_AUTO_FLOW_CONTROL;
		rdacmdset |= UART_RTS;
	} else{
		rdactrl   &= ~UART_AUTO_FLOW_CONTROL;
		rdacmdset |= UART_RTS;
	}

	if(rdaport->rxbreak_int_enable){
		rdactrl |= UART_RX_BREAK_LENGTH(13);
	}

	spin_lock_irqsave(&rdaport->lock, flags);

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

#ifndef CONFIG_RDA_CLK_SIMU
	//set uart baud rate
	apsys_set_uart_clk_rate( rdaport->dev_id - 1, baud);
#endif

	if (rda_use_dma_rx(port) && rda_use_dma_tx(port)){
		rdactrl |= UART_DMA_MODE;
	} else{
		rdactrl &= ~UART_DMA_MODE;
	}
	/* Disable rx and tx */
	hwp_uart->ctrl = 0;

	rdactrl |= UART_ENABLE;

	/*  save and disable all interrupts */
	irq_mask = hwp_uart->irq_mask;
	hwp_uart->irq_mask = 0;
	hwp_uart->triggers  = (UART_AFC_LEVEL(20) | UART_RX_TRIGGER(16));
	hwp_uart->ctrl      = rdactrl;
	hwp_uart->CMD_Set   = rdacmdset;

	/*  restore  former interrupts */
	hwp_uart->irq_mask = irq_mask;

	spin_unlock_irqrestore(&rdaport->lock, flags);
	dev_dbg(port->dev, "irq_mask = %x; ctrl = %x; cmd_set =%x",
             irq_mask, rdactrl, rdacmdset);
}

static void rda_put_char_to_buffer(struct uart_port *port,
                                        unsigned int status,
					unsigned char ch)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	struct circ_buf *ring = & rdaport->rx_ring;
	struct rda_uart_char *c;

	if (!CIRC_SPACE(ring->head, ring->tail, RDA_SERIAL_RINGSIZE)){
		/* Buffer overflow, ignore char */
		return;
	}

	c = &((struct rda_uart_char *)ring->buf)[ring->head];
	c->status  = status;
	c->ch       = ch;

	/* Make sure the character is stored before we update head. */
	smp_wmb();

	ring->head = (ring->head + 1) & (RDA_SERIAL_RINGSIZE - 1);
}

static void rda_uart_rx_chars_tobuf(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	volatile u32 status;
	u8 ch = 0;
	volatile unsigned int read_val;

	if (!GET_BITFIELD(hwp_uart->status, UART_RX_FIFO_LEVEL)) {
		if(rdaport->rxbreak_int_detected){
			rdaport->rxbreak_int_detected = 0;
		}

		if (rdaport->irqstatus & UART_RX_TIMEOUT) {
			read_val = hwp_uart->rxtx_buffer;
		}

		return;
	}

	while (GET_BITFIELD(hwp_uart->status, UART_RX_FIFO_LEVEL)){
		status = hwp_uart->status;

		//if rx break irq received , should ignore the first byte.
		//be careful, if the rx break received but the irq status not set the bit UART_RX_LINE_ERR_U
		//the null data is not intsert to the rx fifo
		if(rdaport->rxbreak_int_detected == 1){
			rdaport->rxbreak_int_detected = 0;

			if((status & UART_RX_LINE_ERR_U) && (rdaport->irqstatus & UART_RX_LINE_ERR_U)){
				ch = (u8)(hwp_uart->rxtx_buffer & 0xff);
				continue;
			}
		}

		if(!rdaport->rxbreak_int_enable_flag)
			rda_enable_rx_break_int(port, true);
		ch = (u8)(hwp_uart->rxtx_buffer & 0xff);
		rda_put_char_to_buffer(port, status, ch);
	}

	tasklet_schedule(&rdaport->tasklet);
}

static void rda_uart_rx_chars(struct rda_uart *pp)
{
	struct rda_uart * rdaport = pp;
	struct uart_port *port = &pp->port;
	struct circ_buf *ring  = &rdaport->rx_ring;
	u32 status, flg, ch;
	unsigned long flag;
	struct rda_uart_char c;

	if (ring->head == ring->tail) {
		return;
	}

	spin_lock_irqsave(&port->lock, flag);
	while (ring->head != ring->tail){
		/* Make sure c is loaded after head. */
		smp_rmb();

		c = ((struct rda_uart_char *)ring->buf)[ring->tail];

		ring->tail = (ring->tail + 1) & (RDA_SERIAL_RINGSIZE - 1);

		status = c.status;
		flg = TTY_NORMAL;
		port->icount.rx++;
		ch = (u32)c.ch;

		/* handle errors and process flag value */
		if ( status & (UART_RX_BREAK_INT | UART_RX_PARITY_ERR
				| UART_RX_FRAMING_ERR | UART_RX_OVERFLOW_ERR)){
			if (status & UART_RX_BREAK_INT){
				/* ignore side-effect */
				status &= ~(UART_RX_PARITY_ERR | UART_RX_FRAMING_ERR);
				port->icount.brk++;
				flg = TTY_BREAK;

				if (uart_handle_break(port)){
					continue;
				}
			}

			if (status & UART_RX_PARITY_ERR){
				port->icount.parity++;
				flg = TTY_PARITY;
			}

			if (status & UART_RX_FRAMING_ERR){
				port->icount.frame++;
				flg = TTY_FRAME;
			}

			if (status & UART_RX_OVERFLOW_ERR){
				port->icount.overrun++;
				flg = TTY_OVERRUN;
			}
		}

		if (uart_handle_sysrq_char(port, ch)){
			continue;
		}

		uart_insert_char(port, 0, 0, ch, flg);
	}

	spin_unlock_irqrestore(&port->lock, flag);
	tty_flip_buffer_push(&port->state->port);
}

static void rda_uart_rx_dma(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	struct rda_dma_buffer *pdmabuf;
	unsigned int count;
	u8 * rxStr = NULL;
	unsigned long flags, flags1;

	pdmabuf = &rdaport->dma_rx;
	count =  pdmabuf->dma_size;

	if (rdaport->irqstatus & UART_RX_DMA_TIMEOUT){
		ifc_transfer_flush(rdaport->rxdmarequestid, rdaport->dmarxchannel);
	}

	count = count - ifc_transfer_get_tc(rdaport->rxdmarequestid,
						rdaport->dmarxchannel);

	if (0 == count){
		/* enable receive again by clear the interrupt register  */
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_cause |= (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
		hwp_uart->irq_mask |=  (UART_RX_DMA_TIMEOUT | UART_RX_DMA_DONE);
		spin_unlock_irqrestore(&rdaport->lock, flags);
		return;
	}


	/* disable the dma rx */
	if (HAL_UNKNOWN_CHANNEL != rdaport->dmarxchannel){
		ifc_transfer_stop(rdaport->rxdmarequestid, rdaport->dmarxchannel);
		rdaport->dmarxchannel = HAL_UNKNOWN_CHANNEL;
	}


	dma_sync_single_for_cpu(port->dev, pdmabuf->dma_addr,
				pdmabuf->dma_size + pdmabuf->ofs, DMA_FROM_DEVICE);
	rxStr = pdmabuf->buf;

	spin_lock_irqsave(&port->lock, flags1);
	//if rx break irq received , should ignore the first byte.
	if(rdaport->rxbreak_int_detected){
		rdaport->rxbreak_int_detected = 0;
		rda_enable_rx_break_int(port, true);
		rxStr += 1;
		count -= 1;
	}
	tty_insert_flip_string(&port->state->port, rxStr, count);
	port->icount.rx += count;

	dma_sync_single_for_device(port->dev, pdmabuf->dma_addr,
				pdmabuf->dma_size, DMA_FROM_DEVICE);

	rdaport->dmarxchannel = ifc_transfer_start(rdaport->rxdmarequestid,
						(u8 *)pdmabuf->dma_addr,pdmabuf->dma_size,
						 HAL_IFC_SIZE_8_MODE_MANUAL);

	spin_lock_irqsave(&rdaport->lock, flags);
	/* enable receive again by clear the interrupt register  */
	hwp_uart->irq_cause |= (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
	hwp_uart->irq_mask |=  (UART_RX_DMA_TIMEOUT | UART_RX_DMA_DONE);
	rdaport->irqstatus = 0;
	spin_unlock_irqrestore(&rdaport->lock, flags);

	spin_unlock_irqrestore(&port->lock, flags1);
	tty_flip_buffer_push(&port->state->port);
}

static void rda_handle_uart_transmit(struct uart_port *port,
                                              unsigned int status)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	if (status & (UART_TX_DATA_NEEDED | UART_TX_DMA_DONE)){
		rdaport->ori_status = status; /* new add */
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_cause |=  UART_TX_DMA_DONE;
		hwp_uart->irq_mask &= ~(UART_TX_DATA_NEEDED | UART_TX_DMA_DONE);
		spin_unlock_irqrestore(&rdaport->lock, flags);

		tasklet_schedule(&rdaport->tasklet);
	}
}

static void rda_uart_tx_dma(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	struct circ_buf *xmit = &port->state->xmit;
	struct rda_dma_buffer *pdmabuf = &rdaport->dma_tx;
	int count;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	xmit->tail += pdmabuf->ofs;
	xmit->tail &= UART_XMIT_SIZE - 1;
	port->icount.tx += pdmabuf->ofs;
	pdmabuf->ofs = 0;

	/* disable the dma transmit */
	if (HAL_UNKNOWN_CHANNEL != rdaport->dmatxchannel) {
		ifc_transfer_stop(rdaport->txdmarequestid, rdaport->dmatxchannel);
		rdaport->dmatxchannel = HAL_UNKNOWN_CHANNEL;
	}

	/* more to transmit - setup next transfer */
	if (!uart_circ_empty(xmit) && !uart_tx_stopped(port)) {
		dma_sync_single_for_device(port->dev, pdmabuf->dma_addr,
					pdmabuf->dma_size, DMA_TO_DEVICE);

		count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		pdmabuf->ofs = count;

		/* put the buffer address and size to dma registers */
		rdaport->dmatxchannel = ifc_transfer_start(rdaport->txdmarequestid,
							(u8 *)(pdmabuf->dma_addr + xmit->tail),
							count, HAL_IFC_SIZE_8_MODE_MANUAL);
	}

	spin_unlock_irqrestore(&port->lock, flags);

	spin_lock_irqsave(&rdaport->lock, flags);
	/* Enable interrupts */
	hwp_uart->irq_cause |= UART_TX_DMA_DONE;
	hwp_uart->irq_mask  |= UART_TX_DMA_DONE;
	spin_unlock_irqrestore(&rdaport->lock, flags);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		uart_write_wakeup(port);
	}
}

static void rda_uart_tx_chars(struct rda_uart *pp)
{
	struct rda_uart * rdaport = pp;
	struct uart_port *port = &pp->port;
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int space_size;
	unsigned long flags, flags1;

	if (!(rdaport->ori_status & (UART_TX_DATA_NEEDED | UART_TX_DMA_DONE))) {
		return;
	}

	if (port->x_char && (GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE))){
		spin_lock_irqsave(&rdaport->lock, flags);
		/* Send special char - probably flow control */
		hwp_uart->rxtx_buffer = (uint32_t)port->x_char;
		spin_unlock_irqrestore(&rdaport->lock, flags);

		port->x_char = 0;
		port->icount.tx++;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)){
		return;
	}

	spin_lock_irqsave(&port->lock, flags1);
	spin_lock_irqsave(&rdaport->lock, flags);
	space_size = GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE);
	while (space_size) {
		hwp_uart->rxtx_buffer = (uint32_t)xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;

		if(uart_circ_empty(xmit))
		{
			break;
		}

		space_size--;
	}
	spin_unlock_irqrestore(&rdaport->lock, flags);
	spin_unlock_irqrestore(&port->lock, flags1);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}

	if (!uart_circ_empty(xmit)) {
		spin_lock_irqsave(&rdaport->lock, flags);
		hwp_uart->irq_mask  |= UART_TX_DATA_NEEDED;
		spin_unlock_irqrestore(&rdaport->lock, flags);
	}
}

static void rda_tasklet_func(unsigned long data)
{

	struct uart_port *port = (struct uart_port *)data;
	struct rda_uart *rdaport = container_of(port, struct rda_uart, port);


	if (rda_use_dma_tx(port)){
		rda_uart_tx_dma(port);
	} else{
		rda_uart_tx_chars(rdaport);
	}

	if (rda_use_dma_rx(port)){
		rda_uart_rx_dma(port);
	} else {
		rda_uart_rx_chars(rdaport);
	}
}

static void rda_uart_config_port(struct uart_port *port, int flags)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long irq_flags;

	port->type = PORT_RDA;

	spin_lock_irqsave(&rdaport->lock, irq_flags);
	/* Clear mask, so no surprise interrupts. */
	hwp_uart->irq_mask = 0;

	/* Clear status register */
	hwp_uart->status   = 0;
	spin_unlock_irqrestore(&rdaport->lock, irq_flags);
}

static void rda_handle_rxerr(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned int status = hwp_uart->status;
	unsigned long flags;

	if (status & UART_RX_PARITY_ERR){
		port->icount.parity++;
	}

	if (status & UART_RX_FRAMING_ERR){
		port->icount.frame++;
	}
	if (status & UART_RX_OVERFLOW_ERR){
		port->icount.overrun++;
	}

	if (status & UART_RX_BREAK_INT){
		port->icount.brk++;

		//first time received rx break
		if(!rdaport->rxbreak_int_detected)
			rdaport->rxbreak_int_detected = 1;
		//close rx break irq
                rda_enable_rx_break_int(port, false);

		if(GET_BITFIELD(hwp_uart->status, UART_RX_FIFO_LEVEL)){
			//received rx break & get break null data
			rdaport->rxbreak_int_detected = 2;
		}
		if(rdaport->rxbreak_int_enable){
			if(!rdaport->rxbreak_int_handle)
				wake_lock_timeout(&rdaport->rxbreak_waklock, HZ*6);
		}
	}

	spin_lock_irqsave(&rdaport->lock, flags);
	/* clear error */
	hwp_uart->status |= (UART_RX_BREAK_INT | UART_RX_PARITY_ERR
				 | UART_RX_FRAMING_ERR | UART_RX_OVERFLOW_ERR);
	spin_unlock_irqrestore(&rdaport->lock, flags);
}

static void rda_handle_receive(struct uart_port *port, unsigned int status)
{
	struct rda_uart *rdaport = container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	//rdaport->hwp_uart->CMD_Clr   = UART_RTS;
	if (rda_use_dma_rx(port)){

		if (status & (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT)){
			/* disable the interrupt when handle , then put it into the taslet  queue to process*/
			rdaport->irqstatus = status;
			spin_lock_irqsave(&rdaport->lock, flags);
			hwp_uart->irq_mask &= ~(UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
			spin_unlock_irqrestore(&rdaport->lock, flags);

			tasklet_schedule(&rdaport->tasklet);
		}
	} else{
		rdaport->irqstatus = status;
		/* Interrupt receive */
		if (status & (UART_RX_DATA_AVAILABLE | UART_RX_TIMEOUT)){
			/* disable the interrupt when handle , then put it into the taslet  queue to process*/
			spin_lock_irqsave(&rdaport->lock, flags);
			rda_uart_rx_chars_tobuf(port);
			spin_unlock_irqrestore(&rdaport->lock, flags);
		}
	}
}

static irqreturn_t rda_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned int irqstatus = 0;

	irqstatus = hwp_uart->irq_cause;

	//clear all irq casue
	hwp_uart->irq_cause = irqstatus;

	/*
	if received rx break interrupt should close all rx interrupt
	*/
	if(irqstatus & UART_RX_LINE_ERR){
		rda_handle_rxerr(port);
	}

	rda_handle_receive(port, irqstatus);
	rda_handle_uart_transmit(port, irqstatus);

	// Poke status register to reset error conditions
	hwp_uart->status = 0;

	return IRQ_HANDLED;
}

static int rda_uart_startup(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	struct tty_struct   *tty = port->state->port.tty;
	int ret;
	unsigned long flags;
	unsigned long irq_flag = 0;

	spin_lock_irqsave(&rdaport->lock, flags);
	/* disable all interrupts of uarts */
	hwp_uart->irq_mask = 0;
	spin_unlock_irqrestore(&rdaport->lock, flags);

	/*
	 * Ensure that no interrupts are enabled otherwise when
	 * request_irq() is called we could get stuck trying to
	 * handle an unexpected interrupt
	 */
	if (rdaport->irq_free_flag) {
		irq_flag = (rdaport->wakeup ? IRQF_NO_SUSPEND : 0);
		ret = request_irq(port->irq, rda_interrupt,
			irq_flag, tty ? tty->name : RDA_UART_DRV_NAME, port);
		if (ret){
			dev_WARN(port->dev,"unable to attach RDA UART %d "
                  	"interrupt vector=%d\r\n", port->line, port->irq);
			return ret;
		} else {
			rdaport->irq_free_flag = 0;
		}
	}
	/* alloc buffer for uart dma, and initialize the dma register */
	if (rda_use_dma_rx(port)) {
		RDA_DMA_BUFFER * pstDmaBuf = &rdaport->dma_rx;

		pstDmaBuf->buf = kmalloc(RDA_UART_DMA_SIZE, GFP_KERNEL);
		if (NULL == pstDmaBuf->buf) {
			dev_WARN(port->dev,"unable to attach RDA UART %d "
                                 "interrupt vector=%d\r\n", port->line, port->irq);
			free_irq(port->irq, port);
			rdaport->irq_free_flag = 1;
			return -1;
		}

		pstDmaBuf->dma_addr = dma_map_single(port->dev,
						pstDmaBuf->buf,
						RDA_UART_DMA_SIZE,
						DMA_FROM_DEVICE);
		pstDmaBuf->dma_size = RDA_UART_DMA_SIZE;
		pstDmaBuf->ofs      = 0;

		rdaport->dmarxchannel = ifc_transfer_start(rdaport->rxdmarequestid,
							(u8 *)pstDmaBuf->dma_addr,
							pstDmaBuf->dma_size,
							HAL_IFC_SIZE_8_MODE_MANUAL);
		if (HAL_UNKNOWN_CHANNEL == rdaport->dmarxchannel){
			dev_WARN(port->dev,"when rda_uart_startup, cannot get dma rx channel");
			dma_unmap_single(port->dev,pstDmaBuf->dma_addr,RDA_UART_DMA_SIZE,DMA_FROM_DEVICE);
			kfree(pstDmaBuf->buf);
			free_irq(port->irq, port);
			rdaport->irq_free_flag = 1;
			return -1;
		}
	}

	if (rda_use_dma_tx(port)){
		RDA_DMA_BUFFER *  pstDmaBuf = &rdaport->dma_tx;
		struct circ_buf   *  xmit       = &port->state->xmit;

		pstDmaBuf->buf = xmit->buf;
		pstDmaBuf->dma_addr = dma_map_single(port->dev,
						pstDmaBuf->buf,
						UART_XMIT_SIZE,
						DMA_TO_DEVICE);
		pstDmaBuf->dma_size = UART_XMIT_SIZE;
		rdaport->dmatxchannel = HAL_UNKNOWN_CHANNEL;
		pstDmaBuf->ofs = 0;
	}

	spin_lock_irqsave(&rdaport->lock, flags);
	/* enable  uart enable interrupt */
	hwp_uart->ctrl |= UART_ENABLE;

	if (rda_use_dma_rx(port)) {
		/* enable dma uart rx interrupt and clear irq cause registers*/
		hwp_uart->irq_cause |= (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
		hwp_uart->irq_mask  |= (UART_RX_DMA_DONE | UART_RX_DMA_TIMEOUT);
	} else{
		 /* enable rx interrupt */
		hwp_uart->irq_mask |= (UART_RX_DATA_AVAILABLE | UART_RX_TIMEOUT);
	}

	if (rdaport->rxbreak_int_enable && !rdaport->rxbreak_int_detected) {
		hwp_uart->irq_mask |= UART_RX_LINE_ERR;
	}
	spin_unlock_irqrestore(&rdaport->lock, flags);
	return 0;
}

static void rda_uart_shutdown(struct uart_port *port)
{
	struct rda_uart *rdaport =
			container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	unsigned long flags;

	rda_uart_stop_tx(port);
	rda_uart_stop_rx(port);

	spin_lock_irqsave(&port->lock, flags);

	 /* alloc buffer for uart dma, and initialize the dma register */
	if (rda_use_dma_rx(port)){
		RDA_DMA_BUFFER * pstDmaBuf = &rdaport->dma_rx;


		dma_unmap_single(port->dev,pstDmaBuf->dma_addr,
                           RDA_UART_DMA_SIZE,DMA_FROM_DEVICE);
		kfree(pstDmaBuf->buf);
		pstDmaBuf->buf = NULL;
		pstDmaBuf->dma_addr = 0;
		pstDmaBuf->dma_size = 0;
		pstDmaBuf->ofs = 0;
	}

	if (rda_use_dma_tx(port)){
		RDA_DMA_BUFFER *  pstDmaBuf = &rdaport->dma_tx;

		dma_unmap_single(port->dev,pstDmaBuf->dma_addr,UART_XMIT_SIZE,DMA_TO_DEVICE);
		pstDmaBuf->buf = NULL;
		pstDmaBuf->dma_addr = 0;
		pstDmaBuf->dma_size = 0;
		pstDmaBuf->ofs = 0;
	}

	/* Disable all interrupts now . if rx break enable should open rx break irq while suspend */
	if(!rdaport->rxbreak_int_enable)
		hwp_uart->irq_mask = 0;
	else
		hwp_uart->irq_mask = UART_RX_LINE_ERR;

	rdaport->rxbreak_int_detected = 0;

	spin_unlock_irqrestore(&port->lock, flags);

	/*if rx break enable, should keep irq func, do not free*/
	if (port->irq){
		if (!rdaport->rxbreak_int_enable) {
			free_irq(port->irq, port);
			rdaport->irq_free_flag = 1;
		}
	} else{
		del_timer_sync(&rdaport->tmr);
	}
}

static const char *rda_uart_type(struct uart_port *port)
{
	return (port->type == PORT_RDA) ? "RDA UART" : NULL;
}

static int rda_uart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void rda_uart_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int rda_uart_verify_port(struct uart_port *port,
				struct serial_struct *ser)
{
	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_RDA))
		return -EINVAL;
	return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int rda_poll_get_char(struct uart_port *port)
{
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	while (!(GET_BITFIELD(hwp_uart->status, UART_RX_FIFO_LEVEL)))
		return NO_POLL_CHAR;

	return (hwp_uart->rxtx_buffer & 0xff);
}
static void rda_poll_put_char(struct uart_port *port, unsigned char ch)
{
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	while (!(GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE)))
		cpu_relax();

	hwp_uart->rxtx_buffer = (u32)ch;
}
#endif

static int rda_uart_ioctl(struct uart_port *port, unsigned int cmd,
                             unsigned long arg)
{
	struct rda_uart *rdaport = container_of(port, struct rda_uart, port);
	switch (cmd) {
		case RDA_UART_ENABLE_RX_BREAK_IOCTL:
			rdaport->rxbreak_int_enable = (u8)arg;
			rda_enable_rx_break_int(port, rdaport->rxbreak_int_enable);
		return 0;

		default:
			return -ENOIOCTLCMD;
	}
}

/*
 *	Define the basic serial functions we support.
 */
static struct uart_ops rda_uart_ops = {
	.tx_empty       = rda_uart_tx_empty,
	.get_mctrl      = rda_uart_get_mctrl,
	.set_mctrl      = rda_uart_set_mctrl,
	.start_tx       = rda_uart_start_tx,
	.stop_tx        = rda_uart_stop_tx,
	.stop_rx        = rda_uart_stop_rx,
	.enable_ms      = rda_uart_enable_ms,
	.break_ctl      = rda_uart_break_ctl,
	.startup        = rda_uart_startup,
	.shutdown       = rda_uart_shutdown,
	.set_termios    = rda_uart_set_termios,
	.type           = rda_uart_type,
	.request_port	= rda_uart_request_port,
	.release_port	= rda_uart_release_port,
	.config_port	= rda_uart_config_port,
	.verify_port	= rda_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= rda_poll_get_char,
	.poll_put_char	= rda_poll_put_char,
#endif
	.ioctl          = rda_uart_ioctl,
};

static struct rda_uart rda_uart_ports[CONFIG_SERIAL_RDA_UART_MAXPORTS];

#if defined(CONFIG_SERIAL_RDA_UART_CONSOLE)
static struct rda_uart rda_uart_early;

static void rda_early_console_putc(struct uart_port *port, const char c)
{
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	while (!(GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE))){
		cpu_relax();
	}

	hwp_uart->rxtx_buffer = (uint32_t)c;
}

static void rda_early_console_write(struct console *co, const char *s,
					unsigned int count)
{
	struct rda_uart *rdauart = &rda_uart_early;
	struct uart_port *port = &rdauart->port;

	for (; count; count--, s++){
		rda_early_console_putc(port, *s);
		if (*s == '\n'){
			rda_early_console_putc(port, '\r');
		}
	}
}

static int __init rda_early_console_setup(struct console *co, char *options)
{
	struct rda_uart *rdauart = &rda_uart_early;
	struct uart_port *port = &rdauart->port;

	port->mapbase = RDA_UART3_PHYS;
	port->membase = ioremap(port->mapbase, 0x1000);
	if (!port->membase){
		return -ENOMEM;
	}

	return 0;
}

static struct console rda_early_console = {
	.name   = RDA_UART_DEV_NAME,
	.write  = rda_early_console_write,
	.device = uart_console_device,
	.setup	= rda_early_console_setup,
	.flags	= CON_BOOT | CON_PRINTBUFFER,
	.index	= -1,
	.data	= NULL,
};

static int __init rda_early_console_init(void)
{
	register_console(&rda_early_console);
	return 0;
}

console_initcall(rda_early_console_init);

static void rda_uart_console_putc(struct uart_port *port, const char c)
{
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	while (!(GET_BITFIELD(hwp_uart->status, UART_TX_FIFO_SPACE))){
		cpu_relax();
	}

	hwp_uart->rxtx_buffer = (uint32_t)c;
}

static void rda_uart_console_write(struct console *co, const char *s,
					unsigned int count)
{
	struct rda_uart *rdauart = rda_uart_ports + co->index;
	struct uart_port *port = &rdauart->port;
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;
	u32 irqmask = 0;
	unsigned long flags;

	spin_lock_irqsave(&rdauart->lock, flags);
	/*
	  * First, save IMR and then disable interrupts
	 */
	irqmask= hwp_uart->irq_mask;
	hwp_uart->irq_mask = 0;

	for (; count; count--, s++){
		rda_uart_console_putc(port, *s);
		if (*s == '\n'){
			rda_uart_console_putc(port, '\r');
		}
	}

	/* Finallyd restore interrupt register */
	hwp_uart->irq_mask = irqmask;
	spin_unlock_irqrestore(&rdauart->lock, flags);
}

static int __init rda_uart_console_setup(struct console *co, char *options)
{
	struct rda_uart *rdauart = &rda_uart_ports[co->index];
	struct uart_port *port = &rdauart->port;
	int baud = RDA_UART_DEFAULT_BAUD;
	int bits = RDA_UART_DEFAULT_DATABITS;
	int parity = 'n';
	int flow = 'n';
	int ret;
	struct rda_uart *early_rdauart = &rda_uart_early;
	struct uart_port *early_port = &early_rdauart->port;

	if (early_port->membase) {
		/* destroy early console first */
		unregister_console(&rda_early_console);
		iounmap(early_port->membase);
		early_port->membase = 0;
	}

	if (co->index < 0 || co->index >= CONFIG_SERIAL_RDA_UART_MAXPORTS) {
		return -EINVAL;
	}

	if (!port->membase) {
		return -ENODEV;
	}

	if (options) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	}

	ret = uart_set_options(port, co, baud, parity, bits, flow);

	return ret;
}

static struct uart_driver rda_uart_driver;

static struct console rda_uart_console = {
	.name   = RDA_UART_DEV_NAME,
	.write  = rda_uart_console_write,
	.device = uart_console_device,
	.setup	= rda_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &rda_uart_driver,
};

#define RDA_UART_CONSOLE	(&rda_uart_console)

#else

#define	RDA_UART_CONSOLE	NULL

#endif /* CONFIG_RDA_UART_CONSOLE */

static void rda_raw_puts_noirq(const char *s, unsigned int count)
{
	rda_uart_console_write(RDA_UART_CONSOLE, s, count);
}

#ifdef CONFIG_PRINTK_TIME
static int rda_puts_add_time(char *buf)
{
	unsigned long long ts;
	unsigned long rem_nsec;

	ts = local_clock();

	rem_nsec = do_div(ts, 1000000000);

	if (!buf)
		return snprintf(NULL, 0, "[%5lu.000000] ", (unsigned long)ts);

	return sprintf(buf, "[%5lu.%06lu] ",
		       (unsigned long)ts, rem_nsec / 1000);

}
#else
static int rda_puts_add_time(*buf) {return 0;}
#endif


void rda_puts_no_irq(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	int i;

	i = rda_puts_add_time(buf);
	va_start(args, fmt);
	i += vsnprintf(buf+i, 255-i, fmt, args);
	va_end(args);

	rda_raw_puts_noirq(buf, i);
}


/*
 *	Define the rda_uart UART driver structure.
 */
static struct uart_driver rda_uart_driver = {
	.owner       = THIS_MODULE,
	.driver_name = RDA_UART_DRV_NAME,
	.dev_name    = RDA_UART_DEV_NAME,
	.major       = SERIAL_RDA_MAJOR,
	.minor       = SERIAL_RDA_MINOR,
	.nr          = CONFIG_SERIAL_RDA_UART_MAXPORTS,
	.cons        = RDA_UART_CONSOLE,
};

static int rda_init_port(struct rda_uart *rda_port,
					struct platform_device *pdev)
{
	struct uart_port *port = &rda_port->port;
	struct resource *res_mem;
	struct resource *res_irq;
	struct rda_uart_device_data *pdata = pdev->dev.platform_data;

	port->dev = &pdev->dev;

#if defined(CONFIG_SERIAL_RDA_UART_DMA)
	dev_info(&pdev->dev, "use dma\n");
	rda_port->use_dma_rx = 1;
	rda_port->use_dma_tx = 1;
#else
	rda_port->use_dma_rx = 0;
	rda_port->use_dma_tx = 0;
#endif

	/* initialize  virtual address of start registers */
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res_mem){
		port->mapbase = res_mem->start;
	}else{
		return -EINVAL;
	}

	port->membase = ioremap(port->mapbase, resource_size(res_mem));
	if (!port->membase)
		return -ENOMEM;

	port->uartclk = pdata->uartclk;
	if(rda_port->use_dma_rx && rda_port->use_dma_tx){
		rda_port->rxdmarequestid = pdata->rxdmarequestid;
		rda_port->txdmarequestid = pdata->txdmarequestid;
	}
	rda_port->dev_id = pdata->dev_id;
	rda_port->wakeup = pdata->wakeup;
	rda_port->irq_free_flag = 1;

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res_irq){
		port->irq = res_irq->start;
	}

	spin_lock_init(&rda_port->lock);

	tasklet_init(&rda_port->tasklet, rda_tasklet_func,
		(unsigned long)port);

	port->regshift = 0;
	port->line = pdev->id;
	port->fifosize = RDA_UART_TX_FIFO_SIZE;
	port->type = PORT_RDA;
	port->iotype = UPIO_MEM;
	port->ops = &rda_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;

	return 0;
}

static int rda_uart_probe(struct platform_device *pdev)
{
	void *  data = NULL;
	int     i    = pdev->id;
	int	result = 0;
	struct rda_uart *rda_port = &rda_uart_ports[i];
	struct uart_port    *port = &rda_port->port;
	HWP_UART_T *hwp_uart = NULL;
	u8  tmpWakeLockName[64];

	result = rda_init_port(rda_port,pdev);

	if(result)
		return result;

	hwp_uart = (HWP_UART_T*)port->membase;

	if (!rda_use_dma_rx(port)){
		data = kmalloc(sizeof(struct rda_uart_char) * RDA_SERIAL_RINGSIZE, GFP_KERNEL);
		if (!data){
			return -ENOMEM;
		}

		rda_port->rx_ring.buf = data;
		rda_port->rx_ring.head = 0;
		rda_port->rx_ring.tail = 0;
	}

	uart_add_one_port(&rda_uart_driver, port);

	dev_set_drvdata(&pdev->dev, port);

	rda_port->rxbreak_int_detected = 0;
	hwp_uart->ctrl |= UART_RX_BREAK_LENGTH(13);
	sprintf(tmpWakeLockName, "rx_break_wake_lock_uart_id_%d", rda_port->dev_id);
	wake_lock_init(&rda_port->rxbreak_waklock, WAKE_LOCK_SUSPEND, tmpWakeLockName);

	dev_info(&pdev->dev, "rda_uart %d initialized\n",i);
	return 0;
}

static int rda_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = dev_get_drvdata(&pdev->dev);
	struct rda_uart *rdaport = container_of(port, struct rda_uart, port);

	if (port)
	{
		tasklet_kill(&rdaport->tasklet);
		if (!rda_use_dma_rx(port)){
			kfree(rdaport->rx_ring.buf);
		}
		uart_remove_one_port(&rda_uart_driver, port);
		dev_set_drvdata(&pdev->dev, NULL);
		iounmap(port->membase);
		port->membase = 0;
		port->mapbase = 0;

		wake_lock_destroy(&rdaport->rxbreak_waklock);
	}

	return 0;
}

static int rda_uart_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct rda_uart *rdaport = container_of(port, struct rda_uart, port);
	HWP_UART_T *hwp_uart = (HWP_UART_T*)port->membase;

	uart_suspend_port(&rda_uart_driver, port);

	if (rdaport->dev_id == 3  && console_suspend_enabled) {
		/* Disable uart3 of termial. */
		hwp_uart->ctrl &= ~(UART_ENABLE);
	}

	return 0;
}

static int rda_uart_resume(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	uart_resume_port(&rda_uart_driver, port);

	return 0;
}

void rda_uart_register_rxbreak_int_handler(void (*func)(void),u8 dev_id)
{
	int i = 0;

	for(i = 0; i < CONFIG_SERIAL_RDA_UART_MAXPORTS; i ++){
		if(rda_uart_ports[i].dev_id == dev_id){
			rda_uart_ports[i].rxbreak_int_handle = func;
			break;
		}
	}
}

static struct platform_driver rda_uart_platform_driver = {
	.probe   = rda_uart_probe,
	.remove  = rda_uart_remove,
	.suspend = rda_uart_suspend,
	.resume  = rda_uart_resume,
	.driver  = {
		.name       = RDA_UART_DRV_NAME,
		.owner      = THIS_MODULE,
	},
};

static int __init rda_uart_init(void)
{
	int rc;

	rc = uart_register_driver(&rda_uart_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&rda_uart_platform_driver);
	if (rc) {
		uart_unregister_driver(&rda_uart_driver);
		return rc;
	}

	return 0;
}

static void __exit rda_uart_exit(void)
{
	platform_driver_unregister(&rda_uart_platform_driver);
	uart_unregister_driver(&rda_uart_driver);
}

EXPORT_SYMBOL(rda_uart_register_rxbreak_int_handler);

module_init(rda_uart_init);
module_exit(rda_uart_exit);

MODULE_DESCRIPTION("RDA UART driver");
MODULE_AUTHOR("Lei Wang <leiwang@rdamicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_ALIAS_CHARDEV_MAJOR(SERIAL_RDA_MAJOR);
