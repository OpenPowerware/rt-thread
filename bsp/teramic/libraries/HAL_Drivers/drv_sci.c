/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-02-02     xuzhuoyi     first version
 */

#include "rtdevice.h"
#include "board.h"
#include "drv_sci.h"
#include "DSP2803x_Device.h"

#ifdef RT_USING_SERIAL

#define LOG_TAG             "drv.sci"

/* c28x uart driver class */
struct c28x_uart
{
    const char *name;
    volatile struct SCI_REGS *sci_regs;
    struct rt_serial_device serial;
};

static struct c28x_uart uart_obj[3] = {0};
static rt_err_t c28x_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
//    struct c28x_uart *uart;
//    RT_ASSERT(serial != RT_NULL);
//    RT_ASSERT(cfg != RT_NULL);
//    uart = (struct c28x_uart *)serial->parent.user_data;
//    RT_ASSERT(uart != RT_NULL);
//
//    EALLOW;
//
//    // default config
//
//    // 1 stop bit,  No loopback
//    // No parity,8 char bits,
//    // async mode, idle-line protocol
//    uart->sci_regs->SCICCR.all = 0x0007;
//
//    // enable TX, RX, internal SCICLK,
//    // Disable RX ERR, SLEEP, TXWAKE
//    uart->sci_regs->SCICTL1.all = 0x0003;
//
//    uart->sci_regs->SCICTL2.bit.TXINTENA = 1;
//    uart->sci_regs->SCICTL2.bit.RXBKINTENA = 1;
//
//    uart->sci_regs->SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
//    uart->sci_regs->SCILBAUD = 53;
//
//    uart->sci_regs->SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
//
//    switch (cfg->data_bits)
//    {
//    case DATA_BITS_5:
//        uart->sci_regs->SCICCR.bit.SCICHAR = 4;
//        break;
//    case DATA_BITS_6:
//        uart->sci_regs->SCICCR.bit.SCICHAR = 5;
//        break;
//    case DATA_BITS_7:
//        uart->sci_regs->SCICCR.bit.SCICHAR = 6;
//        break;
//    case DATA_BITS_8:
//        uart->sci_regs->SCICCR.bit.SCICHAR = 7;
//        break;
//    default:
//        uart->sci_regs->SCICCR.bit.SCICHAR = 7;
//        break;
//    }
//    switch (cfg->stop_bits)
//    {
//    case STOP_BITS_1:
//        uart->sci_regs->SCICCR.bit.STOPBITS = 0;
//        break;
//    case STOP_BITS_2:
//        uart->sci_regs->SCICCR.bit.STOPBITS = 1;
//        break;
//    default:
//        uart->sci_regs->SCICCR.bit.STOPBITS = 0;
//        break;
//    }
//    switch (cfg->parity)
//    {
//    case PARITY_NONE:
//        uart->sci_regs->SCICCR.bit.PARITYENA = 0;
//        break;
//    case PARITY_ODD:
//        uart->sci_regs->SCICCR.bit.PARITYENA = 1;
//        uart->sci_regs->SCICCR.bit.PARITY = 0;
//        break;
//    case PARITY_EVEN:
//        uart->sci_regs->SCICCR.bit.PARITYENA = 1;
//        uart->sci_regs->SCICCR.bit.PARITY = 1;
//        break;
//    default:
//        uart->sci_regs->SCICCR.bit.PARITYENA = 0;
//        break;
//    }
//
//    EDIS;

    return RT_EOK;
}

static rt_err_t c28x_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct c28x_uart *uart;
    uart = (struct c28x_uart *)serial->parent.user_data;

    EALLOW;

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable interrupt */
        uart->sci_regs->SCICTL2.bit.TXINTENA = 0;
        uart->sci_regs->SCICTL2.bit.RXBKINTENA = 0;
        break;
    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable interrupt */
        uart->sci_regs->SCICTL2.bit.TXINTENA = 1;
        uart->sci_regs->SCICTL2.bit.RXBKINTENA = 1;
        break;
    }
    return RT_EOK;
}

static int c28x_putc(struct rt_serial_device *serial, char c)
{
	while(SciaRegs.SCICTL2.bit.TXRDY != 1)
	{
	}
	SciaRegs.SCITXBUF = c;
    return 1;
}

static int c28x_getc(struct rt_serial_device *serial)
{
    char ch;
    if(SciaRegs.SCIRXST.bit.RXRDY)
    {
    	ch = SciaRegs.SCIRXBUF.bit.RXDT;
    	return ch;
    }
    else
    {
    	return -1;
    }
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial) {
    struct c28x_uart *uart = (struct c28x_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all |= 0x100;       // Issue PIE ack
}

static const struct rt_uart_ops c28x_uart_ops =
{
    .configure = c28x_configure,
    .control = c28x_control,
    .putc = c28x_putc,
    .getc = c28x_getc,
};

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
extern rt_base_t rt_hw_save_context(void);
extern void rt_hw_restore_context(rt_base_t);

__interrupt void sciaRxFifoIsr(void)
{

	rt_base_t context;

	context = rt_hw_save_context();

    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&uart_obj[0].serial);

    /* leave interrupt */
    rt_interrupt_leave();

    rt_hw_restore_context(context);
}
void Scia_Config1(uint32 baud);
void Scia_Config1(uint32 baud)
{
	uint32 div = 0;
	uint32 divsel = 0;
	uint32 lospcp = 0;
	uint32 lspclk = 0;
	uint16 brr = 0;

	/*获取系统时钟的倍频、分频和低速外部时钟的值*/
	div = SysCtrlRegs.PLLCR.bit.DIV;
	divsel = SysCtrlRegs.PLLSTS.bit.DIVSEL;
	lospcp = SysCtrlRegs.LOSPCP.bit.LSPCLK;

	if(lospcp != 0)
	{
		lospcp = lospcp * 2;
	}
	else
	{
		lospcp = 1;
	}

	/*分频值设置
	 divsel为 0时，系统时钟4分频
	 divsel为 1时，系统时钟4分频
	 divsel为 2时，系统时钟2分频
	 divsel为 3时，系统时钟1分频*/
	switch(divsel)
	{
	case 0:
	case 1:
		lspclk = 12000000 * div / 4 / lospcp;
		break;
	case 2:
		lspclk = 12000000 * div / 2 / lospcp;
		break;
	case 3:
		lspclk = 12000000 * div / 1 / lospcp;
		break;
	}

	brr = lspclk / (baud * 8) - 1;

	/*SCI 停止位设置    0：一个停止位   1：两个停止位*/
	SciaRegs.SCICCR.bit.STOPBITS = 0;

	/*SCI 奇偶校验位    0：奇偶校验   1：偶偶校验*/
	SciaRegs.SCICCR.bit.PARITY = 0;

	/*SCI 奇偶校验使能   0：关闭   1：启用*/
	SciaRegs.SCICCR.bit.PARITYENA = 0;

	/*SCI 字符长度   0：1个字长  1：2个字长 ... 7：8个字长*/
	SciaRegs.SCICCR.bit.SCICHAR = 7;

	/*使能SCI的发送机和接收机*/
	SciaRegs.SCICTL1.bit.TXENA = 1;
	SciaRegs.SCICTL1.bit.RXENA = 1;

	/*SCI 16位波特率选择寄存器 高8位*/
	SciaRegs.SCIHBAUD = (uint8)((brr >> 8) & 0xff);
	/*SCI 16位波特率选择寄存器 低8位*/
	SciaRegs.SCILBAUD = (uint8)(brr & 0xff);

	/*SCI 软件复位，重新启动SCI*/
	SciaRegs.SCICTL1.bit.SWRESET = 1;
}

int rt_hw_sci_init(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
/* Disable internal pull-up for the selected output pins
   to reduce power consumption. */
// Pull-ups can be enabled or disabled disabled by the user.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;    // Enable pull-up for GPIO19 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up for GPIO7  (SCIRXDA)

	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;	   // Disable pull-up for GPIO29 (SCITXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;	   // Disable pull-up for GPIO18 (SCITXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;	   // Disable pull-up for GPIO12 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDA)
//	GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;   // Asynch input GPIO7 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;    // Configure GPIO7  for SCIRXDA operation

	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;   // Configure GPIO12 for SCITXDA operation

    PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
    EDIS;
    Scia_Config1(115200);
    //
    // Enable interrupts required for this example
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
    IER |= 0x100;                        // Enable CPU INT

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t result = 0;

    uart_obj[0].serial.ops    = &c28x_uart_ops;
    uart_obj[0].serial.config = config;
    uart_obj[0].name          = "scia";
    uart_obj[0].sci_regs      = &SciaRegs;

//    uart_obj[1].serial.ops    = &c28x_uart_ops;
//    uart_obj[1].serial.config = config;
//    uart_obj[1].name          = "scib";
//    uart_obj[1].sci_regs      = &ScibRegs;
//
//    uart_obj[2].serial.ops    = &c28x_uart_ops;
//    uart_obj[2].serial.config = config;
//    uart_obj[2].name          = "scic";
//    uart_obj[2].sci_regs      = &ScicRegs;

    /* register UART device */
    result = rt_hw_serial_register(&uart_obj[0].serial, uart_obj[0].name,
                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                                   &uart_obj[0]);

//    /* register UART device */
//    result = rt_hw_serial_register(&uart_obj[1].serial, uart_obj[1].name,
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
//                                   &uart_obj[1]);
//
//    /* register UART device */
//    result = rt_hw_serial_register(&uart_obj[2].serial, uart_obj[2].name,
//                                   RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
//                                   &uart_obj[2]);

    return result;
}

#endif /* RT_USING_SERIAL */
