/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-28     qiyu     first version
 */
#include <rthw.h>
#include "dsc_config.h"
#include "drv_gpio.h"

#ifdef RT_USING_PIN

// the gpio pin number for each port is 32, while it is 16 for ARM
#define PIN_NUM(port, no) (((((port) & 0xFu) << 5) | ((no) & 0x1F)))
#define PIN_PORT(pin) ((rt_uint16_t)(((pin) >> 5) & 0xFu))
#define PIN_NO(pin) ((rt_uint16_t)((pin) & 0x1Fu))

#define PIN_c28x_PORT(pin) (volatile Uint32 *)&GpioDataRegs + (PIN_PORT(pin))*1
#define PIN_c28x_PIN(pin) ((rt_uint32_t)(1u << PIN_NO(pin)))

#define PIN_c28x_PORT_MAX 1 /* gpioA to GPIOF in total*/
#define PIN_IRQ_MAX 5   /* XINT1 to XINT5 in total */

static rt_err_t c28x_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args);
static rt_err_t c28x_pin_dettach_irq(struct rt_device *device, rt_int32_t pin);
static rt_err_t c28x_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                     rt_uint32_t enabled);

static rt_base_t c28x_pin_get(const char *name)
{
    int hw_pin_num = 0;
    int i, name_len;

    name_len = rt_strlen(name);

    if ((name_len < 3) || (name_len >= 7))
    {
        return -RT_EINVAL;
    }
    /*
     * PX.y
     */
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    for (i = 3; i < name_len; i++)
    {
        hw_pin_num *= 10;
        hw_pin_num += name[i] - '0';
    }
    return hw_pin_num;
}

static void c28x_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;
    if (PIN_PORT(pin) < PIN_c28x_PORT_MAX)
    {
        gpioDataReg = PIN_c28x_PORT(pin);
        pinMask = 1UL << (PIN_NO(pin));
        if (value == 0)
        {
            GpioDataRegs.GPACLEAR.all = pinMask;
        }
        else
        {
            GpioDataRegs.GPASET.all = pinMask;
        }
    }
}

static int c28x_pin_read(rt_device_t dev, rt_base_t pin)
{
    volatile Uint32 *gpioDataReg;
    int value = PIN_LOW;

    if (PIN_PORT(pin) < PIN_c28x_PORT_MAX)
    {
        value = (GpioDataRegs.GPADAT.all >> PIN_NO(pin)) & 0x1;
    }

    return value;
}

static void c28x_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    volatile Uint32 *gpioBaseAddr;
    volatile Uint32 *dir, *pud, *odr;
    if (PIN_PORT(pin) >= PIN_c28x_PORT_MAX)
    {
        return;
    }
    rt_uint32_t pinMask;
    pinMask = 1UL << PIN_NO(pin);
    gpioBaseAddr = (Uint32 *)&GpioCtrlRegs;

    dir = gpioBaseAddr + 9;
    pud = gpioBaseAddr + 12;

    EALLOW;
    if (mode == PIN_MODE_OUTPUT)
    {
        *dir |= pinMask;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        *dir &= ~pinMask;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        *dir &= ~pinMask;
        *pud &= ~pinMask;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        *dir &= ~pinMask;
        *pud |= pinMask;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        *dir |= pinMask;
    }
    EDIS;
}

const static struct rt_pin_ops _c28x_pin_ops =
{
    c28x_pin_mode,
    c28x_pin_write,
    c28x_pin_read,
	RT_NULL,
	RT_NULL,
    RT_NULL,
    c28x_pin_get,
};

int rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &_c28x_pin_ops, RT_NULL);
}

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

#endif /* RT_USING_PIN */



