/***************************************************************************
 *   Copyright (C) 2006 by Joerg Langenberg                                *
 *   joerg.langenberg@gmx.net                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>

#include <rt_usb.h>

#include "rt_ftdi_sio.h"

#define RT_FTDI_VID                 0x0403
#define RT_8U232AM_PID              0x6001
#define MAX_DEVICES                 4

// buffer sizes of the URBs
#define MAX_CTRL_BUFFER_SIZE        64
#define MAX_READ_BUFFER_SIZE        64
#define MAX_WRITE_BUFFER_SIZE       64

// created in the init function (while loading the module)
typedef struct ftdi_private {
    struct usb_device*      usb_dev;
    int                     dev_id;

    struct rtdm_device*     rtdm_dev;
    char                    rtdm_registered;

    struct rt_urb*          ctrl_urb;
    void*                   ctrl_buffer;
    char                    ctrl_registered;

    struct rt_urb*          read_urb;
    void*                   read_buffer;
    char                    read_registered;

    struct rt_urb*          write_urb;
    void*                   write_buffer;
    char                    write_registered;

} ftdi_private_t;

// created while opening a device (from RTDM)
typedef struct ftdi_context {
    ftdi_private_t*         priv;
} ftdi_context_t;

static ftdi_private_t ftdi_priv[MAX_DEVICES];

int rt_ftdi_open(struct rtdm_dev_context *context,
                 rtdm_user_info_t *user_info, int oflags)
{
    struct ftdi_context *ctx;
    int dev_id = context->device->device_id;

    ctx = (struct ftdi_context *)context->dev_private;

    // fill context
    ctx->priv = &ftdi_priv[dev_id];

    // inc module use counter
    try_module_get(THIS_MODULE);

    return 0;
}

int rt_ftdi_close(struct rtdm_dev_context *context,
                   rtdm_user_info_t *user_info)
{
    struct ftdi_context *ctx;

    ctx = (struct ftdi_context *)context->dev_private;

    // ...

    // dec module use counter
    module_put(THIS_MODULE);

    return 0;
}

int rt_ftdi_ioctl(struct rtdm_dev_context *context,
                 rtdm_user_info_t *user_info, int request, void *arg)
{
    struct ftdi_context *ctx;

    ctx = (struct ftdi_context *)context->dev_private;

    // ...

    return -ENOTTY;
}



ssize_t rt_ftdi_read(struct rtdm_dev_context *context,
                     rtdm_user_info_t *user_info, void *buf, size_t nbyte)
{
    struct ftdi_context *ctx;
    ctx = (struct ftdi_context *)context->dev_private;

    // ...

    return 0;
}

ssize_t rt_ftdi_write(struct rtdm_dev_context *context,
                      rtdm_user_info_t *user_info, const void *buf,
                      size_t nbyte)
{
    struct ftdi_context *ctx;
    ctx = (struct ftdi_context *)context->dev_private;

    // ...

    return 0;
}

static const struct rtdm_device __initdata device_tmpl = {
    struct_version:     RTDM_DEVICE_STRUCT_VER,

    device_flags:       RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
    context_size:       sizeof(struct ftdi_context),
    device_name:        "rtserUSB0",

    open_rt:            rt_ftdi_open,
    open_nrt:           rt_ftdi_open,

    ops: {
        close_rt:       rt_ftdi_close,
        close_nrt:      rt_ftdi_close,

        ioctl_rt:       rt_ftdi_ioctl,
        ioctl_nrt:      rt_ftdi_ioctl,

        read_rt:        rt_ftdi_read,
        read_nrt:       NULL,

        write_rt:       rt_ftdi_write,
        write_nrt:      NULL,

        recvmsg_rt:     NULL,
        recvmsg_nrt:    NULL,

        sendmsg_rt:     NULL,
        sendmsg_nrt:    NULL,
    },

    device_class:       RTDM_CLASS_SERIAL,
    device_sub_class:   RTDM_SUBCLASS_USBSERIAL,
    driver_name:        "rt_ftdi_sio",
    driver_version:     RTDM_DRIVER_VER(0, 0, 1),
    peripheral_name:    "UART USB",
    provider_name:      "Joerg Langenberg",
};

static void destroy_urbs(int dev_id)
{
    // destroy ctrl URB

    if (ftdi_priv[dev_id].ctrl_registered)
    {
        nrt_usb_unregister_urb(ftdi_priv[dev_id].ctrl_urb);
        printk("rt_ftdi_sio[%d]: Ctrl URB has been unregistered\n", dev_id);
    }

    if (ftdi_priv[dev_id].ctrl_buffer)
    {
        kfree(ftdi_priv[dev_id].ctrl_buffer);
        printk("rt_ftdi_sio[%d]: Ctrl buffer has been given free\n", dev_id);
    }

    if (ftdi_priv[dev_id].ctrl_urb)
    {
        nrt_usb_destroy_ctrl_semaphore_urb(ftdi_priv[dev_id].ctrl_urb);
        printk("rt_ftdi_sio[%d]: Ctrl URB has been destroyed\n", dev_id);
    }

    return;
}

static int create_urbs(int dev_id)
{
    int ret;
    void* buffer;
    struct rt_urb *p_urb;
    struct usb_device *usb_dev = ftdi_priv[dev_id].usb_dev;
    unsigned char urbname[20];

    // create control URB
    snprintf(urbname, 20, "ctrl-urb[%d]", dev_id);
    p_urb = nrt_usb_create_ctrl_semaphore_urb(urbname, 500000000llu);
    if (!p_urb)
    {
        printk("rt_ftdi_sio[%d]: [ERROR] Can't create ctrl URB\n", dev_id);
        return -EFAULT;
    }
    ftdi_priv[dev_id].ctrl_urb = p_urb;
    printk("rt_ftdi_sio[%d]: Ctrl URB has been created\n", dev_id);

    // allocate buffer
    buffer = kmalloc(MAX_CTRL_BUFFER_SIZE, GFP_ATOMIC);
    if (!buffer)
    {
        printk("rt_ftdi_sio[%d]: [ERROR] No Memory for Buffer \n", dev_id);
        ret = -ENOMEM;
        goto create_error;
    }
    ftdi_priv[dev_id].ctrl_buffer = buffer;
    printk("rt_ftdi_sio[%d]: Ctrl buffer has been allocated\n", dev_id);

    // fill URB
    p_urb->max_buffer_len    = MAX_CTRL_BUFFER_SIZE;
    p_urb->p_transfer_buffer = buffer;
    p_urb->p_hcd             = usb_dev->p_hcd;
    p_urb->p_usbdev          = usb_dev;
    p_urb->pipe              = usb_rcvctrlpipe(usb_dev,0);
    p_urb->max_packet_size   = usb_maxpacket(usb_dev, p_urb->pipe);

    // register ctrl URB
    ret = nrt_usb_register_urb(ftdi_priv[dev_id].ctrl_urb);
    if (ret)
    {
        printk("rt_ftdi_sio[%d]: [ERROR] Can't register ctrl URB, code = %d \n",
               dev_id, ret);
        goto create_error;
    }
    ftdi_priv[dev_id].ctrl_registered = 1;
    printk("rt_ftdi_sio[%d]: Ctrl URB has been registered\n", dev_id);

    return 0;

create_error:
    destroy_urbs(dev_id);

    return ret;
}

static void destroy_rtdm_device(int dev_id)
{
    if (ftdi_priv[dev_id].rtdm_registered)
    {
        rtdm_dev_unregister(ftdi_priv[dev_id].rtdm_dev, 1000);
        printk("rt_ftdi_sio[%d]: RTDM device %s has been unregistered\n",
               dev_id, ftdi_priv[dev_id].rtdm_dev->device_name);
    }

    if (ftdi_priv[dev_id].rtdm_dev)
    {
        kfree(ftdi_priv[dev_id].rtdm_dev);
        printk("rt_ftdi_sio[%d]: RTDM device has been destoyed\n", dev_id);
    }
}

static int create_rtdm_device(int dev_id)
{
    int ret;
    struct rtdm_device* dev;

    dev = kmalloc(sizeof(struct rtdm_device), GFP_KERNEL);
    if (!dev)
    {
        printk("rt_ftdi_sio[%d]: [ERROR] No Memory for RTDM device \n", dev_id);
        return -ENOMEM;
    }

    memcpy(dev, &device_tmpl, sizeof(struct rtdm_device));
    snprintf(dev->device_name, RTDM_MAX_DEVNAME_LEN, "rtserUSB%d", dev_id);
    dev->device_id = dev_id;
    dev->proc_name = dev->device_name;

    ftdi_priv[dev_id].rtdm_dev = dev;
    printk("rt_ftdi_sio[%d]: RTDM device %s has been created\n", dev_id,
           dev->device_name);

    ret = rtdm_dev_register(dev);
    if (ret)
    {
        printk("rt_ftdi_sio[%d]: [ERROR] Can't register device %s, code = %d \n",
               dev_id, dev->device_name, ret);
        kfree(dev);
        return ret;
    }
    ftdi_priv[dev_id].rtdm_registered = 1;
    printk("rt_ftdi_sio[%d]: RTDM device %s has been registered\n", dev_id,
           dev->device_name);

    return 0;
}

void rt_ftdi_exit(void);

int __init rt_ftdi_init (void)
{
    int i = 0;
    int ret;
    struct usb_device *usb_dev = NULL;

    for (i = 0; i < MAX_DEVICES; i++)
    {
        usb_dev = rt_usb_get_device(RT_FTDI_VID, RT_8U232AM_PID, usb_dev);
        if (!usb_dev && !i) // no device has been found
        {
            printk("rt_ftdi_sio[%d]: Sorry, Device not found ... \n", i);
            return -ENODEV;
        }
        else if (!usb_dev && i) // no more devices are found
            break;

        printk("rt_ftdi_sio[%d]: Device found (vendor: 0x%04x, product: 0x%04x) ... \n",
               i, RT_FTDI_VID, RT_8U232AM_PID);
        printk("rt_ftdi_sio[%d]: USB-Device @ 0x%p LOCKED\n", i, usb_dev);
        printk("rt_ftdi_sio[%d]: Host-Controller @ 0x%p \n", i, usb_dev->p_hcd);

        ftdi_priv[i].usb_dev = usb_dev;

        ret = create_urbs(i);
        if (ret)
            goto init_error;

        ret = create_rtdm_device(i);
        if (ret)
            goto init_error;

    }
    return 0;

 init_error:
    rt_ftdi_exit();

    return ret;
}


void __exit rt_ftdi_exit (void)
{
    int i;

    for (i = 0; i < MAX_DEVICES; i++)
    {
        destroy_rtdm_device(i);
        destroy_urbs(i);

        if (ftdi_priv[i].usb_dev)
        {
            printk("rt_ftdi_sio[%d]: USB-Device @ 0x%p UNLOCKED\n", i,
                   ftdi_priv[i].usb_dev);
            rt_usb_put_device(ftdi_priv[i].usb_dev);
        }
    }
}

module_init(rt_ftdi_init);
module_exit(rt_ftdi_exit);

MODULE_LICENSE("GPL");
