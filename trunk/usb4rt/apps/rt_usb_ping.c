/***************************************************************************
 *   Copyright (C) 2005 by Jörg Langenberg                                 *
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

//                        --s-ms-us-ns
#define NS_PER_PERIOD        100125000

#define MAX_BUFFER_SIZE       256
#define TASK_PRIORITY         1
#define ANZ_URBS              16

/* Parameter for Blocking URBs: */
#define MAX_WAIT_TIME   500000000  /* 500ms max sleep-time on RT_SEM */

MODULE_LICENSE("GPL");

int          pings = 0;
unsigned int alloc_bytes = 0;       // allocated Byte
int          anz_retrys = ANZ_URBS; // Number of retrys
int          retry = 0;             // actual retry
int          all_urbs_finished = 0; // ==1, if all URBs finished
int          len;                   // length of Configuration-Descriptor
uint64_t     min, max;
int64_t      sum = 0;
uint64_t     value[ANZ_URBS];

static unsigned int blocking = 0;
module_param(blocking, uint, 0);
MODULE_PARM_DESC(blocking," Set it to 1 to use blocking URBs");

static unsigned int next_in_callback = 0;
module_param(next_in_callback, uint, 0);
MODULE_PARM_DESC(next_in_callback," Send next URB inside callback function");

static unsigned int period_time = NS_PER_PERIOD;
module_param(period_time, uint, 0);
MODULE_PARM_DESC(period_time,"Delay between sending URBs");

static unsigned int vendor = 0x0000;
module_param(vendor, uint, 0);
MODULE_PARM_DESC(vendor,"Hex-Value of Vendor-ID");

static unsigned int product = 0x0000;
module_param(product, uint, 0);
MODULE_PARM_DESC(product,"Hex-Value of Product-ID");

static rtdm_task_t rtTask;

unsigned char *names[] = {  "ping_urb[1]","ping_urb[2]","ping_urb[3]","ping_urb[4]",
                            "ping_urb[5]","ping_urb[6]","ping_urb[7]","ping_urb[8]",
                            "ping_urb[9]","ping_urb[10]","ping_urb[11]","ping_urb[12]",
                            "ping_urb[13]","ping_urb[14]","ping_urb[15]","ping_urb[16]"};

struct urb_struct{
    struct rt_urb*  p_urb[ANZ_URBS];
    __u8            completed[ANZ_URBS];
    __u8            registered[ANZ_URBS];
    __u8            rt_sem_created[ANZ_URBS];
    uint64_t        send_time[ANZ_URBS];
    uint64_t        end_time[ANZ_URBS];
};

struct urb_struct urbs;
struct usb_device *p_device;

void rt_callback_fkt(struct rt_urb *p_urb)
{
    int i, ret;
    int urb_nr = -1;

    for (i=0;i < ANZ_URBS;i++)
    {
        if (p_urb == urbs.p_urb[i])
            urb_nr = i;
    }

    if (urb_nr == -1)
    {
        printk("RT-USB-PING: No mine ... \n");
        return;
    }

    if (p_urb->actual_length < p_urb->transfer_buffer_length )
    {
        printk("RT-USB-PING: [ERROR] URB[%d] : Received/Sent only %d/%d Byte \n",
               urb_nr, p_urb->actual_length, p_urb->transfer_buffer_length);
    }

    urbs.end_time[urb_nr] = rtdm_clock_read();
    printk("RT-USB-PING: End-Time  URB[%2d] (Callback): %llu \n",
           urb_nr, urbs.end_time[urb_nr]);
    urbs.completed[urb_nr] = 1;

    if (likely(!next_in_callback))
        return;

    if ( retry >= (ANZ_URBS - 1) )
    {
        all_urbs_finished = 1;
        return;
    }

    retry++;
    urbs.send_time[retry] = rtdm_clock_read();
    printk("RT-USB-PING: Send-Time URB[%2d] (Callback): %llu \n",
           retry, urbs.send_time[urb_nr]);

    ret = rt_usb_submit_ctrl_urb(urbs.p_urb[retry],
                                 USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                 USB_REQ_GET_DESCRIPTOR,
                                 USB_DT_CONFIG << 8 | 0x00,
                                 0x0000,
                                 len ,
                                 urbs.p_urb[retry]->p_transfer_buffer);
    if ( ret < 0)
    {
        printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
        return;
    }

    return;
}

int not_send_next_in_callback(struct urb_struct *p_urbs)
{
    int ret, i;

    while (retry < anz_retrys)
    {
        p_urbs->send_time[retry] = rtdm_clock_read();

        if (!blocking)
        {
            printk("RT-USB-PING: Send-Time URB[%2d] (Callback): %llu \n",
                   retry, p_urbs->send_time[retry]);

            ret = rt_usb_submit_ctrl_urb(p_urbs->p_urb[retry],
                                         USB_DIR_IN | USB_TYPE_STANDARD |
                                         USB_RECIP_DEVICE,
                                         USB_REQ_GET_DESCRIPTOR,
                                         USB_DT_CONFIG << 8 | 0x00,
                                         0x0000,
                                         len ,
                                         p_urbs->p_urb[retry]->p_transfer_buffer);
            if (ret < 0)
            {
                printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
                return ret;
            }
        }
        else // blocking
        {
            p_urbs->p_urb[retry]->rt_complete_fkt = NULL;
            printk("RT-USB-PING: Send-Time URB[%2d] (Semaphore): %llu \n",
                   retry, p_urbs->send_time[retry]);

            ret = rt_usb_send_ctrl_msg(p_urbs->p_urb[retry],
                                       USB_DIR_IN | USB_TYPE_STANDARD |
                                       USB_RECIP_DEVICE,
                                       USB_REQ_GET_DESCRIPTOR,
                                       USB_DT_CONFIG << 8 | 0x00,
                                       0x0000,
                                       len ,
                                       p_urbs->p_urb[retry]->p_transfer_buffer );

            if (p_urbs->p_urb[retry]->actual_length <
                p_urbs->p_urb[retry]->transfer_buffer_length )
            {
                printk("RT-USB-PING: [ERROR] Received / Sent only %d / %d Byte \n",
                       p_urbs->p_urb[retry]->actual_length,
                       p_urbs->p_urb[retry]->transfer_buffer_length);
            }
            p_urbs->completed[retry] = 1;
            p_urbs->end_time[retry] = rtdm_clock_read();
            printk("RT-USB-PING: End-Time  URB[%2d] (Semaphore): %llu \n",
                   retry, p_urbs->end_time[retry]);
        }
        rtdm_task_wait_period();
        retry++;
    }

    // Waiting until all URBs are completed
    while (!all_urbs_finished )
    {
        rtdm_task_wait_period();
        all_urbs_finished = 1;
        for (i=0; i< anz_retrys; i++)
        {
            if (!p_urbs->completed[i])
                all_urbs_finished = 0;
        }
    }

    return 0;
}

int send_next_in_callback(struct urb_struct *p_urbs)
{
    int ret, old_retry;

    // Sending the first URB, the other called by interrupt handler
    p_urbs->send_time[retry] = rtdm_clock_read();
    printk("RT-USB-PING: Send-Time URB[%2d] (Callback): %llu \n",
           retry, p_urbs->send_time[retry]);

    ret = rt_usb_submit_ctrl_urb(p_urbs->p_urb[retry],
                                 USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                 USB_REQ_GET_DESCRIPTOR,
                                 USB_DT_CONFIG << 8 | 0x00,
                                 0x0000,
                                 len ,
                                 p_urbs->p_urb[retry]->p_transfer_buffer);
    if (ret < 0)
    {
        printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
        return ret;
    }

    old_retry = retry;
    printk("Waiting for URB[%d] \n",retry);
    while (!all_urbs_finished)
    {
        if (old_retry != retry)
        {
            printk("Waiting for URB[%d] \n",retry);
            old_retry = retry;
        }
        rtdm_task_wait_period();
    }

    return 0;
}

void periodic_fkt(void *p_data)
{
    int ret, i;
    struct urb_struct *p_urbs = (struct urb_struct *)p_data;

    len = p_urbs->p_urb[0]->p_usbdev->p_cfg_desc->wTotalLength;

    if (len <= 0)
        return;

    if (!next_in_callback)
    {
        ret = not_send_next_in_callback(p_urbs);
        if (ret)
            return;
    }
    else
    {
        ret = send_next_in_callback(p_urbs);
        if (ret)
            return;
    }

    // calculating times
    min = max = p_urbs->end_time[0] - p_urbs->send_time[0];
    sum = 0;

    printk("RT-USB-PING: *** List of needed URB send times *** \n");
    for (i=0; i<anz_retrys; i++)
    {
        if( !p_urbs->completed[i] )
            continue;

        value[i] = p_urbs->end_time[i] - p_urbs->send_time[i];
        if (value[i] > max) max = value[i];
        if (value[i] < min) min = value[i];
        sum += value[i];

        printk("RT-USB-PING: %3d Ping to USB-Device %d -> %llu ns \n",
               i,p_urbs->p_urb[i]->p_usbdev->address, value[i] );

    }

    printk("Send-Time MIN: %llu ns - AVERAGE: %llu ns - MAX: %llu ns\n ",min, sum >> 4, max );
    printk("All done ( %llu ns )\n",sum);
    return;
}

struct rt_urb *create_urb(  struct usb_device *p_dev,
                            unsigned int pipe,
                            unsigned int transfer_flags,
                            rt_usb_complete_t rt_complete_fkt,
                            unsigned char *rt_sem_name,
                            int64_t rt_sem_timeout)
{
    struct rt_urb           *p_urb    = NULL;
    struct usb_ctrlrequest  *p_ctrl   = NULL;
    unsigned char           *p_buffer = NULL;

    p_urb = kmalloc(sizeof(struct rt_urb),GFP_ATOMIC);
    if (!p_urb)
    {
        printk("RT-USB-PING: [ERROR] No Memory for URB \n");
        return NULL;
    }
    alloc_bytes += sizeof(struct rt_urb);
    memset( p_urb, 0, sizeof(struct rt_urb) );

    p_ctrl = kmalloc(sizeof(struct usb_ctrlrequest),GFP_ATOMIC);
    if (!p_ctrl)
    {
        printk("RT-USB-PING: [ERROR] No Memory for Ctrl-Request \n");
        kfree(p_urb);
        alloc_bytes -= sizeof(struct rt_urb);
        return NULL;
    }
    alloc_bytes += sizeof(struct usb_ctrlrequest);
    memset( p_ctrl, 0, sizeof(struct usb_ctrlrequest) );

    p_buffer = kmalloc(MAX_BUFFER_SIZE,GFP_ATOMIC);
    if (!p_buffer)
    {
        printk("RT-USB-PING: [ERROR] No Memory for Buffer \n");
        kfree(p_ctrl);
        alloc_bytes -= sizeof(struct usb_ctrlrequest);
        kfree(p_urb);
        alloc_bytes -= sizeof(struct rt_urb);
        return NULL;
    }
    alloc_bytes += MAX_BUFFER_SIZE;
    memset( p_buffer, 0, MAX_BUFFER_SIZE);

    p_urb->p_setup_packet = p_ctrl;
    p_urb->p_transfer_buffer = p_buffer;
    p_urb->transfer_flags = transfer_flags;
    p_urb->max_buffer_len = MAX_BUFFER_SIZE;
    p_urb->p_usbdev = p_dev;
    p_urb->p_hcd = p_dev->p_hcd;
    p_urb->pipe = pipe;
    p_urb->max_packet_size = usb_maxpacket(p_dev,pipe);

    if (blocking)
    {
        p_urb->rt_complete_fkt = NULL;
        p_urb->rt_sem_timeout = rt_sem_timeout;
    }
    else
    {
        p_urb->rt_complete_fkt = rt_callback_fkt;
        p_urb->rt_sem_timeout = 0;
    }

    rtdm_sem_init (&p_urb->rt_sem, 0);
    return p_urb;
}

void destroy_urbs(void)
{
    int i;
    for (i=0;i< ANZ_URBS;i++)
    {
        if (urbs.p_urb[i])
        {
            if (urbs.registered[i])
                nrt_usb_unregister_urb(urbs.p_urb[i]);

            if (urbs.rt_sem_created[i])
                rtdm_sem_destroy(&urbs.p_urb[i]->rt_sem);

            if (urbs.p_urb[i]->p_setup_packet)
            {
                kfree(urbs.p_urb[i]->p_setup_packet);
                alloc_bytes -= sizeof(struct usb_ctrlrequest);
            }

            if (urbs.p_urb[i]->p_transfer_buffer)
            {
                kfree(urbs.p_urb[i]->p_transfer_buffer);
                alloc_bytes -= MAX_BUFFER_SIZE;
            }

            kfree(urbs.p_urb[i]);
            alloc_bytes -= sizeof(struct rt_urb);
        }
    }
    printk("RT-USB-PING: All %d URBs deleted \n",ANZ_URBS);
}

int create_urbs( struct usb_device *p_dev)
{
    int i,ret = 0;
    for (i=0;i< ANZ_URBS;i++)
    {
        urbs.p_urb[i] = create_urb(p_dev,
                                   usb_rcvctrlpipe(p_dev,0),
                                   URB_SHORT_NOT_OK,
                                   rt_callback_fkt,
                                   names[i],
                                   500000000);
        if (!urbs.p_urb[i])
        {
            printk("RT-USB-PING: [ERROR] Creating URB[%d], code = %d \n",i,ret);
            destroy_urbs();
            return -1;
        }
        urbs.rt_sem_created[i] = 1;

        ret = nrt_usb_register_urb(urbs.p_urb[i]);
        if (ret)
        {
            printk("RT-USB-PING: [ERROR] Register URB 0x%p failed, code = %d \n",
                   urbs.p_urb[i], ret);
               destroy_urbs();
            return -1;
        }

        urbs.registered[i] = 1;
    }
    return 0;
}

int init_module(void)
{
    int i;
    int ret;

    // small check
    if (blocking)
        next_in_callback = 0;

    printk("RT-USB-PING: Blocking URBs: %s\n", blocking ? "yes" : "no");
    printk("RT-USB-PING: Send next URB in callback: %s\n",
           next_in_callback ? "yes" : "no");
    printk("RT-USB-PING: Period time = %d ns \n", period_time);
    printk("RT-USB-PING: Vendor: 0x%04x ns \n", vendor);
    printk("RT-USB-PING: Product: 0x%04x ns \n", product);

    p_device = NULL;
    p_device = rt_usb_get_device(vendor, product, p_device);
    if (!p_device)
    {
        printk("RT-USB-PING: Sorry, Device not found ... \n");
        return -ENODEV;
    }
    else
    {
        printk("RT-USB-PING: vendor = 0x%04x, product = 0x%04x found ... \n",
               vendor, product);
        printk("RT-USB-PING: USB-Device @ 0x%p \n", p_device);
        printk("RT-USB-PING: Host-Controller @ 0x%p \n", p_device->p_hcd);
    }

    for (i=0;i<ANZ_URBS;i++)
    {
        urbs.p_urb[i] = NULL;
        urbs.registered[i] = 0;
        urbs.rt_sem_created[i] = 0;
        urbs.completed[i] = 0;
    }

    ret = create_urbs(p_device);
    if (ret)
    {
        printk("RT-USB-PING: [ERROR] Creating URBs, code = %d \n",ret);
        rt_usb_put_device(p_device);
        return ret;
    }

    ret = rtdm_task_init(&rtTask, "rt_usb_ping", periodic_fkt, &urbs ,
                         RTDM_TASK_HIGHEST_PRIORITY, (uint64_t)period_time);
    if (ret)
    {
        printk("RT-USB-PING: Failed to create Task, code = %d\n",ret);
        return ret;
    }

    printk("RT-USB-PING: started, %d Byte allocted \n", alloc_bytes);
    return 0;
}

void cleanup_module(void)
{
    destroy_urbs();
    rt_usb_put_device(p_device);

    rtdm_task_destroy(&rtTask);
    printk("RT-USB-PING: STOPPED, %d Byte allocted \n",alloc_bytes);
}

