/***************************************************************************
 *   Copyright (C) 2005 by Joerg Langenberg                                *
 *   joergel@gmx.net                                                       *
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
#include <rtai/task.h>
#include <rtai/timer.h>
#include <rtai/sem.h>

#include "../core/rt_usb.h"
//                        --s-ms-us-ns
#define NS_PER_PERIOD        100125000

#define MAX_BUFFER_SIZE       256
#define TASK_PRIORITY         1
#define ANZ_URBS              16

/* Callback or Blocking URBs (RT_SEM) */
#define BLOCKING
//#undef BLOCKING

/* Parameter for Blocking URBs: */
#define MAX_WAIT_TIME   500000000  /* 500ms max sleep-time on RT_SEM */

/* Parameter for Callback URBs: */
//#define NEXT_URB_IN_CALLBACK     /* Next URB called by Interrupt-Handler */
#undef NEXT_URB_IN_CALLBACK        /* Next URB called in Task-Loop */


MODULE_LICENSE("GPL");

int vendor = 0x0000;
int product= 0x0000;
int pings = 0;
unsigned int alloc_bytes = 0;   // allocated Byte
int anz_retrys = ANZ_URBS;      // Number of retrys
int retry = 0;                  // actual retry
int all_urbs_finished = 0;      // ==1, if all URBs finished
int len;                        // length of Configuration-Descriptor
RTIME min,max;
RTIME sum = 0;
RTIME value[ANZ_URBS];

MODULE_PARM (vendor,"i");
MODULE_PARM_DESC (vendor,"Hex-Value of Vendor-ID");
MODULE_PARM (product,"i");
MODULE_PARM_DESC (product,"Hex-Value of Product-ID");

__u8 stop_timer = 1;

static RT_TASK  rtTask;

unsigned char *names[] = {  "ping_urb[1]","ping_urb[2]","ping_urb[3]","ping_urb[4]",
                            "ping_urb[5]","ping_urb[6]","ping_urb[7]","ping_urb[8]",
                            "ping_urb[9]","ping_urb[10]","ping_urb[11]","ping_urb[12]",
                            "ping_urb[13]","ping_urb[14]","ping_urb[15]","ping_urb[16]"};

struct urb_struct{
  struct rt_urb *p_urb[ANZ_URBS];
  __u8 completed[ANZ_URBS];
  __u8 registered[ANZ_URBS];
  __u8 rt_sem_created[ANZ_URBS];
  RTIME send_time[ANZ_URBS];
  RTIME end_time[ANZ_URBS];
};

struct urb_struct urbs;
struct usb_device *p_device;

void rt_callback_fkt(struct rt_urb *p_urb)
{

  int i;
  int urb_nr = -1;
  for(i=0;i<ANZ_URBS;i++){
    if(p_urb == urbs.p_urb[i]){
      urb_nr = i;
    }
  }
  if(urb_nr == -1){
    printk("RT-USB-PING: No mine ... \n");
    return;
  }

  if(p_urb->actual_length < p_urb->transfer_buffer_length ){
    printk("RT-USB-PING: [ERROR] URB[%d] : Received/Sent only %d/%d Byte \n",urb_nr,
           p_urb->actual_length,p_urb->transfer_buffer_length);
  }

  urbs.end_time[urb_nr] = rt_timer_read();
  printk("RT-USB-PING: End-Time  URB[%d] (Callback) : %llu \n",urb_nr,urbs.end_time[urb_nr]);
  urbs.completed[urb_nr] = 1;

#ifdef NEXT_URB_IN_CALLBACK

  int ret;
  if( retry >= (ANZ_URBS - 1) ){
    all_urbs_finished = 1;
    return;
  }

  retry++;
  urbs.send_time[retry] = rt_timer_read();
  printk("RT-USB-PING: Send-Time URB[%d] (Callback): %llu \n",retry,urbs.send_time[urb_nr]);

  ret = rt_usb_submit_ctrl_urb( urbs.p_urb[retry],
                                USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                USB_REQ_GET_DESCRIPTOR,
                                USB_DT_CONFIG << 8 | 0x00,
                                0x0000,
                                len ,
                                urbs.p_urb[retry]->p_transfer_buffer);
  if( ret < 0){
    printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
    return;
  }
  printk("OK \n");

#endif

  return;
}

void periodic_fkt( void *p_data )
{
  int ret,i;

  struct urb_struct *p_urbs = (struct urb_struct *)p_data;

  len = p_urbs->p_urb[0]->p_usbdev->p_cfg_desc->wTotalLength;

  if(len <= 0){
    return;
  }

  RTIME tick_period;
  ret = rt_timer_start( TM_ONESHOT );
  if(ret){
    if(ret == -EBUSY){
      printk("RT-USB-PING: Timer is running\n");
      stop_timer = 0;

    } else {
      printk("RT-USB-PING: [ERROR] Failed to start Timer, code = %d \n",ret);
      return;
    }
  } else {
    printk("RT-USB-PING: Timer started\n");
    stop_timer = 1;
  }

  tick_period = rt_timer_ns2ticks( NS_PER_PERIOD );
  printk("RT-USB-PING: tick_period = %llu \n",tick_period);

  ret = rt_task_set_periodic ( &rtTask, TM_NOW, tick_period);
  if(ret){
    printk("RT-USB-PING: [ERROR] Failed to set periodic, code = %d\n",ret);
    return;
  }

#ifndef NEXT_URB_IN_CALLBACK

  while (retry < anz_retrys) {
    p_urbs->send_time[retry] = rt_timer_read();
    printk("RT-USB-PING: Retry %d/%d \n",retry,anz_retrys);

    // UNBLOCKING URB
#ifndef BLOCKING
    printk("RT-USB-PING: Send-Time URB[%d] (Callback): %llu \n",retry,p_urbs->send_time[retry]);
    ret = rt_usb_submit_ctrl_urb( p_urbs->p_urb[retry],
                                  USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                  USB_REQ_GET_DESCRIPTOR,
                                  USB_DT_CONFIG << 8 | 0x00,
                                  0x0000,
                                  len ,
                                  p_urbs->p_urb[retry]->p_transfer_buffer);
    if( ret < 0){
      printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
      return;
    }
    printk("OK \n");
#else // BLOCKING
    // BLOCKING URB
    p_urbs->p_urb[retry]->rt_complete_fkt = NULL;
    printk("RT-USB-PING: Send-Time URB[%d] (Semaphore): %llu \n",retry,p_urbs->send_time[retry]);
    ret = rt_usb_send_ctrl_msg( p_urbs->p_urb[retry],
                                  USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                  USB_REQ_GET_DESCRIPTOR,
                                  USB_DT_CONFIG << 8 | 0x00,
                                  0x0000,
                                  len ,
                                  p_urbs->p_urb[retry]->p_transfer_buffer );

    if(p_urbs->p_urb[retry]->actual_length < p_urbs->p_urb[retry]->transfer_buffer_length ){
      printk("RT-USB-PING: [ERROR] Received / Sent only %d / %d Byte \n",
             p_urbs->p_urb[retry]->actual_length,p_urbs->p_urb[retry]->transfer_buffer_length);
    }
    printk("OK \n");
    p_urbs->completed[retry] = 1;
    p_urbs->end_time[retry] = rt_timer_read();
    printk("RT-USB-PING: End-Time  URB[%d] (Semaphore): %llu \n",retry,p_urbs->end_time[retry]);

#endif // BLOCKING

    rt_task_wait_period();
    retry++;
  }

/* Waiting until all URBs completed */
    while(!all_urbs_finished ){
    rt_task_wait_period();

    all_urbs_finished = 1;
    for(i=0; i< anz_retrys; i++){
      if( !p_urbs->completed[i] ){
        all_urbs_finished = 0;
      }
    }
  }

#else //NEXT_URB_IN_CALLBACK

/* Sending the first URB, the other called by Interrupt-Handler */
  p_urbs->send_time[retry] = rt_timer_read();
  printk("RT-USB-PING: Send-Time URB[%d] (Callback): %llu \n",retry,p_urbs->send_time[retry]);
  ret = rt_usb_submit_ctrl_urb( p_urbs->p_urb[retry],
                                USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                USB_REQ_GET_DESCRIPTOR,
                                USB_DT_CONFIG << 8 | 0x00,
                                0x0000,
                                len ,
                                p_urbs->p_urb[retry]->p_transfer_buffer);
  if( ret < 0){
    printk("RT-USB-PING: [ERROR] Sending URB[%d] \n",retry);
    return;
  }

  printk("OK \n");
  int old_retry = retry;
  printk("Waiting for URB[%d] \n",retry);
  while ( !all_urbs_finished ) {

    if(old_retry != retry){
      printk("Waiting for URB[%d] \n",retry);
      old_retry = retry;
    }

    rt_task_wait_period();
  }
#endif //NEXT_URB_IN_CALLBACK

  /* calculating times */
  min = max = p_urbs->end_time[0] - p_urbs->send_time[0];
  sum = 0;

  for(i=0; i<anz_retrys; i++){
    if( !p_urbs->completed[i] ){
      continue;
    }
    value[i] = p_urbs->end_time[i] - p_urbs->send_time[i];
    if(value[i] > max) max = value[i];
    if(value[i] < min) min = value[i];
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
                            RTIME rt_sem_timeout){

  struct rt_urb *p_urb = kmalloc(sizeof(struct rt_urb),GFP_ATOMIC);
  if(!p_urb){
    printk("RT-USB-PING: [ERROR] No Memory for URB \n");
    return NULL;
  }
  alloc_bytes += sizeof(struct rt_urb);
  memset( p_urb, 0, sizeof(struct rt_urb) );

  struct usb_ctrlrequest *p_ctrl = kmalloc(sizeof(struct usb_ctrlrequest),GFP_ATOMIC);
  if(!p_ctrl){
    printk("RT-USB-PING: [ERROR] No Memory for Ctrl-Request \n");
    kfree(p_urb);
    alloc_bytes -= sizeof(struct rt_urb);
    return NULL;
  }
  alloc_bytes += sizeof(struct usb_ctrlrequest);
  memset( p_ctrl, 0, sizeof(struct usb_ctrlrequest) );

  unsigned char *p_buffer = kmalloc(MAX_BUFFER_SIZE,GFP_ATOMIC);
  if(!p_buffer){
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
#ifdef BLOCKING
  p_urb->rt_complete_fkt = NULL;
  p_urb->rt_sem_timeout = rt_sem_timeout;
#else
  p_urb->rt_complete_fkt = rt_callback_fkt;
  p_urb->rt_sem_timeout = 0;
#endif

  int ret = rt_sem_create( &p_urb->rt_sem, rt_sem_name ,0, S_PRIO);
  if(ret){
    if(ret == -EEXIST) printk("RT-USB-PING: [ERROR] RT-Semaphore: The name is already in use by some registered object \n");
    if(ret == -EINVAL) printk("RT-USB-PING: [ERROR] RT-Semaphore: The icount is non-zero and mode specifies a pulse semaphore \n");
    if(ret == -EPERM ) printk("RT-USB-PING: [ERROR] RT-Semaphore: This service was called from an asynchronous context \n");
    kfree(p_buffer);
    alloc_bytes -= MAX_BUFFER_SIZE;
    kfree(p_ctrl);
    alloc_bytes -= sizeof(struct usb_ctrlrequest);
    kfree(p_urb);
    alloc_bytes -= sizeof(struct rt_urb);
    return NULL;
  }

  //printk("RT-SEM @ 0x%p \n",&p_urb->rt_sem);
  return p_urb;
}

void destroy_urbs(void)
{
  int i;
  for(i=0;i< ANZ_URBS;i++){
    if(urbs.p_urb[i]){

      if(urbs.registered[i]){
        nrt_usb_unregister_urb(urbs.p_urb[i]);
      }
      if(urbs.rt_sem_created[i]){
        rt_sem_delete(&urbs.p_urb[i]->rt_sem);
      }

      if(urbs.p_urb[i]->p_setup_packet){
        kfree(urbs.p_urb[i]->p_setup_packet);
        alloc_bytes -= sizeof(struct usb_ctrlrequest);
      }
      if(urbs.p_urb[i]->p_transfer_buffer){
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
  for(i=0;i< ANZ_URBS;i++){

    urbs.p_urb[i] = create_urb( p_dev,
                                usb_rcvctrlpipe(p_dev,0),
                                URB_SHORT_NOT_OK,
                                rt_callback_fkt,
                                names[i],
                                500000000);
    if(!urbs.p_urb[i]){
      printk("RT-USB-PING: [ERROR] Creating URB[%d], code = %d \n",i,ret);
      destroy_urbs();
      return -1;
    }
    urbs.rt_sem_created[i] = 1;

    ret = nrt_usb_register_urb(urbs.p_urb[i]);
    if(ret){
      printk("RT-USB-PING: [ERROR] Register URB 0x%p failed, code = %d \n",urbs.p_urb[i],ret);
      destroy_urbs();
      return -1;
    }
    //printk("URB 0x%p registered \n",urbs.p_urb[i]);
    urbs.registered[i] = 1;

  }
  return 0;
}

int init_module(void)
{
  int ret;
  ret = rt_task_create ( &rtTask, "rt_usb_ping", 0, 1, 0 );
  if(ret){
    printk("RT-USB-PING: Failed to create Task, code = %d\n",ret);
    return -1;
  }

  p_device = NULL;
  p_device = rt_usb_get_device(vendor,product,p_device);
  if(!p_device){
    printk("RT-USB-PING: Sorry, Device not found ... \n");
    rt_task_delete(&rtTask);
    return -ENODEV;
  } else {
    printk("RT-USB-PING: HUB (Vendor = 0x%04x, Product = 0x%04x ) found ... \n",vendor,product);
    printk("RT-USB-PING: USB-Device @ 0x%p \n",p_device);
    printk("RT-USB-PING: Host-Controller @ 0x%p \n",p_device->p_hcd);
  }

  int i;
  for(i=0;i<ANZ_URBS;i++){
    urbs.p_urb[i] = NULL;
    urbs.registered[i] = 0;
    urbs.rt_sem_created[i] = 0;
    urbs.completed[i] = 0;
  }

  ret = create_urbs(p_device);
  if(ret){
    printk("RT-USB-PING: [ERROR] Creating URBs, code = %d \n",ret);
    rt_usb_put_device(p_device);
    return ret;
  }

  printk("RT-USB-PING: Starting Task \n");
  ret = rt_task_start( &rtTask, periodic_fkt, &urbs );
  if(ret){
    printk("RT-USB-PING: [ERROR] Failed to start Task, code = %d \n",ret);
    destroy_urbs();
    rt_usb_put_device(p_device);
    rt_task_delete(&rtTask);
    return -3;
  }

  printk("RT-USB-PING: STARTED, %d Byte allocted \n",alloc_bytes);
  return 0;
}

void cleanup_module(void)
{
  if(stop_timer){
    printk("RT-USB-PING: Stopping Timer \n");
    rt_timer_stop();
  }

  destroy_urbs();
  rt_usb_put_device(p_device);

  rt_task_delete(&rtTask);
  printk("RT-USB-PING: STOPPED, %d Byte allocted \n",alloc_bytes);
}

