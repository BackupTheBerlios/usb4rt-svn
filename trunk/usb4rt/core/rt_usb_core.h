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
#ifndef RT_USB_CORE_H
#define RT_USB_CORE_H

#include <core/rt_usb.h>

#define MAX_USB_DEV           128 /* 7 bit */
#define MAX_EP                16  /* 4 bit */
#define MAX_CTRL_RETRIES      3
#define MAX_CTRL_STS_RETRIES  0
#define MAX_HUBS_TO_POLL      10
#define UHC_RH_MAXCHILD       8

/* Device-Descriptor Types */
#define USB_DT_HID            0x2100
#define USB_DT_REPORT         0x2200
#define USB_DT_PHYSICAL       0x2300
#define USB_DT_HUB            0x2900

/* ERROR-Flags */
#define ERR_FLAG_INVALID_TYPE 0x01
#define ERR_FLAG_INIT_HCD     0x02

/* URB-SUBMIT-FLAGS */
#define URB_CALLBACK          0x0001
#define URB_WAIT_SEM          0x0002
#define URB_WAIT_BUSY         0x0004

struct hcd_funktions {
  /* Host Controller Functions called by Core */
  struct usb_device *(*  nrt_hcd_poll_root_hub_port) ( struct hc_device *p_hcd, __u8 rh_port_nr);
  int(* nrt_hcd_register_urb)  ( struct rt_urb *p_urb);
  int(* nrt_hcd_unregister_urb)( struct rt_urb *p_urb);
  int(*  rt_hcd_submit_urb)    ( struct rt_urb *p_urb, __u16 urb_submit_flags);

  /* Core-Functions called by Host Controller Driver */
  struct usb_device *(* nrt_usb_config_dev)    ( struct hc_device *p_hcd, __u8 rh_port_nr, unsigned int lowspeed);
  void(* nrt_usb_search_devices)( struct hc_device *p_hcd );
};

struct hc_device{
  /* Access by Host Controller */
  __u8 type;
  __u8 rh_numports;
  void *p_private;

  /* Access by Core */
  struct list_head usb_ctrl_list;
  atomic_t use_counter;
  __u8 hcd_nr;

  /* Common use */
  struct hcd_funktions *p_hcd_fkt ;
};

struct hub_device{
  __u8 configured;
  struct usb_device *p_usbdev;
  struct list_head hub_list;
};

/*-----------------*/
/*  Driver API     */
/*-----------------*/
struct rt_urb *nrt_usb_create_ctrl_callback_urb( rt_usb_complete_t rt_callback_fkt );
void nrt_usb_destroy_ctrl_callback_urb( struct rt_urb *p_urb );

struct rt_urb *nrt_usb_create_callback_urb( rt_usb_complete_t rt_callback_fkt );
void nrt_usb_destroy_callback_urb( struct rt_urb *p_urb );

struct rt_urb *nrt_usb_create_ctrl_semaphore_urb( unsigned char *name , RTIME rt_sem_timeout );
void nrt_usb_destroy_ctrl_semaphore_urb( struct rt_urb *p_urb );

struct rt_urb *nrt_usb_create_semaphore_urb( unsigned char *name , RTIME rt_sem_timeout );
void nrt_usb_destroy_semaphore_urb( struct rt_urb *p_urb );

struct rt_urb *nrt_usb_alloc_urb( void );
void nrt_usb_free_urb( struct rt_urb *p_urb );

struct rt_urb *nrt_usb_alloc_ctrl_urb( void );
void nrt_usb_free_ctrl_urb( struct rt_urb *p_urb );

int nrt_usb_register_urb(struct rt_urb *p_urb);
int nrt_usb_unregister_urb(struct rt_urb *p_urb);

struct usb_device *rt_usb_get_device(__u16 vendor, __u16 product ,struct usb_device *p_old_device);
void rt_usb_put_device( struct usb_device *p_usbdev );

int nrt_usb_clear_stall( struct rt_urb *p_urb );

/*--------------------------*/
/*   Synchronous Messages   */
/*--------------------------*/
int rt_usb_send_urb_msg( struct rt_urb *p_urb);
int rt_usb_send_ctrl_msg(struct rt_urb *p_urb, __u8 request_type,__u8 request, __u16 wValue, __u16 wIndex, __u16 wLength , void *data);

/*--------------------------*/
/*   Asynchronous Messages  */
/*--------------------------*/
int rt_usb_submit_urb( struct rt_urb *p_urb);
int rt_usb_submit_ctrl_urb( struct rt_urb *p_urb,__u8 request_type,__u8 request, __u16 wValue, __u16 wIndex, __u16 wLength , void *data);

/*--------------------------*/
/*   Host-Controller API    */
/*--------------------------*/
int nrt_hcd_register_driver( struct hc_device *p_hcd );
int nrt_hcd_unregister_driver( struct hc_device *p_hcd );

EXPORT_SYMBOL(nrt_usb_create_ctrl_callback_urb);
EXPORT_SYMBOL(nrt_usb_destroy_ctrl_callback_urb);

EXPORT_SYMBOL(nrt_usb_create_callback_urb);
EXPORT_SYMBOL(nrt_usb_destroy_callback_urb);

EXPORT_SYMBOL(nrt_usb_create_ctrl_semaphore_urb);
EXPORT_SYMBOL(nrt_usb_destroy_ctrl_semaphore_urb);

EXPORT_SYMBOL(nrt_usb_create_semaphore_urb);
EXPORT_SYMBOL(nrt_usb_destroy_semaphore_urb);

EXPORT_SYMBOL(nrt_usb_alloc_urb);
EXPORT_SYMBOL(nrt_usb_free_urb);

EXPORT_SYMBOL(nrt_usb_alloc_ctrl_urb);
EXPORT_SYMBOL(nrt_usb_free_ctrl_urb);

EXPORT_SYMBOL(nrt_usb_register_urb);
EXPORT_SYMBOL(nrt_usb_unregister_urb);

EXPORT_SYMBOL(nrt_usb_clear_stall);

EXPORT_SYMBOL(rt_usb_get_device);
EXPORT_SYMBOL(rt_usb_put_device);
EXPORT_SYMBOL(rt_usb_send_urb_msg);
EXPORT_SYMBOL(rt_usb_send_ctrl_msg);
EXPORT_SYMBOL(rt_usb_submit_urb);
EXPORT_SYMBOL(rt_usb_submit_ctrl_urb);

EXPORT_SYMBOL(nrt_hcd_register_driver);
EXPORT_SYMBOL(nrt_hcd_unregister_driver);

#endif
