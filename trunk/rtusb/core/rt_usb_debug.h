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

#ifndef RT_USB_DEBUG_H
#define RT_USB_DEBUG_H

#include "rt_usb.h"
#include "rt_usb_hub.h"  /* portstat_t */

//#define TD_DEBUG
#undef TD_DEBUG
//#define QH_DEBUG
#undef QH_DEBUG
#define DEBUG
//#undef DEBUG
#define ERROR
//#undef ERROR

#define PRNT(txt, args...)                         printk(txt, ##args); /* TD debug messages */

# define INFO(txt, args...)                        PRNT(txt, ##args);
# define INFO_MSG1( p_hcd, txt, args...)           PRNT("%03d:" txt, (p_hcd)->hcd_nr, ##args);
# define INFO_MSG2( p_hcd, p_usbdev, txt, args...) PRNT("%03d:%02d-%03d:" txt,\
                                                       (p_hcd)->hcd_nr, \
                                                       (p_usbdev)->rh_port,\
                                                       (p_usbdev)->address, ##args);

#ifdef TD_DEBUG
# define TD_DBG(txt, args...)                      PRNT(txt, ##args); /* TD debug messages */
# define TD_MSG1( p_hcd, txt, args...)             PRNT("%03d:" txt, (p_hcd)->hcd_nr, ##args);
# define TD_ERR(txt, args...)                      PRNT(txt, ##args); /* error messages */
#else
# define TD_DBG(txt, args...)
# define TD_MSG1( p_hcd, txt, args...)
# define TD_ERR(txt, args...)
#endif

#ifdef QH_DEBUG
# define QH_DBG(txt, args...)                      PRNT(txt, ##args); /* QH debug messages */
# define QH_ERR(txt, args...)                      PRNT(txt, ##args); /* error messages */
#else
# define QH_DBG(txt, args...)
# define QH_ERR(txt, args...)
#endif

#ifdef DEBUG
# define DBG(txt, args...)                         PRNT(txt, ##args);
# define DBG_MSG1( p_hcd, txt, args...)            PRNT("%03d:" txt, (p_hcd)->hcd_nr, ##args);
# define DBG_MSG2( p_hcd, p_usbdev, txt, args...)  PRNT("%03d:%02d-%03d:" txt,\
                                                       (p_hcd)->hcd_nr, \
                                                       (p_usbdev)->rh_port,\
                                                       (p_usbdev)->address, ##args);
#else
# define DBG(txt, args...)
# define DBG_MSG1(p_hcd,txt,args...)
# define DBG_MSG2(p_hcd,p_usbdev,txt,args...)
#endif

#ifdef ERROR
# define ERR2(txt, args...)                         PRNT(txt, ##args); /* error messages */
# define ERR_MSG1(p_hcd,txt,args...)                PRNT("%03d: [ERROR]" txt, (p_hcd)->hcd_nr, ##args);
# define ERR_MSG2(p_hcd,p_usbdev,txt,args...)       PRNT("%03d:%02d-%03d: [ERROR]" txt,\
                                                        (p_hcd)->hcd_nr, \
                                                        (p_usbdev)->rh_port,\
                                                        (p_usbdev)->address, ##args);
#else
# define ERR2(txt, args...)
# define ERR_MSG1(p_hcd,txt,args...)
# define ERR_MSG2(p_hcd,p_usbdev,txt,args...)
#endif

void dump_device_descriptor( struct usb_device *p_usbdev, struct usb_device_descriptor *p_desc );
void dump_complete_descriptor( struct usb_device *p_usbdev );
void dump_configuration_descriptor( struct usb_device *p_usbdev );
void dump_interface_descriptor( struct usb_device *p_usbdev, struct usb_interface_descriptor *p_desc );
void dump_endpoint_descriptor( struct usb_device *p_usbdev, struct usb_endpoint_descriptor *p_desc );
void dump_hid_descriptor( struct usb_device *p_usbdev, hid_descriptor_t *p_desc );
void dump_hub_descriptor( struct usb_device *p_usbdev );
void dump_portstatus( struct usb_device *p_usbdev, __u8 port, portstat_t *p_desc );
void dump_usb_device( struct usb_device *p_usbdev );
void list_usb_devices( void );
void dump_urb( struct rt_urb *p_urb );

#endif
