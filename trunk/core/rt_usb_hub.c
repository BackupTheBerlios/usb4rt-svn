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

#include <linux/delay.h>

#include "rt_usb_hub.h"
#include "rt_usb_debug.h"

extern struct usb_device usb_dev[MAX_USB_DEV];
extern unsigned int alloc_bytes;

/* rt_usb */
extern struct usb_device *nrt_usb_config_dev( struct hc_device *p_hcd, __u8 rh_port_nr, unsigned int lowspeed);
extern int nrt_intern_usb_control_msg( struct rt_urb *p_urb, __u8 endpoint, __u8 request_type, __u8 request, __u16 wValue,
																			 __u16 wIndex, __u16 wLength, void *data);

int usb_init_hub( struct rt_urb *p_urb )
{
	if(!p_urb || ! p_urb->p_usbdev){
		return -1;
	}

	int i;

	hub_descriptor_t *p_hub_desc = kmalloc( sizeof(hub_descriptor_t),GFP_KERNEL);
	if(!p_hub_desc){
		return -ENOMEM;
	}

	alloc_bytes += sizeof(hub_descriptor_t);
	DBG2("RT-USBCORE: %d Byte allocated (%s %d Byte)\n", alloc_bytes, "+", sizeof(hub_descriptor_t) );

	memset( p_hub_desc,0,sizeof( hub_descriptor_t ) );
	p_urb->p_usbdev->p_hub_desc = p_hub_desc;

	DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," Hub-Descriptor saved @ 0x%p\n",p_hub_desc);
	if( hub_get_descriptor( p_urb, p_hub_desc ) ){
		kfree(p_hub_desc);
		alloc_bytes -= sizeof(hub_descriptor_t);
		DBG2("RT-USBCORE: %d Byte allocated (%s %d Byte)\n", alloc_bytes, "-", sizeof(hub_descriptor_t) );

		return -1;
	}

	dump_hub_descriptor( p_urb->p_usbdev );

	for(i=1; i<= p_hub_desc->bNbrPorts; i++) {
		if( hub_set_port_feature( p_urb, i, PORT_POWER) < 0 ){
			continue;
		}
	}

	/* Angabe in 2ms -Intervallen */
	udelay( p_hub_desc->bPwrOn2PwrGood * 2000);

	return(0);
}

struct usb_device *usb_poll_hub_port( struct rt_urb *p_urb, __u8 hub_port_nr )
{
	if(! p_urb || !p_urb->p_usbdev || !p_urb->p_usbdev->p_hcd){
		if (!p_urb)														ERR2("[ERROR] %s - Invalid URB-Pointer \n", __FUNCTION__);
		if (!p_urb->p_usbdev)									ERR2("[ERROR] %s - Invalid USB-Device-Pointer \n", __FUNCTION__);
		if (!p_urb->p_usbdev->p_hcd)					ERR2("[ERROR] %s - Invalid Host-Controller-Pointer \n", __FUNCTION__);
		return NULL;
	}

	struct usb_device *p_new_dev = NULL;
	portstat_t p_pstat;


	if (hub_get_portstat( p_urb, hub_port_nr, &p_pstat)){
		ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Get Portstat for Port[%d] failed \n",__FUNCTION__, hub_port_nr);
		return NULL;
	}

	dump_portstatus( p_urb->p_usbdev, hub_port_nr, &p_pstat);

	if( p_pstat.change.c_port_connection ) {	/* Connection detect */
		if( hub_clear_port_feature( p_urb , hub_port_nr, C_PORT_CONNECTION) ){
			return NULL;
		}
		if( p_pstat.stat.port_connection){

			hub_port_resume( p_urb ,hub_port_nr);
			hub_port_reset( p_urb ,hub_port_nr);
			udelay(10);

			// unregister hub_urb
			struct rt_urb *p_urb_save = p_urb;
			struct usb_device *p_usbdev_save = p_urb->p_usbdev;
			nrt_usb_unregister_urb(p_urb);

			p_new_dev = nrt_usb_config_dev( p_urb->p_hcd, p_urb->p_usbdev->rh_port, p_pstat.stat.port_lowspeed);

			// re-register hub_urb
			p_urb = p_urb_save;
			p_urb->p_usbdev = p_usbdev_save;
			p_urb->p_hcd = p_usbdev_save->p_hcd;
			nrt_usb_register_urb(p_urb);

			return p_new_dev;

		}

		hub_set_port_feature(   p_urb, hub_port_nr, PORT_SUSPEND );
		hub_clear_port_feature( p_urb, hub_port_nr, PORT_ENABLE  );

	}

	return NULL;
}

int hub_set_port_feature( struct rt_urb *p_urb, __u16 port_nr, __u8 feature)
{
	if(!p_urb){
		return -ENODEV;
	}

	int ret = 0;
	DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev,"%02d : Set Feature 0x%02x\n", port_nr, feature);

	ret = nrt_intern_usb_control_msg( p_urb,
																		0x00,
																		USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_OTHER,
																		USB_REQ_SET_FEATURE,
																		feature,
																		port_nr,      // port
																		0x0000,   // fix
																		NULL);

	if(ret < 0){
		ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Port %02d : Set Feature 0x%02x FAILED\n",__FUNCTION__, port_nr, feature);

		return -1;
	}
	return 0;
}

int hub_clear_port_feature( struct rt_urb *p_urb, __u16 port_nr, __u8 feature)
{
	int ret;

	DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev,"%02d : Clear Feature 0x%02x\n",port_nr, feature);

	ret = nrt_intern_usb_control_msg( p_urb,
																		0x00,
																		USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_OTHER,
																		USB_REQ_CLEAR_FEATURE,
																		feature,
																		port_nr,
																		0x0000,
																		NULL);

	if(ret < 0){
		DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Port %02d : Clear Feature 0x%02x FAILED\n",__FUNCTION__,port_nr, feature);

		return -1;
	}
	return 0;
}

int hub_port_reset( struct rt_urb *p_urb, __u16 port_nr)
{
	int tries = 100;
	int us_delay = 10000;
	portstat_t pstat;

	if( hub_set_port_feature( p_urb, port_nr, PORT_RESET) ){
		return -1;
	}

	do{
		udelay(us_delay);
		if( hub_get_portstat( p_urb, port_nr, &pstat) ){
			return -1;
		}
	} while( tries-- && !pstat.change.c_port_reset );

	if(!tries){
		return(-1);
	}

	hub_clear_port_feature( p_urb, port_nr, C_PORT_RESET);
	return 0;
}

int hub_port_resume( struct rt_urb *p_urb, __u16 port_nr)
{
	int tries = 100;
	int us_delay = 10000;
	portstat_t pstat;

	if( hub_get_portstat( p_urb, port_nr, &pstat) ){
		return -1;
	}

	if( !pstat.stat.port_suspend ){
		return 0;
	}

	if( hub_clear_port_feature( p_urb, port_nr, PORT_SUSPEND) ){
		return -1;
	}

	do{
		udelay(us_delay);
		if( hub_get_portstat( p_urb, port_nr, &pstat) ){
			return -1;
		}
	} while( tries-- && !pstat.change.c_port_suspend );

	if(!tries){
		return(-1);
	}

	hub_clear_port_feature( p_urb, port_nr, C_PORT_SUSPEND);
	return 0;
}

int hub_get_portstat( struct rt_urb *p_urb, __u16 port_nr, portstat_t *p_stat )
{
	int ret;

	DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev,"%02d : Get Port-Status\n",port_nr);

	ret = nrt_intern_usb_control_msg( p_urb,
																		0x00,
																		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_OTHER,
																		USB_REQ_GET_STATUS,
																		0x0000,     // fix Value
																		port_nr,    // port
																		0x0004,     // fix length
																		p_stat);

	if(ret < 0x0004){
		DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Port %02d : Get Port-Status FAILED\n",__FUNCTION__, port_nr);

		return -1;
	}
	return 0;
}

int hub_get_descriptor( struct rt_urb *p_urb, hub_descriptor_t *p_desc)
{
	int ret = 0;

	ret = nrt_intern_usb_control_msg( p_urb,
																		0x00,
																		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_DEVICE,
																		USB_REQ_GET_DESCRIPTOR,
																		USB_DT_HUB,   // Value
																		0x0000,   // Index
																		0x0002,   // length
																		p_desc);

	if(ret < 0x0002){
		return -1;
	}

	ret = nrt_intern_usb_control_msg( p_urb,
																		0x00,
																		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_DEVICE,
																		USB_REQ_GET_DESCRIPTOR,
																		USB_DT_HUB,
																		0x0000,
																		p_desc->bLength,
																		p_desc);

	if(ret < p_desc->bLength){
		return -1;
	}
	return 0;
}
