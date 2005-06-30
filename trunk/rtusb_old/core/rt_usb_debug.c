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

#include "rt_usb_core.h"
#include "rt_usb_debug.h"

extern struct usb_device usb_dev[MAX_USB_DEV];

void dump_portstatus( struct usb_device *p_usbdev, __u8 port, portstat_t *p_desc )
{
  if(!p_desc){
    return;
  }

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : *** PORTSTATUS-INFO *** \n",port);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Connection   : %x \n",port,p_desc->stat.port_connection);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Enable       : %x \n",port,p_desc->stat.port_enable);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Suspend      : %x \n",port,p_desc->stat.port_suspend);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Over-Current : %x \n",port,p_desc->stat.port_over_current);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Reset        : %x \n",port,p_desc->stat.port_reset);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Power        : %x \n",port,p_desc->stat.port_power);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Lowspeed     : %x \n",port,p_desc->stat.port_lowspeed);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Highspeed    : %x \n",port,p_desc->stat.port_highspeed);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Test         : %x \n",port,p_desc->stat.port_test);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Indicator    : %x \n",port,p_desc->stat.port_indicator);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d -- Change- Bits---------\n",port);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Connection   : %x \n",port,p_desc->change.c_port_connection);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Enable       : %x \n",port,p_desc->change.c_port_enable);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Suspend      : %x \n",port,p_desc->change.c_port_suspend);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Over-Current : %x \n",port,p_desc->change.c_port_over_current);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev,"%02d : Reset        : %x \n",port,p_desc->change.c_port_reset);
}

void dump_device_descriptor( struct usb_device *p_usbdev, struct usb_device_descriptor *p_desc )
{
	if(!p_desc || !p_usbdev){
    return;
  }

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** DEVICE DESCRIPTOR -INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length        : 0x%02x (%d Bytes)\n",p_desc->bLength,p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type          : 0x%02x \n",p_desc->bDescriptorType);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," USB-Version   : 0x%04x \n",le16_to_cpu(p_desc->bcdUSB));
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Class         : 0x%02x \n",p_desc->bDeviceClass);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," SubClass      : 0x%02x \n",p_desc->bDeviceSubClass);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Protocol      : 0x%02x \n",p_desc->bDeviceProtocol);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," MaxPacket     : 0x%02x \n",p_desc->bMaxPacketSize0);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Vendor-ID     : 0x%04x \n",le16_to_cpu(p_desc->idVendor));
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Product-ID    : 0x%04x \n",le16_to_cpu(p_desc->idProduct));
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Device        : 0x%04x \n",le16_to_cpu(p_desc->bcdDevice));
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Manuf-Nr      : 0x%02x \n",p_desc->iManufacturer);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Product-Nr    : 0x%02x \n",p_desc->iProduct);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Serial-Nr     : 0x%02x \n",p_desc->iSerialNumber);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Num Config    : 0x%02x \n",p_desc->bNumConfigurations);

  return;
}

void dump_configuration_descriptor( struct usb_device *p_usbdev )
{
	struct usb_config_descriptor *p_desc = p_usbdev->p_cfg_desc;
  if(!p_desc){
    return;
  }

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** CONFIG-INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length        : 0x%02x (%d Bytes)\n",p_desc->bLength,p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type          : 0x%02x \n",p_desc->bDescriptorType);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," TotalLength   : 0x%04x (%d Bytes)\n",p_desc->wTotalLength,p_desc->wTotalLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Num Interf    : 0x%02x \n",p_desc->bNumInterfaces);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Conf Value    : 0x%02x \n",p_desc->bConfigurationValue);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Config        : 0x%02x \n",p_desc->iConfiguration);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Attributes    : 0x%02x -" ,p_desc->bmAttributes);
	if(p_desc->bmAttributes & 0x20 ) DBG(" Remote-Wakeup");
	if(p_desc->bmAttributes & 0x40 ) DBG(" Self-Powered");
	if(p_desc->bmAttributes & 0x80 ) DBG(" Bus-Powered");
	DBG("\n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Max Power     : %d mA \n", 2 * p_desc->bMaxPower);
  return;
}

void dump_interface_descriptor( struct usb_device *p_usbdev, struct usb_interface_descriptor *p_desc )
{
	if(!p_desc){
		return;
	}

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** INTERFACE - INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length        : 0x%02x (%d Bytes)\n",p_desc->bLength,p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type          : 0x%02x \n",p_desc->bDescriptorType);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Interface-Nr  : 0x%02x \n",p_desc->bInterfaceNumber);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Alternate Set : 0x%02x \n",p_desc->bAlternateSetting);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Num Endpoints : 0x%02x \n",p_desc->bNumEndpoints);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Class-Code    : 0x%02x \n",p_desc->bInterfaceClass);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Subclass-Code : 0x%02x \n",p_desc->bInterfaceSubClass);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Protocol-Code : 0x%02x \n",p_desc->bInterfaceProtocol);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Interface-Str : 0x%02x \n",p_desc->iInterface);
	return;
}

void dump_endpoint_descriptor( struct usb_device *p_usbdev, struct usb_endpoint_descriptor *p_desc )
{
	if(!p_desc){
		return;
	}

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** ENDPOINT - INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length        : 0x%02x (%d Bytes)\n",p_desc->bLength,p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type          : 0x%02x \n",p_desc->bDescriptorType);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Endpoint-Nr   : 0x%02x - ",p_desc->bEndpointAddress);
	if( (p_desc->bEndpointAddress & 0x80))	DBG("IN-EP \n");
	if(!(p_desc->bEndpointAddress & 0x80))	DBG("OUT-EP \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Attributes    : 0x%02x - ",p_desc->bmAttributes);
	if( p_desc->bmAttributes == 0x00 )			DBG("Control-Transfer \n");
	if( p_desc->bmAttributes == 0x01)				DBG("Isochronous-Transfer \n");
	if( p_desc->bmAttributes == 0x02)				DBG("Bulk-Transfer \n");
	if( p_desc->bmAttributes == 0x03)				DBG("Interrupt-Transfer \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Max Packetsize: %d Bytes \n",p_desc->wMaxPacketSize);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Interval      : %d ms \n",p_desc->bInterval);
	return;
}

void dump_hid_descriptor( struct usb_device *p_usbdev, hid_descriptor_t *p_desc )
{
	if(!p_desc){
		return;
	}

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** HID - INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length        : 0x%02x (%d Bytes)\n",p_desc->bLength,p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type          : 0x%02x \n",p_desc->type);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," BCD-HID       : 0x%04x \n",p_desc->bcdHID);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Country-Code  : 0x%02x \n",p_desc->bCountryCode);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Num Descript. : 0x%02x \n",p_desc->bNumDescriptors);
	return;
}

void dump_hub_descriptor( struct usb_device *p_usbdev )
{
	hub_descriptor_t *p_desc = p_usbdev->p_hub_desc;
	if(!p_desc){
		return;
	}

	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," *** HUB DESCRIPTOR -INFO *** \n");
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Length     : 0x%02x \n",p_desc->bLength);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Type       : 0x%02x \n",p_desc->type);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Num Ports  : %d 		\n",p_desc->bNbrPorts);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Character. : 0x%04x \n",p_desc->wHubCharacteristics);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Time2Power : %d ms 	\n",p_desc->bPwrOn2PwrGood);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," max Current: %d mA 	\n",p_desc->bHubCntrCurrent);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," RemoveMask : 0x%02x \n",p_desc->DeviceRemovable);
	DBG_MSG2(p_usbdev->p_hcd,p_usbdev," --- Characteristics ---- \n");
	if( (p_desc->wHubCharacteristics & 0x0003) == 0x00 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Spannungsversorgung nur gleichzeitig schaltbar\n");
	if( (p_desc->wHubCharacteristics & 0x0003) == 0x01 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Spannungsversorgung pro Port schaltbar\n");
	if(!(p_desc->wHubCharacteristics & 0x0004) )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Hub ist kein Bestandteil eines Compound-Device\n");
	if(  p_desc->wHubCharacteristics & 0x0004 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Hub ist Bestandteil eines Compound-Device\n");
	if( (p_desc->wHubCharacteristics & 0x0018) >> 3 == 0x00 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Globaler Ueberstromschutz\n");
	if( (p_desc->wHubCharacteristics & 0x0018) >> 3 == 0x01 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Selektiver Ueberstromschutz\n");
	if( (p_desc->wHubCharacteristics & 0x0018) >> 3 == 0x20 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Kein Ueberstromschutz\n");
	if( (p_desc->wHubCharacteristics & 0x0060) >> 5 == 0x00 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," TT-Think-Time 8 FS-Taktzyklen\n");
	if( (p_desc->wHubCharacteristics & 0x0060) >> 5 == 0x01 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," TT-Think-Time 16 FS-Taktzyklen\n");
	if( (p_desc->wHubCharacteristics & 0x0060) >> 5 == 0x02 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," TT-Think-Time 26 FS-Taktzyklen\n");
	if( (p_desc->wHubCharacteristics & 0x0060) >> 5 == 0x03 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," TT-Think-Time 32 FS-Taktzyklen\n");
	if(!(p_desc->wHubCharacteristics & 0x0080))
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Port-Indikatoren werden nicht unterstuetzt\n");
	if(  p_desc->wHubCharacteristics & 0x0080 )
		DBG_MSG2(p_usbdev->p_hcd,p_usbdev," Port-Indikatoren werden unterstuetzt\n");

  return;
}

void list_usb_devices( void )
{
	int i;
	struct usb_device *p_dev;

	INFO("--- LIST USB-DEVICES -------------------------\n");
	INFO(" No HCD VENDOR   PROD  CLS SCLS PROT ->CTRL CTRL-> ->BULK BULK->  ->INT  INT-> ->ISOC ISOC-> STATE SPEED USED\n");
	for(i=1;i<MAX_USB_DEV;i++){
		p_dev = &usb_dev[i];
		if(!p_dev->p_hcd){
			continue;
		}
		INFO("%03d %03d 0x%04x 0x%04x 0x%02x 0x%02x 0x%02x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x %05d %05d %s\n",
				i,p_dev->p_hcd->hcd_nr,p_dev->vendor,p_dev->product,p_dev->class,p_dev->subclass,p_dev->protocol,
				p_dev->ctrl_mask[0],p_dev->ctrl_mask[1],p_dev->bulk_mask[0],p_dev->bulk_mask[1],
				p_dev->int_mask[0],p_dev->int_mask[1],p_dev->iso_mask[0],p_dev->iso_mask[1],p_dev->state,p_dev->speed,
				p_dev->in_use ? "   X" : "   -");
	}
	INFO("--- END USB-DEVICES --------------------------\n");
}

void dump_usb_device( struct usb_device *p_usbdev )
{
	INFO("--- DUMP USB-DEVICE @ %p -------------------------\n",p_usbdev);
	INFO(" Number          : %d    \n",p_usbdev->address);
	INFO(" Vendor          : 0x%04x\n",p_usbdev->vendor);
	INFO(" Product         : 0x%04x\n",p_usbdev->product);
	INFO(" Root-Hub-Port   : 0x%02x\n",p_usbdev->rh_port);
	INFO(" Class           : 0x%02x\n",p_usbdev->class);
	INFO(" Subclass        : 0x%02x\n",p_usbdev->subclass);
	INFO(" Protocol        : 0x%02x\n",p_usbdev->protocol);
	INFO(" In use          : %s    \n",p_usbdev->in_use ? "yes" : "no" );
	INFO(" Status          : 0x%02x\n",p_usbdev->state);
	INFO(" Speed           : 0x%02x\n",p_usbdev->speed);
	INFO(" Host-Controller @ 0x%p  \n",p_usbdev->p_hcd);
	INFO(" \n");
	INFO(" Endpoint         ---0 ---1 ---2 ---3 ---4 ---5 ---6 ---7 ---8 ---9 --10 --11 --12 --13 --14 --15 \n");
	INFO(" Ctrl-Mask IN        %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->ctrl_mask[0] & ( 1 <<  0) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 <<  2) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 <<  4) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 <<  6) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 <<  8) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 << 10) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 << 11) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 << 12) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 << 13) ? "X" : "-",
			p_usbdev->ctrl_mask[0] & ( 1 << 14) ? "X" : "-", p_usbdev->ctrl_mask[0] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Ctrl-Mask OUT       %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->ctrl_mask[1] & ( 1 <<  0) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 <<  2) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 <<  4) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 <<  6) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 <<  8) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 << 10) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 << 11) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 << 12) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 << 13) ? "X" : "-",
			p_usbdev->ctrl_mask[1] & ( 1 << 14) ? "X" : "-", p_usbdev->ctrl_mask[1] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Bulk-Mask IN        %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->bulk_mask[0] & ( 1 <<  0) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 <<  2) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 <<  4) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 <<  6) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 <<  8) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 << 10) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 << 11) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 << 12) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 << 13) ? "X" : "-",
			p_usbdev->bulk_mask[0] & ( 1 << 14) ? "X" : "-", p_usbdev->bulk_mask[0] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Bulk-Mask OUT       %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->bulk_mask[1] & ( 1 <<  0) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 <<  2) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 <<  4) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 <<  6) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 <<  8) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 << 10) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 << 11) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 << 12) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 << 13) ? "X" : "-",
			p_usbdev->bulk_mask[1] & ( 1 << 14) ? "X" : "-", p_usbdev->bulk_mask[1] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Int-Mask IN         %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->int_mask[0] & ( 1 <<  0) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 <<  2) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 <<  4) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 <<  6) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 <<  8) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 << 10) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 << 11) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 << 12) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 << 13) ? "X" : "-",
			p_usbdev->int_mask[0] & ( 1 << 14) ? "X" : "-", p_usbdev->int_mask[0] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Int-Mask OUT        %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->int_mask[1] & ( 1 <<  0) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 <<  2) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 <<  4) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 <<  6) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 <<  8) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 << 10) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 << 11) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 << 12) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 << 13) ? "X" : "-",
			p_usbdev->int_mask[1] & ( 1 << 14) ? "X" : "-", p_usbdev->int_mask[1] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Iso-Mask IN         %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->iso_mask[0] & ( 1 <<  0) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 <<  2) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 <<  4) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 <<  6) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 <<  8) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 << 10) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 << 11) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 << 12) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 << 13) ? "X" : "-",
			p_usbdev->iso_mask[0] & ( 1 << 14) ? "X" : "-", p_usbdev->iso_mask[0] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Iso-Mask OUT        %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->iso_mask[1] & ( 1 <<  0) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 <<  2) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 <<  4) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 <<  6) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 <<  8) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 << 10) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 << 11) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 << 12) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 << 13) ? "X" : "-",
			p_usbdev->iso_mask[1] & ( 1 << 14) ? "X" : "-", p_usbdev->iso_mask[1] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Toggle-Mask IN      %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->toggle_mask[0] & ( 1 <<  0) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 <<  2) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 <<  4) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 <<  6) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 <<  8) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 << 10) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 << 11) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 << 12) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 << 13) ? "X" : "-",
			p_usbdev->toggle_mask[0] & ( 1 << 14) ? "X" : "-", p_usbdev->toggle_mask[0] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Toggle-Mask OUT     %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s    %s",
			p_usbdev->toggle_mask[1] & ( 1 <<  0) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 <<  1) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 <<  2) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 <<  3) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 <<  4) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 <<  5) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 <<  6) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 <<  7) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 <<  8) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 <<  9) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 << 10) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 << 11) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 << 12) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 << 13) ? "X" : "-",
			p_usbdev->toggle_mask[1] & ( 1 << 14) ? "X" : "-", p_usbdev->toggle_mask[1] & ( 1 << 15) ? "X\n" : "-\n"
		 );

	INFO(" Max-Packet IN    %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\n",
			p_usbdev->epmaxpacketin[ 0],p_usbdev->epmaxpacketin[ 1],p_usbdev->epmaxpacketin[ 2],p_usbdev->epmaxpacketin[ 3],
			p_usbdev->epmaxpacketin[ 4],p_usbdev->epmaxpacketin[ 5],p_usbdev->epmaxpacketin[ 6],p_usbdev->epmaxpacketin[ 7],
			p_usbdev->epmaxpacketin[ 8],p_usbdev->epmaxpacketin[ 9],p_usbdev->epmaxpacketin[10],p_usbdev->epmaxpacketin[11],
			p_usbdev->epmaxpacketin[12],p_usbdev->epmaxpacketin[13],p_usbdev->epmaxpacketin[14],p_usbdev->epmaxpacketin[15]);

	INFO(" Max-Packet OUT   %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\n",
			p_usbdev->epmaxpacketout[ 0],p_usbdev->epmaxpacketout[ 1],p_usbdev->epmaxpacketout[ 2],p_usbdev->epmaxpacketout[ 3],
			p_usbdev->epmaxpacketout[ 4],p_usbdev->epmaxpacketout[ 5],p_usbdev->epmaxpacketout[ 6],p_usbdev->epmaxpacketout[ 7],
			p_usbdev->epmaxpacketout[ 8],p_usbdev->epmaxpacketout[ 9],p_usbdev->epmaxpacketout[10],p_usbdev->epmaxpacketout[11],
			p_usbdev->epmaxpacketout[12],p_usbdev->epmaxpacketout[13],p_usbdev->epmaxpacketout[14],p_usbdev->epmaxpacketout[15]);
	/*
	hub_descriptor_t *p_hub_desc;
	struct usb_config_descriptor *p_cfg_desc;
	struct usb_device_descriptor *p_dev_desc;
	__u32 halted[2];
	*/
	INFO("--- END USB-DEVICE --------------------------\n");
}

void dump_urb( struct rt_urb *p_urb )
{
	if(!p_urb){
		return;
	}

	DBG("========== DUMP URB 0x%p ======================\n",p_urb);
	DBG(" Host-Controller   @ 0x%p\n",p_urb->p_hcd);
	DBG(" Max Buffer-Length : %d Byte\n",p_urb->max_buffer_len);
	DBG(" Max Packet-Size   : %d Byte\n",p_urb->max_packet_size);
	DBG(" \n");
	DBG(" USB-Device        @ 0x%p\n",p_urb->p_usbdev);
	DBG(" Pipe              : %d \n",p_urb->pipe);
	DBG(" Flags             : %d \n",p_urb->transfer_flags);
	DBG(" Transfer-Buffer   @ 0x%p\n",p_urb->p_transfer_buffer);
	DBG(" Transfer-DMA      @ 0x%p\n",(void *)p_urb->transfer_dma);
	DBG(" Transf-Buff-Length: %d Byte\n",p_urb->transfer_buffer_length);
	DBG(" Ctrl-Request-Pack @ 0x%p\n",p_urb->p_setup_packet);
	DBG(" Ctrl-Request-DMA  @ 0x%p\n",(void *)p_urb->setup_dma);
	DBG(" Private           @ 0x%p\n",p_urb->p_private);
	DBG(" RT-Complete-Funct @ 0x%p\n",p_urb->rt_complete_fkt);
	DBG(" Status            : %d\n",p_urb->status);
	DBG(" Actual Length     : %d Byte \n",p_urb->actual_length);
	DBG("======================================================\n");

}
