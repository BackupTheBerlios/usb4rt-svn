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

#ifndef RT_USB_HUB_H
#define RT_USB_HUB_H

#include <core/rt_usb_core.h>

 // some port features
#define PORT_CONNECTION         0x00
#define PORT_ENABLE             0x01
#define PORT_SUSPEND            0x02
#define PORT_OVER_CURRENT       0x03
#define PORT_RESET              0x04
#define PORT_POWER              0x08
#define PORT_LOW_SPEED          0x09
#define PORT_HIGH_SPEED         0x0a
#define PORT_TEST               0x0b
#define PORT_INDICATOR          0x0c

#define C_PORT_CONNECTION       0x01
#define C_PORT_ENABLE           0x02
#define C_PORT_SUSPEND          0x04
#define C_PORT_OVER_CURRENT     0x13
#define C_PORT_RESET            0x14

typedef struct port_charge {
  unsigned short c_port_connection:1;   // C_PORT_CONNECTION
  unsigned short c_port_enable:1;       // C_PORT_ENABLE
  unsigned short c_port_suspend:1;      // C_PORT_SUSPEND
  unsigned short c_port_over_current:1; // C_PORT_OVER_CURRENT
  unsigned short c_port_reset:1;        // C_PORT_RESET
  unsigned short reserved:11;           // RESERVED
}port_change_t;

typedef struct port_status {
  unsigned short port_connection:1;     // PORT_CONNECTION
  unsigned short port_enable:1;         // PORT_ENABLE
  unsigned short port_suspend:1;        // PORT_SUSPEND
  unsigned short port_over_current:1;   // PORT_OVER_CURRENT
  unsigned short port_reset:1;          // PORT_RESET
  unsigned short reserved:3;            // RESERVED
  unsigned short port_power:1;          // PORT_POWER
  unsigned short port_lowspeed:1;       // PORT_LOW_SPEED
  unsigned short port_highspeed:1;      // PORT_HIGH_SPEED
  unsigned short port_test:1;           // PORT_TEST
  unsigned short port_indicator:1;      // PORT_INDICATOR
  unsigned short reserved2:3;           // RESERVED
} __attribute__ ((packed)) portstatus_t;

typedef struct portstat {
  portstatus_t  stat;
  port_change_t change;
} __attribute__ ((packed)) portstat_t;

int usb_poll_hub( struct rt_urb *p_urb );
int usb_init_hub( struct rt_urb *p_urb );

int hub_set_port_feature( struct rt_urb *p_urb, __u16 port_nr, __u8 feature );
int hub_clear_port_feature( struct rt_urb *p_urb, __u16 port_nr, __u8 feature );

int hub_port_reset( struct rt_urb *p_urb, __u16 port_nr);
int hub_port_resume( struct rt_urb *p_urb, __u16 port_nr);

int hub_get_portstat( struct rt_urb *p_urb, __u16 port_nr, portstat_t *p_stat );
int hub_get_descriptor( struct rt_urb *p_urb, hub_descriptor_t *p_desc);

#endif
