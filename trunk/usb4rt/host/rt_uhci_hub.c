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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "rt_uhci_hub.h"
#include <rt_usb.h>
#include <core/rt_usb_hub.h>
#include <core/rt_usb_debug.h>

extern struct uhc_device uhc_dev[MAX_UHC_CONTROLLER];

extern int uhc_clear_stat( struct uhc_device *p_uhcd  );

/*
static void clear_uport_stat(int port)
{
  unsigned short value = inw(port);
  outw(value, port);
}
*/

static void rh_port_suspend( int port)
{
  unsigned short value = inw(port);
  value |= PORTSC_SUSPEND;
  outw( value, port);
}

static void rh_port_wakeup( int port)
{
  unsigned short value = inw(port);
  value &= ~PORTSC_SUSPEND;
  outw( value, port);
}

static void rh_port_enable( int port)
{
  unsigned short value = inw(port);
  value |= PORTSC_PE;
  outw( value, port);
  do {
    value = inw(port);
  } while( !(value & PORTSC_PE) && (value & PORTSC_CCS));
}

static void rh_port_disable( int port)
{
  unsigned short value = inw(port);
  value &= ~PORTSC_PE;
  outw( value, port);
}

static void rh_port_reset( int port)
{
  int i,usec_offset = 0;
  unsigned short value = inw(port);
  value |= PORTSC_PR;
  outw( value, port);
  for(i=0;i<5;i++)
    udelay(10000 + usec_offset);
  value &= ~PORTSC_PR;
  outw( value, port);
}

static void rh_port_reset_long( int port)
{
  unsigned short value = 0;
  int i;

  value = inw(port);
  value |= PORTSC_PR;
  outw( value, port);
  for(i=0; i<20; i++)
    udelay(10000);
  value &= ~PORTSC_PR;
  outw( value, port);
}

int rh_init( struct uhc_device *p_uhcd )
{
  unsigned long io_size = 0;
  unsigned int port,portstatus;

  if(!p_uhcd->p_io){
    return -ENODEV;
  }

  io_size = p_uhcd->p_io->end - p_uhcd->p_io->start + 1;

  for (port = 0; port < (io_size - 0x10) / 2; port++) {
    portstatus = inw( p_uhcd->p_io->start + 0x10 + (port * 2));
    /* If Bit 7 is not set */
    if (!(portstatus & 0x0080))
      break;
  }

  if (port < 2 || port > UHC_RH_MAXCHILD) {
    DBG("RT-UHC-Driver: Root-Hub: Port count misdetected? Forcing to 2 ports\n");
    port = 2;
  }
  p_uhcd->p_hcd->rh_numports = port;

  return 0;
}

struct usb_device *nrt_uhci_poll_root_hub_port( struct hc_device *p_hcd , __u8 rh_port_nr)
{
  struct usb_device *p_usbdev = NULL;
  int i               = 0;
  static int do_over  = 0;
  int usec_offset     = 0;
  int rh_port         = 0;

  unsigned short value = 0;

  struct uhc_device *p_uhcd = p_hcd->p_private;

  if(!p_uhcd || !p_uhcd->p_io){
    ERR("RT-UHC-Driver: %s - Invalid Pointer to UHCD \n",__FUNCTION__);
    return NULL;
  }

  rh_port = p_uhcd->p_io->start + 0x10 + (rh_port_nr * 0x02);
  value   = inw(rh_port);

  if(value & PORTSC_CSC || do_over == rh_port) {

    if (value & PORTSC_CSC){
      DBG_MSG1(p_hcd,"%02d : Connect Status Change\n",rh_port_nr);
    }

    if(do_over == rh_port){
      DBG_MSG1(p_hcd,"%02d : DO OVER Port %x\n",rh_port_nr,do_over);
    }

    do_over=0;

    if(value & PORTSC_CCS ) {             // if port connected
      DBG_MSG1(p_hcd,"%02d : Connection detect\n",rh_port_nr);

      /* ACK */
      outw(value, rh_port);
      for(i=0; i<40; i++) {
        udelay(10000 + usec_offset);
        value = inw(rh_port);
        if(value & PORTSC_CSC) {
          outw(value, rh_port);
          i=0;
          ERR_MSG1(p_hcd,"%02d : BOUNCE!\n",rh_port_nr);
        }
      }

      DBG_MSG1(p_hcd,"%02d : Wake Up!\n",rh_port_nr);
      rh_port_wakeup(rh_port);

      DBG_MSG1(p_hcd,"%02d : Reset\n",rh_port_nr);
      rh_port_reset(rh_port);

      udelay(10);

      DBG_MSG1(p_hcd,"%02d : Enable\n",rh_port_nr);
      rh_port_enable(rh_port);

      if(!value & PORTSC_CCS) {
        ERR_MSG1(p_hcd,"%02d : Device went away!\n",rh_port_nr);
        return NULL;
      }
      if(p_hcd->p_hcd_fkt->nrt_usb_config_dev){
        p_usbdev = p_uhcd->p_hcd->p_hcd_fkt->nrt_usb_config_dev( p_hcd, rh_port_nr, value & PORTSC_LSDA);
      } else {
        ERR_MSG1(p_hcd,"%02d : %s Device found, but can't configure it !!! \n",
            rh_port_nr,( value & PORTSC_LSDA) ? "Lowspeed" : "Fullspeed" );
      }
      if( !p_usbdev ) {
        DBG_MSG1(p_hcd,"%02d : Disable\n",rh_port_nr);
        rh_port_disable(rh_port);

        udelay(5000);

        DBG_MSG1(p_hcd,"%02d : Reset\n",rh_port_nr);
        rh_port_reset_long(rh_port);

        DBG_MSG1(p_hcd,"%02d : Suspend\n",rh_port_nr);
        rh_port_suspend(rh_port);

        do_over = rh_port;

        DBG_MSG1(p_hcd,"%02d : DO OVER = %x\n",rh_port,rh_port_nr);
        uhc_clear_stat(p_uhcd);
      }
    } else {
      rh_port_suspend(rh_port);
      DBG_MSG1(p_hcd,"%02d : Suspend\n",rh_port_nr);
      rh_port_disable(rh_port);
      DBG_MSG1(p_hcd,"%02d : Disable\n",rh_port_nr);
    }
  }
  return p_usbdev;
}

