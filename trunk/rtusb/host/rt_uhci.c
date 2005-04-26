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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include "rt_uhci.h"
#include "rt_uhci_hub.h"
#include "../core/rt_usb_debug.h"

#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Joerg Langenberg - joergel@gmx.net"
#define DRIVER_DESC "Realtime Driver for Universal Host Controller"

#define BANDWIDTH_RECLAMATION

int list=0;
int device=0x0000;
MODULE_PARM (list,"i");
MODULE_PARM_DESC (list,"Listet alle UHCI-Controller auf");
MODULE_PARM (device,"i");
MODULE_PARM_DESC (device,"Treiber nur fuer Geraet mit Device-ID device (HEX)");

/* rt_uhci_hub */
extern int rh_init( struct uhc_device *p_uhcd );
extern struct usb_device *nrt_uhci_poll_root_hub_port( struct hc_device *p_hcd , __u8 rh_port_nr);

unsigned short anz_uhc_ctrl;
unsigned short reg_uhc_ctrl;
struct uhc_device uhc_dev[MAX_UHC_CONTROLLER];
struct hcd_funktions hcd_fkt;
struct pci_dev *p_pcidev_old = NULL;
unsigned int alloc_bytes = 0;
RTIME rt_timer_overhead = 0;
RTIME frame_start;

static u32 pci_io_addr[] = {
  PCI_BASE_ADDRESS_0,
  PCI_BASE_ADDRESS_1,
  PCI_BASE_ADDRESS_2,
  PCI_BASE_ADDRESS_3,
  PCI_BASE_ADDRESS_4,
  PCI_BASE_ADDRESS_5,
  0
};

/***************************************************************/
/*  Transfer-Deskriptor Funktionen                             */
/***************************************************************/

static void dump_td_table( struct rt_privurb *p_purb, int all )
{
  if(!p_purb || !p_purb->p_td_table){
    return;
  }

  u32 link,status,token,buffer;
  struct list_head *p_list = p_purb->active_td_list.next;
  td_t *p_td = NULL;

  PRNT("========== DUMP TD-TABLE @ 0x%p ( %d TDs, %d Byte) =========== \n",p_purb->p_td_table,p_purb->anz_tds,p_purb->table_size);
  PRNT("   URB         @ 0x%p \n", p_purb->p_urb);
  PRNT("   PRIVATE-URB @ 0x%p \n", p_purb);
  if(p_purb->p_qh){
    PRNT("   QH          @ 0x%p ,DMA: 0x%p \n",p_purb->p_qh,(void *)p_purb->p_qh->dma_handle);
  }


  PRNT("A    NR         TD        LINK      STATUS       TOKEN      BUFFER  ACT ACTLEN MAXLEN TYPE ERRCNT STALL DBUF BABL TIME BITST SPD IOC NAK DT ADR EP LS\n");

  while(p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list,td_t,active_td_list);

    link   = le32_to_cpu(p_td->link);
    status = td_status(p_td);
    token  = td_token(p_td);
    buffer = le32_to_cpu(p_td->buffer);

    PRNT("%s  %4d 0x%p  0x%p  0x%p  0x%p  0x%p %4s %6d %6d %4x %6d %5s %4s %4s %4s %5s %3s %3s %3s %2s %3d %2d %2s\n",
         "*",p_td->td_nr,
         (void *)p_td->dma_handle,(void *)link,(void *)status,(void *)token,(void *)buffer,
         (get_status_active(p_td) ? "X" : "-"),get_status_actlen(p_td),get_max_len(p_td),get_packet_type(p_td),
         get_status_errcount(p_td),(status & TD_STAT_STALLED) ? "X":"-",(status & TD_STAT_DBUFERR) ? "X":"-",
         (status & TD_STAT_BABBLE)?"X":"-",(status & TD_STAT_CRCTIMEO)? "X":"-",(status & TD_STAT_BITSTUFF) ? "X":"-",
         (status & TD_STAT_SPD) ? "X":"-",(status & TD_STAT_IOC) ?"X":"-",(status & TD_STAT_NAK) ? "X":"-",
         (token & TD_TOKEN_TOGGLE) ? "1":"0", get_address(p_td),get_endpoint(p_td),(status & TD_STAT_LS) ? "X":"-");
    p_list = p_list->next;
  }

  if(!all){
    PRNT("========== END DUMP ========= \n");
    return;
  }
  PRNT("------------------------------------------------------------------------------------------------------------------------------------------------------\n");

  p_list = p_purb->free_td_list.next;
  while(p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list,td_t,active_td_list);

    link   = le32_to_cpu(p_td->link);
    status = td_status(p_td);
    token  = td_token(p_td);
    buffer = le32_to_cpu(p_td->buffer);

    PRNT("%s  %4d 0x%p  0x%p  0x%p  0x%p  0x%p %4s %6d %6d %4x %6d %5s %4s %4s %4s %5s %3s %3s %3s %2s %3d %2d %2s\n",
         "-",p_td->td_nr,
         (void *)p_td->dma_handle,(void *)link,(void *)status,(void *)token,(void *)buffer,
         (get_status_active(p_td) ? "X" : "-"),get_status_actlen(p_td),get_max_len(p_td),get_packet_type(p_td),
         get_status_errcount(p_td),(status & TD_STAT_STALLED) ? "X":"-",(status & TD_STAT_DBUFERR) ? "X":"-",
         (status & TD_STAT_BABBLE)?"X":"-",(status & TD_STAT_CRCTIMEO)? "X":"-",(status & TD_STAT_BITSTUFF) ? "X":"-",
         (status & TD_STAT_SPD) ? "X":"-",(status & TD_STAT_IOC) ?"X":"-",(status & TD_STAT_NAK) ? "X":"-",
         (token & TD_TOKEN_TOGGLE) ? "1":"0", get_address(p_td),get_endpoint(p_td),(status & TD_STAT_LS) ? "X":"-");
    p_list = p_list->next;
  }

  PRNT("========== END DUMP ========= \n");
}

static int create_td_table( struct rt_privurb *p_purb )
{
  td_t *p_td_table;
  td_t *p_list;
  dma_addr_t td_dma;
  int td_size = sizeof(td_t);
  p_purb->table_size = td_size * p_purb->anz_tds;

  p_td_table = (td_t *)dma_alloc_coherent(&p_purb->p_uhcd->p_pcidev->dev,
                                          p_purb->table_size,
                                          &td_dma,
                                          0);
  if(!p_td_table || !td_dma){
    return -ENOMEM;
  }

  int i;
  TD_MSG1( p_purb->p_hcd," TD-Table for URB 0x%p created @ 0x%p, DMA: 0x%p (%d Byte)\n",
           p_purb->p_urb, p_td_table, (void *)td_dma, p_purb->table_size);

  for(i=0; i < p_purb->anz_tds ; i++){
    p_list = &p_td_table[i];

    memset(p_list,0,td_size);
    p_list->link   = cpu_to_le32( LINK_TERM );
    INIT_LIST_HEAD(&p_list->free_td_list);
    INIT_LIST_HEAD(&p_list->active_td_list);

    p_list->td_nr = i;
    p_list->dma_handle = td_dma + (i * td_size);
    p_list->p_uhcd = p_purb->p_uhcd;

    list_add_tail( &p_list->free_td_list, &p_purb->free_td_list);

    p_purb->free_tds++;

  }

  p_purb->p_td_table   = p_td_table;
  p_purb->td_table_dma = td_dma;

  TD_MSG1(p_purb->p_hcd," %d free TDs for URB 0x%p available\n",p_purb->free_tds,p_purb->p_urb);

  return 0;
}

static void destroy_td_table( struct rt_privurb *p_purb )
{
  if(!p_purb->p_td_table || !p_purb->td_table_dma){
    return;
  }

  TD_DBG("RT-UHC-Driver: Deleting TD-Table for URB 0x%p (%d Byte)\n",p_purb->p_urb,p_purb->table_size);

  dma_free_coherent(  &p_purb->p_uhcd->p_pcidev->dev,
                      p_purb->table_size,
                      p_purb->p_td_table,
                      p_purb->td_table_dma);
  p_purb->p_td_table = NULL;
  p_purb->free_tds   = 0;
  return;
}

static td_t *create_td( struct uhc_device *p_uhcd )
{
  td_t *p_td;
  dma_addr_t td_dma;

  p_td = (td_t *)pci_alloc_consistent(p_uhcd->p_pcidev,
                                      sizeof(td_t),
                                      &td_dma);
  if(!p_td || !td_dma){
    return NULL;
  }
  TD_DBG("RT-UHC-Driver: Creating TD @ 0x%p, DMA: 0x%p (%d Byte)\n",p_td, (void *)td_dma, sizeof(td_t));

  memset(p_td,0,sizeof(td_t));
  p_td->link   = cpu_to_le32( LINK_TERM );
  INIT_LIST_HEAD(&p_td->free_td_list);
  INIT_LIST_HEAD(&p_td->active_td_list);

  p_td->p_uhcd = p_uhcd;
  p_td->dma_handle = td_dma;

  return p_td;
}

static void destroy_td( td_t *p_td )
{
  if(!p_td || !p_td->dma_handle || !p_td->p_uhcd){
    return;
  }

  TD_DBG("RT-UHC-Driver: Deleting TD @ 0x%p (%d Byte)\n",p_td,sizeof(td_t));

  pci_free_consistent(p_td->p_uhcd->p_pcidev,
                      sizeof(td_t),
                      p_td,
                      p_td->dma_handle);

  return;
}

 /**************************************************************/
/*  QUEUE-HEADER Funktionen                                    */
/***************************************************************/

static qh_t *create_qh( struct uhc_device *p_uhcd )
{
  qh_t *p_qh;
  dma_addr_t qh_dma;

  p_qh = (qh_t *)pci_alloc_consistent(p_uhcd->p_pcidev,
                                      sizeof(qh_t),
                                      &qh_dma);
  if(!p_qh || !qh_dma){
    return NULL;
  }
  QH_DBG("RT-UHC-Driver: Creating QH @ 0x%p, DMA: 0x%p (%d Byte)\n",p_qh, (void *)qh_dma, sizeof(qh_t));

  memset(p_qh,0,sizeof(qh_t));
  p_qh->link    = cpu_to_le32( LINK_TERM );
  p_qh->element = cpu_to_le32( LINK_TERM );
  INIT_LIST_HEAD(&p_qh->qh_link_list);

  p_qh->p_uhcd = p_uhcd;
  p_qh->dma_handle = qh_dma;

  return p_qh;
}

static void destroy_qh( qh_t *p_qh )
{
  if(!p_qh || !p_qh->dma_handle || !p_qh->p_uhcd){
    return;
  }

  QH_DBG("RT-UHC-Driver: Deleting QH @ 0x%p (%d Byte)\n",p_qh,sizeof(qh_t));

  pci_free_consistent(p_qh->p_uhcd->p_pcidev,
                      sizeof(qh_t),
                      p_qh,
                      p_qh->dma_handle);

  return;
}

 /**************************************************************/
/*  FRAME-LIST Funktionen                                      */
/***************************************************************/

/**
 * Initialiert die Frame-Liste.
 * Nachdem der Speicher fuer eine Frame-Liste angefordert ist, wird der Bereich wird mit Nullen gefuellt.
 * Der Zeiger auf die Frame-Liste muss mit 0x000 enden, da der Controller diese Bits fuer die Frame-Nummer (10 Bit),
 * fuer das QH-Bit und das Terminate-Bit verwendet.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0 bei Erfolg
 * @return -ENODEV, wenn p_hdc nicht auf ein gueltiges struct hc_device zeigt.
 * @return -ENOMEM, wenn kein Speicher vorhanden oder Frame-List-Pointer ungueltig.
 */
static int init_framelist( struct uhc_device *p_uhcd )
{
  if(!p_uhcd->p_pcidev){
    return -ENODEV;
  }
  if(p_uhcd->p_fl){
    return -EBUSY;
  }

  struct frame_list *p_fl;
  dma_addr_t fl_dma;

  p_fl = (struct frame_list *)dma_alloc_coherent(&p_uhcd->p_pcidev->dev,sizeof(struct frame_list),&fl_dma, 0);
  if(!p_fl || !fl_dma){
    return -ENOMEM;
  }

  memset(p_fl,0,sizeof(struct frame_list));

  p_uhcd->p_fl = p_fl;
  p_uhcd->p_fl->dma_handle = fl_dma;

  int i;
  for(i=0; i<1024;i++){
    p_uhcd->p_fl->frame_dma[i] = LINK_TERM;
    p_uhcd->p_fl->frame_cpu[i] = NULL;
  }

  return 0;
}

/**
 * Gibt den Speicher der Frame-Liste wieder frei.
 * @param hcd_nr Nummer des Host-Controllers
 */
static void destroy_flame_list( struct uhc_device *p_uhcd )
{
  if(!p_uhcd){
    return;
  }
  if(!p_uhcd->p_fl || !p_uhcd->p_fl->dma_handle){
    return;
  }

  DBG("RT-UHC-Driver: Deleting Framelist @ 0x%p (%d Byte)\n",p_uhcd->p_fl, sizeof(struct frame_list));

  dma_free_coherent(&p_uhcd->p_pcidev->dev,sizeof(struct frame_list),p_uhcd->p_fl,p_uhcd->p_fl->dma_handle);
  p_uhcd->p_fl = NULL;

  return;
}

/**
 * Haengt einen QH an eine Frame-List an.
 * @param p_qh Zeiger auf ein QH, welcher an die Frame-Liste angehaengt werden soll.
 * @param p_fl Zeiger auf ein struct frame_list.
 * @param nr Nummer des Frame-Pointers.
 * @return -EINVAL, wenn Zeiger p_td oder p_qh ungueltig
 * @return 0, wenn erfolgreich.
 */
static int append_qh_on_fl( qh_t *p_qh,struct frame_list *p_fl,int nr)
{
  if(!p_qh || !p_qh->dma_handle || !p_fl){
    return -EINVAL;
  }
  if(nr < 0 || nr > MAX_FRAMES){
    return -EINVAL;
  }
  p_fl->frame_dma[nr] = cpu_to_le32(p_qh->dma_handle | LINK_TO_QH | LINK_NO_TERM);
  p_fl->frame_cpu[nr] = p_qh;

  return 0;
}

/******************************************************************************/
/*                   UHC - Funktionen                                         */
/*           ( fuer jeden gefundenen Controller )                             */
/******************************************************************************/

/**
 * Initialisiert die grundlegende Datenstruktur fuer einen Universal Host Controller.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn hcd_nr ungueltig
 * @return -ENOMEM, wenn keine TDs oder QHs mehr frei
 */
static int init_skel( struct uhc_device *p_uhcd )
{
  int i;

  p_uhcd->p_qh_lowspeed = create_qh(p_uhcd);
  if(!p_uhcd->p_qh_lowspeed) {
    ERR2("RT-UHC-Driver: [ERROR] Cannot allocate Low-Speed-QH \n");
    return -ENOMEM;
  }
  QH_DBG("RT-UHC-Driver: Low-Speed-QH  @ 0x%p, DMA: 0x%p \n",p_uhcd->p_qh_lowspeed,(void *)p_uhcd->p_qh_lowspeed->dma_handle);

  p_uhcd->p_qh_fullspeed = create_qh(p_uhcd);
  if(!p_uhcd->p_qh_fullspeed) {
    ERR2("RT-UHC-Driver: [ERROR] Cannot allocate Full-Speed-QH \n");
    return -ENOMEM;
  }
  QH_DBG("RT-UHC-Driver: Full-Speed-QH  @ 0x%p, DMA: 0x%p \n",p_uhcd->p_qh_fullspeed,(void *)p_uhcd->p_qh_fullspeed->dma_handle);

  p_uhcd->p_qh_term = create_qh(p_uhcd);
  if(!p_uhcd->p_qh_term) {
    ERR2("RT-UHC-Driver: [ERROR] Cannot allocate Terminate-QH \n");
    return -ENOMEM;
  }
  QH_DBG("RT-UHC-Driver: Terminate-QH @ 0x%p, DMA: 0x%p \n",p_uhcd->p_qh_term,(void *)p_uhcd->p_qh_term->dma_handle);


  p_uhcd->p_td_loop = create_td(p_uhcd);
  if(!p_uhcd->p_td_loop) {
    ERR2("RT-UHC-Driver: [ERROR] Cannot allocate Loop-TD \n");
    return -ENOMEM;
  }
  TD_DBG("RT-UHC-Driver: Loop-TD      @ 0x%p, DMA: 0x%p \n",p_uhcd->p_td_loop,(void *)p_uhcd->p_td_loop->dma_handle);

  p_uhcd->p_td_loop->buffer = 0;
  p_uhcd->p_td_loop->link   = cpu_to_le32(  p_uhcd->p_td_loop->dma_handle | LINK_NO_TERM | LINK_TO_TD );
  p_uhcd->p_td_loop->status = cpu_to_le32(  td_status_errcount(0) );
  p_uhcd->p_td_loop->token  = cpu_to_le32(  td_token_pid(IN_TOKEN) |
                                            td_token_addr(0x7f) |
                                            td_token_maxlen(7) );

  p_uhcd->p_qh_lowspeed->link    = cpu_to_le32( p_uhcd->p_qh_fullspeed->dma_handle | LINK_TO_QH | LINK_NO_TERM );
  p_uhcd->p_qh_lowspeed->element = cpu_to_le32( LINK_TERM );

  p_uhcd->p_qh_fullspeed->link    = cpu_to_le32( p_uhcd->p_qh_term->dma_handle | LINK_TO_QH | LINK_NO_TERM );
  p_uhcd->p_qh_fullspeed->element = cpu_to_le32( LINK_TERM );


#ifdef BANDWIDTH_RECLAMATION
  p_uhcd->p_qh_term->link    = cpu_to_le32( p_uhcd->p_qh_fullspeed->dma_handle | LINK_TO_QH | LINK_NO_TERM );
#else
  p_uhcd->p_qh_term->link    = cpu_to_le32( LINK_TERM );
#endif

//p_uhcd->p_qh_term->element = cpu_to_le32( LINK_TERM );
  p_uhcd->p_qh_term->element = cpu_to_le32( p_uhcd->p_td_loop->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_NO_VF);

  int ret;
  for(i=0; i<MAX_FRAMES; i++) {
    ret = append_qh_on_fl( p_uhcd->p_qh_lowspeed, p_uhcd->p_fl, i);
  }

  return 0;
}

/**
 * Ausgabe der Controller-Informationen.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static void dump_uhcd( struct uhc_device *p_uhcd )
{
#ifdef DEBUG
  if(!p_uhcd->p_io){
    return;
  }

  unsigned short stat;
  unsigned short cmd;
  unsigned short intr;
  unsigned int port_base;
  unsigned short port[p_uhcd->p_hcd->rh_numports];

  cmd  = inw(p_uhcd->p_io->start + USBCMD);
  stat = inw(p_uhcd->p_io->start + USBSTS);
  intr = inw(p_uhcd->p_io->start + USBINTR);

  DBG("========== DUMP UHC-Controller [%d] @ 0x%p ===========\n",p_uhcd->uhcd_nr,p_uhcd);
#ifdef CONFIG_PCI_NAMES
  DBG("   Name : %s \n",p_uhcd->p_pcidev->pretty_name);
#else
  DBG("   Name : USB Universal Host Controller\n");
#endif
  DBG("   Vendor = 0x%4x, Device = 0x%4x, Class = 0x%x, IRQ = %d ",
      p_uhcd->p_pcidev->vendor,p_uhcd->p_pcidev->device,p_uhcd->p_pcidev->class,p_uhcd->p_pcidev->irq);
  DBG("\n   COMMAND (0x%04x): ", cmd);
  if (!(cmd & USBCMD_RS) )    DBG("STOP ");
  if (cmd & USBCMD_RS )       DBG("RUN ");
  if (cmd & USBCMD_HCRESET )  DBG("HCRESET "); //Host-Controller Reset
  if (cmd & USBCMD_GRESET )   DBG("GRESET ");  //Global-Reset
  if (cmd & USBCMD_EGSM )     DBG("EGSM ");  //Enter global Suspend Mode
  if (cmd & USBCMD_FGR )      DBG("FGR ");     //Force global Resume
  DBG("\n   USBSTS  (0x%04x): ", stat);
  if (! stat )                DBG("OK ");
  if (stat & USBSTS_HCH )     DBG("HCH ");  //HC Halted
  if (stat & USBSTS_HCPE )    DBG("HCPE "); //Host Controller Process Error - the scripts were buggy
  if (stat & USBSTS_USBINT )  DBG("IOC ");  //Interrupt due to IOC
  if (stat & USBSTS_ERROR )   DBG("ERROR ");  //Interrupt due to error
  if (stat & USBSTS_HSE )     DBG("HSE ");  //Host System Error - basically PCI problems
  if (stat & USBSTS_RD )      DBG("RD "); //Resume Detect
  DBG("\n   USBINTR (0x%04x): ", intr);
  if (! intr )                DBG("NONE "); //No Interrupts
  if (intr & USBINTR_TIMEOUT) DBG("TIMEOUT ");//Timeout / CRC Interrupt Enable
  if (intr & USBINTR_RESUME)  DBG("RESUME "); //Resume Interrupt Enable
  if (intr & USBINTR_IOC )    DBG("IOC ");  //Interrupt on Complete (IOC)
  if (intr & USBINTR_SP )     DBG("SP "); //Short Packet Interrupt Enable \n");
  DBG("\n   FRNUM           : 0x%04x", inw(p_uhcd->p_io->start + FRNUM));
  DBG("\n   FLBASEADD       : 0x%08x", inl(p_uhcd->p_io->start + FRBASEADD));
  DBG("\n   SOFMOD          : 0x%02x", inb(p_uhcd->p_io->start + SOFMOD));

  unsigned short k=0;
  for(k=0 ; k < p_uhcd->p_hcd->rh_numports; k++){
    port_base = p_uhcd->p_io->start + 0x10 + ( 0x02 * k);
    port[k] = inw(port_base);
    DBG("\n   PORTSC%d @ 0x%04x (0x%04x) : ",k,port_base,port[k]);
    if (port[k] & PORTSC_CSC )      DBG("CSC ");  //Connect Status Change
    if (port[k] & PORTSC_PE)        DBG("PE "); //Port Enabled
    if (port[k] & PORTSC_PEC)       DBG("PEC ");  //Port Enable/Disable Change
    if (port[k] & PORTSC_DPLUS)     DBG("D+ "); //D+ active
    if (port[k] & PORTSC_DMINUS)    DBG("D- "); //D- active
    if (port[k] & PORTSC_RD)        DBG("RD "); //Resume Detect on Port
    if (port[k] & PORTSC_CCS) {
      if (port[k] & PORTSC_LSDA)    DBG("LS "); //Low Speed Device Attached
      if (!(port[k] & PORTSC_LSDA)) DBG("FS "); //Full Speed Device Attached
    }
    if (port[k] & PORTSC_PR)        DBG("PR "); //Port k is in Reset
    if (port[k] & PORTSC_OC)        DBG("OC "); //Overcurrent-Pin is Active
    if (port[k] & PORTSC_OCC)       DBG("OCC ");  //Overcurrent-Pin changed from Low to High
    if (port[k] & PORTSC_SUSPEND)   DBG("SUSPEND ");//Port k in suspend Mode
  }
  DBG("\n========== END DUMP UHC-Controller [%d] @ 0x%p ==========\n",p_uhcd->uhcd_nr,p_uhcd);
#endif
  return;
}

/**
 * Loescht das Statusregister des Host Controllers.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_clear_stat( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Clear Status\n");

  unsigned short stat;
  stat = inw(p_uhcd->p_io->start + USBSTS);
  outw(stat,p_uhcd->p_io->start + USBSTS);
  return 0;
}

void uhc_enable_all_interrupts( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io){
    return;
  }

  /* Turn on all interrupts */
  PRNT("RT-UHC-Driver: Turn on all Interrupts  \n");
  outw( USBINTR_TIMEOUT | USBINTR_RESUME | USBINTR_IOC | USBINTR_SP ,p_uhcd->p_io->start + USBINTR );
}

void uhc_disable_all_interrupts( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io){
    return;
  }

  /* Turn off all interrupts */
  PRNT("RT-UHC-Driver: Turn off all Interrupts  \n");
  outw( 0 ,p_uhcd->p_io->start + USBINTR );
}

/**
 * Fuehrt einen globalen Reset eines Universal Host Controller aus.
 * ZEIT : 15ms
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_global_reset( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io || !p_uhcd->p_pcidev){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Global Reset\n");

  /* Turn off PIRQ .This also turns off the BIOS's USB Legacy Support.*/
  pci_write_config_word(p_uhcd->p_pcidev,USBLEGSUP, 0);

  uhc_disable_all_interrupts(p_uhcd);

  /* Global reset for 50ms */
  outw(USBCMD_GRESET,p_uhcd->p_io->start + USBCMD);

  msleep(50);

  /* Clear Command-Register */
  outw(0,p_uhcd->p_io->start + USBCMD);

  return 0;
}

/**
 * Startet einen Universal Host Controller aus.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_start( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io || !p_uhcd->p_pcidev){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Starte Host Controller\n");
  int timeout = 1000;

  /* Host Controller Reset */
  outw(USBCMD_HCRESET, p_uhcd->p_io->start + USBCMD);
  while (inw(p_uhcd->p_io->start + USBCMD) & USBCMD_HCRESET) {
    if (!--timeout) {
      ERR2("RT-UHC-Driver: [ERROR] %s - USBCMD_HCRESET timed out! \n",__FUNCTION__);
      break;
    }
  }

  /* Turn on PIRQ */
  pci_write_config_word(p_uhcd->p_pcidev, USBLEGSUP,USBLEGSUP_DEFAULT);

  uhc_enable_all_interrupts(p_uhcd);

  /* Run and mark it configured with a 64-byte max packet */
  outw(USBCMD_RS | USBCMD_CF | USBCMD_MAXP, p_uhcd->p_io->start + USBCMD);

  return(0);
}

/**
 * Stoppt einen Universal Host Controller.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_stop( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Stoppe Host Controller\n");

  unsigned short tmp;
  tmp = inw(p_uhcd->p_io->start + USBCMD);
  tmp &= ~USBCMD_RS;
  outw(tmp,p_uhcd->p_io->start + USBCMD);
  while( !(inw(p_uhcd->p_io->start + USBSTS) & USBSTS_HCH) ){
    outw(USBSTS_HCH,p_uhcd->p_io->start + USBSTS);
  }
  if(uhc_global_reset(p_uhcd)) {
    ERR2("RT-UHC-Driver: [ERROR] %s - Resetting Host-Controller failed\n",__FUNCTION__);
  }
  return (0);
}

/**
 * Initialisiert einen Universal Host Controller.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn hcd_nr ungueltig
 */
static int uhc_init( struct uhc_device *p_uhcd )
{
  if(!p_uhcd || !p_uhcd->p_io || !p_uhcd->irq){
    return -ENODEV;
  }
  int ret,i,rh_port;

  /* Initialisiere Framelist */
  ret = init_framelist( p_uhcd );
  if( ret ){
    ERR2("RT-UHC-Driver: [ERROR] %s - Creating Framelist failed\n",__FUNCTION__);
    return ret;
  }
  DBG("RT-UHC-Driver: Framelist    @ 0x%p, DMA: 0x%p (%d Byte) \n",
      (void *)p_uhcd->p_fl,(void *)p_uhcd->p_fl->dma_handle, sizeof(struct frame_list));

  /* Initialisiere Scheduler */
  ret = init_skel( p_uhcd );
  if( ret ) {
    ERR2("RT-UHC-Driver: [ERROR] %s - Init Skeleton failed\n",__FUNCTION__);
    return ret;
  }
  DBG("RT-UHC-Driver: Skeleton initialized\n");

  ret = rh_init( p_uhcd );
  if(ret) {
    ERR2("RT-UHC-Driver: [ERROR] %s - Init Root-Hub failed\n",__FUNCTION__);
    return ret;
  }

  /* Setting Bus-Master */
  DBG("RT-UHC-Driver: Setting Bus-Master\n");
  unsigned short tmp;
  pci_read_config_word(p_uhcd->p_pcidev,PCICMD,&tmp);
  tmp |= PCICMD_BME;
  pci_write_config_word(p_uhcd->p_pcidev,PCICMD,tmp);

  /* Writing Framellist */
  DBG("RT-UHC-Driver: Write Framellist 0x%p \n",(void *)p_uhcd->p_fl->dma_handle);
  outl( p_uhcd->p_fl->dma_handle, p_uhcd->p_io->start + FRBASEADD);

   /* Set Max-Packet to 64 */
  DBG("RT-UHC-Driver: Setting Max-Packet to 64 \n");
  outw( inw( p_uhcd->p_io->start + USBCMD ) | USBCMD_MAXP , p_uhcd->p_io->start + USBCMD);

  /* Setting Framenummer to 0 */
  DBG("RT-UHC-Driver: Set Framenummer to 0\n");
  outw(0x0000,p_uhcd->p_io->start + FRNUM);

  uhc_disable_all_interrupts(p_uhcd);

  /* Setting Ports into Suspend-Mode */
  DBG("RT-UHC-Driver: Set Ports into Suspend-Mode\n");
  for(i=0;i<p_uhcd->p_hcd->rh_numports;i++){
    rh_port = p_uhcd->p_io->start + 0x10 + (i * 0x02);
    outw(0x1000,rh_port);
  }

  return 0;
}

static void uhc_clear( struct uhc_device *p_uhcd )
{
  __u8 nr = p_uhcd->uhcd_nr;
  memset(p_uhcd,0,sizeof(struct uhc_device));
  p_uhcd->uhcd_nr = nr;

  INIT_LIST_HEAD(&p_uhcd->reg_urb_list);
  INIT_LIST_HEAD(&p_uhcd->handle_urb_list);

}

/******************************************************************/
/*  !!! Functions called IN INTERRUPT !!!                         */
/******************************************************************/

void dump_urb( struct rt_urb *p_urb );
static void unmap_dma( struct rt_privurb *p_purb );
static inline void rt_complete_urb( struct rt_privurb *p_purb );
static void rt_unschedule_ctrl_bulk_urb( struct rt_privurb *p_purb );
static void rt_unschedule_int_urb( struct rt_privurb *p_purb );
static void rt_unschedule_isoc_urb( struct rt_privurb *p_purb );


static inline void handle_urb(__u16 stat, struct uhc_device *p_uhcd, struct rt_privurb *p_purb)
{
  if(p_purb->urb_wait_flags == URB_WAIT_BUSY){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: BUSY WAIT\n",p_purb->p_urb);
    // handled in rt_uhci_send_busywait_urb
    return;
  }

  // UHCD-Error
  if(p_uhcd->status & UHC_HOST_ERROR){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: HOST-CONTROLLER ERROR\n",p_purb->p_urb);
    p_purb->p_urb->status = -2;
    goto handle;
  }

    // UHCD-Not Running
  if( !(p_uhcd->status & UHC_RUNNING) ){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: HOST-CONTROLLER STOPPED\n",p_purb->p_urb);
    p_purb->p_urb->status = -3;
    goto handle;
  }

  // Check if completed
  if(p_purb->p_qh->element & LINK_TERM){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: COMPLETED\n",p_purb->p_urb);
    p_purb->p_urb->status = 0;
    goto handle;
  }

  // Check for Errors
  if (stat & USBSTS_ERROR) {  //Interrupt due to error

    td_t *p_td = NULL;
    struct list_head *p_list = p_purb->active_td_list.next;

    while( p_list != &p_purb->active_td_list ){

      p_td = list_entry( p_list, td_t, active_td_list);

      if( td_status(p_td) & TD_STAT_ANY_ERROR ){
        DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," ===> URB 0x%p: ERROR AT TD[%d]: %s%s%s%s%s",p_purb->p_urb,p_td->td_nr,
          (td_status(p_td) & TD_STAT_STALLED)  ? "STALL ":"",
          (td_status(p_td) & TD_STAT_DBUFERR)  ? "DATA-BUFFER ":"",
          (td_status(p_td) & TD_STAT_BABBLE)   ? "BABBLE ":"",
          (td_status(p_td) & TD_STAT_CRCTIMEO) ? "CRC or TIMEOUT ":"",
          (td_status(p_td) & TD_STAT_BITSTUFF) ? "BITSTUFF ":"");
        DBG("\n");
        dump_td_table(p_purb,0);

        int toggle = ( td_status(p_td) & TD_TOKEN_TOGGLE ? 1 : 0 );
        usb_set_data_toggle( p_purb->p_urb->p_usbdev,p_purb->p_urb->pipe, toggle);

        p_purb->p_urb->status = -1;

        goto handle;
      }

      p_list = p_list->next;
    }
}
  // no handle
  return;

handle:
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: HANDLING \n",p_purb->p_urb);

    if(p_purb->urb_wait_flags == URB_WAIT_SEM){
      DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Wake up Driver\n",p_purb->p_urb);
      p_purb->status |= PURB_WAKE_UP_BY_IRQ;
      rt_sem_v(&p_purb->p_urb->rt_sem);
      // handled in rt_uhci_send_semaphore_urb
      return;
    }

    rt_complete_urb(p_purb);

    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Calling Complete-Function\n",p_purb->p_urb);
    p_purb->p_urb->rt_complete_fkt(p_purb->p_urb);

    return;
}

static inline void handle_controller( struct uhc_device *p_uhcd, __u16 stat )
{
  DBG_MSG1(p_uhcd->p_hcd," =================== BEGIN RTAI-INTERRUPT HANDLER =============================\n");

  if (stat & USBSTS_ERROR) {  //Interrupt due to error
    DBG_MSG1(p_uhcd->p_hcd," ===> Incoming IRQ %d, ERROR \n",p_uhcd->irq);
  }
  if (stat & USBSTS_USBINT) { //Interrupt due to IOC
    DBG_MSG1(p_uhcd->p_hcd," ===> Incoming IRQ %d, COMPLETE \n",p_uhcd->irq);
    if( td_status(p_uhcd->p_td_loop) & TD_STAT_IOC ){
      DBG_MSG1(p_uhcd->p_hcd," ===> Interrupt called by Loop-TD \n");
    }
  }
  if (stat & USBSTS_RD) {     // Resume Detect
    DBG_MSG1(p_uhcd->p_hcd," ===> Incoming IRQ %d, Resume Detect \n",p_uhcd->irq);
  }

  if (stat & USBSTS_HSE || stat & USBSTS_HCPE || stat & USBSTS_HCH ) {
    DBG_MSG1(p_uhcd->p_hcd," ===> Incoming IRQ %d \n",p_uhcd->irq);

    if(stat & USBSTS_HSE){
      ERR_MSG1(p_uhcd->p_hcd," ===> Host System Error \n");
      p_uhcd->status |= UHC_HOST_ERROR;
    }
    if (stat & USBSTS_HCPE) {
      ERR_MSG1(p_uhcd->p_hcd," ===> Host Controller Process Error \n");
      p_uhcd->status |= UHC_HOST_ERROR;
    }
    if (stat & USBSTS_HCH) {
      ERR_MSG1(p_uhcd->p_hcd," ===> Host-Controller Halted \n");
      p_uhcd->status &= ~UHC_RUNNING;
    }

    dump_uhcd(p_uhcd);
  }

  struct rt_privurb *p_purb;
  struct list_head *p_list, *p_next;
  p_list = p_uhcd->reg_urb_list.next;
  while(p_list != &p_uhcd->reg_urb_list){
    p_next = p_list->next;
    p_purb = list_entry(p_list, struct rt_privurb, reg_urb_list);
    if( (p_purb->status == PURB_IN_PROGRESS) || (p_uhcd->status & UHC_HOST_ERROR) || !(p_uhcd->status & UHC_RUNNING) ){
      handle_urb(stat,p_uhcd,p_purb);
    }
    p_list = p_next;
  }

  outw(stat, p_uhcd->p_io->start + USBSTS);                 // Clear it
  DBG_MSG1(p_uhcd->p_hcd," =================== ENDE RTAI-INTERRUPT HANDLER ==============================\n");
}

int rt_irq_handler(struct xnintr *p_xnintr)
{
  struct uhc_device *p_uhcd = NULL;
  struct uhc_device *p_test = (struct uhc_device *)p_xnintr->cookie;

  int i,fs=0;
  __u16 stat = 0x0000;

  for(i=0;i<MAX_UHC_CONTROLLER;i++){
    if(p_test == &uhc_dev[i]){

      if(!p_test || !p_test->p_io){ /* no I/O-Pointer */
        continue;                   /* searching */
      }

      if(!fs){
        frame_start = rt_timer_read();
      }

      stat = inw(p_test->p_io->start + USBSTS);

      if (!(stat & ~USBSTS_HCH)) {  /* no State-Bit set */
        continue;                   /* searching */
      } else {                      /* Interrupt-Controller found */
        p_uhcd = p_test;
        handle_controller(p_uhcd,stat);  /* handle this Controller */
        /* search next Controller (IRQ-Sharing) */
      }
    }
  }

  if(!p_uhcd){
    return RT_INTR_CHAINED;         /* shared interrupt, not mine */
  }

  return RT_INTR_ENABLE;
}

/******************************************************************/
/*  Ressource-Functions                                           */
/******************************************************************/

/**
 * Versucht fuer einen Controller einen IO-Port anzufordern.
 * @param p_uhcd Zeiger auf ein gueltiges struct hc_device
 * @return 0, wenn erfolgreich
 * @return -EBUSY, wenn IO-Port schon belegt
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static int request_ioport( struct uhc_device *p_uhcd )
{
  if(!p_uhcd->p_pcidev){
    return -ENODEV;
  }
  p_uhcd->p_io = NULL;

  int i;
  unsigned long io_flags,io_start,io_end,io_size;
  io_flags = io_start = io_end = io_size = 0;

  for(i=0; pci_io_addr[i]; i++){

    io_flags = pci_resource_flags(p_uhcd->p_pcidev,i);

    if(io_flags & IORESOURCE_IO){
      io_start = pci_resource_start(p_uhcd->p_pcidev,i);
      io_end   = pci_resource_end(p_uhcd->p_pcidev,i);
      io_size  = pci_resource_len(p_uhcd->p_pcidev,i);

      PRNT("RT-UHC-Driver: Request IO-Port @ 0x%08lx (%d Byte) for Device 0x%04x ... ",
           io_start,(int)io_size,p_uhcd->p_pcidev->device);

      p_uhcd->p_io = NULL;
      p_uhcd->p_io = request_region(io_start,io_size,"rt_uhci");

      if(p_uhcd->p_io){
        PRNT("[OK]\n");
        return 0;
      }

      PRNT("[BUSY]\n");
      return -EBUSY;

    }
  }
  return -ENODEV;
}

/**
 * Gibt IO-Port fuer Controller wieder frei.
 * @param p_uhcd : Zeiger auf ein gueltiges struct hc_device
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static void release_ioport( struct uhc_device *p_uhcd )
{
  if(!p_uhcd->p_io){
    return ;
  }

  unsigned long size = p_uhcd->p_io->end - p_uhcd->p_io->start + 1;
  PRNT("RT-UHC-Driver: Release IO-Port 0x%08lx (%lu Byte)\n",p_uhcd->p_io->start,size);
  release_region(p_uhcd->p_io->start,size);
  p_uhcd->p_io = NULL;
  return;
}

/**
 * Versucht fuer einen Controller eine Interruptleitung anzufordern.
 * @param p_uhcd : Zeiger auf ein gueltiges struct hc_device
 * @return 0, wenn erfolgreich
 * @return -EBUSY, wenn Interrupt schon belegt
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static int get_irq( struct uhc_device *p_uhcd )
{
  if(!p_uhcd->p_pcidev){
    return -ENODEV;
  }

  int i;
  for(i=0;i<MAX_UHC_CONTROLLER;i++){
    if( uhc_dev[i].irq &&
        uhc_dev[i].irq == p_uhcd->p_pcidev->irq &&
        &uhc_dev[i] != p_uhcd){
      PRNT("RT-UHC-Driver: RTAI-IRQ %d registered by UHC %d, use it.\n",p_uhcd->p_pcidev->irq,uhc_dev[i].uhcd_nr);
      return 0;
    }
  }

  int ret;
  PRNT("RT-UHC-Driver: Request RTAI-IRQ %d ... ",p_uhcd->p_pcidev->irq);
  ret = rt_intr_create ( &p_uhcd->rt_intr, p_uhcd->p_pcidev->irq, (rt_isr_t) &rt_irq_handler );
  if(ret){
    p_uhcd->irq = 0;
    PRNT("[BUSY]\n");
    return -EBUSY;
  }
  p_uhcd->irq = p_uhcd->p_pcidev->irq;
  PRNT("[OK]\n");

  xnintr_attach ( &p_uhcd->rt_intr.intr_base,(void *)p_uhcd );

  PRNT("RT-UHC-Driver: Enable  RTAI-IRQ %d ... ",p_uhcd->p_pcidev->irq );
  ret = rt_intr_enable ( &p_uhcd->rt_intr);
  if(ret){
    p_uhcd->irq = 0;
    PRNT("[BUSY]\n");
    rt_intr_delete( &p_uhcd->rt_intr);
    return -EBUSY;
  }
  PRNT("[OK]\n");

  return 0;
}

/**
 * Gibt Interruptleitung fuer Controller wieder frei.
 * @param p_uhcd : Zeiger auf ein gueltiges struct hc_device
 */
static void put_irq( struct uhc_device *p_uhcd )
{
  if(!p_uhcd->p_pcidev || !p_uhcd->irq ){
    return;
  }

  int i;
  for(i=0;i<MAX_UHC_CONTROLLER;i++){
    if( uhc_dev[i].irq &&
        uhc_dev[i].irq == p_uhcd->p_pcidev->irq &&
        &uhc_dev[i] != p_uhcd ){
      PRNT("RT-UHC-Driver: RTAI-IRQ %d used by UHC %d\n",p_uhcd->p_pcidev->irq,uhc_dev[i].uhcd_nr);
      return;
    }
  }

  PRNT("RT-UHC-Driver: Disable RTAI-IRQ %d\n",p_uhcd->irq);
  rt_intr_disable ( &p_uhcd->rt_intr );

  PRNT("RT-UHC-Driver: Delete  RTAI-IRQ %d\n",p_uhcd->irq);
  rt_intr_delete ( &p_uhcd->rt_intr );

  p_uhcd->irq = 0;

  return;
}

/******************************************************************/
/*  URB Funktionen                                                */
/******************************************************************/

void dump_urb( struct rt_urb *p_urb )
{
  if(!p_urb){
    ERR2("[ERROR] %s - Invalid URB-Pointer \n",__FUNCTION__);
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

static int map_dma( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return -ENODEV;
  }

  struct rt_urb *p_urb = p_purb->p_urb;

  if(usb_pipecontrol(p_urb->pipe)){
    if( !(p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP) ){

      /* Setup-Packet to DMA */
      dma_addr_t dma_handle = pci_map_single( p_purb->p_uhcd->p_pcidev,
                                              p_urb->p_setup_packet,
                                              sizeof( struct usb_ctrlrequest ),
                                              PCI_DMA_TODEVICE);
      if(!dma_handle){
        ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Couldn't map Setup-Packet to DMA!\n",__FUNCTION__);
        return -ENOMEM;
      }
      p_urb->setup_dma = dma_handle;
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Setup-Packet @ 0x%p (%d Byte) mapped to DMA: 0x%p [OUT]\n",
          p_urb,p_urb->p_setup_packet,sizeof(struct usb_ctrlrequest),(void *)p_urb->setup_dma);
    }
  }

  if(p_urb->transfer_buffer_length){
    if( !(p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) ){

      /* Transfer-Buffer to DMA */
      int direction = usb_pipein(p_urb->pipe) ? PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE ;
      dma_addr_t dma_handle = pci_map_single( p_purb->p_uhcd->p_pcidev,
                                              p_urb->p_transfer_buffer,
                                              p_urb->transfer_buffer_length,
                                              direction);
      if(!dma_handle){
        ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Couldn't map Transfer-Buffer to DMA!\n",__FUNCTION__);
        return -ENOMEM;
      }
      p_urb->transfer_dma = dma_handle;
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Transfer-Buffer @ 0x%p (%d Byte) mapped to DMA: 0x%p [%s]\n",
          p_urb,p_urb->p_transfer_buffer,p_urb->transfer_buffer_length,
          (void *)p_urb->transfer_dma,usb_pipein(p_urb->pipe) ? "IN" : "OUT");
    }
  }
  return 0;
}

static void unmap_dma( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return;
  }

  struct rt_urb *p_urb = p_purb->p_urb;

  /* Unmappe evtl Transfer-Buffer */
  if( p_purb->p_urb->transfer_buffer_length ){
    if( !(p_purb->p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP) ){
      int direction = usb_pipein(p_purb->p_urb->pipe) ? PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE ;
      pci_unmap_single( p_purb->p_uhcd->p_pcidev, p_urb->transfer_dma, p_urb->transfer_buffer_length, direction);

      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Unmap Transfer-Buffer @ 0x%p (%d Byte) from DMA: 0x%p\n",
          p_urb,p_purb->p_urb->p_transfer_buffer,p_urb->transfer_buffer_length,
          (void *)p_purb->p_urb->transfer_dma);

      p_urb->transfer_dma = 0;
    }
  }

  /* Unmappe evtl Setup-Packet */
  if(usb_pipecontrol(p_purb->p_urb->pipe)){
    if( !(p_purb->p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP) ){
      int direction = usb_pipein(p_purb->p_urb->pipe) ? PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE ;
      pci_unmap_single( p_purb->p_uhcd->p_pcidev, p_urb->setup_dma, sizeof( struct usb_ctrlrequest ), direction);

      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Unmap Setup-Packet @ 0x%p (%d Byte) from DMA: 0x%p\n",
          p_urb,p_purb->p_urb->p_setup_packet,sizeof(struct usb_ctrlrequest),
          (void *)p_purb->p_urb->setup_dma);

      p_urb->setup_dma = 0;
    }
  }
}

static int wait_for_urb( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return -ENODEV;
  }

  if(list_empty( &p_purb->active_td_list )){
    return 0; // No TDs to do
  }

  td_t *p_td = NULL;
  RTIME ns_per_retry =     80; // 80ns (12MHz @ Full-Speed)
  unsigned int retrys_per_td = 6250000; // 500ms (500ms/80ns)
  unsigned int retrys;
  RTIME start, stop, urb_start, urb_end;

  struct list_head *p_list = p_purb->active_td_list.next;
  urb_start = p_purb->p_urb->schedule_time;

  while(p_list !=  &p_purb->active_td_list){
    p_td = list_entry(p_list,td_t,active_td_list);

    retrys = retrys_per_td;
    start = rt_timer_read();

    while(retrys && get_status_active(p_td)) {
      /* TD still aktive, waiting ns_per_retry */
      rt_timer_spin( ns_per_retry );
      retrys--;
    }

    stop = rt_timer_read();

    if(retrys == retrys_per_td){
      TDBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: TD[%d] completed in %llu ns @ %llu ns\n",p_purb->p_urb,
             p_td->td_nr, 0llu , stop - frame_start - rt_timer_overhead );
    } else {
      TDBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: TD[%d] completed in %llu ns @ %llu ns\n",p_purb->p_urb,
             p_td->td_nr, stop - start - rt_timer_overhead, stop - frame_start - rt_timer_overhead );
    }

    if(!retrys){
      /* Time-Out for this TD */
      DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p : TIMEOUT FOR TD[%d]\n",p_purb->p_urb, p_td->td_nr);
      return -1;
    }
    /* TD inactive --> Goto Next TD in List */
    p_list = p_list->next;
  }

  urb_end = rt_timer_read();

  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Complete Wait-Time: %llu ns\n",p_purb->p_urb,
             urb_end - urb_start - rt_timer_overhead );

  return 0;
}

static void clear_rt_privurb( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return;
  }

  memset(p_purb,0,sizeof(struct rt_privurb) );

  INIT_LIST_HEAD(&p_purb->reg_urb_list);
  INIT_LIST_HEAD(&p_purb->handle_urb_list);

  INIT_LIST_HEAD(&p_purb->free_td_list);
  INIT_LIST_HEAD(&p_purb->active_td_list);
}

static int rt_uhci_bytes_send( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return -ENODEV;
  }

  td_t *p_td = NULL;
  int act_len, max_len;
  int data_len = 0;
  struct list_head *p_list = p_purb->active_td_list.next;

  while(p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list,td_t,active_td_list);

    act_len = get_status_actlen(p_td);
    max_len = get_max_len(p_td);

    if( get_packet_type(p_td) != SETUP_TOKEN){
      if (act_len != 0x7ff) { /* If no Zero-Paket */
        data_len += act_len + 1;
      }
    }

    if(act_len < max_len){  /* short packet */
      DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Short Packet (%d/%d Bytes) \n", p_purb->p_urb,act_len, max_len);
      break;
    }

    p_list = p_list->next;
  }

  return data_len;
}

static void rt_free_urb_tds( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return;
  }

  if( list_empty(&p_purb->active_td_list) ){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Active-TD-List is empty \n",p_purb->p_urb);
    return;
  }

  td_t *p_td, *p_prev;
  struct list_head *p_list, *p_next;
  p_list = p_purb->active_td_list.next;
  while(p_list != &p_purb->active_td_list){
    p_td = list_entry( p_list, td_t, active_td_list);
    p_next = p_list->next;

    if(p_td->td_nr == 0){
      list_add(&p_td->free_td_list,&p_purb->free_td_list);
    } else {
      p_prev = &p_purb->p_td_table[p_td->td_nr - 1];
      list_add(&p_td->free_td_list, &p_prev->free_td_list);
    }
    list_del_init(&p_td->active_td_list);

    p_list = p_next;
  }
}

static void rt_unschedule_ctrl_bulk_urb( struct rt_privurb *p_purb )
{
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Pointer to Private URB \n",__FUNCTION__);
    return;
  }

  qh_t *p_urb_qh = p_purb->p_qh;
  qh_t *p_prev_qh;
//  qh_t *p_next_qh;

  if( list_empty(&p_urb_qh->qh_link_list) ){
    ERR2("[ERROR] %s - URB 0x%p: Not in QH-Link-List\n",__FUNCTION__,p_purb->p_urb);
    return;
  }

  p_prev_qh = list_entry( p_urb_qh->qh_link_list.prev, qh_t, qh_link_list);
//  p_next_qh = list_entry( p_urb_qh->qh_link_list.next, qh_t, qh_link_list);
//  if(p_next_qh == p_purb->p_uhcd->p_qh_sched){
//    p_next_qh = p_purb->p_uhcd->p_qh_term;
//  }

  /*
  DBG("VORHER  : ------------   ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p     0x%p\n",p_prev_qh,p_urb_qh,p_next_qh);
  DBG("DMA     :  0x%p     0x%p     0x%p\n",(void*)p_prev_qh->dma_handle,(void *)p_urb_qh->dma_handle,(void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->link),(void *)le32_to_cpu(p_urb_qh->link),(void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element),(void *)le32_to_cpu(p_urb_qh->element),(void *)le32_to_cpu(p_next_qh->element));
  DBG("VORHER  : ------------   ------------   ------------\n");

  DBG("NACHHER : ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p\n",p_prev_qh,p_next_qh);
  DBG("DMA     :  0x%p     0x%p\n",(void*)p_prev_qh->dma_handle,(void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_urb_qh->link),(void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element),(void *)le32_to_cpu(p_next_qh->element));
  DBG("NACHHER : ------------   ------------\n");
  */

  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: ---UNSCHEDULE from %s-List ---\n",p_purb->p_urb,
    p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW ? "Low-Speed" : "Full-Speed" );

  p_purb->p_urb->unschedule_time = rt_timer_read();

  wmb();
  p_prev_qh->link = p_urb_qh->link;

      // Aus QH-Liste entfernen
  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Remove from QH-List\n",p_purb->p_urb);
  list_del_init(&p_urb_qh->qh_link_list);

  return;
}

// T O D O
static void rt_unschedule_int_urb( struct rt_privurb *p_purb )
{
  return;
}

// T O D O
static void rt_unschedule_isoc_urb( struct rt_privurb *p_purb )
{
  return;
}

static int rt_schedule_ctrl_bulk_urb( struct rt_privurb *p_purb )
{
  if(!p_purb){
    return -ENODEV;
  }

  qh_t *p_urb_qh = p_purb->p_qh;
  qh_t *p_prev_qh;
  qh_t *p_sched_qh;

  if( p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW){
    p_sched_qh = p_purb->p_uhcd->p_qh_lowspeed;
  } else {
    p_sched_qh = p_purb->p_uhcd->p_qh_fullspeed;
  }

  if(list_empty(&p_sched_qh->qh_link_list)){  // Dieser URB ist erster
    p_prev_qh = p_sched_qh;
  } else {          // andere URBs vorhanden, diesen hinten anhängen
    p_prev_qh = list_entry(p_sched_qh->qh_link_list.prev,qh_t,qh_link_list);
  }

  // An UHC-Schedule-QH anhängen
  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Add to QH-List\n",p_purb->p_urb);
  list_add_tail( &p_urb_qh->qh_link_list, &p_sched_qh->qh_link_list);

  /*
  qh_t *p_next_qh = p_purb->p_uhcd->p_qh_term;
  DBG("VORHER  : ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p\n",p_prev_qh,p_next_qh);
  DBG("DMA     :  0x%p     0x%p\n",(void*)p_prev_qh->dma_handle,(void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->link),(void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element),(void *)le32_to_cpu(p_next_qh->element));
  DBG("VORHER  : ------------   ------------\n");

  DBG("NACHHER : ------------   ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p     0x%p\n",p_prev_qh,p_urb_qh,p_next_qh);
  DBG("DMA     :  0x%p     0x%p     0x%p\n",(void*)p_prev_qh->dma_handle,(void *)p_urb_qh->dma_handle,(void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p --> 0x%p\n",
      (void *)(p_urb_qh->dma_handle | LINK_NO_TERM | LINK_TO_QH),(void *)le32_to_cpu(p_prev_qh->link),
      (void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element),(void *)le32_to_cpu(p_urb_qh->element),(void *)le32_to_cpu(p_next_qh->element));
  DBG("NACHHER : ------------   ------------   ------------\n");

  */

  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: ---SCHEDULE in %s-List ---\n",p_purb->p_urb,
    p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW ? "Low-Speed" : "Full-Speed" );

  p_purb->p_urb->schedule_time = rt_timer_read();

  p_urb_qh->link = p_prev_qh->link;
  wmb();
  p_prev_qh->link = cpu_to_le32( p_urb_qh->dma_handle | LINK_NO_TERM | LINK_TO_QH );

  return 0;
}

// T O D O
static int rt_schedule_int_urb( struct rt_privurb *p_purb )
{
  return 0;
}

// T O D O
static int rt_schedule_isoc_urb( struct rt_privurb *p_purb )
{
  return 0;
}

static int rt_uhci_send_ctrl_urb( struct rt_privurb *p_purb )
{
  if(!p_purb || !p_purb->p_urb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return -ENODEV;
  }

  if( !list_empty(&p_purb->active_td_list) ){
    DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," %s - Active-TD-List is NOT empty !!!\n",__FUNCTION__);
    dump_td_table(p_purb,0);
    return -EINVAL;
  }

  struct rt_urb *p_urb = p_purb->p_urb;

#ifdef DEBUG
  struct usb_ctrlrequest *p_ctrl = p_purb->p_urb->p_setup_packet;
  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Sending CTRL-MESSAGE: RT: 0x%02x, RQ: 0x%02x, Val: 0x%04x, Idx: 0x%04x, %d Byte \n",
           p_urb,p_ctrl->bRequestType, p_ctrl->bRequest, p_ctrl->wValue, p_ctrl->wIndex, p_ctrl->wLength);
#endif

  __u8 toggle   = 1;
  int out       = usb_pipeout(p_urb->pipe);
  int max_pl    = p_purb->max_packet_size;
  int remaining = p_urb->transfer_buffer_length;
  __u32 buff = (__u32) p_urb->transfer_dma;
  int length = 0;

  td_t *p_last = NULL;
  td_t *p_list = p_purb->p_td_table;
  td_t *p_end  = &p_purb->p_td_table[p_purb->anz_tds -1];

  p_purb->p_qh->element = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF );

  // TD[0] (SETUP)
  p_list->status = cpu_to_le32( td_status_active(1) |
                                td_status_actlen(0) |
                                td_status_bits(0) |
                                td_status_lowspeed( p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0 ) |
                                td_status_errcount(MAX_CTRL_RETRIES));
  p_list->token  = cpu_to_le32( td_token_pid(SETUP_TOKEN) |
                                td_token_maxlen(7) |
                                td_token_endpoint( usb_pipeendpoint(p_urb->pipe) ) |
                                td_token_addr( usb_pipedevice(p_urb->pipe) ) |
                                td_token_toggle(0) );
  p_list->buffer = cpu_to_le32( p_urb->setup_dma );

  list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
  list_del_init(&p_list->free_td_list);

  //TD[1-(N-1)] (DATA)
  p_last = p_list;
  p_list++;

  while( remaining > 0 && p_list != p_end){
    length = remaining > max_pl ? max_pl : remaining  ;

    p_list->status = cpu_to_le32( td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed( p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0 ) |
                                  td_status_spd( p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0 ) |
                                  td_status_errcount(MAX_CTRL_RETRIES));
    p_list->token  = cpu_to_le32( td_token_pid( out ? OUT_TOKEN : IN_TOKEN ) |
                                  td_token_maxlen( length - 1) |
                                  td_token_endpoint( usb_pipeendpoint(p_urb->pipe) ) |
                                  td_token_addr( usb_pipedevice(p_urb->pipe) ) |
                                  td_token_toggle(toggle) );
    p_list->buffer = cpu_to_le32( buff );
    buff += length;
    remaining -= length;
    toggle ^= 1;

    p_last->link   = cpu_to_le32( p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF );

    list_add_tail(&p_list->active_td_list,&p_purb->active_td_list);
    list_del_init(&p_list->free_td_list);

    p_last = p_list;
    p_list++;
  }

  // LAST TD (STATUS)
  p_list->status  = cpu_to_le32(  td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed( p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0 ) |
                                  td_status_spd( p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0 ) |
                                  td_status_errcount(0) |
                                  td_status_ioc( 1 ) );
  p_list->token   = cpu_to_le32(  td_token_pid( out ? IN_TOKEN : OUT_TOKEN ) |
                                  td_token_maxlen(0x7ff) |
                                  td_token_endpoint( usb_pipeendpoint(p_urb->pipe) ) |
                                  td_token_addr( usb_pipedevice(p_urb->pipe) ) |
                                  td_token_toggle(1) );
  p_list->link    = cpu_to_le32(  LINK_TERM );
  p_list->buffer  = cpu_to_le32(  0 );

  p_last->link    = cpu_to_le32(  p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF );

  list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
  list_del_init(&p_list->free_td_list);

  //dump_td_table( p_purb,0);

  return rt_schedule_ctrl_bulk_urb( p_purb );
}

static int rt_uhci_send_bulk_urb( struct rt_privurb *p_purb )
{
  if(!p_purb || !p_purb->p_urb){
    ERR2("[ERROR] %s - Invalid Private-URB-Pointer \n",__FUNCTION__);
    return -ENODEV;
  }
  struct rt_urb *p_urb = p_purb->p_urb;

  __u8 toggle   = usb_get_data_toggle(p_urb->p_usbdev,p_urb->pipe);
  int out       = usb_pipeout(p_urb->pipe);
  int max_pl    = p_purb->max_packet_size;
  int remaining = p_urb->transfer_buffer_length;
  __u32 buff    = ( __u32 ) p_urb->transfer_dma;
  int length    = 0;

  td_t *p_last = NULL;
  td_t *p_list = p_purb->p_td_table;
  td_t *p_end  = &p_purb->p_td_table[p_purb->anz_tds -1];

  p_purb->p_qh->element = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF );

  while(remaining > 0 && p_list != p_end){
    length = remaining > max_pl ? max_pl  : remaining  ;

    p_list->buffer = cpu_to_le32( buff );
    buff += length;
    remaining -= length;

    p_list->status = cpu_to_le32( td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed( p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0 ) |
                                  td_status_spd( p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0 ) |
                                  td_status_ioc( 1 ) | //remaining ? 0 : 1 ) |
                                  td_status_errcount(0) );
    p_list->token  = cpu_to_le32( td_token_pid( out ? OUT_TOKEN : IN_TOKEN ) |
                                  td_token_endpoint( usb_pipeendpoint(p_urb->pipe) ) |
                                  td_token_addr( usb_pipedevice(p_urb->pipe) ) |
                                  td_token_maxlen( length -1) |
                                  td_token_toggle(toggle) );
    toggle ^= 1;

    if(p_last){
      p_last->link = cpu_to_le32( p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF );
    }
    if(!remaining){
      usb_set_data_toggle(p_urb->p_usbdev,p_urb->pipe, toggle);
      p_list->link = cpu_to_le32( LINK_TERM );
    }

    list_add_tail(&p_list->active_td_list,&p_purb->active_td_list);
    list_del_init(&p_list->free_td_list);

    p_last = p_list;
    p_list++;
  }

  //dump_td_table( p_purb,0);

  return rt_schedule_ctrl_bulk_urb( p_purb );
}

// T O D O
static int rt_uhci_send_int_urb( struct rt_privurb *p_purb )
{
  rt_schedule_int_urb(p_purb);
  rt_unschedule_int_urb(p_purb);
  return 0;
}

// T O D O
static int rt_uhci_send_isoc_urb( struct rt_privurb *p_purb )
{
  rt_schedule_isoc_urb(p_purb);
  rt_unschedule_isoc_urb(p_purb);
  return 0;
}

static inline void rt_complete_urb( struct rt_privurb *p_purb )
{
  p_purb->status = PURB_IN_HANDLE;
  DBG_MSG2(p_purb->p_urb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: PURB_IN_HANDLE \n",p_purb->p_urb);

  switch(usb_pipetype(p_purb->p_urb->pipe)){
    case USB_ENDPOINT_XFER_CONTROL:
      rt_unschedule_ctrl_bulk_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_BULK:
      rt_unschedule_ctrl_bulk_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_INT:
      rt_unschedule_int_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_ISOC:
      rt_unschedule_isoc_urb(p_purb);
      break;
  }

  p_purb->p_urb->actual_length = rt_uhci_bytes_send(p_purb);

  DBG_MSG2(p_purb->p_urb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: %d / %d Byte %s\n",p_purb->p_urb,
           p_purb->p_urb->actual_length, p_purb->p_urb->transfer_buffer_length,
           usb_pipeout(p_purb->p_urb->pipe) ? "sent" : "received");

  // Unmap DMA
  unmap_dma(p_purb);


  DBG_MSG2(p_purb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: Free TDs\n",p_purb->p_urb);
  rt_free_urb_tds(p_purb);

  p_purb->status = PURB_IDLE;
  DBG_MSG2(p_purb->p_urb->p_hcd,p_purb->p_urb->p_usbdev," URB 0x%p: PURB_IDLE \n",p_purb->p_urb);

}

static inline int rt_uhci_send_busywait_urb( struct rt_privurb *p_purb )
{
  struct rt_urb *p_urb = p_purb->p_urb;
  int ret = 0;

  /* Mappe evtl DMA */
  ret = map_dma(p_purb);
  if(ret){
    dump_urb(p_urb);
    return -ENOMEM;
  }

  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: BUSY WAIT\n",p_urb);

  p_purb->status = PURB_IN_PROGRESS;
  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IN_PROGRESS \n",p_urb);

  switch(usb_pipetype(p_urb->pipe)){
    case USB_ENDPOINT_XFER_CONTROL:
      ret = rt_uhci_send_ctrl_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_BULK:
      ret = rt_uhci_send_bulk_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_INT:
      ret = rt_uhci_send_int_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_ISOC:
      ret = rt_uhci_send_isoc_urb(p_purb);
      break;
  }
  if(ret){
    p_purb->status = PURB_IDLE;
    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IDLE \n",p_urb);
    unmap_dma(p_purb);
    return ret;
  }

  ret = wait_for_urb(p_purb);
  if(ret){
    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p BUSY-WAIT -> TIMEOUT\n",p_urb);
    dump_td_table(p_purb,0);
  }

  rt_complete_urb(p_purb);

  return p_urb->actual_length;
}

static inline int rt_uhci_send_semaphore_urb( struct rt_privurb *p_purb )
{
  struct rt_urb *p_urb = p_purb->p_urb;
  int ret = 0;

  /* Mappe evtl DMA */
  ret = map_dma(p_purb);
  if(ret){
    dump_urb(p_urb);
    return -ENOMEM;
  }

  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: SLEEPING MAX %llu ns\n",p_urb,p_urb->rt_sem_timeout);

  p_purb->status = PURB_IN_PROGRESS;
  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IN_PROGRESS \n",p_urb);

  switch( usb_pipetype(p_urb->pipe) ){
    case USB_ENDPOINT_XFER_CONTROL:
      ret = rt_uhci_send_ctrl_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_BULK:
      ret = rt_uhci_send_bulk_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_INT:
      ret = rt_uhci_send_int_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_ISOC:
      ret = rt_uhci_send_isoc_urb(p_purb);
      break;
  }
  if(ret){
    p_purb->status = PURB_IDLE;
    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IDLE \n",p_urb);
    unmap_dma(p_purb);
    return ret;
  }

  ret = rt_sem_p( &p_purb->p_urb->rt_sem, p_purb->p_urb->rt_sem_timeout);
  switch(ret){
    case -EINVAL:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM is not a semaphore descriptor \n",p_urb);
      break;
    case -EIDRM:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM is a deleted semaphore descriptor \n",p_urb);
      break;
    case -EWOULDBLOCK:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM would block \n",p_urb);
      break;
    case -EINTR:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM interrupted \n",p_urb);
      break;
    case -ETIMEDOUT:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM timed out \n",p_urb);
      break;
    case -EPERM:
      DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM cannot sleep \n",p_urb);
      break;
    case 0:
      if( p_purb->status & PURB_WAKE_UP_BY_IRQ ){
        DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM waked up by ISR, Sleeptime: (%llu ns)\n",
        p_urb, rt_timer_read() - p_urb->schedule_time );
      } else {
        DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: RT_SEM waked up by someone else, sleeptime: %llu ns\n",
        p_urb, rt_timer_read() - p_urb->schedule_time );
      }
      break;
  }

  rt_complete_urb(p_purb);

  return p_urb->actual_length;
}

static inline int rt_uhci_send_callback_urb( struct rt_privurb *p_purb )
{
  struct rt_urb *p_urb = p_purb->p_urb;
  int ret = 0;

  /* Mappe evtl DMA */
  ret = map_dma(p_purb);
  if(ret){
    dump_urb(p_urb);
    return -ENOMEM;
  }

  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Sent Callback\n",p_urb);

  p_purb->status = PURB_IN_PROGRESS;
  DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IN_PROGRESS \n",p_urb);

  switch(usb_pipetype(p_urb->pipe)){
    case USB_ENDPOINT_XFER_CONTROL:
      ret = rt_uhci_send_ctrl_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_BULK:
      ret = rt_uhci_send_bulk_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_INT:
      ret = rt_uhci_send_int_urb(p_purb);
      break;
    case USB_ENDPOINT_XFER_ISOC:
      ret = rt_uhci_send_isoc_urb(p_purb);
      break;
  }
  if(ret){
    p_purb->status = PURB_IDLE;
    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: PURB_IDLE \n",p_urb);
    unmap_dma(p_purb);
    return ret;
  }

  return 0;

}

/******************************************************************/
/*  HCD-FKT Funktionen                                            */
/******************************************************************/

void nrt_uhci_remove_controller( struct uhc_device *p_uhcd )
{
  if(p_uhcd->status & UHC_REGISTERED){
    nrt_hcd_unregister_driver( p_uhcd->p_hcd );
  }

  if(p_uhcd->status & UHC_RUNNING){
    uhc_stop(p_uhcd);
  }

  if(p_uhcd->p_hcd){
    kfree(p_uhcd->p_hcd);
    alloc_bytes -= sizeof(struct hc_device);
  }

  destroy_td(p_uhcd->p_td_loop);
  destroy_qh(p_uhcd->p_qh_fullspeed);
  destroy_qh(p_uhcd->p_qh_lowspeed);
  destroy_qh(p_uhcd->p_qh_term);

  destroy_flame_list(p_uhcd);

  put_irq(p_uhcd);

  release_ioport(p_uhcd);
}

void nrt_uhci_init_controller( struct uhc_device *p_uhcd )
{
  if(!p_uhcd){
    return;
  }

  /* Initialize Controller */
  if (uhc_init( p_uhcd ) ){
    return;
  }
  p_uhcd->status |= UHC_INITIALIZED;

  /* Starte Controller */
  if( uhc_start( p_uhcd ) ){
    return;
  }
  p_uhcd->status |= UHC_RUNNING;

  return;
}

int nrt_uhci_search_controller( struct uhc_device *p_uhcd )
{
  struct pci_dev *p_pcidev_new = NULL;

start_search:
    uhc_clear(p_uhcd);

  /* searching controller */
  p_pcidev_new = pci_find_class(UHC_CLASS_CODE,p_pcidev_old);
  if(!p_pcidev_new) {
    p_pcidev_old = NULL;
    return 1; // No device found
  }
  p_pcidev_old = p_pcidev_new;

  /* device located but not wanted */
  if(device && device != p_pcidev_new->device){
    goto start_search;
  }

  /* Device gefunden */
  PRNT("USB Universal Host Controller found : Vendor = 0x%04x, Device = 0x%04x\n",
       p_pcidev_new->vendor,p_pcidev_new->device);

  struct hc_device *p_hcd = kmalloc( sizeof(struct hc_device),GFP_KERNEL);
  if(!p_hcd){
    return -1;
  }
  alloc_bytes += sizeof(struct hc_device);

  p_hcd->type  = 0x00;  /* UHC Device */
  p_hcd->p_hcd_fkt = &hcd_fkt;

  p_uhcd->p_pcidev = p_pcidev_new;
  p_uhcd->p_hcd = p_hcd;

  /* Pruefe ob Ressourcen schon belegt*/
  if(request_ioport(p_uhcd)){
    kfree(p_hcd);
    alloc_bytes -= sizeof(struct hc_device);
    goto start_search;
  }

  if(get_irq(p_uhcd)){
    release_ioport(p_uhcd);
    kfree(p_hcd);
    alloc_bytes -= sizeof(struct hc_device);
    goto start_search;
  }

  return 0;

}

int nrt_uhci_register_urb( struct rt_urb *p_urb)
{
  /*
  In Core überprüft:
  p_urb
  p_urb->p_hcd
  p_urb->p_hcd->p_hcd_fkt
  p_urb->max_packet_size
  p_urb->max_buffer_len
  */

  int i;
  struct uhc_device *p_uhcd = NULL;
  for(i=0;i< MAX_UHC_CONTROLLER;i++){
    if( uhc_dev[i].p_hcd == p_urb->p_hcd){
      p_uhcd = &uhc_dev[i];
      break;
    }
  }

  if(!p_uhcd || !p_uhcd->p_pcidev ){
    ERR2("[ERROR] %s - Invalid Pointer to UHC \n",__FUNCTION__);
    return -ENODEV;
  }

  if(p_uhcd->status & UHC_HOST_ERROR){
    ERR2("[ERROR] %s - Host-Controller-Error, REJECT \n",__FUNCTION__);
    return -ENODEV;
  }

  if( !(p_uhcd->status & UHC_RUNNING) ) {
    ERR2("[ERROR] %s - Host-Controller not running, REJECT \n",__FUNCTION__);
    return -ENODEV;
  }

  /* Pruefe, ob URB schon registriert */
  struct rt_privurb *p_purb = NULL;
  struct list_head *p_list = p_uhcd->reg_urb_list.next;;

  while(p_list != &p_uhcd->reg_urb_list){
    p_purb = list_entry(p_list, struct rt_privurb, reg_urb_list);
    if( p_purb->p_urb == p_urb){
      break;
    } else {
      p_purb = NULL;
    }
    p_list = p_list->next;
  }

  if( p_purb ){
    DBG_MSG1(p_purb->p_hcd," URB 0x%p is already registered\n",p_urb);
    return -EINVAL;
  }

  /* Erstelle struct rt_privurb */
  p_purb = kmalloc(sizeof(struct rt_privurb), GFP_ATOMIC);
  if(!p_purb){
    ERR_MSG1(p_urb->p_hcd," %s - No Memory for Private URB-Data \n",__FUNCTION__);
    return -ENOMEM;
  }

  alloc_bytes += sizeof(struct rt_privurb);

  clear_rt_privurb(p_purb);

  p_purb->p_uhcd          = p_uhcd;
  p_purb->p_urb           = p_urb;
  p_purb->p_hcd           = p_urb->p_hcd;
  p_purb->max_packet_size = p_urb->max_packet_size;
  p_purb->max_buffer_len  = p_urb->max_buffer_len;
  p_purb->status          = PURB_IDLE;

  p_urb->p_private        = p_purb;

  /* Erstelle TD-Tabelle anhand der Paketgroesse des Endpoints und der max Puffergroesse */
  if(p_purb->max_buffer_len % p_purb->max_packet_size){
    p_purb->anz_tds = (p_purb->max_buffer_len / p_purb->max_packet_size) + 1;
  } else {
    p_purb->anz_tds = p_purb->max_buffer_len / p_purb->max_packet_size;
  }

  /* Control-Transfer braucht 2 TDs mehr (Setup & Status) */
  p_purb->anz_tds += 2;

//  DBG_MSG1(p_urb->p_hcd," Create %d TDs. Max_Packet = %d, Max_Buffer = %d\n",
//      p_purb->anz_tds, p_purb->max_packet_size, p_purb->max_buffer_len);

  int ret = create_td_table( p_purb );
  if(ret){
    kfree(p_purb);
    alloc_bytes -= sizeof(struct rt_privurb);

    ERR_MSG1(p_urb->p_hcd," %s - Error while creating TD-Table \n",__FUNCTION__);
    return ret;
  }

  /* Erstelle Queue-Header */
  p_purb->p_qh = create_qh( p_uhcd );
  if( !p_purb->p_qh ){
    destroy_td_table( p_purb );
    kfree(p_purb);
    alloc_bytes -= 1 * sizeof(struct rt_privurb);
    ERR_MSG1(p_purb->p_hcd," %s - Error while creating Queue-Header \n",__FUNCTION__);
    return ret;
  }

  list_add_tail( &p_purb->reg_urb_list, &p_uhcd->reg_urb_list);
  p_purb->get_module = try_module_get(THIS_MODULE);

  DBG_MSG1(p_urb->p_hcd," URB @ 0x%p registered, %d TDs, Max-Buffer: %d, Max-Packet-Size: %d\n",
    p_urb, p_purb->anz_tds, p_purb->max_buffer_len, p_purb->max_packet_size );

  return 0;
}

int nrt_uhci_unregister_urb( struct rt_urb *p_urb)
{
  /*
  In Core überprüft:
  p_urb
  p_urb->p_hcd
  p_urb->p_hcd->p_hcd_fkt
  */

  struct rt_privurb *p_purb = p_urb->p_private;
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Pointer to Private URB-Data \n",__FUNCTION__);
    return -ENODEV;
  }

  struct uhc_device *p_uhcd = p_purb->p_uhcd;
  if(!p_uhcd){
    ERR2("[ERROR] %s - Invalid Pointer to UHC \n",__FUNCTION__);
    return -ENODEV;
  }

  if( (p_purb->status == PURB_IN_PROGRESS) ||
        (p_purb->status == PURB_IN_HANDLE) ){

    ERR_MSG1(p_purb->p_hcd," %s - URB 0x%p: IN PROGRESS, UNSCHEDULE !!!\n",__FUNCTION__,p_purb->p_urb);

    rt_complete_urb(p_purb);

    return 0;
  }

  list_del_init(&p_purb->reg_urb_list);

  if(p_purb->get_module) {
    module_put(THIS_MODULE);
  }

  /* Lösche QH */
  destroy_qh(p_purb->p_qh);

  /* Lösche TD-Tabelle */
  destroy_td_table(p_purb);

  /* Lösche urb_priv */
  kfree(p_purb);
  alloc_bytes -= 1 * sizeof(struct rt_privurb);

  DBG_MSG1(p_purb->p_hcd," URB 0x%p unregistered\n",p_urb);
  return 0;
}

int rt_uhci_submit_urb(struct rt_urb *p_urb , __u16 urb_submit_flags )
{
  /*
  In Core überprüft:
  p_urb
  p_urb->p_hcd
  p_urb->p_hcd->p_hcd_fkt
  ...
  */

  struct rt_privurb *p_purb = p_urb->p_private;
  if(!p_purb){
    ERR2("[ERROR] %s - Invalid Pointer to Private URB-Data \n",__FUNCTION__);
    return -ENODEV;
  }

  struct uhc_device *p_uhcd = p_purb->p_uhcd;
  if(!p_uhcd){
    ERR2("[ERROR] %s - Invalid Pointer to UHC \n",__FUNCTION__);
    return -ENODEV;
  }

  if( p_uhcd->status & UHC_HOST_ERROR ) {
    ERR2("[ERROR] %s - Host-Controller-Error, REJECT \n",__FUNCTION__);
    return -ENODEV;
  }

  if( !(p_uhcd->status & UHC_RUNNING) ) {
    ERR2("[ERROR] %s - Host-Controller not running, REJECT \n",__FUNCTION__);
    return -ENODEV;
  }

  /* Pruefe, ob sich die konstanten Werte geändert haben */
  if(p_urb->p_hcd != p_purb->p_hcd ){
    ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Invalid Host-Controller: 0x%p sent, 0x%p allowed\n",
        __FUNCTION__, p_urb->p_hcd, p_purb->p_hcd);
    return -EINVAL;
  }

  if(p_urb->max_packet_size != p_purb->max_packet_size ){
    ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Invalid Max-Packet-Size: %d sent, %d allowed \n",
        __FUNCTION__, p_urb->max_packet_size, p_purb->max_packet_size );
    return -EINVAL;
  }

  if(p_urb->max_buffer_len != p_purb->max_buffer_len ){
    ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," %s - Invalid Max-Buffer-Length: %d sent, %d allowed \n",
        __FUNCTION__, p_urb->max_buffer_len, p_purb->max_buffer_len );
    return -EINVAL;
  }

  if( !(p_purb->status == PURB_IDLE) ){
    ERR_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: URB IN PROGRESS \n",p_urb);
    return -EBUSY;
  }

  /* Pruefe Complete-Function */
  if( urb_submit_flags & URB_WAIT_SEM ){

    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Blocking URB (Semaphore) \n",p_urb);
    p_purb->urb_wait_flags = URB_WAIT_SEM;

  } else if( urb_submit_flags & URB_CALLBACK ){

    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Non-Blocking URB (Callback-Function @ 0x%p) \n",p_urb,p_urb->rt_complete_fkt);
    p_purb->urb_wait_flags = URB_CALLBACK;

  } else if( urb_submit_flags & URB_WAIT_BUSY ){

    DBG_MSG2(p_urb->p_hcd,p_urb->p_usbdev," URB 0x%p: Blocking URB (Busy Wait) \n",p_urb);
    p_purb->urb_wait_flags = URB_WAIT_BUSY;

  }

  // save pipe
  p_purb->pipe_save = p_urb->pipe;

  // OK - send URB
  switch(p_purb->urb_wait_flags){

    case URB_WAIT_BUSY:

      return rt_uhci_send_busywait_urb(p_purb);

    case URB_WAIT_SEM:

      return rt_uhci_send_semaphore_urb(p_purb);

    case URB_CALLBACK:

      return rt_uhci_send_callback_urb(p_purb);
  }

  return -EINVAL;
}

/******************************************************************/
/*  Modul - Funktionen                                            */
/******************************************************************/

int __init mod_start(void)
{
  PRNT(KERN_INFO "********** " DRIVER_DESC " " DRIVER_VERSION " ***********\n");

  int i,ret;

  /* Controller Functions */
  hcd_fkt.nrt_hcd_register_urb        = nrt_uhci_register_urb;
  hcd_fkt.nrt_hcd_unregister_urb      = nrt_uhci_unregister_urb;
  hcd_fkt.rt_hcd_submit_urb           = rt_uhci_submit_urb;

  /* Root-Hub Functions */
  hcd_fkt.nrt_hcd_poll_root_hub_port  = nrt_uhci_poll_root_hub_port;

  anz_uhc_ctrl = 0;
  struct uhc_device *p_uhcd;
  PRNT("RT-UHC-Driver: Searching for UHCI-Host-Controller \n");
  for(i=0;i < MAX_UHC_CONTROLLER;i++) {
    p_uhcd = &uhc_dev[i];
    p_uhcd->uhcd_nr = i;
    ret = nrt_uhci_search_controller(p_uhcd);
    if(!ret){

      nrt_uhci_init_controller(p_uhcd);
      if(!(p_uhcd->status & UHC_INITIALIZED) || !(p_uhcd->status & UHC_RUNNING) ){
        PRNT("RT-UHC-Driver: Error while initializing UHC (%d)\n",ret);
        nrt_uhci_remove_controller(p_uhcd);
        break;
      }

      ret = nrt_hcd_register_driver(p_uhcd->p_hcd);
      p_uhcd->p_hcd->p_private = (void *)p_uhcd;

      if(ret){
        PRNT("RT-UHC-Driver: Error while registering UHC (%d)\n",ret);
        nrt_uhci_remove_controller(p_uhcd);
        break;
      }
      p_uhcd->status |= UHC_REGISTERED;

      anz_uhc_ctrl++;
      DBG_MSG1(p_uhcd->p_hcd," USB Universal Host Controller @ 0x%p registered \n",p_uhcd);

      p_uhcd->p_hcd->p_hcd_fkt->nrt_usb_search_devices(p_uhcd->p_hcd);

    } else {
      break;
    }
  }

  if(!anz_uhc_ctrl){
    PRNT("RT-UHC-Driver: No UHC-Driver registered -> Exit Module \n");
    return -ENODEV;
  }

  RTIME start, end;
  start = rt_timer_read();
  end = rt_timer_read();
  rt_timer_overhead = end - start;

  PRNT("RT-UHC-Driver: Loading Completed (%d Byte allocated) \n",alloc_bytes);


  return 0;
}

void mod_exit(void)
{
  int i=0;
  for(i=0; i < anz_uhc_ctrl; i++) {

    nrt_uhci_remove_controller(&uhc_dev[i]);
  }

  PRNT("RT-UHC-Driver : Unloading complete (%d byte allocated)\n",alloc_bytes);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_init (mod_start);
module_exit (mod_exit);
