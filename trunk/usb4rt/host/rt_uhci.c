/***************************************************************************
 *   Copyright (C) 2005, 2006 by Joerg Langenberg                          *
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include "rt_uhci.h"
#include "rt_uhci_hub.h"
#include <core/rt_usb_debug.h>
#include <core/usb4rt_config.h>

#define DRIVER_VERSION USB4RT_PACKAGE_VERSION
#define DRIVER_AUTHOR "Joerg Langenberg - joergel@gmx.net"
#define DRIVER_DESC "Realtime Driver for Universal Host Controller"

/* init module-parameter */
/*
int list=0;
MODULE_PARM (list, "i");
MODULE_PARM_DESC (list, "Listet alle UHCI-Controller auf");
*/
#define MAX_DEVICES    4

static int deviceid[MAX_DEVICES];
static int irq[MAX_DEVICES];
static int ioport[MAX_DEVICES];

static int deviceid_c;
static int irq_c;
static int ioport_c;

module_param_array(deviceid, int, &deviceid_c, 0);
MODULE_PARM_DESC (deviceid, "Load driver only for controller with a specific device-id (HEX).");

module_param_array(irq, int, &irq_c, 0);
MODULE_PARM_DESC (irq, "Load driver only for controller with a specific irq-number (INT).");

module_param_array(ioport, int, &ioport_c, 0);
MODULE_PARM_DESC (ioport, "Load driver only for controller with a specific io-port (HEX).");

/* rt_uhci_hub */
extern int rh_init(struct uhc_device *p_uhcd);
extern struct usb_device *nrt_uhci_poll_root_hub_port(struct hc_device *p_hcd , __u8 rh_port_nr);

unsigned short anz_uhc_ctrl;
unsigned short reg_uhc_ctrl;
struct uhc_device uhc_dev[MAX_DEVICES];
struct uhc_irq uhc_irq_tab[MAX_DEVICES];
struct hcd_funktions hcd_fkt;
struct pci_dev *p_pcidev_old = NULL;
unsigned int alloc_bytes = 0;
uint64_t rt_timer_overhead = 0;
uint64_t frame_start;

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

static void dump_td_table(struct rt_privurb *p_purb, int all)
{
  u32 link, status, token, buffer;
  struct list_head *p_list;
  td_t *p_td = NULL;

  if (!p_purb || !p_purb->p_td_table){
    return;
  }

  p_list = p_purb->active_td_list.next;

  PRNT("========== DUMP TD-TABLE @ 0x%p (%d TDs, %d Byte) =========== \n", p_purb->p_td_table, p_purb->anz_tds, p_purb->table_size);
  PRNT("   URB         @ 0x%p \n", p_purb->p_urb);
  PRNT("   PRIVATE-URB @ 0x%p \n", p_purb);
  if (p_purb->p_qh){
    PRNT("   QH          @ 0x%p , DMA: 0x%p \n", p_purb->p_qh, (void *)p_purb->p_qh->dma_handle);
  }


  PRNT("A    NR         TD        LINK      STATUS       TOKEN      BUFFER  ACT ACTLEN MAXLEN TYPE ERRCNT STALL DBUF BABL TIME BITST SPD IOC NAK DT ADR EP LS\n");

  while (p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list, td_t, active_td_list);

    link   = le32_to_cpu(p_td->link);
    status = td_status(p_td);
    token  = td_token(p_td);
    buffer = le32_to_cpu(p_td->buffer);

    PRNT("%s  %4d 0x%p  0x%p  0x%p  0x%p  0x%p %4s %6d %6d %4x %6d %5s %4s %4s %4s %5s %3s %3s %3s %2s %3d %2d %2s\n",
         "*", p_td->td_nr,
         (void *)p_td->dma_handle, (void *)link, (void *)status, (void *)token, (void *)buffer,
         (get_status_active(p_td) ? "X" : "-"), get_status_actlen(p_td), get_max_len(p_td), get_packet_type(p_td),
         get_status_errcount(p_td), (status & TD_STAT_STALLED) ? "X":"-", (status & TD_STAT_DBUFERR) ? "X":"-",
         (status & TD_STAT_BABBLE)?"X":"-", (status & TD_STAT_CRCTIMEO)? "X":"-", (status & TD_STAT_BITSTUFF) ? "X":"-",
         (status & TD_STAT_SPD) ? "X":"-", (status & TD_STAT_IOC) ?"X":"-", (status & TD_STAT_NAK) ? "X":"-",
         (token & TD_TOKEN_TOGGLE) ? "1":"0", get_address(p_td), get_endpoint(p_td), (status & TD_STAT_LS) ? "X":"-");
    p_list = p_list->next;
  }

  if (!all){
    PRNT("========== END DUMP ========= \n");
    return;
  }
  PRNT("------------------------------------------------------------------------------------------------------------------------------------------------------\n");

  p_list = p_purb->free_td_list.next;
  while (p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list, td_t, active_td_list);

    link   = le32_to_cpu(p_td->link);
    status = td_status(p_td);
    token  = td_token(p_td);
    buffer = le32_to_cpu(p_td->buffer);

    PRNT("%s  %4d 0x%p  0x%p  0x%p  0x%p  0x%p %4s %6d %6d %4x %6d %5s %4s %4s %4s %5s %3s %3s %3s %2s %3d %2d %2s\n",
         "-", p_td->td_nr,
         (void *)p_td->dma_handle, (void *)link, (void *)status, (void *)token, (void *)buffer,
         (get_status_active(p_td) ? "X" : "-"), get_status_actlen(p_td), get_max_len(p_td), get_packet_type(p_td),
         get_status_errcount(p_td), (status & TD_STAT_STALLED) ? "X":"-", (status & TD_STAT_DBUFERR) ? "X":"-",
         (status & TD_STAT_BABBLE)?"X":"-", (status & TD_STAT_CRCTIMEO)? "X":"-", (status & TD_STAT_BITSTUFF) ? "X":"-",
         (status & TD_STAT_SPD) ? "X":"-", (status & TD_STAT_IOC) ?"X":"-", (status & TD_STAT_NAK) ? "X":"-",
         (token & TD_TOKEN_TOGGLE) ? "1":"0", get_address(p_td), get_endpoint(p_td), (status & TD_STAT_LS) ? "X":"-");
    p_list = p_list->next;
  }

  PRNT("========== END DUMP ========= \n");
}

static int create_td_table(struct rt_privurb *p_purb)
{
  td_t *p_td_table;
  td_t *p_list;
  dma_addr_t td_dma;
  int td_size = sizeof(td_t);
  int i;

  p_purb->table_size = td_size * p_purb->anz_tds;

  p_td_table = (td_t *)dma_alloc_coherent(&p_purb->p_uhcd->p_pcidev->dev,
                                          p_purb->table_size,
                                          &td_dma,
                                          0);
  if (!p_td_table || !td_dma){
    return -ENOMEM;
  }

  TD_MSG1(p_purb->p_hcd, " TD-Table for URB 0x%p created @ 0x%p, DMA: 0x%p (%d Byte)\n",
           p_purb->p_urb, p_td_table, (void *)td_dma, p_purb->table_size);

  for(i=0; i < p_purb->anz_tds ; i++){
    p_list = &p_td_table[i];

    memset(p_list, 0, td_size);
    p_list->link   = cpu_to_le32(LINK_TERM);
    INIT_LIST_HEAD(&p_list->free_td_list);
    INIT_LIST_HEAD(&p_list->active_td_list);

    p_list->td_nr = i;
    p_list->dma_handle = td_dma + (i * td_size);
    p_list->p_uhcd = p_purb->p_uhcd;

    list_add_tail(&p_list->free_td_list, &p_purb->free_td_list);

    p_purb->free_tds++;

  }

  p_purb->p_td_table   = p_td_table;
  p_purb->td_table_dma = td_dma;

  TD_MSG1(p_purb->p_hcd, " %d free TDs for URB 0x%p available\n", p_purb->free_tds, p_purb->p_urb);

  return 0;
}

static void destroy_td_table(struct rt_privurb *p_purb)
{
  if (!p_purb->p_td_table || !p_purb->td_table_dma){
    return;
  }

  TD_DBG("RT-UHC-Driver: Deleting TD-Table for URB 0x%p (%d Byte)\n", p_purb->p_urb, p_purb->table_size);

  dma_free_coherent( &p_purb->p_uhcd->p_pcidev->dev,
                      p_purb->table_size,
                      p_purb->p_td_table,
                      p_purb->td_table_dma);
  p_purb->p_td_table = NULL;
  p_purb->free_tds   = 0;
  return;
}

static td_t *create_td(struct uhc_device *p_uhcd)
{
  td_t *p_td;
  dma_addr_t td_dma;

  p_td = (td_t *)pci_alloc_consistent(p_uhcd->p_pcidev,
                                      sizeof(td_t),
                                      &td_dma);
  if (!p_td || !td_dma){
    return NULL;
  }
  TD_DBG("RT-UHC-Driver: Creating TD @ 0x%p, DMA: 0x%p (%d Byte)\n", p_td, (void *)td_dma, sizeof(td_t));

  memset(p_td, 0, sizeof(td_t));
  p_td->link   = cpu_to_le32(LINK_TERM);
  INIT_LIST_HEAD(&p_td->free_td_list);
  INIT_LIST_HEAD(&p_td->active_td_list);

  p_td->p_uhcd = p_uhcd;
  p_td->dma_handle = td_dma;

  return p_td;
}

static void destroy_td(td_t *p_td)
{
  if (!p_td || !p_td->dma_handle || !p_td->p_uhcd){
    return;
  }

  TD_DBG("RT-UHC-Driver: Deleting TD @ 0x%p (%d Byte)\n", p_td, sizeof(td_t));

  pci_free_consistent(p_td->p_uhcd->p_pcidev,
                      sizeof(td_t),
                      p_td,
                      p_td->dma_handle);

  return;
}

 /**************************************************************/
/*  QUEUE-HEADER Funktionen                                    */
/***************************************************************/

static qh_t *create_qh(struct uhc_device *p_uhcd)
{
  qh_t *p_qh;
  dma_addr_t qh_dma;

  p_qh = (qh_t *)pci_alloc_consistent(p_uhcd->p_pcidev,
                                      sizeof(qh_t),
                                      &qh_dma);
  if (!p_qh || !qh_dma){
    return NULL;
  }
  QH_DBG("RT-UHC-Driver: Creating QH @ 0x%p, DMA: 0x%p (%d Byte)\n", p_qh, (void *)qh_dma, sizeof(qh_t));

  memset(p_qh, 0, sizeof(qh_t));
  p_qh->link    = cpu_to_le32(LINK_TERM);
  p_qh->element = cpu_to_le32(LINK_TERM);
  INIT_LIST_HEAD(&p_qh->qh_link_list);

  p_qh->p_uhcd = p_uhcd;
  p_qh->dma_handle = qh_dma;

  return p_qh;
}

static void destroy_qh(qh_t *p_qh)
{
  if (!p_qh || !p_qh->dma_handle || !p_qh->p_uhcd){
    return;
  }

  QH_DBG("RT-UHC-Driver: Deleting QH @ 0x%p (%d Byte)\n", p_qh, sizeof(qh_t));

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
 * - NO DEBUG-MESSAGES -
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0 bei Erfolg
 * @return -ENODEV, wenn p_hdc nicht auf ein gueltiges struct hc_device zeigt.
 * @return -ENOMEM, wenn kein Speicher vorhanden oder Frame-List-Pointer ungueltig.
 * @return -EBUSY, Frameliste existiert bereits fuer diesen Controller.
 */
static int init_framelist(struct uhc_device *p_uhcd)
{
  int i;
  struct frame_list *p_fl;
  dma_addr_t fl_dma;

  if (!p_uhcd->p_pcidev){
    return -ENODEV;
  }
  if (p_uhcd->p_fl){
    return -EBUSY;
  }

  p_fl = (struct frame_list *)dma_alloc_coherent(&p_uhcd->p_pcidev->dev,
                                                  sizeof(struct frame_list),
                                                  &fl_dma,
                                                  0);
  if (!p_fl || !fl_dma){
    return -ENOMEM;
  }

  memset(p_fl, 0, sizeof(struct frame_list));

  p_uhcd->p_fl = p_fl;
  p_uhcd->p_fl->dma_handle = fl_dma;

  for(i=0; i<1024;i++){
    p_uhcd->p_fl->frame_dma[i] = LINK_TERM;
    p_uhcd->p_fl->frame_cpu[i] = NULL;
  }

  return 0;
}

/**
 * Release the memory of the framelist.
 * @param hcd_nr number of the host-controller
 */
static void destroy_flame_list(struct uhc_device *p_uhcd)
{
  if (!p_uhcd){
    return;
  }
  if (!p_uhcd->p_fl || !p_uhcd->p_fl->dma_handle){
    return;
  }

  DBG("RT-UHC-Driver: Deleting Framelist @ 0x%p (%d Byte)\n", p_uhcd->p_fl, sizeof(struct frame_list));

  dma_free_coherent(&p_uhcd->p_pcidev->dev, sizeof(struct frame_list), p_uhcd->p_fl, p_uhcd->p_fl->dma_handle);
  p_uhcd->p_fl = NULL;

  return;
}

/**
 * Insert QH-pointer into the frame-list.
 * @param p_qh Pointer to a Queue-Head.
 * @param p_fl Pointer to the frame_list.
 * @param nr Number of the frame-list entry.
 * @return -EINVAL The pointers p_qh or p_fl are invalid. Furthermore the index could be wrong.
 * @return 0, successful.
 */
static int append_qh_on_fl(qh_t *p_qh, struct frame_list *p_fl, int nr)
{
  if (!p_qh || !p_qh->dma_handle || !p_fl){
    return -EINVAL;
  }
  if (nr < 0 || nr > MAX_FRAMES){
    return -EINVAL;
  }
  p_fl->frame_dma[nr] = cpu_to_le32(p_qh->dma_handle | LINK_TO_QH | LINK_NO_TERM);
  p_fl->frame_cpu[nr] = p_qh;

  return 0;
}

/******************************************************************************/
/*                   UHC - Functions                                          */
/*               (for any Controllers)                                      */
/******************************************************************************/

/**
 * Initialisiert die grundlegende Datenstruktur fuer einen Universal Host Controller.
 * Wenn ein Fehler aufteten sollte, werden alle erzeugten Datenstrukturen wieder geloescht.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, if p_uhcd invalid
 * @return -EINVAL,
 * @return -ENOMEM, no memory for HCD available
 */
static int init_skel(struct uhc_device *p_uhcd)
{
  int i;
  int ret = 0;
  if (!p_uhcd){
    return -ENODEV;
  }

  p_uhcd->p_qh_lowspeed = create_qh(p_uhcd);
  if (!p_uhcd->p_qh_lowspeed) {
    goto err_skel_ls;
  }
  QH_DBG("RT-UHC-Driver: Low-Speed-QH  @ 0x%p, DMA: 0x%p \n", p_uhcd->p_qh_lowspeed, (void *)p_uhcd->p_qh_lowspeed->dma_handle);

  p_uhcd->p_qh_fullspeed = create_qh(p_uhcd);
  if (!p_uhcd->p_qh_fullspeed) {
    goto err_skel_fs;
  }
  QH_DBG("RT-UHC-Driver: Full-Speed-QH  @ 0x%p, DMA: 0x%p \n", p_uhcd->p_qh_fullspeed, (void *)p_uhcd->p_qh_fullspeed->dma_handle);

  p_uhcd->p_qh_term = create_qh(p_uhcd);
  if (!p_uhcd->p_qh_term) {
    goto err_skel_term;
  }
  QH_DBG("RT-UHC-Driver: Terminate-QH @ 0x%p, DMA: 0x%p \n", p_uhcd->p_qh_term, (void *)p_uhcd->p_qh_term->dma_handle);

  p_uhcd->p_td_loop = create_td(p_uhcd);
  if (!p_uhcd->p_td_loop) {
    goto err_skel_loop;
  }
  TD_DBG("RT-UHC-Driver: Loop-TD      @ 0x%p, DMA: 0x%p \n", p_uhcd->p_td_loop, (void *)p_uhcd->p_td_loop->dma_handle);

  /* initialize loop-td */
  p_uhcd->p_td_loop->buffer = 0;
  p_uhcd->p_td_loop->link   = cpu_to_le32( p_uhcd->p_td_loop->dma_handle | LINK_NO_TERM | LINK_TO_TD);
  p_uhcd->p_td_loop->status = cpu_to_le32( td_status_errcount(0));
  p_uhcd->p_td_loop->token  = cpu_to_le32( td_token_pid(IN_TOKEN) |
                                            td_token_addr(0x7f) |
                                            td_token_maxlen(7));

  /* initialize lowspeed-qh */
  p_uhcd->p_qh_lowspeed->link    = cpu_to_le32(p_uhcd->p_qh_fullspeed->dma_handle | LINK_TO_QH | LINK_NO_TERM);
  p_uhcd->p_qh_lowspeed->element = cpu_to_le32(LINK_TERM);

  /* initialize fullspeed-qh */
  p_uhcd->p_qh_fullspeed->link    = cpu_to_le32(p_uhcd->p_qh_term->dma_handle | LINK_TO_QH | LINK_NO_TERM);
  p_uhcd->p_qh_fullspeed->element = cpu_to_le32(LINK_TERM);

  /* initialize terminate-qh */
#ifdef CONFIG_USB4RT_BANDW_RECLAM
  p_uhcd->p_qh_term->link    = cpu_to_le32(p_uhcd->p_qh_fullspeed->dma_handle | LINK_TO_QH | LINK_NO_TERM);
#else
  p_uhcd->p_qh_term->link    = cpu_to_le32(LINK_TERM);
#endif
  p_uhcd->p_qh_term->element = cpu_to_le32(p_uhcd->p_td_loop->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_NO_VF);

  /* insert pointer of the lowspeed-qh into the frame_list */
  for(i=0; i<MAX_FRAMES; i++) {
    ret = append_qh_on_fl(p_uhcd->p_qh_lowspeed, p_uhcd->p_fl, i);
    if (ret){
      goto err_skel_append;
    }
  }

  return 0;

/* Error - Handling */

err_skel_append:
  destroy_td(p_uhcd->p_td_loop);
  ERR("RT-UHC-Driver: [ERROR] Cannot insert Adress of LS-QH into the framelist \n");
  ret = -EINVAL;

err_skel_loop:
  destroy_qh(p_uhcd->p_qh_term);
  ERR("RT-UHC-Driver: [ERROR] Cannot allocate memory for Loop-TD \n");
  ret = -ENOMEM;

err_skel_term:
  destroy_qh(p_uhcd->p_qh_fullspeed);
  ERR("RT-UHC-Driver: [ERROR] Cannot allocate memory for Terminate-QH \n");
  ret = -ENOMEM;

err_skel_fs:
  destroy_qh(p_uhcd->p_qh_lowspeed);
  ERR("RT-UHC-Driver: [ERROR] Cannot allocate memory for Full-Speed-QH \n");
  ret = -ENOMEM;

err_skel_ls:
  ERR("RT-UHC-Driver: [ERROR] Cannot allocate memory for Low-Speed-QH \n");
  ret = -ENOMEM;

  return ret;
}

/**
 * Ausgabe der Controller-Informationen.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static void dump_uhcd(struct uhc_device *p_uhcd)
{
#ifdef DEBUG
  if (!p_uhcd->p_io){
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

  DBG("========== DUMP UHC-Controller [%d] @ 0x%p ===========\n", p_uhcd->uhcd_nr, p_uhcd);
#ifdef CONFIG_PCI_NAMES
  DBG("   Name : %s \n", p_uhcd->p_pcidev->pretty_name);
#else
  DBG("   Name : USB Universal Host Controller\n");
#endif
  DBG("   Vendor = 0x%4x, Device = 0x%4x, Class = 0x%x, IRQ = %d ",
      p_uhcd->p_pcidev->vendor, p_uhcd->p_pcidev->device, p_uhcd->p_pcidev->class, p_uhcd->p_pcidev->irq);
  DBG("\n   COMMAND (0x%04x): ", cmd);
  if (!(cmd & USBCMD_RS))    DBG("STOP ");
  if (cmd & USBCMD_RS)       DBG("RUN ");
  if (cmd & USBCMD_HCRESET)  DBG("HCRESET "); //Host-Controller Reset
  if (cmd & USBCMD_GRESET)   DBG("GRESET ");  //Global-Reset
  if (cmd & USBCMD_EGSM)     DBG("EGSM ");  //Enter global Suspend Mode
  if (cmd & USBCMD_FGR)      DBG("FGR ");     //Force global Resume
  DBG("\n   USBSTS  (0x%04x): ", stat);
  if (! stat)                DBG("OK ");
  if (stat & USBSTS_HCH)     DBG("HCH ");  //HC Halted
  if (stat & USBSTS_HCPE)    DBG("HCPE "); //Host Controller Process Error - the scripts were buggy
  if (stat & USBSTS_USBINT)  DBG("IOC ");  //Interrupt due to IOC
  if (stat & USBSTS_ERROR)   DBG("ERROR ");  //Interrupt due to error
  if (stat & USBSTS_HSE)     DBG("HSE ");  //Host System Error - basically PCI problems
  if (stat & USBSTS_RD)      DBG("RD "); //Resume Detect
  DBG("\n   USBINTR (0x%04x): ", intr);
  if (! intr)                DBG("NONE "); //No Interrupts
  if (intr & USBINTR_TIMEOUT) DBG("TIMEOUT ");//Timeout / CRC Interrupt Enable
  if (intr & USBINTR_RESUME)  DBG("RESUME "); //Resume Interrupt Enable
  if (intr & USBINTR_IOC)    DBG("IOC ");  //Interrupt on Complete (IOC)
  if (intr & USBINTR_SP)     DBG("SP "); //Short Packet Interrupt Enable \n");
  DBG("\n   FRNUM           : 0x%04x", inw(p_uhcd->p_io->start + FRNUM));
  DBG("\n   FLBASEADD       : 0x%08x", inl(p_uhcd->p_io->start + FRBASEADD));
  DBG("\n   SOFMOD          : 0x%02x", inb(p_uhcd->p_io->start + SOFMOD));

  unsigned short k=0;
  for(k=0 ; k < p_uhcd->p_hcd->rh_numports; k++){
    port_base = p_uhcd->p_io->start + 0x10 + (0x02 * k);
    port[k] = inw(port_base);
    DBG("\n   PORTSC%d @ 0x%04x (0x%04x) : ", k, port_base, port[k]);
    if (port[k] & PORTSC_CSC)      DBG("CSC ");  //Connect Status Change
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
  DBG("\n========== END DUMP UHC-Controller [%d] @ 0x%p ==========\n", p_uhcd->uhcd_nr, p_uhcd);
#endif
  return;
}

/**
 * Loescht das Statusregister des Host Controllers.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_clear_stat(struct uhc_device *p_uhcd)
{
  unsigned short stat;

  if (!p_uhcd || !p_uhcd->p_io){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Clear Status\n");

  stat = inw(p_uhcd->p_io->start + USBSTS);
  outw(stat, p_uhcd->p_io->start + USBSTS);
  return 0;
}

void uhc_enable_all_interrupts(struct uhc_device *p_uhcd)
{
  if (!p_uhcd || !p_uhcd->p_io){
    return;
  }

  /* Turn on all interrupts */
  DBG("RT-UHC-Driver: Turn on all Interrupts  \n");
  outw(USBINTR_TIMEOUT | USBINTR_RESUME | USBINTR_IOC | USBINTR_SP , p_uhcd->p_io->start + USBINTR);
}

void uhc_disable_all_interrupts(struct uhc_device *p_uhcd)
{
  if (!p_uhcd || !p_uhcd->p_io){
    return;
  }

  /* Turn off all interrupts */
  DBG("RT-UHC-Driver: Turn off all Interrupts  \n");
  outw(0 , p_uhcd->p_io->start + USBINTR);
}

/**
 * Fuehrt einen globalen Reset eines Universal Host Controller aus.
 * ZEIT : 15ms
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_global_reset(struct uhc_device *p_uhcd)
{
  if (!p_uhcd || !p_uhcd->p_io || !p_uhcd->p_pcidev){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Global Reset\n");

  /* Turn off PIRQ .This also turns off the BIOS's USB Legacy Support.*/
  pci_write_config_word(p_uhcd->p_pcidev, USBLEGSUP, 0);

  uhc_disable_all_interrupts(p_uhcd);

  /* Global reset for 50ms */
  outw(USBCMD_GRESET, p_uhcd->p_io->start + USBCMD);

  msleep(50);

  /* Clear Command-Register */
  outw(0, p_uhcd->p_io->start + USBCMD);

  return 0;
}

void reset_framelist(struct uhc_device *p_uhcd)
{
  /* writing framellist pointer */
  DBG("RT-UHC-Driver: Write framellist 0x%p \n", (void *)p_uhcd->p_fl->dma_handle);
  outl(p_uhcd->p_fl->dma_handle, p_uhcd->p_io->start + FRBASEADD);

  /* setting framenummer to 0 */
  DBG("RT-UHC-Driver: Setting framenumber to 0\n");
  outw(0x0000, p_uhcd->p_io->start + FRNUM);
}

/**
 * Startet einen Universal Host Controller aus.
 * @param hcd_nr Nummer des Host-Controllers
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
int uhc_start(struct uhc_device *p_uhcd)
{
  int timeout = 1000;

  if (!p_uhcd || !p_uhcd->p_io || !p_uhcd->p_pcidev){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Starte Host Controller\n");

  /* Host Controller Reset */
  outw(USBCMD_HCRESET, p_uhcd->p_io->start + USBCMD);
  while (inw(p_uhcd->p_io->start + USBCMD) & USBCMD_HCRESET) {
    if (!--timeout) {
      ERR("RT-UHC-Driver: [ERROR] %s - USBCMD_HCRESET timed out! \n", __FUNCTION__);
      break;
    }
  }

  /* reset framelist */
  reset_framelist(p_uhcd);

  /* Turn on PIRQ */
  pci_write_config_word(p_uhcd->p_pcidev, USBLEGSUP, USBLEGSUP_DEFAULT);

  uhc_enable_all_interrupts(p_uhcd);

  /* Run and mark it configured with a 64-byte max packet */
  outw(USBCMD_RS | USBCMD_CF | USBCMD_MAXP, p_uhcd->p_io->start + USBCMD);

  return(0);
}

/**
 * stops an universal-host-controller.
 * @param hcd_nr number of the host-controller
 * @return 0, if successful
 * @return -ENODEV, if p_uhcd invalid
 */
int uhc_stop(struct uhc_device *p_uhcd)
{
  unsigned short tmp;

  if (!p_uhcd || !p_uhcd->p_io){
    return -ENODEV;
  }

  DBG("RT-UHC-Driver: Stopping host-controller\n");

  tmp = inw(p_uhcd->p_io->start + USBCMD);
  tmp &= ~USBCMD_RS;
  outw(tmp, p_uhcd->p_io->start + USBCMD);
  while (!(inw(p_uhcd->p_io->start + USBSTS) & USBSTS_HCH)){
    outw(USBSTS_HCH, p_uhcd->p_io->start + USBSTS);
  }
  if (uhc_global_reset(p_uhcd)) {
    ERR("RT-UHC-Driver: [ERROR] %s - Resetting host-controller failed\n", __FUNCTION__);
  }
  return (0);
}

/**
 * Initializes the framelist, the data-structures and the root-hub of an universal-host-controller.
 * Furthermore all params of the controller will be defined.
 * @param hcd_nr Number of the host-controller
 * @return 0, if successful
 * @return -ENODEV, if p_uhcd invalid or the host-controller has'n become needed resources.
 */
static int uhc_init(struct uhc_device *p_uhcd)
{
  int ret, i, rh_port;
  unsigned short tmp;

  if (!p_uhcd || !p_uhcd->p_io || !p_uhcd->irq){
    return -ENODEV;
  }

  /* 1. initialize framelist */
  ret = init_framelist(p_uhcd);
  if (ret){
    ERR("RT-UHC-Driver: [ERROR] %s - Creating framelist failed\n", __FUNCTION__);
    return -1;
  }
  DBG("RT-UHC-Driver: Framelist    @ 0x%p, DMA: 0x%p (%d Byte) \n",
      (void *)p_uhcd->p_fl, (void *)p_uhcd->p_fl->dma_handle, sizeof(struct frame_list));

  /* 2. initialize skeleton */
  ret = init_skel(p_uhcd);
  if (ret) {
    ERR("RT-UHC-Driver: [ERROR] %s - Init skeleton failed\n", __FUNCTION__);
    return -2;
  }
  DBG("RT-UHC-Driver: Skeleton initialized\n");

  /* 3. initialize Root-Hub*/
  ret = rh_init(p_uhcd);
  if (ret) {
    ERR("RT-UHC-Driver: [ERROR] %s - Init root-hub failed\n", __FUNCTION__);
    return -3;
  }

  /* Setting Bus-Master */
  DBG("RT-UHC-Driver: Setting bus-master\n");
  pci_read_config_word(p_uhcd->p_pcidev, PCICMD, &tmp);
  tmp |= PCICMD_BME;
  pci_write_config_word(p_uhcd->p_pcidev, PCICMD, tmp);

  /* init Framelist */
  reset_framelist(p_uhcd);

  /* Set Max-Packet to 64 */
  DBG("RT-UHC-Driver: Setting max-packet-size to 64 \n");
  outw(inw(p_uhcd->p_io->start + USBCMD) | USBCMD_MAXP , p_uhcd->p_io->start + USBCMD);

  uhc_disable_all_interrupts(p_uhcd);

  /* Setting Ports into Suspend-Mode */
  DBG("RT-UHC-Driver: Setting ports into suspend-mode\n");
  for(i=0;i<p_uhcd->p_hcd->rh_numports;i++){
    rh_port = p_uhcd->p_io->start + 0x10 + (i * 0x02);
    outw(0x1000, rh_port);
  }

  return 0;
}

static void uhc_clear(struct uhc_device *p_uhcd)
{
  __u8 nr = p_uhcd->uhcd_nr;

  memset(p_uhcd, 0, sizeof(struct uhc_device));
  p_uhcd->uhcd_nr = nr;

  INIT_LIST_HEAD(&p_uhcd->irq_list);
  INIT_LIST_HEAD(&p_uhcd->reg_urb_list);
  INIT_LIST_HEAD(&p_uhcd->handle_urb_list);
}

/******************************************************************/
/*  !!! Functions called IN INTERRUPT !!!                         */
/******************************************************************/

void dump_urb(struct rt_urb *p_urb);
static void unmap_dma(struct rt_privurb *p_purb);
static void rt_complete_urb(struct rt_privurb *p_purb);
static void rt_unschedule_ctrl_bulk_urb(struct rt_privurb *p_purb);
static void rt_unschedule_int_urb(struct rt_privurb *p_purb);
static void rt_unschedule_isoc_urb(struct rt_privurb *p_purb);


static void handle_first_irq(struct rt_privurb *p_purb)
{
  td_t *p_td = NULL;
  unsigned int bytes = 0;
  struct list_head *p_list = p_purb->active_td_list.next;
  unsigned long transfer_time;
  unsigned long offset;
  __u32 status;

  while (p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list, td_t, active_td_list);
    if (get_status_active(p_td)){

      p_purb->byte_at_first_irq = bytes;
      p_purb->compl_tds_at_first_irq = p_td->td_nr;
      p_purb->time_at_first_irq = frame_start;
      p_purb->get_time_of_first_bit = 2;

      INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 1: active TD[%d] \n", p_purb->p_urb, p_td->td_nr);
      INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 1: %u Byte in %d TDs transferred \n", p_purb->p_urb,
        bytes, p_purb->compl_tds_at_first_irq);

      p_list = p_list->next;
      if (p_list != &p_purb->active_td_list){
        p_td = list_entry(p_list->next, td_t, active_td_list);

        //set_status_ioc(p_td);                /* Set IOC */
        status = le32_to_cpu(p_td->status);
        status |= TD_STAT_IOC;
        p_td->status = cpu_to_le32(status);

      }

      return;
    } else {
       bytes += get_status_actlen(p_td)+1 + /* data-bytes*/
                4 + /* Token-Overhead */
                4 + /* Data-Overhead */
                2 + /* Handshake-Overhead */
                3 * EOP_DELAY_BYTE; /* 3 packets EOP and Delay */
    }
    p_list = p_list->next;
  }
  /* URB completed in the first frame */
  transfer_time = FS_NS_PER_BYTE * bytes;
  offset = FS_NS_PER_preSOF + transfer_time + 1000000 + FS_NS_PER_SOF;
  p_purb->p_urb->timestamp_first_data = frame_start - (offset >> 1);
  INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 1: Timestamp: %llu \n", p_purb->p_urb,
        p_purb->p_urb->timestamp_first_data);
  p_purb->get_time_of_first_bit = 0;
}

static void handle_second_irq(struct rt_privurb *p_purb)
{
  unsigned long transfer_time;
  unsigned long offset;
  unsigned int bytes = 0;
  td_t *p_td = &p_purb->p_td_table[p_purb->compl_tds_at_first_irq];
  struct list_head *p_list = &p_td->active_td_list;
  unsigned int bytes_per_frame;
  unsigned int ns_per_frame;
  unsigned int ns_per_byte;
  unsigned int bit_per_s;

  while (p_list != &p_purb->active_td_list){
    p_td = list_entry(p_list, td_t, active_td_list);

    if (get_status_active(p_td)){

      bytes_per_frame = bytes - p_purb->byte_at_first_irq;
      ns_per_frame = (unsigned long)(frame_start - p_purb->time_at_first_irq);
      ns_per_byte = ns_per_frame / bytes_per_frame;
      bit_per_s = (bytes_per_frame * 8) * 1000 ;
      offset = p_purb->byte_at_first_irq * ns_per_byte;
      p_purb->p_urb->timestamp_first_data = p_purb->time_at_first_irq - offset;
      p_purb->get_time_of_first_bit = 0;

      INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 2: %u Bytes in %u ns \n", p_purb->p_urb,
        bytes_per_frame, ns_per_frame);
      INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 2: Bitrate: %u Bit/s \n", p_purb->p_urb,
        bit_per_s);
      INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 2: Timestamp: %llu \n", p_purb->p_urb,
        p_purb->p_urb->timestamp_first_data);
      return;
    } else {
      bytes += get_status_actlen(p_td)+1 + /* data-bytes*/
               4 + /* Token-Overhead */
               4 + /* Data-Overhead */
               2 + /* Handshake-Overhead */
               3 * EOP_DELAY_BYTE; /* 3 packets EOP and Delay */
    }
    p_list = p_list->next;
  }

  /* URB completed in the second frame */
  transfer_time = FS_NS_PER_BYTE * p_purb->byte_at_first_irq;
  offset = (FS_NS_PER_preSOF >> 1) + transfer_time;
  p_purb->p_urb->timestamp_first_data = p_purb->time_at_first_irq - offset;
  INFO_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: IRQ 2: Timestamp: %llu \n", p_purb->p_urb,
        p_purb->p_urb->timestamp_first_data);
  p_purb->get_time_of_first_bit = 0;
}

static inline void handle_urb(__u16 stat, struct uhc_device *p_uhcd, struct rt_privurb *p_purb)
{
  int toggle;
  td_t *p_td = NULL;
  struct list_head *p_list = p_purb->active_td_list.next;

  if (p_purb->urb_wait_flags == URB_WAIT_BUSY){
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: BUSY WAIT\n", p_purb->p_urb);
    // handled in rt_uhci_send_busywait_urb
    return;
  }

  // UHCD-Error
  if (p_uhcd->status & UHC_HOST_ERROR){
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: HOST-CONTROLLER ERROR\n", p_purb->p_urb);
    p_purb->p_urb->status = -2;
    goto handle;
  }

  // UHCD-Not Running
  if (!(p_uhcd->status & UHC_RUNNING)){
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: HOST-CONTROLLER STOPPED\n", p_purb->p_urb);
    p_purb->p_urb->status = -3;
    goto handle;
  }

  // Check if completed
  if (p_purb->p_qh->element & LINK_TERM){
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: COMPLETED\n", p_purb->p_urb);
    p_purb->p_urb->status = 0;
    goto handle;
  }

  // Check for Errors
  if (stat & USBSTS_ERROR) {  //Interrupt due to error

    p_td = NULL;
    p_list = p_purb->active_td_list.next;

    while (p_list != &p_purb->active_td_list){

      p_td = list_entry(p_list, td_t, active_td_list);

      if (td_status(p_td) & TD_STAT_ANY_ERROR){
        DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " ===> URB 0x%p: ERROR AT TD[%d]: %s%s%s%s%s", p_purb->p_urb, p_td->td_nr,
          (td_status(p_td) & TD_STAT_STALLED)  ? "STALL ":"",
          (td_status(p_td) & TD_STAT_DBUFERR)  ? "DATA-BUFFER ":"",
          (td_status(p_td) & TD_STAT_BABBLE)   ? "BABBLE ":"",
          (td_status(p_td) & TD_STAT_CRCTIMEO) ? "CRC or TIMEOUT ":"",
          (td_status(p_td) & TD_STAT_BITSTUFF) ? "BITSTUFF ":"");
        DBG("\n");
        dump_td_table(p_purb, 0);

        toggle = (td_status(p_td) & TD_TOKEN_TOGGLE ? 1 : 0);
        usb_set_data_toggle(p_purb->p_urb->p_usbdev, p_purb->p_urb->pipe, toggle);

        p_purb->p_urb->status = -1;

        goto handle;
      }

      p_list = p_list->next;
    }
  }
  // no handle
  return;

handle:
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: HANDLING \n", p_purb->p_urb);

    if (p_purb->urb_wait_flags == URB_WAIT_SEM){
      DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: Wake up Driver\n", p_purb->p_urb);
      p_purb->status |= PURB_WAKE_UP_BY_IRQ;
      rtdm_sem_up(&p_purb->p_urb->rt_sem);
      // handled in rt_uhci_send_semaphore_urb
      return;
    }

    rt_complete_urb(p_purb);

    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: Calling Complete-Function\n", p_purb->p_urb);
    p_purb->p_urb->rt_complete_fkt(p_purb->p_urb);

    return;
}

static inline void handle_controller(struct uhc_device *p_uhcd, __u16 stat)
{
  struct rt_privurb *p_purb;
  struct list_head *p_list, *p_next;

  if (stat & USBSTS_ERROR) {  //Interrupt due to error
    DBG_MSG1(p_uhcd->p_hcd, " ===> Incoming IRQ %d, ERROR \n", p_uhcd->irq);
  }
  if (stat & USBSTS_USBINT) { //Interrupt due to IOC
    DBG_MSG1(p_uhcd->p_hcd, " ===> Incoming IRQ %d, COMPLETE \n", p_uhcd->irq);
    if (td_status(p_uhcd->p_td_loop) & TD_STAT_IOC){
      DBG_MSG1(p_uhcd->p_hcd, " ===> Interrupt called by Loop-TD \n");
    }
  }
  if (stat & USBSTS_RD) {     // Resume Detect
    DBG_MSG1(p_uhcd->p_hcd, " ===> Incoming IRQ %d, Resume Detect \n", p_uhcd->irq);
  }

  if (stat & USBSTS_HSE || stat & USBSTS_HCPE || stat & USBSTS_HCH) {
    DBG_MSG1(p_uhcd->p_hcd, " ===> Incoming IRQ %d \n", p_uhcd->irq);

    if (stat & USBSTS_HSE){
      ERR_MSG1(p_uhcd->p_hcd, " ===> Host System Error \n");
      p_uhcd->status |= UHC_HOST_ERROR;
    }
    if (stat & USBSTS_HCPE) {
      ERR_MSG1(p_uhcd->p_hcd, " ===> Host Controller Process Error \n");
      p_uhcd->status |= UHC_HOST_ERROR;
    }
    if (stat & USBSTS_HCH) {
      ERR_MSG1(p_uhcd->p_hcd, " ===> Host-Controller Halted \n");
      p_uhcd->status &= ~UHC_RUNNING;
    }

    dump_uhcd(p_uhcd);
  }

  p_list = p_uhcd->reg_urb_list.next;

  while (p_list != &p_uhcd->reg_urb_list){
    p_next = p_list->next;
    p_purb = list_entry(p_list, struct rt_privurb, reg_urb_list);

    if (p_purb->get_time_of_first_bit == 2){
      handle_second_irq(p_purb);
    }

    if (p_purb->get_time_of_first_bit == 1){
      handle_first_irq(p_purb);
    }

    if ((p_purb->status == PURB_IN_PROGRESS) || (p_uhcd->status & UHC_HOST_ERROR) || !(p_uhcd->status & UHC_RUNNING)){
      handle_urb(stat, p_uhcd, p_purb);
    }

    p_list = p_next;
  }
}

int rt_irq_handler(rtdm_irq_t *irq_handle)
{
  struct uhc_irq *p_uhc_irq = rtdm_irq_get_arg(irq_handle, struct uhc_irq);
  int fs=0;
  int handled_uhc = 0;
  __u16 stat = 0x0000;
  struct uhc_device *p_uhcd = NULL;
  struct list_head  *p_list = NULL;

  if (!p_uhc_irq || list_empty(&p_uhc_irq->irq_list)){
    return RTDM_IRQ_NONE;           /* shared interrupt, not mine */
  }

  DBG(" =================== BEGIN RTAI-INTERRUPT HANDLER =============================\n");


  p_list = p_uhc_irq->irq_list.next;
  while (p_list != &p_uhc_irq->irq_list){
    p_uhcd = list_entry(p_list, struct uhc_device, irq_list);
    DBG("UHC @ 0x%p: Checking for Interrupts \n", p_uhcd);
    if (!p_uhcd ||                    /* invalid pointer */
        !p_uhcd->p_io ||              /* no io-port */
        !p_uhcd->irq){                /* no interrupt-number */
      DBG("UHC @ 0x%p: p_uhcd invalid \n", p_uhcd);
      goto irq_next_uhc;
    }

    /* get time */
    if (!fs){
      frame_start = rtdm_clock_read();
    }

    /* read controller-state */
    stat = inw(p_uhcd->p_io->start + USBSTS);
    if (!(stat & ~USBSTS_HCH)) {                /* no State-Bit set */
      DBG("UHC[%d]: No state-bit set\n", p_uhcd->uhcd_nr);
      goto irq_next_uhc;                          /* searching */
    } else {                                      /* Interrupt-Controller found */
      DBG("UHC[%d]: Handling \n", p_uhcd->uhcd_nr);
      handle_controller(p_uhcd, stat);           /* handle this Controller */
      handled_uhc++;
      outw(stat, p_uhcd->p_io->start + USBSTS);   /* Clear it */
    }

irq_next_uhc:
    p_list = p_list->next;
  }

  DBG(" =================== ENDE RTAI-INTERRUPT HANDLER ==============================\n");

  if (!handled_uhc){
    return RTDM_IRQ_NONE;         /* shared interrupt, not mine */
  }

  return RTDM_IRQ_HANDLED;
}

/******************************************************************/
/*  Ressource-Functions                                           */
/******************************************************************/

static int get_ioport_info(struct pci_dev *p_pcidev, unsigned long *io_start, unsigned long *io_size){
  int i;
  unsigned long io_flags = 0;

  if (!p_pcidev || !io_start || !io_size){
    return -EINVAL;
  }

  /* searching for the first io-port */
  for(i=0; pci_io_addr[i]; i++){
    io_flags = pci_resource_flags(p_pcidev, i);
    if (io_flags & IORESOURCE_IO){
      *io_start = pci_resource_start(p_pcidev, i);
      *io_size  = pci_resource_len(p_pcidev, i);
      return 0;
    }
  }
  return -ENODEV;
}

/**
 * Versucht fuer einen Controller einen IO-Port anzufordern.
 * @param p_uhcd Zeiger auf ein gueltiges struct hc_device
 * @return 0, wenn erfolgreich
 * @return -EBUSY, wenn IO-Port schon belegt
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static int request_ioport(struct uhc_device *p_uhcd, unsigned long io_start, unsigned long io_size)
{
  char uhc_name[20];

  if (!p_uhcd || !io_start || !io_size){
    return -EINVAL;
  }

  p_uhcd->p_io = NULL;

  PRNT("RT-UHC-Driver: Request IO-Port @ 0x%08lx (%lu Byte) for UHC[%d] ... ",
        io_start, io_size, p_uhcd->uhcd_nr);

  sprintf(uhc_name, "rt_uhcd[%02d]", p_uhcd->uhcd_nr);
  p_uhcd->p_io = NULL;
  p_uhcd->p_io = request_region(io_start, io_size, uhc_name);

  if (p_uhcd->p_io){
    PRNT("[OK]\n");
    return 0;
  }
  PRNT("[BUSY]\n");
  return -EBUSY;

}

/**
 * Gibt IO-Port fuer Controller wieder frei.
 * @param p_uhcd : Zeiger auf ein gueltiges struct hc_device
 * @return 0, wenn erfolgreich
 * @return -ENODEV, wenn p_uhcd ungueltig
 */
static void release_ioport(struct uhc_device *p_uhcd)
{
  unsigned long size;

  if (!p_uhcd->p_io){
    return ;
  }

  size = p_uhcd->p_io->end - p_uhcd->p_io->start + 1;
  PRNT("RT-UHC-Driver: Release IO-Port 0x%08lx (%lu Byte)\n", p_uhcd->p_io->start, size);
  release_region(p_uhcd->p_io->start, size);
  p_uhcd->p_io = NULL;
  return;
}

/**
 * Try to get an interrupt line for the host controller.
 *
 * @param p_uhcd Pointer to the host controller.
 * @return 0, if successful
 * @return -EBUSY, Interrupt line is busy
 * @return -ENODEV, Pointer to host controller is invalid
 */
static int get_irq(struct uhc_device *p_uhcd)
{
    int i, ret, wanted_irq;
    int uhc_irq_idx = -1;
    int uhc_irq_free = -1;
    struct uhc_irq *p_uhc_irq = NULL;

    if (!p_uhcd->p_pcidev || !p_uhcd->p_pcidev->irq)
        return -ENODEV;

    wanted_irq = p_uhcd->p_pcidev->irq;

    // checking irq table
    DBG("searching IRQ %d in the IRQ-Table\n", wanted_irq);
    for (i=0; i < MAX_DEVICES; i++)
    {
        DBG("uhc_irq_tab[%d].irq = %d\n", i, uhc_irq_tab[i].irq);
        if (uhc_irq_tab[i].irq == wanted_irq)
        {
            // irq ist registered
            p_uhc_irq = &uhc_irq_tab[i];
            uhc_irq_idx = i;
            DBG("Entry found, p_uhc_irq[%d] @ 0x%p, \n", uhc_irq_idx, p_uhc_irq);
            break;
        }

        if (uhc_irq_free == -1 &&
            uhc_irq_tab[i].irq == 0) // free entry
        {
            DBG("uhc_irq_tab[%d].irq = 0 -> free entry\n", i);
            uhc_irq_free = i;
        }
    }

    if (p_uhc_irq)
    {
        INFO("RT-UHC-Driver: RTDM IRQ %d registered yet, add UHC to IRQ-List\n",
             wanted_irq);
        p_uhcd->irq = wanted_irq;
        list_add_tail(&p_uhcd->irq_list, &p_uhc_irq->irq_list);
        return 0;
    }

    // first UHC with this irq -> init uhc_irq-structure
    if (uhc_irq_free == -1)
        return -EBUSY;

    p_uhc_irq = &uhc_irq_tab[uhc_irq_free];
    DBG("New Entry: uhc_irq_tab[%d] @ 0x%p\n", uhc_irq_free, p_uhc_irq);
    p_uhc_irq->irq = wanted_irq;
    INIT_LIST_HEAD(&p_uhc_irq->irq_list);
    list_add_tail(&p_uhcd->irq_list, &p_uhc_irq->irq_list);

    // request irq
    INFO("RT-UHC-Driver: Request RTDM IRQ %d ... ", wanted_irq);
    ret = rtdm_irq_request(&p_uhc_irq->irq_handle, wanted_irq, &rt_irq_handler,
                           0, "rt_uhci", (void *)p_uhc_irq);
    if (ret)
    {
        INFO("[BUSY]\n");

        // reset irq-values of p_uhcd
        p_uhcd->irq = 0;
        list_del_init(&p_uhcd->irq_list);

        // reset irq-values of struct uhc_irq
        memset(p_uhc_irq, 0, sizeof(struct uhc_irq));
        INIT_LIST_HEAD(&p_uhc_irq->irq_list);

        return -EBUSY;
    }
    INFO("[OK]\n");
    p_uhcd->irq = wanted_irq;

    INFO("RT-UHC-Driver: Enable  RTDM-IRQ %d ... ", wanted_irq);
    ret = rtdm_irq_enable(&p_uhc_irq->irq_handle);
    if (ret)
    {
        INFO("[BUSY]\n");

        // delete rtdm interrupt
        rtdm_irq_free(&p_uhc_irq->irq_handle);

        // reset irq values of p_uhcd
        p_uhcd->irq = 0;
        list_del_init(&p_uhcd->irq_list);

        // reset irq-values of struct uhc_irq
        memset(p_uhc_irq, 0, sizeof(struct uhc_irq));
        INIT_LIST_HEAD(&p_uhc_irq->irq_list);

        return -EBUSY;
    }
    INFO("[OK]\n");
    return 0;
}

/**
 * Releases the interrupt line of the controller
 * @param p_uhcd Pointer to the host controller
 */
static void put_irq(struct uhc_device *p_uhcd)
{
    int i, release_irq;
    int uhc_irq_idx = -1;
    struct uhc_irq    *p_uhc_irq    = NULL;
    struct uhc_device *p_uhcd_dummy = NULL;
    struct list_head  *p_list       = NULL;

    if (!p_uhcd->p_pcidev || !p_uhcd->irq)
        return;

    release_irq = p_uhcd->irq;

    // checking irq-table
    for(i=0; i < MAX_DEVICES; i++)
    {
        if (uhc_irq_tab[i].irq == release_irq) // irq ist registered
        {
            p_uhc_irq = &uhc_irq_tab[i];
            uhc_irq_idx = i;
            break;
        }
    }

    if (!p_uhc_irq)
        return;

    // searching for p_uhcd in irq-list
    p_list = p_uhc_irq->irq_list.next;

    while (p_list != &p_uhc_irq->irq_list)
    {
        p_uhcd_dummy = list_entry(p_list, struct uhc_device, irq_list);
        if (p_uhcd_dummy == p_uhcd)
        {
            // reset irq-values of p_uhcd
              p_uhcd->irq = 0;
              list_del_init(&p_uhcd->irq_list);
            break;
        }
        p_list = p_list->next;
    }

    // checking if irq-list empty
    if (!list_empty(&p_uhc_irq->irq_list))
    {
        INFO("RT-UHC-Driver: RTDM-IRQ %d still needed by another UHC\n",
             release_irq);
        return;
    }

    // irq-list is empty -> disable and delete rtdm irq
    INFO("RT-UHC-Driver: Disable RTDM IRQ %d\n", release_irq);
    rtdm_irq_disable(&p_uhc_irq->irq_handle);

    INFO("RT-UHC-Driver: Delete  RTDM IRQ %d\n", release_irq);
    rtdm_irq_free(&p_uhc_irq->irq_handle);

    // reset irq values of struct uhc_irq
    memset(p_uhc_irq, 0, sizeof(struct uhc_irq));
    INIT_LIST_HEAD(&p_uhc_irq->irq_list);

    return;
}

/******************************************************************/
/*  URB Funktionen                                                */
/******************************************************************/

void dump_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
    {
        ERR("[ERROR] %s - Invalid URB-Pointer \n", __FUNCTION__);
        return;
    }

    DBG("========== DUMP URB 0x%p ======================\n", p_urb);
    DBG(" Host-Controller   @ 0x%p\n",     p_urb->p_hcd);
    DBG(" Max Buffer-Length : %d Byte\n",  p_urb->max_buffer_len);
    DBG(" Max Packet-Size   : %d Byte\n",  p_urb->max_packet_size);
    DBG(" \n");
    DBG(" USB-Device        @ 0x%p\n",     p_urb->p_usbdev);
    DBG(" Pipe              : %d \n",      p_urb->pipe);
    DBG(" Flags             : %d \n",      p_urb->transfer_flags);
    DBG(" Transfer-Buffer   @ 0x%p\n",     p_urb->p_transfer_buffer);
    DBG(" Transfer-DMA      @ 0x%p\n",     (void *)p_urb->transfer_dma);
    DBG(" Transf-Buff-Length: %d Byte\n",  p_urb->transfer_buffer_length);
    DBG(" Ctrl-Request-Pack @ 0x%p\n",     p_urb->p_setup_packet);
    DBG(" Ctrl-Request-DMA  @ 0x%p\n",     (void *)p_urb->setup_dma);
    DBG(" Private           @ 0x%p\n",     p_urb->p_private);
    DBG(" RT-Complete-Funct @ 0x%p\n",     p_urb->rt_complete_fkt);
    DBG(" Status            : %d\n",       p_urb->status);
    DBG(" Actual Length     : %d Byte \n", p_urb->actual_length);
    DBG("======================================================\n");
}

static int map_dma(struct rt_privurb *p_purb)
{
    struct rt_urb *p_urb = NULL;

    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return -ENODEV;
    }

    p_urb = p_purb->p_urb;

    if (usb_pipecontrol(p_urb->pipe))
    {
        if (!(p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP))
        { // mapping setup-packet to DMA
            dma_addr_t dma_handle = pci_map_single(p_purb->p_uhcd->p_pcidev,
                                                   p_urb->p_setup_packet,
                                                   sizeof(struct usb_ctrlrequest),
                                                   PCI_DMA_TODEVICE);

            if (!dma_handle)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " %s - Couldn't map "
                         "Setup-Packet to DMA!\n", __FUNCTION__);
                return -ENOMEM;
              }
            p_urb->setup_dma = dma_handle;
            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Setup-Packet "
                     "@ 0x%p (%d Byte) mapped to DMA: 0x%p [OUT]\n", p_urb,
                     p_urb->p_setup_packet, sizeof(struct usb_ctrlrequest),
                     (void *)p_urb->setup_dma);
        }
    }

    if (p_urb->transfer_buffer_length)
    {
        if (!(p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP))
        { // mapping transfer buffer to DMA
            dma_addr_t dma_handle = pci_map_single(p_purb->p_uhcd->p_pcidev,
                                                   p_urb->p_transfer_buffer,
                                                   p_urb->transfer_buffer_length,
                                                   usb_pipein(p_purb->p_urb->pipe) ? PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE);
            if (!dma_handle)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " %s - Couldn't map "
                         "Transfer-Buffer to DMA!\n", __FUNCTION__);
                return -ENOMEM;
            }
            p_urb->transfer_dma = dma_handle;
            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Transfer-Buffer "
                     "@ 0x%p (%d Byte) mapped to DMA: 0x%p [%s]\n",
                     p_urb, p_urb->p_transfer_buffer, p_urb->transfer_buffer_length,
                     (void *)p_urb->transfer_dma, usb_pipein(p_urb->pipe) ? "IN" : "OUT");

            p_urb->transfer_dma = dma_handle;
        }
    }
    return 0;
}

static void unmap_dma(struct rt_privurb *p_purb)
{
    struct rt_urb *p_urb = NULL;

    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return;
    }

    p_urb = p_purb->p_urb;

    // Unmapping transfer buffer
    if (p_purb->p_urb->transfer_buffer_length)
    {
        if (!(p_purb->p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP))
        {
              pci_unmap_single(p_purb->p_uhcd->p_pcidev,
                             p_urb->transfer_dma,
                             p_urb->transfer_buffer_length,
                             usb_pipein(p_purb->p_urb->pipe) ?
                             PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE);

            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Unmap Transfer-Buffer "
                     "@ 0x%p (%d Byte) from DMA: 0x%p\n", p_urb,
                     p_purb->p_urb->p_transfer_buffer,
                     p_urb->transfer_buffer_length,
                     (void *)p_purb->p_urb->transfer_dma);

            p_urb->transfer_dma = 0;
        }
    }

    // Unmapping setup packet
    if (usb_pipecontrol(p_purb->p_urb->pipe))
    {
        if (!(p_purb->p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP))
        {
            pci_unmap_single(p_purb->p_uhcd->p_pcidev,
                             p_urb->setup_dma,
                             sizeof(struct usb_ctrlrequest),
                             usb_pipein(p_purb->p_urb->pipe) ?
                             PCI_DMA_FROMDEVICE : PCI_DMA_TODEVICE);

            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Unmap Setup-Packet "
                     "@ 0x%p (%d Byte) from DMA: 0x%p\n",
                     p_urb, p_purb->p_urb->p_setup_packet,
                     sizeof(struct usb_ctrlrequest),
                     (void *)p_purb->p_urb->setup_dma);

            p_urb->setup_dma = 0;
        }
    }
}

static int wait_for_urb(struct rt_privurb *p_purb)
{
    td_t *p_td = NULL;
    int64_t ns_per_retry = 80; // 80ns (12MHz @ Full-Speed)
    unsigned int retrys_per_td = 6250000; // 500ms (500ms/80ns)
    unsigned int retrys;
    int64_t time, time_old, urb_start, urb_end;

    struct list_head *p_list = NULL;

    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return -ENODEV;
    }

    if (list_empty(&p_purb->active_td_list))
        return 0; // No TDs to do

    p_list    = p_purb->active_td_list.next;
    urb_start = p_purb->p_urb->schedule_time;

    time = rtdm_clock_read();
    while (p_list !=  &p_purb->active_td_list)
    {
        p_td = list_entry(p_list, td_t, active_td_list);

        retrys = retrys_per_td;

        while (retrys && get_status_active(p_td))
        {
            // current TD is still active, waiting ns_per_retry
            rtdm_task_busy_sleep(ns_per_retry);
            retrys--;
        }

        time_old = time;
        time = rtdm_clock_read();

        TDBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: TD[%d] "
                  "completed in %llu ns @ %llu ns\n", p_purb->p_urb,
                  p_td->td_nr, time - time_old, time);

        if (!retrys) // timeout for this TD
        {
            DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p : "
                     "TIMEOUT FOR TD[%d]\n", p_purb->p_urb, p_td->td_nr);
            return -1;
        }

        // TD inactive --> goto next TD in list
        p_list = p_list->next;
    }

    urb_end = rtdm_clock_read();

    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: "
             "Complete Wait-Time: %llu ns\n", p_purb->p_urb,
             urb_end - urb_start);

    return 0;
}

static void clear_rt_privurb(struct rt_privurb *p_purb)
{
    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return;
    }

    memset(p_purb, 0, sizeof(struct rt_privurb));

    INIT_LIST_HEAD(&p_purb->reg_urb_list);
    INIT_LIST_HEAD(&p_purb->handle_urb_list);

    INIT_LIST_HEAD(&p_purb->free_td_list);
    INIT_LIST_HEAD(&p_purb->active_td_list);
}

static int rt_uhci_bytes_send(struct rt_privurb *p_purb)
{
    td_t *p_td = NULL;
    int act_len, max_len;
    int data_len = 0;
    struct list_head *p_list = NULL;

    if(!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return -ENODEV;
    }

    p_list = p_purb->active_td_list.next;
    while (p_list != &p_purb->active_td_list)
    {
        p_td = list_entry(p_list, td_t, active_td_list);

        act_len = get_status_actlen(p_td);
        max_len = get_max_len(p_td);

        if (get_packet_type(p_td) != SETUP_TOKEN)
        {
            if (act_len != 0x7ff) // if no zero paket
            {
                data_len += act_len + 1;
            }
        }

        if (act_len < max_len) // short packet
        {
            DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev,
                     " URB 0x%p: Short Packet (%d/%d Bytes) \n",
                     p_purb->p_urb, act_len, max_len);
            break;
        }

        p_list = p_list->next;
    }
    return data_len;
}

static void rt_free_urb_tds(struct rt_privurb *p_purb)
{
    td_t *p_td, *p_prev;
    struct list_head *p_list, *p_next;

    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
        return;
    }

    if (list_empty(&p_purb->active_td_list))
    {
        DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev,
        " URB 0x%p: Active-TD-List is empty \n", p_purb->p_urb);
        return;
    }

    p_list = p_purb->active_td_list.next;
    while (p_list != &p_purb->active_td_list)
    {
        p_td = list_entry(p_list, td_t, active_td_list);
        p_next = p_list->next;

        if (p_td->td_nr == 0)
        {
            list_add(&p_td->free_td_list, &p_purb->free_td_list);
        }
        else
        {
            p_prev = &p_purb->p_td_table[p_td->td_nr - 1];
            list_add(&p_td->free_td_list, &p_prev->free_td_list);
        }
        list_del_init(&p_td->active_td_list);
        p_list = p_next;
    }
}

static void rt_unschedule_ctrl_bulk_urb(struct rt_privurb *p_purb)
{
  qh_t *p_urb_qh = NULL;
  qh_t *p_prev_qh;
//  qh_t *p_next_qh;

  if (!p_purb){
    ERR("[ERROR] %s - Invalid Pointer to Private URB \n", __FUNCTION__);
    return;
  }

  p_urb_qh = p_purb->p_qh;

  if (list_empty(&p_urb_qh->qh_link_list)){
    ERR("[ERROR] %s - URB 0x%p: Not in QH-Link-List\n", __FUNCTION__, p_purb->p_urb);
    return;
  }

  p_prev_qh = list_entry(p_urb_qh->qh_link_list.prev, qh_t, qh_link_list);
//  p_next_qh = list_entry(p_urb_qh->qh_link_list.next, qh_t, qh_link_list);
//  if (p_next_qh == p_purb->p_uhcd->p_qh_sched){
//    p_next_qh = p_purb->p_uhcd->p_qh_term;
//  }

  /*
  DBG("VORHER  : ------------   ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p     0x%p\n", p_prev_qh, p_urb_qh, p_next_qh);
  DBG("DMA     :  0x%p     0x%p     0x%p\n", (void*)p_prev_qh->dma_handle, (void *)p_urb_qh->dma_handle, (void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->link), (void *)le32_to_cpu(p_urb_qh->link), (void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element), (void *)le32_to_cpu(p_urb_qh->element), (void *)le32_to_cpu(p_next_qh->element));
  DBG("VORHER  : ------------   ------------   ------------\n");

  DBG("NACHHER : ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p\n", p_prev_qh, p_next_qh);
  DBG("DMA     :  0x%p     0x%p\n", (void*)p_prev_qh->dma_handle, (void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_urb_qh->link), (void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element), (void *)le32_to_cpu(p_next_qh->element));
  DBG("NACHHER : ------------   ------------\n");
  */

  DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: ---UNSCHEDULE from %s-List ---\n", p_purb->p_urb,
    p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW ? "Low-Speed" : "Full-Speed");

  p_purb->p_urb->unschedule_time = rtdm_clock_read();

  wmb();
  p_prev_qh->link = p_urb_qh->link;

      // Aus QH-Liste entfernen
  DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: Remove from QH-List\n", p_purb->p_urb);
  list_del_init(&p_urb_qh->qh_link_list);

  return;
}

//TODO
static void rt_unschedule_int_urb(struct rt_privurb *p_purb)
{
    return;
}

//TODO
static void rt_unschedule_isoc_urb(struct rt_privurb *p_purb)
{
    return;
}

static int rt_schedule_ctrl_bulk_urb(struct rt_privurb *p_purb)
{
  qh_t *p_urb_qh = NULL;
  qh_t *p_prev_qh;
  qh_t *p_sched_qh;

  if (!p_purb){
    return -ENODEV;
  }

  p_urb_qh = p_purb->p_qh;

  if (p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW){
    p_sched_qh = p_purb->p_uhcd->p_qh_lowspeed;
  } else {
    p_sched_qh = p_purb->p_uhcd->p_qh_fullspeed;
  }

  if (list_empty(&p_sched_qh->qh_link_list)){  // Dieser URB ist erster
    p_prev_qh = p_sched_qh;
  } else {          // other URBs exists, append this one
    p_prev_qh = list_entry(p_sched_qh->qh_link_list.prev, qh_t, qh_link_list);
  }

  // append on schedule-QH
  DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: Add to QH-List\n", p_purb->p_urb);
  list_add_tail(&p_urb_qh->qh_link_list, &p_sched_qh->qh_link_list);

  /*
  qh_t *p_next_qh = p_purb->p_uhcd->p_qh_term;
  DBG("VORHER  : ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p\n", p_prev_qh, p_next_qh);
  DBG("DMA     :  0x%p     0x%p\n", (void*)p_prev_qh->dma_handle, (void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->link), (void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element), (void *)le32_to_cpu(p_next_qh->element));
  DBG("VORHER  : ------------   ------------\n");

  DBG("NACHHER : ------------   ------------   ------------\n");
  DBG("POINTER :  0x%p     0x%p     0x%p\n", p_prev_qh, p_urb_qh, p_next_qh);
  DBG("DMA     :  0x%p     0x%p     0x%p\n", (void*)p_prev_qh->dma_handle, (void *)p_urb_qh->dma_handle, (void *)p_next_qh->dma_handle);
  DBG("LINK    :  0x%p --> 0x%p --> 0x%p\n",
      (void *)(p_urb_qh->dma_handle | LINK_NO_TERM | LINK_TO_QH), (void *)le32_to_cpu(p_prev_qh->link),
      (void *)le32_to_cpu(p_next_qh->link));
  DBG("ELEMENT :  0x%p     0x%p     0x%p\n",
      (void *)le32_to_cpu(p_prev_qh->element), (void *)le32_to_cpu(p_urb_qh->element), (void *)le32_to_cpu(p_next_qh->element));
  DBG("NACHHER : ------------   ------------   ------------\n");

  */

  DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: ---SCHEDULE in %s-List ---\n", p_purb->p_urb,
    p_purb->p_urb->p_usbdev->speed == USB_SPEED_LOW ? "Low-Speed" : "Full-Speed");

  p_purb->p_urb->schedule_time = rtdm_clock_read();

  p_urb_qh->link = p_prev_qh->link;
  wmb();
  p_prev_qh->link = cpu_to_le32(p_urb_qh->dma_handle | LINK_NO_TERM | LINK_TO_QH);

  return 0;
}

//TODO
static int rt_schedule_int_urb(struct rt_privurb *p_purb)
{
    return 0;
}

//TODO
static int rt_schedule_isoc_urb(struct rt_privurb *p_purb)
{
    return 0;
}

static int rt_uhci_send_ctrl_urb(struct rt_privurb *p_purb)
{
  struct rt_urb *p_urb = NULL;
  __u8 toggle   = 1;
  int out       = 0;
  int max_pl    = 0;
  int remaining = 0;
  __u32 buff    = 0;
  int length    = 0;

  td_t *p_last = NULL;
  td_t *p_list = NULL;
  td_t *p_end  = NULL;

#ifdef DEBUG
  struct usb_ctrlrequest *p_ctrl = NULL;
#endif

  if (!p_purb || !p_purb->p_urb){
    ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
    return -ENODEV;
  }

  if (!list_empty(&p_purb->active_td_list)){
    DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " %s - Active-TD-List is NOT empty !!!\n", __FUNCTION__);
    dump_td_table(p_purb, 0);
    return -EINVAL;
  }

  p_urb = p_purb->p_urb;

#ifdef DEBUG
  p_ctrl = p_purb->p_urb->p_setup_packet;
  DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Sending CTRL-MESSAGE: RT: 0x%02x, RQ: 0x%02x, Val: 0x%04x, Idx: 0x%04x, %d Byte \n",
           p_urb, p_ctrl->bRequestType, p_ctrl->bRequest, p_ctrl->wValue, p_ctrl->wIndex, p_ctrl->wLength);
#endif

  out       = usb_pipeout(p_urb->pipe);
  max_pl    = p_purb->max_packet_size;
  remaining = p_urb->transfer_buffer_length;
  buff      = (__u32) p_urb->transfer_dma;

  p_list    = p_purb->p_td_table;
  p_end     = &p_purb->p_td_table[p_purb->anz_tds -1];

  p_purb->p_qh->element = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF);

  // TD[0] (SETUP)
  p_list->status = cpu_to_le32(td_status_active(1) |
                                td_status_actlen(0) |
                                td_status_bits(0) |
                                td_status_lowspeed(p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0) |
                                td_status_errcount(MAX_CTRL_RETRIES));
  p_list->token  = cpu_to_le32(td_token_pid(SETUP_TOKEN) |
                                td_token_maxlen(7) |
                                td_token_endpoint(usb_pipeendpoint(p_urb->pipe)) |
                                td_token_addr(usb_pipedevice(p_urb->pipe)) |
                                td_token_toggle(0));
  p_list->buffer = cpu_to_le32(p_urb->setup_dma);

  list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
  list_del_init(&p_list->free_td_list);

  //TD[1-(N-1)] (DATA)
  p_last = p_list;
  p_list++;

  while (remaining > 0 && p_list != p_end){
    length = remaining > max_pl ? max_pl : remaining  ;

    p_list->status = cpu_to_le32(td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed(p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0) |
                                  td_status_spd(p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0) |
                                  td_status_errcount(MAX_CTRL_RETRIES));
    p_list->token  = cpu_to_le32(td_token_pid(out ? OUT_TOKEN : IN_TOKEN) |
                                  td_token_maxlen(length - 1) |
                                  td_token_endpoint(usb_pipeendpoint(p_urb->pipe)) |
                                  td_token_addr(usb_pipedevice(p_urb->pipe)) |
                                  td_token_toggle(toggle));
    p_list->buffer = cpu_to_le32(buff);
    buff += length;
    remaining -= length;
    toggle ^= 1;

    p_last->link   = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF);

    list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
    list_del_init(&p_list->free_td_list);

    p_last = p_list;
    p_list++;
  }

  // LAST TD (STATUS)
  p_list->status  = cpu_to_le32( td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed(p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0) |
                                  td_status_spd(p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0) |
                                  td_status_errcount(0) |
                                  td_status_ioc(p_purb->urb_wait_flags == URB_WAIT_BUSY ? 0 : 1));
  p_list->token   = cpu_to_le32( td_token_pid(out ? IN_TOKEN : OUT_TOKEN) |
                                  td_token_maxlen(0x7ff) |
                                  td_token_endpoint(usb_pipeendpoint(p_urb->pipe)) |
                                  td_token_addr(usb_pipedevice(p_urb->pipe)) |
                                  td_token_toggle(1));
  p_list->link    = cpu_to_le32( LINK_TERM);
  p_list->buffer  = cpu_to_le32( 0);

  p_last->link    = cpu_to_le32( p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF);

  list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
  list_del_init(&p_list->free_td_list);

  //dump_td_table(p_purb, 0);

  return rt_schedule_ctrl_bulk_urb(p_purb);
}

static int rt_uhci_send_bulk_urb(struct rt_privurb *p_purb)
{
  struct rt_urb *p_urb = p_purb->p_urb;

  __u8 toggle   = 0;
  int out       = 0;
  int max_pl    = 0;
  int remaining = 0;
  __u32 buff    = 0;
  int length    = 0;

  td_t *p_last = NULL;
  td_t *p_list = NULL;
  td_t *p_end  = NULL;

  int ioc_after_first_td = 0;
  int ioc_after_last_td = 0;


  if (!p_purb || !p_purb->p_urb){
    ERR("[ERROR] %s - Invalid Private-URB-Pointer \n", __FUNCTION__);
    return -ENODEV;
  }

  p_urb     = p_purb->p_urb;
  toggle    = usb_get_data_toggle(p_urb->p_usbdev, p_urb->pipe);
  out       = usb_pipeout(p_urb->pipe);
  max_pl    = p_purb->max_packet_size;
  remaining = p_urb->transfer_buffer_length;
  buff      = (__u32) p_urb->transfer_dma;

  p_list    = p_purb->p_td_table;
  p_end     = &p_purb->p_td_table[p_purb->anz_tds -1];

  p_purb->compl_tds_at_first_irq = 0;
  p_purb->byte_at_first_irq = 0;
  p_purb->time_at_first_irq = 0;

  if (p_purb->urb_wait_flags == URB_WAIT_SEM ||
      p_purb->urb_wait_flags == URB_CALLBACK) {
    if (p_purb->p_urb->transfer_flags & URB_TIMESTAMP_DATA){ /* get Timestamp */
      ioc_after_first_td = 1;  /* to get time of the first bits */
      p_purb->get_time_of_first_bit = 1;
    }
    ioc_after_last_td = 1;   /* Interrupt IOC (wake up caller-task) */
  }

  if (out){
   INFO("Sending %d Bytes: [0]: 0x%02x, [1]: 0x%02x \n",
      remaining,
      *(unsigned char *)p_urb->p_transfer_buffer,
      *(unsigned char *)(p_urb->p_transfer_buffer+1));
  }

  p_purb->p_qh->element = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF);

  while (remaining > 0 && p_list != p_end){
    length = remaining > max_pl ? max_pl  : remaining  ;

    p_list->buffer = cpu_to_le32(buff);
    buff += length;
    remaining -= length;

    p_list->status = cpu_to_le32(td_status_active(1) |
                                  td_status_actlen(0) |
                                  td_status_bits(0) |
                                  td_status_lowspeed(p_urb->p_usbdev->speed == USB_SPEED_LOW ? 1 : 0) |
                                  td_status_spd(p_urb->transfer_flags & URB_SHORT_NOT_OK ? 1 : 0) |
                                  td_status_ioc(((p_list->td_nr == 0 && ioc_after_first_td) | /* first TD */
                                                   (remaining == 0 && ioc_after_last_td)) ? 1 : 0) |
                                  td_status_errcount(0));
    p_list->token  = cpu_to_le32(td_token_pid(out ? OUT_TOKEN : IN_TOKEN) |
                                  td_token_endpoint(usb_pipeendpoint(p_urb->pipe)) |
                                  td_token_addr(usb_pipedevice(p_urb->pipe)) |
                                  td_token_maxlen(length -1) |
                                  td_token_toggle(toggle));
    toggle ^= 1;

    if (p_last){
      p_last->link = cpu_to_le32(p_list->dma_handle | LINK_NO_TERM | LINK_TO_TD | LINK_VF);
    }
    if (!remaining){
      usb_set_data_toggle(p_urb->p_usbdev, p_urb->pipe, toggle);
      p_list->link = cpu_to_le32(LINK_TERM);
    }

    list_add_tail(&p_list->active_td_list, &p_purb->active_td_list);
    list_del_init(&p_list->free_td_list);

    p_last = p_list;
    p_list++;
  }

  if (out){
    dump_td_table(p_purb, 0);
  }

  return rt_schedule_ctrl_bulk_urb(p_purb);
}

// T O D O
static int rt_uhci_send_int_urb(struct rt_privurb *p_purb)
{
  rt_schedule_int_urb(p_purb);
  rt_unschedule_int_urb(p_purb);
  return 0;
}

// T O D O
static int rt_uhci_send_isoc_urb(struct rt_privurb *p_purb)
{
  rt_schedule_isoc_urb(p_purb);
  rt_unschedule_isoc_urb(p_purb);
  return 0;
}

static void rt_complete_urb(struct rt_privurb *p_purb)
{
  p_purb->status = PURB_IN_HANDLE;
  DBG_MSG2(p_purb->p_urb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: PURB_IN_HANDLE \n", p_purb->p_urb);

  switch (usb_pipetype(p_purb->p_urb->pipe)){
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

  DBG_MSG2(p_purb->p_urb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: %d / %d Byte %s\n", p_purb->p_urb,
           p_purb->p_urb->actual_length, p_purb->p_urb->transfer_buffer_length,
           usb_pipeout(p_purb->p_urb->pipe) ? "sent" : "received");

  // Unmap DMA
  unmap_dma(p_purb);


  DBG_MSG2(p_purb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: Free TDs\n", p_purb->p_urb);
  rt_free_urb_tds(p_purb);

  p_purb->status = PURB_IDLE;
  DBG_MSG2(p_purb->p_urb->p_hcd, p_purb->p_urb->p_usbdev, " URB 0x%p: PURB_IDLE \n", p_purb->p_urb);

}

static inline int rt_uhci_send_busywait_urb(struct rt_privurb *p_purb)
{
  struct rt_urb *p_urb = p_purb->p_urb;
  int ret = 0;

  /* Mappe evtl DMA */
  ret = map_dma(p_purb);
  if (ret){
    dump_urb(p_urb);
    return -ENOMEM;
  }

  DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: BUSY WAIT\n", p_urb);

  p_purb->status = PURB_IN_PROGRESS;
  DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IN_PROGRESS \n", p_urb);

  switch (usb_pipetype(p_urb->pipe)){
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
  if (ret){
    p_purb->status = PURB_IDLE;
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IDLE \n", p_urb);
    unmap_dma(p_purb);
    return ret;
  }

  ret = wait_for_urb(p_purb);
  if (ret){
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p BUSY-WAIT -> TIMEOUT\n", p_urb);
    dump_td_table(p_purb, 0);
  }

  rt_complete_urb(p_purb);

  return p_urb->actual_length;
}

static inline int rt_uhci_send_semaphore_urb(struct rt_privurb *p_purb)
{
  struct rt_urb *p_urb = p_purb->p_urb;
  int ret = 0;

  /* Mappe evtl DMA */
  ret = map_dma(p_purb);
  if (ret){
    dump_urb(p_urb);
    return -ENOMEM;
  }

  DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: SLEEPING MAX %llu ns\n", p_urb, p_urb->rt_sem_timeout);

  p_purb->status = PURB_IN_PROGRESS;
  DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IN_PROGRESS \n", p_urb);

  switch (usb_pipetype(p_urb->pipe)){
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
  if (ret){
    p_purb->status = PURB_IDLE;
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IDLE \n", p_urb);
    unmap_dma(p_purb);
    return ret;
  }

  ret = rtdm_sem_timeddown(&p_purb->p_urb->rt_sem,
                           p_purb->p_urb->rt_sem_timeout,
                           NULL);
  switch (ret){
    case -EINVAL:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM is not a semaphore descriptor \n", p_urb);
      break;
    case -EIDRM:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM is a deleted semaphore descriptor \n", p_urb);
      break;
    case -EWOULDBLOCK:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM would block \n", p_urb);
      break;
    case -EINTR:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM interrupted \n", p_urb);
      break;
    case -ETIMEDOUT:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM timed out \n", p_urb);
      break;
    case -EPERM:
      DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM cannot sleep \n", p_urb);
      break;
    case 0:
      if (p_purb->status & PURB_WAKE_UP_BY_IRQ){
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM waked up by ISR, Sleeptime: (%llu ns)\n",
        p_urb, rtdm_clock_read() - p_urb->schedule_time);
      } else {
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: RT_SEM waked up by someone else, sleeptime: %llu ns\n",
        p_urb, rtdm_clock_read() - p_urb->schedule_time);
      }
      break;
  }

  rt_complete_urb(p_purb);

  return p_urb->actual_length;
}

static inline int rt_uhci_send_callback_urb(struct rt_privurb *p_purb)
{
    struct rt_urb *p_urb = p_purb->p_urb;
    int ret = 0;

    // DMA mapping (if needed)
    ret = map_dma(p_purb);
    if (ret)
    {
        dump_urb(p_urb);
        return -ENOMEM;
    }

    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Sent Callback\n", p_urb);

    p_purb->status = PURB_IN_PROGRESS;
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IN_PROGRESS \n",
             p_urb);

    switch (usb_pipetype(p_urb->pipe))
    {
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
    if (ret)
    {
        p_purb->status = PURB_IDLE;
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: PURB_IDLE \n", p_urb);
        unmap_dma(p_purb);
        return ret;
    }
    return 0;
}

/******************************************************************/
/*  HCD-FKT Funktionen                                            */
/******************************************************************/

void nrt_uhci_remove_controller(struct uhc_device *p_uhcd)
{
    if (p_uhcd->status & UHC_REGISTERED)
    {
        nrt_hcd_unregister_driver(p_uhcd->p_hcd);
        p_uhcd->status &= ~UHC_REGISTERED;
    }

    if (p_uhcd->status & UHC_RUNNING)
    {
        uhc_stop(p_uhcd);
        p_uhcd->status &= ~UHC_RUNNING;
    }

    if (p_uhcd->status & UHC_INITIALIZED)
    {
        destroy_td(p_uhcd->p_td_loop);
        destroy_qh(p_uhcd->p_qh_fullspeed);
        destroy_qh(p_uhcd->p_qh_lowspeed);
        destroy_qh(p_uhcd->p_qh_term);
        destroy_flame_list(p_uhcd);
        p_uhcd->status &= ~UHC_INITIALIZED;
    }

    if (p_uhcd->status & UHC_ALLOCATED)
    {
        if (p_uhcd->p_hcd)
        {
            kfree(p_uhcd->p_hcd);
            alloc_bytes -= sizeof(struct hc_device);
        }
        put_irq(p_uhcd);
        release_ioport(p_uhcd);
        p_uhcd->status &= ~UHC_ALLOCATED;
    }
    return;
}

/**
 * Searching for UH-Controller on the Host. Furthermore this function creates
 * the struct hc_device for the rt-usbcore.
 *
 * @param p_uhcd Pointer to an empty uhc_device
 * @return 0, device found
 * @return > 0, no device found
 * @return -ENOMEM, no memory for hc_device available
 */
int nrt_uhci_search_controller(struct uhc_device *p_uhcd)
{
    struct pci_dev *p_pcidev_new = NULL;
    int i;
    unsigned long io_start = 0;
    unsigned long io_size  = 0;
    int ret                = 0;
    struct hc_device *p_hcd = NULL;

start_search:
    uhc_clear(p_uhcd);

    // searching for controller
    p_pcidev_new = pci_get_class(UHC_CLASS_CODE, p_pcidev_old);
    if (!p_pcidev_new)
    {
        p_pcidev_old = NULL;
        return 1; // no device found
    }
    p_pcidev_old = p_pcidev_new;

    // device found but the device-id is undesired
    for (i=0; i < deviceid_c; i++)
    {
        if (deviceid[i] && deviceid[i] != p_pcidev_new->device)
            goto start_search;
    }

    // device found but the irq-number is invalid
    for (i=0; i < irq_c; i++)
    {
        if (irq[i] && irq[i] != p_pcidev_new->irq)
            goto start_search;
    }

    // device found but the io-port is invalid
    ret = get_ioport_info(p_pcidev_new, &io_start, &io_size);
    if (ret)
    {
        PRNT("Couldn't get informations of the IO-Port \n");
        goto start_search;
    }

    for (i=0; i < ioport_c; i++)
    {
        if (ioport[i] && ioport[i] != io_start)
            goto start_search;
    }

    // device has been found
    PRNT("USB Universal Host Controller found : Vendor = 0x%04x, Device = 0x%04x, "
         "IRQ = %d, IO-Port = 0x%08lx (%lu Bytes)\n", p_pcidev_new->vendor,
         p_pcidev_new->device, p_pcidev_new->irq, io_start, io_size);

    // generate hc_device for the rt-core-module
    p_hcd = kmalloc(sizeof(struct hc_device), GFP_KERNEL);
    if (!p_hcd)
    {
        uhc_clear(p_uhcd);
        return -ENOMEM;
    }
    alloc_bytes += sizeof(struct hc_device);

    p_hcd->type  = 0x00;
    p_hcd->p_hcd_fkt = &hcd_fkt;
    p_uhcd->p_pcidev = p_pcidev_new;
    p_uhcd->p_hcd = p_hcd;

    // checking io-resource
    if (request_ioport(p_uhcd, io_start, io_size))
    {
        kfree(p_hcd);
        alloc_bytes -= sizeof(struct hc_device);
        goto start_search;
    }

    // allocate RTDM interrupt
    if (get_irq(p_uhcd))
    {
        release_ioport(p_uhcd);
        kfree(p_hcd);
        alloc_bytes -= sizeof(struct hc_device);
        goto start_search;
    }
    return 0;
}

int nrt_uhci_register_urb(struct rt_urb *p_urb)
{
    /*
    checked in core-module:
        p_urb
        p_urb->p_hcd
        p_urb->p_hcd->p_hcd_fkt
        p_urb->max_packet_size
        p_urb->max_buffer_len
    */

    int i , ret;
    struct uhc_device *p_uhcd = NULL;
    struct rt_privurb *p_purb = NULL;
    struct list_head  *p_list = NULL;

    // get host controller
    for (i=0; i <  MAX_DEVICES; i++)
    {
        if (uhc_dev[i].p_hcd == p_urb->p_hcd)
        {
            p_uhcd = &uhc_dev[i];
            break;
        }
    }

    if (!p_uhcd || !p_uhcd->p_pcidev)
    {
        ERR("[ERROR] %s - Invalid Pointer to UHC \n", __FUNCTION__);
        return -ENODEV;
    }

    if (p_uhcd->status & UHC_HOST_ERROR)
    {
        ERR("[ERROR] %s - Host-Controller-Error, REJECT \n", __FUNCTION__);
        return -ENODEV;
    }

    if (!(p_uhcd->status & UHC_RUNNING))
    {
        ERR("[ERROR] %s - Host-Controller not running, REJECT \n", __FUNCTION__);
        return -ENODEV;
    }

    // check if the URB is registered yet
    p_purb = NULL;
    p_list = p_uhcd->reg_urb_list.next;;

    while (p_list != &p_uhcd->reg_urb_list)
    {
        p_purb = list_entry(p_list, struct rt_privurb, reg_urb_list);
        if (p_purb->p_urb == p_urb)
            break;
        else
            p_purb = NULL;

        p_list = p_list->next;
    }

    if (p_purb)
    {
        DBG_MSG1(p_purb->p_hcd, " URB 0x%p is already registered\n", p_urb);
        return -EINVAL;
    }

    // creating struct rt_privurb
    p_purb = kmalloc(sizeof(struct rt_privurb), GFP_ATOMIC);
    if (!p_purb)
    {
        ERR_MSG1(p_urb->p_hcd, " %s - No Memory for Private URB-Data \n",
                 __FUNCTION__);
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

    // create TD-table by means of the endpoint's packet size and the
    // maximal buffer size
    if (p_purb->max_buffer_len % p_purb->max_packet_size)
        p_purb->anz_tds = (p_purb->max_buffer_len / p_purb->max_packet_size) + 1;
    else
        p_purb->anz_tds = p_purb->max_buffer_len / p_purb->max_packet_size;

    // control transfer needs 2 additional TDs (setup & status)
    p_purb->anz_tds += 2;

    ret = create_td_table(p_purb);
    if (ret)
    {
        kfree(p_purb);
        alloc_bytes -= sizeof(struct rt_privurb);

        ERR_MSG1(p_urb->p_hcd, " %s - Error while creating TD-Table \n",
                 __FUNCTION__);
        return ret;
    }

    // creating queue-heads
    p_purb->p_qh = create_qh(p_uhcd);
    if (!p_purb->p_qh)
    {
        destroy_td_table(p_purb);
        kfree(p_purb);
        alloc_bytes -= 1 * sizeof(struct rt_privurb);
        ERR_MSG1(p_purb->p_hcd, " %s - Error while creating Queue-Header \n",
                 __FUNCTION__);
        return ret;
    }

    list_add_tail(&p_purb->reg_urb_list, &p_uhcd->reg_urb_list);
    p_purb->get_module = try_module_get(THIS_MODULE);

    DBG_MSG1(p_urb->p_hcd, " URB @ 0x%p registered, %d TDs, Max-Buffer: %d, "
             "Max-Packet-Size: %d\n", p_urb, p_purb->anz_tds,
             p_purb->max_buffer_len, p_purb->max_packet_size);
    return 0;
}

int nrt_uhci_unregister_urb(struct rt_urb *p_urb)
{
    struct uhc_device *p_uhcd = NULL;
    struct rt_privurb *p_purb = NULL;

    p_purb = p_urb->p_private;
    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Pointer to Private URB-Data \n", __FUNCTION__);
        return -ENODEV;
    }

    p_uhcd = p_purb->p_uhcd;
    if (!p_uhcd)
    {
        ERR("[ERROR] %s - Invalid Pointer to UHC \n", __FUNCTION__);
        return -ENODEV;
    }

    if ((p_purb->status == PURB_IN_PROGRESS) ||
        (p_purb->status == PURB_IN_HANDLE))
    {
        ERR_MSG1(p_purb->p_hcd, " %s - URB 0x%p: IN PROGRESS, UNSCHEDULE !!!\n",
                 __FUNCTION__, p_purb->p_urb);

        rt_complete_urb(p_purb);
    }

    list_del_init(&p_purb->reg_urb_list);

    if (p_purb->get_module)
        module_put(THIS_MODULE);

      // deleting QH
    destroy_qh(p_purb->p_qh);

    // deleting TD-table
    destroy_td_table(p_purb);

    // deleting urb_priv
    kfree(p_purb);
    alloc_bytes -= 1 * sizeof(struct rt_privurb);

    DBG_MSG1(p_purb->p_hcd, " URB 0x%p unregistered\n", p_urb);
    return 0;
}

int rt_uhci_submit_urb(struct rt_urb *p_urb , __u16 urb_submit_flags)
{
    struct rt_privurb *p_purb = NULL;
    struct uhc_device *p_uhcd = NULL;

    p_purb = p_urb->p_private;
    if (!p_purb)
    {
        ERR("[ERROR] %s - Invalid Pointer to Private URB-Data \n", __FUNCTION__);
        return -ENODEV;
    }

    p_uhcd = p_purb->p_uhcd;
    if (!p_uhcd)
    {
        ERR("[ERROR] %s - Invalid Pointer to UHC \n", __FUNCTION__);
        return -ENODEV;
    }

    if (p_uhcd->status & UHC_HOST_ERROR)
    {
        ERR("[ERROR] %s - Host-Controller-Error, REJECT \n", __FUNCTION__);
        return -ENODEV;
    }

    if (!(p_uhcd->status & UHC_RUNNING))
    {
        ERR("[ERROR] %s - Host-Controller not running, REJECT \n", __FUNCTION__);
        return -ENODEV;
    }

    // checking (normally) constant values
    if (p_urb->p_hcd != p_purb->p_hcd)
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " %s - Invalid Host-Controller: "
                 "0x%p sent, 0x%p allowed\n", __FUNCTION__, p_urb->p_hcd,
                 p_purb->p_hcd);
        return -EINVAL;
    }

    if (p_urb->max_packet_size != p_purb->max_packet_size)
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " %s - Invalid Max-Packet-Size: "
                 "%d sent, %d allowed \n", __FUNCTION__, p_urb->max_packet_size,
                 p_purb->max_packet_size);
        return -EINVAL;
    }

    if (p_urb->max_buffer_len != p_purb->max_buffer_len)
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " %s - Invalid Max-Buffer-Length: "
                 "%d sent, %d allowed \n", __FUNCTION__, p_urb->max_buffer_len,
                 p_purb->max_buffer_len);
        return -EINVAL;
    }

    if (!(p_purb->status == PURB_IDLE))
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: URB IN PROGRESS \n",
                 p_urb);
        return -EBUSY;
    }

    // checking complete function
    if (urb_submit_flags & URB_WAIT_SEM)
    {
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Blocking URB "
                 "(Semaphore) \n", p_urb);
        p_purb->urb_wait_flags = URB_WAIT_SEM;

    }
    else if (urb_submit_flags & URB_CALLBACK)
    {
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Non-Blocking URB "
                 "(Callback-Function @ 0x%p) \n", p_urb, p_urb->rt_complete_fkt);
        p_purb->urb_wait_flags = URB_CALLBACK;

    }
    else if (urb_submit_flags & URB_WAIT_BUSY)
    {
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB 0x%p: Blocking URB "
                 "(Busy Wait) \n", p_urb);
        p_purb->urb_wait_flags = URB_WAIT_BUSY;
    }

    // save pipe
    p_purb->pipe_save = p_urb->pipe;

    // OK - send URB
    switch (p_purb->urb_wait_flags)
    {
        case URB_WAIT_BUSY:
            return rt_uhci_send_busywait_urb(p_purb);

        case URB_WAIT_SEM:
            return rt_uhci_send_semaphore_urb(p_purb);

        case URB_CALLBACK:
            return rt_uhci_send_callback_urb(p_purb);
    }

    return -EINVAL;
}

/******************************************************************
 *  module - functions                                            *
 ******************************************************************/

int __init mod_start(void)
{
    int i, ret;
    struct uhc_device *p_uhcd;
    uint64_t start, end;

    PRNT(KERN_INFO "********* " DRIVER_DESC " " DRIVER_VERSION " **********\n");

    // controller functions
    hcd_fkt.nrt_hcd_register_urb        = nrt_uhci_register_urb;
    hcd_fkt.nrt_hcd_unregister_urb      = nrt_uhci_unregister_urb;
    hcd_fkt.rt_hcd_submit_urb           = rt_uhci_submit_urb;

    // root-hub functions
    hcd_fkt.nrt_hcd_poll_root_hub_port  = nrt_uhci_poll_root_hub_port;

    // init uhc_irq-table
    for (i=0; i < MAX_DEVICES; i++)
    {
        memset(&uhc_irq_tab[i], 0, sizeof(struct uhc_irq));
        INIT_LIST_HEAD(&uhc_irq_tab[i].irq_list);
    }

    anz_uhc_ctrl = 0;

    PRNT("RT-UHC-Driver: Searching for Universal-Host-Controller \n");

    for (i=0; i < MAX_DEVICES; i++)
    {
        if (deviceid[i] > 0)
        {
            PRNT("RT-UHC-Driver: -> Use only HC with Device-ID 0x%04x \n",
                 deviceid[i])
        }
    }

    for (i=0; i < MAX_DEVICES; i++)
    {
        if (irq[i] > 0)
        {
            PRNT("RT-UHC-Driver: -> Use only HC with IRQ %d \n", irq[i])
        }
    }

    for (i=0; i < MAX_DEVICES; i++)
    {
        p_uhcd = &uhc_dev[i];
        p_uhcd->uhcd_nr = i;

        ret = nrt_uhci_search_controller(p_uhcd);
        if (ret) // no controller found or no memory for new controller
            break;

        p_uhcd->status |= UHC_ALLOCATED;

        // initializing controller
        ret = uhc_init(p_uhcd );
        if (ret)
        {
            PRNT("RT-UHC-Driver: Error while initializing UHC (%d)\n", ret);
            nrt_uhci_remove_controller(p_uhcd);
        }
        p_uhcd->status |= UHC_INITIALIZED;

        // starting controller
        ret = uhc_start(p_uhcd);
        if (ret)
        {
            PRNT("RT-UHC-Driver: Error while starting UHC (%d)\n", ret);
            nrt_uhci_remove_controller(p_uhcd);
        }
        p_uhcd->status |= UHC_RUNNING;

        // registering controller
        ret = nrt_hcd_register_driver(p_uhcd->p_hcd);
        if (ret)
        {
            PRNT("RT-UHC-Driver: Error while registering UHC (%d)\n", ret);
            nrt_uhci_remove_controller(p_uhcd);
            break;
        }
        p_uhcd->status |= UHC_REGISTERED;
        p_uhcd->p_hcd->p_private = (void *)p_uhcd;

        anz_uhc_ctrl++;
        DBG_MSG1(p_uhcd->p_hcd, " USB Universal Host Controller @ 0x%p "
                 "registered \n", p_uhcd);

        // searching for USB devices on this controller
        p_uhcd->p_hcd->p_hcd_fkt->nrt_usb_search_devices(p_uhcd->p_hcd);
    }

    if (!anz_uhc_ctrl)
    {
        PRNT("RT-UHC-Driver: No UHC-Driver registered -> Exit Module \n");
        return -ENODEV;
    }

    start = rtdm_clock_read();
    end = rtdm_clock_read();
    rt_timer_overhead = end - start;

    PRNT("RT-UHC-Driver: Loading Completed (%d Byte allocated) \n", alloc_bytes);
    return 0;
}

void mod_exit(void)
{
    int i=0;
    for (i=0; i < anz_uhc_ctrl; i++)
    {
        nrt_uhci_remove_controller(&uhc_dev[i]);
    }

    PRNT("RT-UHC-Driver : Unloading complete (%d byte allocated)\n",
         alloc_bytes);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_init (mod_start);
module_exit (mod_exit);
