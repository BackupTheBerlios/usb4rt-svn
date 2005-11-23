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
#ifndef RT_UHCI_H
#define RT_UHCI_H

#include <linux/pci.h>
#include <xenomai/native/intr.h>
#include <core/rt_usb_core.h>

#define MAX_UHC_CONTROLLER      4

/* PCI-HC Register */
#define PCIVID                0x0000    /* PCI Vendor ID */
#define PCIDID                0x0002    /* PCI Device ID*/
#define PCICMD                0x0004    /* PCI Command Register */
#define PCISTA                0x0006    /* PCI Status Register */
#define PCIIOBASE             0x0020    /* PCI - I/O Basisadresse */

/* BITs im PCI-Command-Register */
#define PCICMD_IOSE           0x0001    /* I/O-Space Enable */
#define PCICMD_MSE            0x0002    /* Memory Space Enable */
#define PCICMD_BME            0x0004    /* BUS Master Enable */
/*...*/

/* USB-IO Register */
#define USBCMD                0x0000    /* USB Command (16 Bit) */
#define   USBCMD_RS             0x0001    /* Run(1) / Stop(0) [R/W] */
#define   USBCMD_HCRESET        0x0002    /* Host Controller Reset [R/W] */
#define   USBCMD_GRESET         0x0004    /* Global Reset [R/W] */
#define   USBCMD_EGSM           0x0008    /* Enter global Suspend Mode [R/W] */
#define   USBCMD_FGR            0x0010    /* Force global Resume [R/W] */
#define   USBCMD_SWDBG          0x0020    /* SW Debug mode */
#define   USBCMD_CF             0x0040    /* Config Flag (sw only) */
#define   USBCMD_MAXP           0x0080    /* Max Packet (0 = 32, 1 = 64) */

#define USBSTS                0x0002    /* USB Status (16 Bit) */
#define   USBSTS_USBINT         0x0001    /* Interrupt due to IOC */
#define   USBSTS_ERROR          0x0002    /* Interrupt due to error */
#define   USBSTS_RD             0x0004    /* Resume Detect */
#define   USBSTS_HSE            0x0008    /* Host System Error - basically PCI problems */
#define   USBSTS_HCPE           0x0010    /* Host Controller Process Error - the scripts were buggy */
#define   USBSTS_HCH            0x0020    /* HC Halted */

#define USBINTR               0x0004    /* USB Interrupt Enable (16 Bit) */
#define   USBINTR_TIMEOUT       0x0001    /* Timeout / CRC Interrupt Enable */
#define   USBINTR_RESUME        0x0002    /* Resume Interrupt Enable */
#define   USBINTR_IOC           0x0004    /* Interrupt on Complete (IOC) */
#define   USBINTR_SP            0x0008    /* Short Packet Interrupt Enable */

#define FRNUM                 0x0006    /* USB Frame-Nummer (16 Bit) */
#define FRBASEADD             0x0008    /* USB Frame-List Base Adresse (32 Bit)*/
#define SOFMOD                0x000C    /* USB Start of Frame Modify (8 Bit)*/
#define PORTSC0               0x0010    /* Port 0 Status/Control */
#define PORTSC1               0x0012    /* Port 1 Status/Control */

/* Legacy support register */
#define USBLEGSUP             0xc0
#define   USBLEGSUP_DEFAULT   0x2000          /* only PIRQ enable set */

#define IN_TOKEN              0x69
#define OUT_TOKEN             0xe1
#define SETUP_TOKEN           0x2d

#define UHC_CLASS_CODE        0x0c0300
#define MAX_FRAMES            1024

#define LINK_TERM             (1 << 0)
#define LINK_NO_TERM          (0 << 0)
#define LINK_TO_QH            (1 << 1)
#define LINK_TO_TD            (0 << 1)
#define LINK_VF               (1 << 2)  /* Vertikal zuerst */
#define LINK_NO_VF            (0 << 2)  /* Horizontal zuerst */

#define FS_NS_PER_BYTE        667
#define FS_NS_PER_SOF         3500  /* 5 Byte @ 12MHz */
#define FS_NS_PER_preSOF      52000 /* preSOF for 64-Byte-Packets */
#define EOP_DELAY_BYTE        1     /* Time for EOP and Delay between two packets */

 /**
  * Frame-Liste.
  * Der Frame-Pointer setzt sich aus frame_base (obere 20 Bit) und frame_index (untere 10 Bit) zusammen.
  */
struct frame_list {
  __le32 frame_dma[MAX_FRAMES];
  void *frame_cpu[MAX_FRAMES];
  dma_addr_t dma_handle;
};

typedef struct queue_head {
  __le32 link;
  __le32 element;

  dma_addr_t dma_handle;
  struct list_head qh_link_list;

  struct uhc_device *p_uhcd;
  __u8 qh_nr;
}  __attribute__((aligned(16))) qh_t;


/************************************************
  Transfer-Descriptor
 ***********************************************/

/* for TD <status> */
#define td_status(p_td)       le32_to_cpu((p_td)->status)
#define TD_STAT_SPD           (1 << 29)       /* Short Packet Detect */
#define TD_STAT_C_ERR_MASK    (3 << 27)       /* Error Counter bits */
#define TD_STAT_C_ERR_SHIFT   27
#define TD_STAT_LS            (1 << 26)       /* Low Speed Device */
#define TD_STAT_ISO           (1 << 25)       /* Isochronous Select */
#define TD_STAT_IOC           (1 << 24)       /* Interrupt on Complete */
#define TD_STAT_ACTIVE        (1 << 23)       /* TD Active */
#define TD_STAT_BIT_SHIFT     17
#define TD_STAT_BIT_MASK      (0x3f << 17)    /* Status-Bits set by Controller */
#define TD_STAT_STALLED       (1 << 22)       /* TD Stalled */
#define TD_STAT_DBUFERR       (1 << 21)       /* Data Buffer Error */
#define TD_STAT_BABBLE        (1 << 20)       /* Babble Detected */
#define TD_STAT_NAK           (1 << 19)       /* NAK Received */
#define TD_STAT_CRCTIMEO      (1 << 18)       /* CRC/Time Out Error */
#define TD_STAT_BITSTUFF      (1 << 17)       /* Bit Stuff Error */
#define TD_STAT_ACTLEN_MASK   0x7FF           /* actual length, encoded as n - 1 */
#define TD_STAT_ANY_ERROR     (TD_STAT_STALLED | TD_STAT_DBUFERR | TD_STAT_BABBLE | TD_STAT_CRCTIMEO | TD_STAT_BITSTUFF)

#define td_status_errcount(err)   ( (u32)(((err) << TD_STAT_C_ERR_SHIFT) & TD_STAT_C_ERR_MASK) )
#define td_status_active(act)     ( (u32)((act) ? TD_STAT_ACTIVE : 0 ) )
#define td_status_lowspeed(ls)    ( (u32)((ls)  ? TD_STAT_LS     : 0 ) )
#define td_status_ioc(val)        ( (u32)((val) ? TD_STAT_IOC    : 0 ) )
#define td_status_spd(val)        ( (u32)((val) ? TD_STAT_SPD    : 0 ) )
#define td_status_iso(val)        ( (u32)((val) ? TD_STAT_ISO    : 0 ) )
#define td_status_bits(val)       ( (u32)(((val) << TD_STAT_BIT_SHIFT) & TD_STAT_BIT_MASK) )
#define td_status_actlen(val)     ( (u32)((val) & TD_STAT_ACTLEN_MASK ) )

#define set_status_ioc(td)        ( td_status(td) |= TD_STAT_IOC )
#define clear_status_ioc(td)      ( td_status(td) &= ~TD_STAT_IOC )

#define get_status_active(td)     ( td_status(td) & TD_STAT_ACTIVE )
#define get_status_actlen(td)     ( td_status(td) & TD_STAT_ACTLEN_MASK )
#define get_status_errcount(td)   ( ( td_status(td)  & TD_STAT_C_ERR_MASK ) >> TD_STAT_C_ERR_SHIFT )

/* for TD <token> */
/* TOKEN : XXXXXXXX XXX-TEEE EDDDDDDD PPPPPPPP */
/* D = Device */
/* E = Endpoint */
/* T = Toggle-Bit */
/* X = Expected Length */
/* P = PID */
#define td_token(td)              le32_to_cpu((td)->token)
#define TD_TOKEN_DEVADDR_SHIFT    8
#define TD_TOKEN_DEVADDR_MASK     0x7f
#define TD_TOKEN_ENDPOINT_SHIFT   15
#define TD_TOKEN_ENDPOINT_MASK    0xf
#define TD_TOKEN_TOGGLE_SHIFT     19
#define TD_TOKEN_TOGGLE           (1 << 19)
#define TD_TOKEN_EXPLEN_SHIFT     21
#define TD_TOKEN_EXPLEN_MASK      0x7ff   /* expected length, encoded as n - 1 */
#define TD_TOKEN_PID_MASK         0xff

#define td_token_pid(pid)         ( (u32)((pid) & TD_TOKEN_PID_MASK) )
#define td_token_addr(addr)       ( (u32)((addr & TD_TOKEN_DEVADDR_MASK) << TD_TOKEN_DEVADDR_SHIFT) )
#define td_token_endpoint(ep)     ( (u32)((ep & TD_TOKEN_ENDPOINT_MASK) << TD_TOKEN_ENDPOINT_SHIFT) )
#define td_token_maxlen(len)      ( (u32)(((len) & TD_TOKEN_EXPLEN_MASK) << TD_TOKEN_EXPLEN_SHIFT) )
#define td_token_toggle(val)      ( (u32)((val & 1) << TD_TOKEN_TOGGLE_SHIFT) )

#define get_packet_type(td)       ( td_token(td) & TD_TOKEN_PID_MASK )
#define get_max_len(td)           ( ((td_token(td) >> TD_TOKEN_EXPLEN_SHIFT)   & TD_TOKEN_EXPLEN_MASK   ))
#define get_endpoint(td)          ( ((td_token(td) >> TD_TOKEN_ENDPOINT_SHIFT) & TD_TOKEN_ENDPOINT_MASK ))
#define get_address(td)           ( ((td_token(td) >> TD_TOKEN_DEVADDR_SHIFT)  & TD_TOKEN_DEVADDR_MASK  ))


typedef struct transf_desc {
  __le32 link;
  __le32 status;
  __le32 token;
  __le32 buffer;

  /* private Data */
  dma_addr_t dma_handle;
  struct list_head free_td_list;
  struct list_head active_td_list;

  __u16 td_nr;
  struct uhc_device *p_uhcd;
} __attribute__((aligned(16))) td_t;

/* UHC-STATUS */
#define UHC_ALLOCATED     0x0001
#define UHC_INITIALIZED   0x0002
#define UHC_HOST_ERROR    0x0004
#define UHC_RUNNING       0x0008
#define UHC_REGISTERED    0x0010


struct uhc_device {
  int irq;

  __u16 status;
  __u8 uhcd_nr;
  struct hc_device *p_hcd;

  struct resource *p_io;
  struct pci_dev *p_pcidev;
  struct frame_list *p_fl;

  struct list_head irq_list;          /* list of all uhc with the same rtai-irq */
  struct list_head reg_urb_list;      /* list pointer to all registered urbs */
  struct list_head handle_urb_list;   /* list pointer to all active urbs */

  qh_t *p_qh_lowspeed;
  qh_t *p_qh_fullspeed;
  qh_t *p_qh_term;
  td_t *p_td_loop;
};

/* URB-STATUS */
#define PURB_IDLE             0x0001
#define PURB_IN_PROGRESS      0x0002
#define PURB_IN_HANDLE        0x0004

/* PURB_WAIT_FLAGS */
#define PURB_WAKE_UP_BY_IRQ   0x0100

struct rt_privurb {

  struct list_head reg_urb_list;      /* list pointer to all registered urbs */
  struct list_head handle_urb_list;   /* list pointer to all active urbs */

  //int bandwidth;                /* bandwidth for INT/ISO request */
  u8 reject;                      /* submissions will fail */

  struct rt_urb *p_urb;           /* Pointer to the URB */

  qh_t *p_qh;                     /* Queue-Head for this URB */

  td_t  *p_td_table;              /* Pointer to the TD-Table */
  int table_size;                 /* Size of the TD-Table in Byte */
  dma_addr_t td_table_dma;        /* DMA-Handle for the TD-Table */
  int anz_tds;                    /* Number of TDs in the TD-Table */
  int free_tds;                   /* Number of free TDs in the Table */

  // save constant Values of the registered URB
  int max_packet_size;            /* Bits per TD */
  int max_buffer_len;             /* Max Transfer-Buffer-Length */
  struct hc_device *p_hcd;        /* The Host-Controller of the USB-Device */

  struct list_head free_td_list;  /* Intern List of free TDs */
  struct list_head active_td_list;/* Intern List of active TDs */

  struct uhc_device *p_uhcd;      /* Pointer to the Universal Host-Controller */
  __u8 get_module;
  // save
  int pipe_save;

  __u16 status;                   /* URB-state IDLE,PROGRESS,...*/
  __u16 urb_wait_flags;           /* Callback or Wait */

  int get_time_of_first_bit;
  int compl_tds_at_first_irq;
  RTIME time_at_first_irq;
  unsigned int byte_at_first_irq;

};

struct uhc_irq{
  RT_INTR rt_intr;
  int irq;
  struct list_head irq_list;
};

struct rt_urb_priv{
  struct rt_urbpriv *p_purb;
  struct uhc_device *p_uhcd;
};

#endif
