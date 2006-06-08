/***************************************************************************
 *   Copyright (C) 2005 by Jörg Langenberg                                 *
 *   joerg.langenberg@gmx.net                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful, *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc., *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "rt_usb_core.h"
#include "rt_usb_debug.h"
#include "rt_usb_hub.h"
#include <core/usb4rt_config.h>

#define DRIVER_VERSION  USB4RT_PACKAGE_VERSION
#define DRIVER_AUTHOR "Joerg Langenberg - joerg.langenberg@gmx.net"
#define DRIVER_DESC "Realtime USB-Core Module"

#define MAX_CONTROLLER    16

struct usb_device usb_dev[MAX_USB_DEV];
struct list_head usb_ctrl_list;
struct list_head hub_list;

__u8 next_usbdev = 1;
__u8 usb_anz_ctrl = 0;

unsigned long ctrl_mask = 0;
unsigned int alloc_bytes = 0;

struct rt_urb *p_ctrl_08_urb;
struct rt_urb *p_ctrl_16_urb;
struct rt_urb *p_ctrl_32_urb;
struct rt_urb *p_ctrl_64_urb;

int m_buff_size  = 512;

/* rt_usb_debug */
extern void dump_hub_descriptor(struct usb_device *p_usbdev);

/* rt_usb_hub */
extern int hub_get_descriptor(struct rt_urb *p_urb, hub_descriptor_t *p_desc);
extern int hub_set_port_feature(struct rt_urb *p_urb, __u16 port_nr,
                                __u8 feature);
extern struct usb_device *usb_poll_hub_port(struct rt_urb *p_urb,
                                            __u8 hub_port_nr);

int nrt_intern_usb_control_msg(struct rt_urb *p_urb, __u8 endpoint,
                               __u8 request_type, __u8 request, __u16 wValue,
                               __u16 wIndex, __u16 wLength, void *data);

int nrt_usb_register_urb(struct rt_urb *p_urb);
int nrt_usb_unregister_urb(struct rt_urb *p_urb);

/******************************************
 * RT-CORE Host-Controller Help-Functions *
 ******************************************/

/**
 * Ermittelt eine freie Host-Controller-Nummer.
 * Die belegten Nummern sind in der Variablen ctrl_mask gespeichert.
 * @return Nummer des naechsten freien Host-Controllers
 */
static __u8 get_free_hcd_nr(void)
{
    int i = 0;
    while ((ctrl_mask & (1 << i)) && i < MAX_CONTROLLER)
    {
        i++;
    }
    ctrl_mask |= (1 << i);
    DBG("RT-USBCORE: Get free HCD %d. Mask: 0x%08lx \n", i, ctrl_mask);
    return i;
}

/**
 * Gibt eine Host-Controller-Nummer wieder frei.
 * Das Bit wird in der Variablen ctrl_mask geloescht.
 * @param p_hcd Zeiger auf ein struct hc_device.
 * @return -
 */
static void put_free_hcd_nr(struct hc_device *p_hcd)
{
    if (!p_hcd)
        return;

    ctrl_mask &= ~(1 << p_hcd->hcd_nr);
    DBG("RT-USBCORE: Delete HCD %d. Mask: 0x%08lx \n",
        p_hcd->hcd_nr, ctrl_mask);
    return;
}

/**************************
 * RT-CORE Help-Functions *
 **************************/

/**
 * Sucht nach an einem Host-Controller angeschlossenen Geraeten.
 * Da die Geraete waehrend des Ladens der Host-Controller-Driver (HCD) gesucht
 * werden, muss der HCD dem Core mitteilen, wann die Suche nach Geraeten
 * beginnen kann. Zuvor muessen vom HCD alle notwendigen Vorbereitungen
 * getroffen sein. Ein am Host-Controller ermittelter HUB wird zusaetzlich
 * in einer HUB-Liste eingefuegt. Diese Hubs werden ebenfalls nach
 * angeschlossenen Geraeten untersucht.
 * @param p_hcd Zeiger auf ein struct hc_device.
 * @return -
 */
void nrt_usb_search_devices(struct hc_device *p_hcd)
{
    struct usb_device *p_new_usbdev;
    struct hub_device *p_hub;
    __u8 i;
    __u8 hub_port_nr;
    struct rt_urb *p_urb;
    struct hub_device *p_hub_new;
    struct list_head *p_list;


    if (!p_hcd ||
        !p_hcd->p_hcd_fkt ||
        !p_hcd->p_hcd_fkt->nrt_hcd_poll_root_hub_port)
    {
        return;
    }

    DBG_MSG1(p_hcd, " Searching for USB-Devices\n");

    for (i = 0; i < p_hcd->rh_numports; i++)
    {
        DBG_MSG1(p_hcd, "%02d: Root-Hub portscan start \n", i);
        p_new_usbdev = p_hcd->p_hcd_fkt->nrt_hcd_poll_root_hub_port(p_hcd, i);
        if (!p_new_usbdev)
        {
            DBG_MSG1(p_hcd, "%02d: No Device found\n", i);
            continue; // next Port
        }
        else
        {
            DBG_MSG1(p_hcd, "%02d: Device found ... ", i);
            if (p_new_usbdev->class == 0x09 && p_new_usbdev->p_hub_desc)
            {
                DBG("-> New HUB\n");
                p_hub = kmalloc(sizeof(struct hub_device), GFP_KERNEL);
                if (!p_hub)
                {
                    ERR_MSG1(p_hcd, " No Memory for HUB-Device");
                }
                else
                {
                    alloc_bytes += sizeof(struct hub_device);
                    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                        alloc_bytes, "+", sizeof(struct hub_device));
                    p_hub->p_usbdev = p_new_usbdev;
                    p_hub->configured = 0;
                    DBG_MSG2(p_hcd, p_new_usbdev, " Add new HUB to HUB-List \n");
                    list_add_tail(&p_hub->hub_list, &hub_list);
                }
            }
            else
            {
                DBG("-> No HUB\n");
            }
        }
        DBG_MSG1(p_hcd, "%02d: Root-Hub portscan complete\n", i);
    }

    if (list_empty(&hub_list))
        return;

    // now search for devices on the new found hubs
    p_urb = NULL;
    p_list = hub_list.next;

    while (p_list != &hub_list)
    {
        p_hub = list_entry(p_list, struct hub_device, hub_list);

    if (p_hub->configured)
    {
        DBG_MSG2(p_hcd, p_hub->p_usbdev, " HUB is configured\n");
    }
    else
    {
        DBG_MSG2(p_hcd, p_hub->p_usbdev,
                 " Search for Devices connected on this HUB \n");
        switch (p_hub->p_usbdev->epmaxpacketin[0])
        {
            case 8:
                p_urb = p_ctrl_08_urb;
                p_urb->p_usbdev = p_hub->p_usbdev;
                p_urb->p_hcd = p_hub->p_usbdev->p_hcd;
                nrt_usb_register_urb(p_urb);
                INFO_MSG2(p_hcd, p_hub->p_usbdev,
                          " Switch to URB with 8 Byte Packet-Size \n");
                break;
            case 16:
                p_urb = p_ctrl_16_urb;
                p_urb->p_usbdev = p_hub->p_usbdev;
                p_urb->p_hcd = p_hub->p_usbdev->p_hcd;
                nrt_usb_register_urb(p_urb);
                INFO_MSG2(p_hcd, p_hub->p_usbdev,
                          " Switch to URB with 16 Byte Packet-Size \n");
                break;
            case 32:
                p_urb = p_ctrl_32_urb;
                p_urb->p_usbdev = p_hub->p_usbdev;
                p_urb->p_hcd = p_hub->p_usbdev->p_hcd;
                nrt_usb_register_urb(p_urb);
                INFO_MSG2(p_hcd, p_hub->p_usbdev,
                          " Switch to URB with 32 Byte Packet-Size \n");
                break;
            case 64:
                p_urb = p_ctrl_64_urb;
                p_urb->p_usbdev = p_hub->p_usbdev;
                p_urb->p_hcd = p_hub->p_usbdev->p_hcd;
                nrt_usb_register_urb(p_urb);
                INFO_MSG2(p_hcd, p_hub->p_usbdev,
                          " Switch to URB with 64 Byte Packet-Size \n");
                break;
            default:
                  return;
        }

        hub_port_nr = 0;
        for (hub_port_nr=1;
             hub_port_nr <= p_hub->p_usbdev->p_hub_desc->bNbrPorts;
             hub_port_nr++)
        {
            p_new_usbdev = usb_poll_hub_port(p_urb, hub_port_nr);
            if (!p_new_usbdev)
            {
                DBG_MSG2(p_hcd, p_hub->p_usbdev, "%02d: No Device on HUB-Port found\n",
                         hub_port_nr);
                continue; // next Port
            }
            else
            {
                DBG_MSG2(p_hcd, p_new_usbdev, "%02d: Device on HUB-Port found ... ",
                         hub_port_nr);
                if (p_new_usbdev->class == 0x09 && p_new_usbdev->p_hub_desc)
                {
                    DBG("-> New HUB (%d Ports)\n",
                        p_new_usbdev->p_hub_desc->bNbrPorts);
                    p_hub_new = kmalloc(sizeof(struct hub_device), GFP_KERNEL);
                    if (!p_hub_new)
                    {
                        ERR_MSG1(p_hcd, " No Memory for HUB-Device");
                    }
                    else
                    {
                        alloc_bytes += sizeof(struct hub_device);
                        DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                            alloc_bytes, "+", sizeof(struct hub_device));
                        p_hub_new->p_usbdev = p_new_usbdev;
                        p_hub_new->configured = 0;
                        DBG_MSG2(p_hcd, p_new_usbdev, " Add new HUB to HUB-List \n");
                        list_add_tail(&p_hub_new->hub_list, &hub_list);
                    }
                }
                else
                {
                    DBG("-> No HUB\n");
                }
            }
            DBG_MSG2(p_hcd, p_hub->p_usbdev, "%02d: HUB-Portscan complete\n",
                     hub_port_nr);
        }
        nrt_usb_unregister_urb(p_urb);
        p_hub->configured = 1;
    }
    p_list = p_list->next;
  }
  return;
}

/**
 * Ermittelt die Sprache(n) der String-Deskriptoren eines USB-Geraetes.
 * @param p_urb Zeiger auf einen RT-URB
 * @param len Laenge des Puffers im Byte
 * @param p_buffer Zeiger auf einen Puffer
 * @return 0, wenn ein Fehler auftrat
 * @return !0, 1.Language ID
 */
static __u16 get_language(struct rt_urb *p_urb, __u16 len, __u8 *p_buffer)
{
    int ret;
    __u16 lang  = 0x0000;
    __u8 length = 0x00;
    __u8 desc_t = 0x00;
    __u32 desc  = 0;

    if (len < 0x02)
        return 0x0000; // buffer too small

    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_STRING << 8 | 0x00, // language
                                     0x0000,
                                     0x0002,
                                     p_buffer);

    length = p_buffer[0];

    if ((ret < 0x0002) || (length < 0x04))
        return 0x0000; // error while sending message or no language

    length = (length > len) ? len : length;
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " Get Language-Deskriptor (%d Byte)\n",
             length);

    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_STRING << 8 | 0x00, // Language
                                     0x0000,
                                     length,
                                     p_buffer);

    if (ret < length)
        return 0x0000; // error while sending message

    desc   = le32_to_cpu(*(__u32 *)p_buffer); // Extract the first 32 Bit
    lang   = (desc & 0xffff0000) >> 16;
    desc_t = (desc & 0x0000ff00) >> 8;
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " Language Descriptor : "
             "desc = 0x%04x, lang = 0x%04x\n", desc_t, lang);
    return lang;
}

/**
 * List einen String-Deskriptor aus einem USB-Geraet.
 * @param p_urb Zeiger auf einen RT-URB
 * @param string_id String-Deskriptor-ID
 * @param lang Language-ID
 * @param len Laenge des Puffers in Byte
 * @param p_buffer Zeiger auf einen Puffer
 * @return Laenge des String-Deskriptors
 * @return 0, wenn Fehler auftrat
 */
static __u16 get_string(struct rt_urb *p_urb, __u8 string_id, __u16 lang,
                        __u16 len, __u8 *p_buffer)
{
    int ret;
    int i, j;
    int length = 0;

    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_STRING << 8 | string_id,
                                     lang,
                                     0x0002,
                                     p_buffer);

    if (ret < 0x0002)
        return 0;

    length = (len > p_buffer[0]) ? p_buffer[0] : len;

    if (length <= 0x04)
    {
        strcpy(p_buffer, "USB\0");
        return 3;
    }

    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
             " Get String-Deskriptor 0x%02d (%d Byte)\n", string_id, length);

    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_STRING << 8 | string_id,
                                     lang,
                                     length - 1,
                                     p_buffer);

    if (ret < length-1)
    {
        return 0;
    }
    else // de-unicode it!
    {
        for (i=0, j=2; j < length; i++, j+= 2)
        {
            p_buffer[i] = p_buffer[j];
        }
        p_buffer[i]=0;
        length = length >> 1; // /2
    }
    return(length);
}

/**
 * Loescht das Feature USB_ENDPOINT_HALT eines USB-Geraetes.
 * @param p_urb Zeiger auf einen RT-URB
 * @return < 0, wenn ein Fehler auftrat
 * @return 0, wenn erfolgreich
 */
int nrt_usb_clear_stall(struct rt_urb *p_urb)
{
    int ret;

    if (!p_urb->p_hcd || !p_urb->p_usbdev)
        return -ENODEV;

    ret = nrt_intern_usb_control_msg(p_urb,
                                     usb_pipeendpoint(p_urb->pipe),
                                     USB_DIR_OUT | USB_TYPE_STANDARD |
                                     USB_RECIP_ENDPOINT,
                                     USB_REQ_CLEAR_FEATURE,
                                     USB_ENDPOINT_HALT,
                                     usb_pipeendpoint(p_urb->pipe),
                                     0x0000,
                                     NULL);

    if (ret < 0)
        return ret;

    usb_set_data_toggle(p_urb->p_usbdev, p_urb->pipe, 0);
    return 0;
}

/*************************************
 * RT-CORE USB-Device Help-Functions *
 *************************************/

/**
 * Initialisiert ein USB-Geraet.
 * Der Speicherbereich wird mit Nullen berschrieben und einigen Variablen
 * werden Initialwerte zugewiesen.
 * @param p_usbdev Zeiger auf ein USB-Device (struct usb_device)
 */
static void usb_init_device(struct usb_device *p_usbdev)
{
    int i = p_usbdev->address;
    memset(p_usbdev, 0, sizeof(struct usb_device));
    p_usbdev->address = i;

    p_usbdev->state       = USB_STATE_NOTATTACHED;
    p_usbdev->speed       = USB_SPEED_UNKNOWN;

    p_usbdev->p_hcd       = NULL;

    p_usbdev->lock        = SPIN_LOCK_UNLOCKED;

    // ENDPOINT 0
    p_usbdev->epmaxpacketin[0]  = 8;
    p_usbdev->epmaxpacketout[0] = 8;
    p_usbdev->ctrl_mask[0] = 0x0001;  // IN- EP 0 is Ctrl-Endpoint
    p_usbdev->ctrl_mask[1] = 0x0001;  // OUT-EP 0 is Ctrl-Endpoint
}

/**
 * Loescht und initialisiert ein USB-Geraet.
 * Vor dem Initialisieren des Geraetes werden hier noch Speicherbereiche wieder
 * freigegeben.
 * @param p_usbdev Zeiger auf ein USB-Device (struct usb_device)
 */
static void usb_clear_device(struct usb_device *p_usbdev)
{
    int len = 0;
    struct hc_device *p_hcd = NULL;

    if (!p_usbdev)
        return;

    p_hcd = p_usbdev->p_hcd;
    if (p_hcd)
    {
        if (p_usbdev->p_hub_desc)
        {
            DBG_MSG2(p_hcd, p_usbdev, " Free Hub-Descriptor @ 0x%p\n",
                     p_usbdev->p_hub_desc);
            kfree(p_usbdev->p_hub_desc);
            alloc_bytes -= sizeof(hub_descriptor_t);
            DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                alloc_bytes, "-", sizeof(hub_descriptor_t));
        }
        if (p_usbdev->p_cfg_desc)
        {
            len = p_usbdev->p_cfg_desc->wTotalLength;
            DBG_MSG2(p_hcd, p_usbdev, " Free Configuration-Descriptor @ 0x%p\n",
                     p_usbdev->p_cfg_desc);
            kfree(p_usbdev->p_cfg_desc);
            alloc_bytes -= len;
            DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                alloc_bytes, "-", len);
        }
        if (p_usbdev->p_dev_desc)
        {
            len = p_usbdev->p_dev_desc->bLength;
            DBG_MSG2(p_hcd, p_usbdev, " Free Device-Descriptor @ 0x%p\n",
                     p_usbdev->p_dev_desc);
            kfree(p_usbdev->p_dev_desc);
            alloc_bytes -= len;
            DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                alloc_bytes, "-", len);
        }
    }
    usb_init_device(p_usbdev);
}

/**
 * Sucht eine freie USB-Device-Datenstruktur.
 * @return Zeiger auf ein USB-Device (struct usb_device)
 */
static struct usb_device *usb_get_free_device(void)
{
    __u8 free_nr = 1;

    while (free_nr < MAX_USB_DEV && usb_dev[free_nr].p_hcd != NULL)
    {
        free_nr++;
    }

    if (free_nr == MAX_USB_DEV)
        return NULL;

    usb_clear_device(&usb_dev[free_nr]);
    return &usb_dev[free_nr];
}

/**
 * Blockiert ein USB-Device.
 * @param p_usbdev Zeiger auf das zu blockierende USB-Device
 * @return Zeiger auf das blockierte USB-Device (struct usb_device)
 * @return NULL, wenn der Zeiger ungltig oder das Geraet schon blockiert ist.
 */
static struct usb_device *usb_lock_device(struct usb_device *p_usbdev)
{
    unsigned long flags;
    struct usb_device *p_dev = NULL;

    if (!p_usbdev)
        return NULL;

    rthal_spin_lock_irqsave(&p_usbdev->lock, flags);
    if (!p_usbdev->in_use)
    {
        p_usbdev->in_use = 1;
        p_dev = p_usbdev;
    }
    rthal_spin_unlock_irqrestore(&p_usbdev->lock, flags);
    return p_dev;
}

/**
 * Gibt ein blockiertes USB-Device wieder frei.
 * @param p_usbdev Zeiger auf das freizugebene USB-Device
 */
static void usb_unlock_device(struct usb_device *p_usbdev)
{
  unsigned long flags;

    if (!p_usbdev)
        return;

    rthal_spin_lock_irqsave(&p_usbdev->lock, flags);
    p_usbdev->in_use = 0;
    rthal_spin_unlock_irqrestore(&p_usbdev->lock, flags);

    return;
}

/**
 * Untersucht ein USB-Device auf Transferart, Packetsize, etc.
 * @param p_usbdev Zeiger auf das zu untersuchende USB-Device
 */
void usb_check_device(struct usb_device *p_usbdev)
{
    struct usb_interface_descriptor *p_ifd;
    struct usb_endpoint_descriptor  *p_epd;
    struct usb_descriptor_header    *p_head;
    struct usb_config_descriptor    *p_desc;
/*
    hid_descriptor_t                *p_hid;
*/

    int bytes;
    void *buffer;
    __u16 epa;
    __u8 out;

    if (!p_usbdev || !p_usbdev->p_hcd)
        return;

    p_desc = p_usbdev->p_cfg_desc;
    if (!p_desc)
    {
        ERR_MSG2(p_usbdev->p_hcd, p_usbdev,
                 " No valid pointer to struct usb_config_descriptor !!!\n");
       return;
    }

    bytes = p_desc->wTotalLength;
    buffer = (void *)(p_desc);  // first Interface

    while (bytes > 0)
    {
        p_head = (struct usb_descriptor_header *)buffer;
        switch(p_head->bDescriptorType)
        {

        case USB_DT_CONFIG:
        {
            p_desc = (struct usb_config_descriptor *)buffer;
            buffer += p_desc->bLength;
            bytes -= p_desc->bLength;
            //dump_configuration_descriptor(addr);
            break;
        }

        case USB_DT_INTERFACE:
        {
            p_ifd = (struct usb_interface_descriptor *)buffer;
            buffer += p_ifd->bLength;
            bytes -= p_ifd->bLength;
            //dump_interface_descriptor(addr, p_ifd);
            break;
        }

        case USB_DT_ENDPOINT:
        {
            p_epd = (struct usb_endpoint_descriptor *)buffer;
            epa = p_epd->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
            out = p_epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK ? 0 : 1 ;
            buffer += p_epd->bLength;
            bytes -= p_epd->bLength;
            //dump_endpoint_descriptor(addr, p_epd);
            if (out)
                p_usbdev->epmaxpacketout[epa] = le16_to_cpu(p_epd->wMaxPacketSize);
            else
                p_usbdev->epmaxpacketin[epa] = le16_to_cpu(p_epd->wMaxPacketSize);

            switch (p_epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
            {
                case USB_ENDPOINT_XFER_CONTROL:
                //DBG("USB-DEVICE[%d]: Ctrl-%s Endpoint found: 0x%02x (%d Byte)\n", addr, out ? "OUT":"IN", epa, p_epd->wMaxPacketSize);
                if (out)
                    p_usbdev->ctrl_mask[1] |= (1 << epa);
                else
                    p_usbdev->ctrl_mask[0] |= (1 << epa);

                break;
              case USB_ENDPOINT_XFER_BULK:
                //DBG("USB-DEVICE[%d]: Bulk-%s Endpoint found: 0x%02x (%d Byte)\n", addr, out ? "OUT":"IN", epa, p_epd->wMaxPacketSize);
                if (out)
                    p_usbdev->bulk_mask[1] |= (1 << epa);
                else
                    p_usbdev->bulk_mask[0] |= (1 << epa);

                break;
              case USB_ENDPOINT_XFER_INT:
                //DBG("USB-DEVICE[%d]: Interrupt-%s Endpoint found: 0x%02x (%d Byte)\n", addr, out ? "OUT":"IN", epa, p_epd->wMaxPacketSize);
                if (out)
                    p_usbdev->int_mask[1] |= (1 << epa);
                else
                    p_usbdev->int_mask[0] |= (1 << epa);

                break;
              case USB_ENDPOINT_XFER_ISOC:
                //DBG("USB-DEVICE[%d]: Iso-%s Endpoint found: 0x%02x (%d Byte)\n", addr, out ? "OUT":"IN", epa, p_epd->wMaxPacketSize);
                if (out)
                    p_usbdev->iso_mask[1] |= (1 << epa);
                else
                    p_usbdev->iso_mask[0] |= (1 << epa);

                break;
            }
            break;
        }

/*
        case USB_DT_HID: {
            p_hid = (hid_descriptor_t *)buffer;
            buffer += p_hid->bLength;
            bytes -= p_hid->bLength;
            //dump_hid_descriptor(addr, p_hid);
            break;
        }
*/

            default:
            {
                buffer += p_head->bLength;
                bytes -= p_head->bLength;
            }
        }
    }
    DBG_MSG2(p_usbdev->p_hcd, p_usbdev, " Ctrl-Mask: IN: 0x%04x - OUT: 0x%04x\n",
             p_usbdev->ctrl_mask[0], p_usbdev->ctrl_mask[1]);
    DBG_MSG2(p_usbdev->p_hcd, p_usbdev, " Bulk-Mask: IN: 0x%04x - OUT: 0x%04x\n",
             p_usbdev->bulk_mask[0], p_usbdev->bulk_mask[1]);
    DBG_MSG2(p_usbdev->p_hcd, p_usbdev, " Int- Mask: IN: 0x%04x - OUT: 0x%04x\n",
             p_usbdev->int_mask[0], p_usbdev->int_mask[1]);
    DBG_MSG2(p_usbdev->p_hcd, p_usbdev, " Iso- Mask: IN: 0x%04x - OUT: 0x%04x\n",
             p_usbdev->iso_mask[0], p_usbdev->iso_mask[1]);
  return;
}

/**
 * This function configures a new found USB device. The new USB device gets a
 * unique address. All descriptore are read out and were saved.
 *
 * @param p_hcd Pointer to the host controller the device was found on.
 * @param rh_port_nr Root-hub port number (used for debugging messages)
 * @param lowspeed Is unequal to zero if it is a lowspee device.
 *
 * @return Pointer to the new USB device (struct usb_device), NULL if an error
 * occurs
 */
struct usb_device *nrt_usb_config_dev(struct hc_device *p_hcd, __u8 rh_port_nr,
                                      unsigned int lowspeed)
{
    struct usb_device_descriptor    *p_dev_desc;
    struct usb_config_descriptor    *p_conf_desc;
    struct usb_interface_descriptor *p_if_desc;
    struct usb_endpoint_descriptor  *p_ep_desc;
    struct usb_device               *p_usbdev;
    struct rt_urb                   *p_urb = NULL;

    __u16 lang      = 0x0000;
    __u16 ret_len   = 0x0000;
    __u8 language   = 0;
    __u16 len       = 0;

    int ret         = 0;
    int string_len  = 255;

    unsigned char string[string_len];
    unsigned char buffer[m_buff_size];

    DBG_MSG2(p_hcd, &usb_dev[0], " LOCK USB-Device[0]\n");
    if (!usb_lock_device(&usb_dev[0]))
    {
        ERR("RT-USBCORE: [ERROR] Can't get lock for USB-Device[0] \n");
        return NULL;
    }

    usb_dev[0].epmaxpacketin[0]  = 8;
    usb_dev[0].epmaxpacketout[0] = 8;
    usb_dev[0].rh_port = rh_port_nr;
    usb_dev[0].p_hcd = p_hcd;

    p_urb = p_ctrl_08_urb;
    p_urb->p_hcd = p_hcd;
    p_urb->p_usbdev = &usb_dev[0];

    PRNT("RT-USBCORE: Registering CTRL-URB (8 Byte) @ Host-Controller %d \n",
         p_hcd->hcd_nr);

    ret = nrt_usb_register_urb(p_urb);
    if (ret)
    {
        ERR("RT-USBCORE: [ERROR] while registering CTRL-URB (8 Byte), "
            "code = %d\n", ret);
        usb_unlock_device(&usb_dev[0]);
        return NULL;
    }

    if (lowspeed)
        usb_dev[0].speed =  USB_SPEED_LOW;
    else
        usb_dev[0].speed =  USB_SPEED_FULL;

    DBG_MSG2(p_hcd, &usb_dev[0], " New USB device found\n");

    p_dev_desc = (struct usb_device_descriptor *) buffer;

    len = 0x0008;
    DBG_MSG2(p_hcd, &usb_dev[0], " Get Device-Descriptor (first %d Byte)\n", len);
    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_DEVICE << 8 | 0x00, // index
                                     0x0000, // no language ID
                                     len, // first 8 byte
                                     p_dev_desc);

    if (ret < len)
    {
        ERR_MSG2(p_hcd, &usb_dev[0], " Received / Sent only %d / %d Byte \n",
                 ret, len);
        nrt_usb_unregister_urb(p_urb);
        usb_unlock_device(&usb_dev[0]);
        return NULL;
    }

    DBG_MSG2(p_hcd, &usb_dev[0], " Device supports packet size %d\n",
             p_dev_desc->bMaxPacketSize0);

    p_usbdev = usb_get_free_device();
    if (!p_usbdev)
    {
        ERR_MSG1(p_hcd, " MAX %d USB-Devices\n", MAX_USB_DEV);
        nrt_usb_unregister_urb(p_urb);
        usb_unlock_device(&usb_dev[0]);
        return NULL;
    }

    INFO_MSG2(p_hcd, &usb_dev[0], " Setting Address %d\n", p_usbdev->address);
    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00, // endpoint
                                     0x00,
                                     USB_REQ_SET_ADDRESS,
                                     0x0000 | p_usbdev->address,
                                     0x0000,
                                     0x0000,
                                     NULL);

    DBG_MSG2(p_hcd, &usb_dev[0], " UNLOCK USB-Device[0]\n");
    usb_unlock_device(&usb_dev[0]);

    if (ret < 0)
    {
        ERR_MSG2(p_hcd, p_usbdev, " Setting Address FAILED \n");
        nrt_usb_unregister_urb(p_urb);
        return NULL;
    }

    DBG_MSG2(p_hcd, p_usbdev, " LOCK USB-Device[%d]\n", p_usbdev->address);
    if (!usb_lock_device(p_usbdev))
    {
        DBG_MSG2(p_hcd, p_usbdev, " Can't get lock for USB-Device[%d]\n",
                 p_usbdev->address);
        nrt_usb_unregister_urb(p_urb);
        return NULL;
    }

    p_usbdev->speed             = usb_dev[0].speed;
    p_usbdev->rh_port           = rh_port_nr;
    p_usbdev->p_hcd             = p_hcd;
    p_usbdev->epmaxpacketin[0]  = p_dev_desc->bMaxPacketSize0;
    p_usbdev->epmaxpacketout[0] = p_dev_desc->bMaxPacketSize0;

    switch (p_usbdev->epmaxpacketin[0])
    {
        case 8:
            p_urb->p_usbdev = p_usbdev;
            break;
        case 16:
            nrt_usb_unregister_urb(p_urb);
            p_urb = p_ctrl_16_urb;
            p_urb->p_usbdev = p_usbdev;
            p_urb->p_hcd = p_hcd;
            INFO_MSG2(p_hcd, p_usbdev, " Switch to URB with 16 Byte Packet-Size \n");
            nrt_usb_register_urb(p_urb);
            break;
        case 32:
            nrt_usb_unregister_urb(p_urb);
            p_urb = p_ctrl_32_urb;
            p_urb->p_usbdev = p_usbdev;
            p_urb->p_hcd = p_hcd;
            INFO_MSG2(p_hcd, p_usbdev, " Switch to URB with 32 Byte Packet-Size \n");
            nrt_usb_register_urb(p_urb);
            break;
        case 64:
            nrt_usb_unregister_urb(p_urb);
            p_urb = p_ctrl_64_urb;
            p_urb->p_usbdev = p_usbdev;
            p_urb->p_hcd = p_hcd;
            INFO_MSG2(p_hcd, p_usbdev, " Switch to URB with 64 Byte Packet-Size \n");
            nrt_usb_register_urb(p_urb);
            break;
        default:
            nrt_usb_unregister_urb(p_urb);
            usb_clear_device(p_usbdev);
            return 0;
    }

    mdelay(10); // let the SET_ADDRESS settle

    len = p_dev_desc->bLength;

    DBG_MSG2(p_hcd, p_usbdev, " Get Device-Descriptor (%d Byte)\n", len);
    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_DEVICE << 8 | 0x00, // index
                                     0x0000, // no language ID
                                     len,
                                     buffer);

    if (ret < len)
    {
        ERR_MSG2(p_hcd, p_usbdev, " Received / Sent only %d / %d Byte \n", ret, len);
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }

    // save device descriptor
    p_usbdev->p_dev_desc = (struct usb_device_descriptor *)kmalloc(len, GFP_KERNEL);
    if (!p_usbdev->p_dev_desc)
    {
        ERR_MSG2(p_hcd, p_usbdev, " No Memory for Device-Descriptor \n");
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }

    alloc_bytes += len;
    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n", alloc_bytes, "+", len);

    memcpy(p_usbdev->p_dev_desc, p_dev_desc, len);

    dump_device_descriptor(p_usbdev, p_usbdev->p_dev_desc);

    p_usbdev->vendor  = p_dev_desc->idVendor;
    p_usbdev->product = p_dev_desc->idProduct;

    if (p_dev_desc->iManufacturer ||
        p_dev_desc->iProduct ||
        p_dev_desc->iSerialNumber)
    {
        language = 1;
    }

    p_conf_desc = (struct usb_config_descriptor *)
                  (buffer + sizeof(struct usb_device_descriptor));

    len = 0x0004;
    DBG_MSG2(p_hcd, p_usbdev, " Get Configuration-Descriptor (first %d Byte)\n",
             len);
    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_CONFIG << 8 | 0x00,
                                     0x0000,
                                     len,
                                     p_conf_desc);
    if (ret < len)
    {
        ERR_MSG2(p_hcd, p_usbdev, " Received / Sent only %d / %d Byte \n", ret, len);
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }

    len = p_conf_desc->wTotalLength;
    DBG_MSG2(p_hcd, p_usbdev, " Get Configuration-Descriptor (%d Byte)\n", len);
    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_IN | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_GET_DESCRIPTOR,
                                     USB_DT_CONFIG << 8 | 0x00, // Index
                                     0x0000,
                                     len,
                                     p_conf_desc);
    if (ret < len)
    {
        ERR_MSG2(p_hcd, p_usbdev, " Received / Sent only %d / %d Byte \n",
                 ret, len);
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }

    // save configuration descriptor
    p_usbdev->p_cfg_desc = (struct usb_config_descriptor *)kmalloc(len, GFP_KERNEL);
    if (!p_usbdev->p_cfg_desc)
    {
        ERR_MSG2(p_hcd, p_usbdev, " No Memory for Configuration-Descriptor \n");
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }

    alloc_bytes += len;
    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n", alloc_bytes, "+", len);

    memcpy(p_usbdev->p_cfg_desc, p_conf_desc, len);

    usb_check_device(p_usbdev);

    p_if_desc = (struct usb_interface_descriptor *)
                (buffer + sizeof(struct usb_device_descriptor) +
                          p_conf_desc->bLength);
    p_ep_desc = (struct usb_endpoint_descriptor *)
                (buffer + sizeof(struct usb_device_descriptor) +
                          p_conf_desc->bLength +
                          p_if_desc->bLength);

    DBG_MSG2(p_hcd, p_usbdev, " Set Configuration 0x%0x\n",
              p_conf_desc->bConfigurationValue);

    ret = nrt_intern_usb_control_msg(p_urb,
                                     0x00,
                                     USB_DIR_OUT | USB_TYPE_STANDARD |
                                     USB_RECIP_DEVICE,
                                     USB_REQ_SET_CONFIGURATION,
                                     p_conf_desc->bConfigurationValue,
                                     0x0000,
                                     0x0000,
                                     NULL);
    if (ret < 0)
    {
        ERR_MSG2(p_hcd, p_usbdev, " Can't set Configuration \n");
        nrt_usb_unregister_urb(p_urb);
        usb_clear_device(p_usbdev);
        return NULL;
    }
    mdelay(20);

    // determine device class
    if (p_dev_desc->bDeviceClass)
    {
        p_usbdev->class    = p_dev_desc->bDeviceClass;
        p_usbdev->subclass = p_dev_desc->bDeviceSubClass;
        p_usbdev->protocol = p_dev_desc->bDeviceProtocol;
    }
    else
    {
        p_usbdev->class    = p_if_desc->bInterfaceClass;
        p_usbdev->subclass = p_if_desc->bInterfaceSubClass;
        p_usbdev->protocol = p_if_desc->bInterfaceProtocol;
    }

    DBG_MSG2(p_hcd, p_usbdev, " Class %02x, Subclass %02x, Protocol %02x\n",
             p_usbdev->class, p_usbdev->subclass, p_usbdev->protocol);

    lang = 0x0000;
    ret_len  = 0x0000;
    if (language)
    {
        lang = get_language(p_urb, string_len, string);
    }

    if (p_dev_desc->iManufacturer)
    {
        ret_len = get_string(p_urb, p_dev_desc->iManufacturer, lang,
                             string_len, string);
        if (ret_len > 0)
        {
            INFO_MSG2(p_hcd, p_usbdev, " Manufacturer : %s\n", string);
        }
    }

    if (p_dev_desc->iProduct)
    {
        ret_len = get_string(p_urb, p_dev_desc->iProduct, lang, string_len, string);
        if (ret_len > 0)
        {
            INFO_MSG2(p_hcd, p_usbdev, " Product : %s\n", string);
        }
    }

    if (p_dev_desc->iSerialNumber)
    {
        ret_len = get_string(p_urb, p_dev_desc->iSerialNumber, lang, string_len, string);
        if (ret_len > 0)
        {
            INFO_MSG2(p_hcd, p_usbdev, " Serial : %s\n", string);
        }
    }

    switch(p_usbdev->class)
    {
        case 0x09:      // hub
            if (usb_init_hub(p_urb))
            {
                DBG_MSG2(p_hcd, p_usbdev, " Error while init Hub \n");
                nrt_usb_unregister_urb(p_urb);
                usb_clear_device(p_usbdev);
                return NULL;
            }
            break;

        default:
            break;
    }

    nrt_usb_unregister_urb(p_urb);

    DBG_MSG2(p_hcd, p_usbdev, " UNLOCK USB-Device \n");
    usb_unlock_device(p_usbdev);

    INFO_MSG2(p_hcd, p_usbdev, " Device %d configured\n", p_usbdev->address);

    dump_usb_device(p_usbdev);
    list_usb_devices();

    return (p_usbdev);
}

/**************************************
 * RT-CORE message functions          *
 **************************************/

/**
 * Core internal funktion to submit an URB.
 * Abhaengig vom submit_flag blockiert diese Funktion bis die Nachricht
 * uebertragen wurde oder kehrt sofort zurck, nachdem der Host-Controller die Daten
 * empfangen hat.
 * @param p_urb Zeiger auf den zu versendenen URB
 * @param urb_submit_flags Flags, wie URB_CALLBACK, URB_WAIT_SEM und URB_WAIT_BUSY (Core-Intern).
 * @return -ENODEV, wenn im URB ein ungltiger Zeiger auf einen Host-Controller oder ein USB-Device gespeichert ist
 * @return -EINVAL, wenn im URB ein ungltiger Wert gespeichert ist (siehe Debug-Meldungen)
 * @return 0, wenn erfolgreich
 */
static int rt_intern_usb_submit_urb(struct rt_urb *p_urb, __u16 urb_submit_flags)
{
    /*
    Checked before:
    p_urb
    */

    __u8 out      = 0;
    __u8 endpoint = 0;
    __u16 max_pl  = 0;

#ifdef DEBUG
    __u8 device   = 0;
#endif

    if (!p_urb->p_usbdev)
    {
        ERR("[ERROR] %s - Invalid USB-Device in URB 0x%p\n", __FUNCTION__, p_urb);
        return -EINVAL;
    }

    p_urb->p_hcd = p_urb->p_usbdev->p_hcd;

    if (!p_urb->p_hcd ||
        list_empty(&p_urb->p_hcd->usb_ctrl_list) ||
        !p_urb->p_hcd->p_hcd_fkt ||
        !p_urb->p_hcd->p_hcd_fkt->nrt_hcd_register_urb)
    {
        ERR("[ERROR] %s - Invalid Host-Controller-Pointer in URB 0x%p \n",
            __FUNCTION__, p_urb);
        return -ENODEV;
    }

    if (usb_pipedevice(p_urb->pipe) != p_urb->p_usbdev->address)
    {
        ERR("[ERROR] %s - Pipe-Device != USB-Device-Address \n", __FUNCTION__);
        return -EINVAL;
    }

    out      = usb_pipeout(p_urb->pipe) ? 1 : 0;
    endpoint = usb_pipeendpoint(p_urb->pipe);
    max_pl   = usb_maxpacket(p_urb->p_usbdev, p_urb->pipe);

#ifdef DEBUG
    __u8 device   = usb_pipedevice(p_urb->pipe);

    if (p_urb->transfer_flags & URB_SHORT_NOT_OK)
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB_SHORT_NOT_OK \n");

    if (p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB_NO_TRANSFER_DMA_MAP \n");

    if (p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP)
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB_NO_SETUP_DMA_MAP \n");

    if (p_urb->transfer_flags & URB_TIMESTAMP_DATA)
        DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev, " URB_TIMESTAMP_DATA \n");
#endif

    switch (usb_pipetype(p_urb->pipe))
    {
        case USB_ENDPOINT_XFER_CONTROL:
            DBG("URB Type: CTRL, Address: %d, Pipe: %s, Endpoint %d\n",
                p_urb->p_usbdev->address, out ? "OUT" : "IN", endpoint);

            if (!(p_urb->p_usbdev->ctrl_mask[(out?1:0)] & (1 << endpoint)))
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - %s-Endpoint %d is no CTRL \n",
                         __FUNCTION__, out ? "OUT" : "IN", endpoint);
                return -ENODEV;
            }

            // Wenn Control-Transfer, dann muss p_setup_packet gueltig sein
            if (!p_urb->p_setup_packet)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - Invalid Pointer to Control-Request \n",
                         __FUNCTION__);
                return -EINVAL;
            }
            if (p_urb->transfer_flags & URB_NO_SETUP_DMA_MAP)
            {
                if (!p_urb->setup_dma)
                {
                    ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                             " %s - Invalid DMA-Handle for Control-Request \n",
                             __FUNCTION__);
                    return -EINVAL;
                }
                DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         "  URB 0x%p: Setup-Packet 0x%p, DMA : 0x%p\n",
                         p_urb, p_urb->p_setup_packet, (void*)p_urb->setup_dma);
            }
            else
            {
                DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " URB 0x%p: Setup-Packet 0x%p, NOT_DMA_MAPPED\n",
                         p_urb, p_urb->p_setup_packet);
            }
            break;

        case USB_ENDPOINT_XFER_BULK:
            DBG("URB Type: BULK, Address: %d, Pipe: %s, Endpoint %d\n",
                p_urb->p_usbdev->address, out ? "OUT" : "IN", endpoint);

            if (!(p_urb->p_usbdev->bulk_mask[(out?1:0)] & (1 << endpoint)))
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - %s-Endpoint %d is no BULK \n",
                         __FUNCTION__, out ? "OUT" : "IN", endpoint);
                return -ENODEV;
            }
            if (p_urb->p_usbdev->speed == USB_SPEED_LOW)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s -Device is no Full/Highspeed-Device \n",
                         __FUNCTION__);
                return -ENODEV;
            }
            break;

        case USB_ENDPOINT_XFER_INT:
            DBG("URB Type: INT, Address: %d, Pipe: %s, Endpoint %d\n",
                p_urb->p_usbdev->address, out ? "OUT" : "IN", endpoint);

            if (!(p_urb->p_usbdev->int_mask[(out?1:0)] & (1 << endpoint)))
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " URB 0x%p: %s-Endpoint %d is no INT \n",
                         p_urb, out ? "OUT" : "IN", endpoint);
                return -ENODEV;
            }
            break;

        case USB_ENDPOINT_XFER_ISOC:
            DBG("URB Type: ISO, Address: %d, Pipe: %s, Endpoint %d\n",
                p_urb->p_usbdev->address, out ? "OUT" : "IN", endpoint);

            if (!(p_urb->p_usbdev->iso_mask[(out?1:0)] & (1 << endpoint)))
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - %s-Endpoint %d is no ISO \n",
                         __FUNCTION__, out ? "OUT" : "IN", endpoint);
                return -ENODEV;
            }
            if (p_urb->p_usbdev->speed == USB_SPEED_LOW)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - Device is no Full/Highspeed-Device \n",
                         __FUNCTION__);
                return -ENODEV;
            }

          break;
      }

    if (p_urb->max_packet_size != max_pl)
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                 "  %s - Invalid Max-Packet-Size: %d sent, %d allowed \n",
                 __FUNCTION__, p_urb->max_packet_size, max_pl);
        return -EINVAL;
    }
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
             " URB 0x%p: Max Packet-Size %d\n", p_urb, max_pl);

    if (p_urb->transfer_buffer_length < 0 ||
        p_urb->max_buffer_len < 0 ||
        p_urb->transfer_buffer_length > p_urb->max_buffer_len)
    {
        ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                 " %s - Invalid Transfer-Buffer-Length: %d sent, %d Byte allowed\n",
                 __FUNCTION__, p_urb->transfer_buffer_length, p_urb->max_buffer_len);
        return -EINVAL;
    }
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
             " URB 0x%p: Max Transfer-Buffer-Length %d\n",
             p_urb, p_urb->max_buffer_len);
    DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
             " URB 0x%p: Transfer-Buffer-Length %d\n",
             p_urb, p_urb->transfer_buffer_length);

    if (p_urb->transfer_buffer_length)
    {
        if (!p_urb->p_transfer_buffer)
        {
            ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                     " %s - Invalid Pointer to Transfer-Buffer \n",
                     __FUNCTION__);
            return -EINVAL;
        }
        if (p_urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
        {
            if (!p_urb->transfer_dma)
            {
                ERR_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                         " %s - Invalid DMA-Handle for Transfer-Buffer \n",
                         __FUNCTION__);
                return -EINVAL;
            }
            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                     " URB 0x%p: Transfer-Buffer 0x%p, DMA : 0x%p\n",
                     p_urb, p_urb->p_transfer_buffer, (void*)p_urb->transfer_dma);
        }
        else
        {
            DBG_MSG2(p_urb->p_hcd, p_urb->p_usbdev,
                     " URB 0x%p: Transfer-Buffer 0x%p, NOT_DMA_MAPPED\n",
                     p_urb, p_urb->p_transfer_buffer);
        }
    }
    return p_urb->p_hcd->p_hcd_fkt->rt_hcd_submit_urb(p_urb, urb_submit_flags);
}

/**
 * Core internal function to send a control URB.
 *
 * @param p_urb Pointer to the URB.
 * @param endpoint Endpoint number.
 * @param request_type Request type (e.g.  USB_DIR_IN | USB_TYPE_STANDARD |
 * USB_RECIP_DEVICE), 8 Bit
 * @param request Request (z.B. USB_REQ_GET_DESCRIPTOR), 8 Bit
 * @param wValue Wert, 16 Bit
 * @param wIndex Index, 16 Bit
 * @param wLength Length of the transfer buffer, 16 Bit
 * @param data Pointer to the transfer data.
 *
 * @return -ENODEV, Invalid host controller or usb device pointer has been saved
 * in th URB.
 * @return -EINVAL, Pointer to the URB invalid or invalid values inside the URB.
 */
int nrt_intern_usb_control_msg(struct rt_urb *p_urb, __u8 endpoint,
                               __u8 request_type, __u8 request, __u16 wValue,
                               __u16 wIndex, __u16 wLength, void *data)
{
    if (!p_urb)
    {
        ERR("[ERROR] Invalid URB-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    if (!p_urb->p_setup_packet)
    {
        ERR("[ERROR] Invalid Control-Request-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    p_urb->pipe = (request_type & USB_DIR_IN
        ? usb_rcvctrlpipe(p_urb->p_usbdev, endpoint)
        : usb_sndctrlpipe(p_urb->p_usbdev, endpoint));

    p_urb->p_setup_packet->bRequestType = request_type;
    p_urb->p_setup_packet->bRequest     = request;
    p_urb->p_setup_packet->wValue       = cpu_to_le16(wValue);
    p_urb->p_setup_packet->wIndex       = cpu_to_le16(wIndex);
    p_urb->p_setup_packet->wLength      = cpu_to_le16(wLength);

    p_urb->transfer_buffer_length       = wLength;
    p_urb->p_transfer_buffer            = data;

    return rt_intern_usb_submit_urb(p_urb, URB_WAIT_BUSY);
}

/**************************************
 * RT-CORE-API to the Host-Controller *
 **************************************/

/**
 *
 * @param p_hcd Zeiger auf ein struct hc_device (Angefordert durch den HCD)
 */
int nrt_hcd_register_driver(struct hc_device *p_hcd)
{
    PRNT("RT-USBCORE: Register Host-Controller Driver \n");

    if (usb_anz_ctrl >= MAX_CONTROLLER)
    {
        ERR("RT-USBCORE: [ERROR] Max %d USB-Host-Controller !!! \n",
            MAX_CONTROLLER);
        return -1;
    }

    // Checking p_hcd_fkt
    if (!p_hcd ||
        !p_hcd->p_hcd_fkt ||
        !p_hcd->p_hcd_fkt->nrt_hcd_poll_root_hub_port ||
        !p_hcd->p_hcd_fkt->nrt_hcd_register_urb ||
        !p_hcd->p_hcd_fkt->nrt_hcd_unregister_urb ||
        !p_hcd->p_hcd_fkt->rt_hcd_submit_urb)
    {
        ERR("RT-USBCORE: [ERROR] Invalid Host-Controller Functions !!! \n");
        return -2;
    }

    if (p_hcd->type != 0x00 && // UHCI
        p_hcd->type != 0x10 && // OHCI
        p_hcd->type != 0x20)   // EHCI
    {
        ERR("RT-USBCORE: [ERROR] Invalid Host-Controller Type !!! \n");
        return -3;
    }

    p_hcd->p_hcd_fkt->nrt_usb_config_dev     = nrt_usb_config_dev;
    p_hcd->p_hcd_fkt->nrt_usb_search_devices = nrt_usb_search_devices;

    p_hcd->hcd_nr = get_free_hcd_nr();

    INIT_LIST_HEAD(&p_hcd->usb_ctrl_list);
    atomic_set(&p_hcd->use_counter, 0);
    list_add_tail(&p_hcd->usb_ctrl_list, &usb_ctrl_list);

    PRNT("RT-USBCORE: Host-Controller added to USB-Controller-List \n");
    usb_anz_ctrl++;

    return 0;
}

/**
 *
 * @param p_hcd Zeiger auf die Datenstruktur des Host-Controllers, welcher entfernt werden soll.
 */
int nrt_hcd_unregister_driver(struct hc_device *p_hcd)
{
    struct list_head  *p_list;
    struct list_head  *p_next;
    struct hub_device *p_hub;
    int i;

    if (!p_hcd)
        return -EINVAL;

    if (list_empty(&p_hcd->usb_ctrl_list))
    {
        PRNT("RT-USBCORE: [ERROR] Host-Controller Driver 0x%p not registered \n",
             p_hcd);
        return -EINVAL;
    }

    PRNT("RT-USBCORE: Unregister Host-Controller Driver %d \n", p_hcd->hcd_nr);
    list_del_init(&p_hcd->usb_ctrl_list);
    put_free_hcd_nr(p_hcd);

    usb_anz_ctrl--;

    // entferne Hubs
    p_hub = NULL;
    p_list = hub_list.next;

    while (p_list != &hub_list)
    {
        p_next = p_list->next;
        p_hub = list_entry(p_list, struct hub_device, hub_list);
        if (p_hub->p_usbdev->p_hcd == p_hcd)
        {
            list_del_init(&p_hub->hub_list);
            kfree(p_hub);
            alloc_bytes -= sizeof(struct hub_device);
            DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
                alloc_bytes, "-", sizeof(struct hub_device));
        }
        p_list = p_next;
    }

    // loesche USB-Devices
    for (i=0;i<MAX_USB_DEV;i++)
    {
        if (usb_dev[i].p_hcd == p_hcd)
        {
            PRNT("RT-USBCORE: Remove USB-Device %d \n", i);
            usb_clear_device(&usb_dev[i]);
        }
    }
    return -ENODEV;
}

/******************************
 * RT-CORE-API to the Drivers *
 ******************************/

/**
 * Sucht in der Liste der USB-Geraete ein bestimmtes Geraet.
 * Wenn ein zutreffendes Geraet gefunden wurde, wird dieses gleich blockiert
 * und somit vor anderen Zugriffen geschuetzt.
 * @param vendor Vendor-ID des zu suchenden Geraetes.
 * @param product Product-ID des zu suchenden Geraetes.
 * @param p_old_device Zeiger auf ein USB-Geraet, ab dem gesucht werden soll.
 * @return Zeiger auf ein USB-Device, auf das die Beschreibung passt.
 * @return NULL, wenn kein solches Geraet gefunden wurde.
 */
struct usb_device *rt_usb_get_device(__u16 vendor, __u16 product,
                                     struct usb_device *p_old_device)
{
    struct usb_device *p_dev  = NULL;
    struct usb_device *p_list = NULL;
    int i, start = 1;

    if ((vendor  == 0x0000) &&
        (product == 0x0000))
    {
        return NULL;
    }

    if (p_old_device && p_old_device->address < MAX_USB_DEV)
    {
        start = p_old_device->address + 1;
    }

    DBG("RT-USBCORE: Driver is searching for Device with Vendor = 0x%04x, "
        "Product = 0x%04x \n", vendor, product);

    for (i = start; i < MAX_USB_DEV ; i++)
    {
        p_list = &usb_dev[i];

        if ((p_list->vendor  == vendor) &&
            (p_list->product == product))
        {
            p_dev = usb_lock_device(p_list);

            list_usb_devices();

            if (!p_dev)
            {
                DBG("RT-USBCORE: Cannot lock USB-Device[%d] \n",
                    p_list->address);
            }
            else
            {
                DBG("RT-USBCORE: USB-Device[%d] LOCKED \n", p_dev->address);
                return p_dev;
            }
        }
    }
    return NULL;
}

/**
 * Gibt ein zuvor mit der Funktion rt_usb_get_device blockiertes USB-Geraete
 * wieder frei.
 * @param p_usbdev Zeiger auf ein USB-Geraet, das freigegeben werden soll.
 */
void rt_usb_put_device(struct usb_device *p_usbdev)
{
    if (!p_usbdev)
        return;

    DBG("RT-USBCORE: Driver releases USB-Device[%d] \n", p_usbdev->address);
    usb_unlock_device(p_usbdev);
    list_usb_devices();
}

/**
 * Generiert einen URB.
 * @return NULL, wenn kein Speicher mehr vorhanden.
 * @return Zeiger auf den erstellten URB
 */
struct rt_urb *nrt_usb_alloc_urb(void)
{
    struct rt_urb *p_urb = kmalloc(sizeof(struct rt_urb), GFP_KERNEL);
    if (!p_urb)
    {
        ERR("RT-USBCORE: [ERROR] No memory for RT-URB \n");
        return NULL;
    }
    DBG("RT-USBCORE: URB 0x%p created (%d Byte)\n",
        p_urb, sizeof(struct rt_urb));
    memset(p_urb, 0, sizeof(struct rt_urb));

    alloc_bytes += sizeof(struct rt_urb);
    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
        alloc_bytes, "+", sizeof(struct rt_urb));

    p_urb->p_setup_packet = NULL;
    p_urb->setup_dma = 0;

    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_alloc_urb erstellten URB.
 * @param p_urb Zeiger auf den zu loeschenden URB
 */
void nrt_usb_free_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
        return;

    DBG("RT-USBCORE: URB 0x%p deleted\n", p_urb);
    kfree(p_urb);

    alloc_bytes -= sizeof(struct rt_urb);
    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
        alloc_bytes, "-", sizeof(struct rt_urb));

}

/**
 * Generiert einen Control-URB.
 * Im Gegensatz zu der Funktion nrt_usb_alloc_urb wird hier zusaetzlich noch
 * der Speicherbereich fuer den Control-Request angefordern und der Zeiger
 * in p_urb->p_setup_packet abgelegt.
 * @return NULL, wenn kein Speicher mehr vorhanden.
 * @return Zeiger auf den erstellten Control-URB
 */
struct rt_urb *nrt_usb_alloc_ctrl_urb(void)
{
    struct rt_urb           *p_urb  = NULL;
    struct usb_ctrlrequest  *p_ctrl = NULL;

    p_urb = nrt_usb_alloc_urb();
    if (!p_urb)
        return NULL;

    p_ctrl = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
    if (!p_ctrl)
    {
        ERR("RT-USBCORE: [ERROR] No memory for USB-Control-Request \n");
        nrt_usb_free_urb(p_urb);
        return NULL;
    }

    DBG("RT-USBCORE: URB 0x%p: Control-Request created (%d Byte)\n",
        p_urb, sizeof(struct usb_ctrlrequest));
    memset(p_ctrl, 0, sizeof(struct usb_ctrlrequest));

    alloc_bytes += sizeof(struct usb_ctrlrequest);
    DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
        alloc_bytes, "+", sizeof(struct usb_ctrlrequest));

    p_urb->p_setup_packet = p_ctrl;
    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_alloc_ctrl_urb erstellten Control-URB.
 * @param p_urb Zeiger auf den zu loeschenden Control-URB
 */
void nrt_usb_free_ctrl_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
        return;

    if (p_urb->p_setup_packet)
    {
        kfree(p_urb->p_setup_packet);
        DBG("RT-USBCORE: URB 0x%p: Control-Request deleted (%d Byte)\n",
            p_urb, sizeof(struct usb_ctrlrequest));

        alloc_bytes -= sizeof(struct usb_ctrlrequest);
        DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
            alloc_bytes, "-", sizeof(struct usb_ctrlrequest));
    }

    nrt_usb_free_urb(p_urb);
    return;
}

/**
 * Erstellt einen nicht blockierenden Control-URB.
 * Zuerst wird ein URB mit Hilfe der Funktion nrt_usb_alloc_ctrl_urb erstellt.
 * Abschliessend werden alle noetigen Daten im URB gespeichert.
 * @return NULL, wenn kein Speicher mehr vorhanden.
 * @return Zeiger auf den erstellten Control-URB
 */
struct rt_urb *nrt_usb_create_ctrl_callback_urb(rt_usb_complete_t rt_callback_fkt)
{
    struct rt_urb *p_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_urb)
        return NULL;

    p_urb->rt_complete_fkt = rt_callback_fkt;
    p_urb->rt_sem_timeout = 0;

    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_create_ctrl_callback_urb erstellten Control-URB.
 * @param p_urb Zeiger auf den zu loeschenden Control-URB
 */
void nrt_usb_destroy_ctrl_callback_urb(struct rt_urb *p_urb)
{
    nrt_usb_free_ctrl_urb(p_urb);
}

/**
 * Erstellt einen nicht blockierenden URB.
 * Zuerst wird ein URB mit Hilfe der Funktion nrt_usb_alloc_urb erstellt.
 * Abschliessend werden alle noetigen Daten im URB gespeichert.
 * @return NULL, wenn kein Speicher mehr vorhanden.
 * @return Zeiger auf den erstellten URB
 */
struct rt_urb *nrt_usb_create_callback_urb(rt_usb_complete_t rt_callback_fkt)
{
    struct rt_urb *p_urb = nrt_usb_alloc_urb();
    if (!p_urb)
        return NULL;

    p_urb->rt_complete_fkt = rt_callback_fkt;
    p_urb->rt_sem_timeout = 0;
    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_create_callback_urb erstellten URB.
 * @param p_urb Zeiger auf den zu loeschenden URB
 */
void nrt_usb_destroy_callback_urb(struct rt_urb *p_urb)
{
    nrt_usb_free_urb(p_urb);
}

/**
 * Erstellt einen blockierenden Control-URB.
 * Zuerst wird ein URB mit Hilfe der Funktion nrt_usb_alloc_ctrl_urb erstellt.
 * Abschliessend werden alle noetigen Daten im URB gespeichert und die RT-Semaphore
 * bei Xenomai registriert.
 * @return NULL, wenn kein Speicher mehr vorhanden oder Fehler beim Registrieren der RT-Semaphore.
 * @return Zeiger auf den erstellten Control-URB
 */
struct rt_urb *nrt_usb_create_ctrl_semaphore_urb(unsigned char *name,
                                                 int64_t rt_sem_timeout)
{
    struct rt_urb *p_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_urb)
        return NULL;

    p_urb->rt_complete_fkt = NULL;
    p_urb->rt_sem_timeout = rt_sem_timeout;

    DBG("RT-USBCORE: URB 0x%p: Init RT-Semaphore (%s)\n", p_urb, name);
    rtdm_sem_init(&p_urb->rt_sem, 0);

    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_create_ctrl_semaphore_urb erstellten Control-URB.
 * @param p_urb Zeiger auf den zu loeschenden Control-URB
 */
void nrt_usb_destroy_ctrl_semaphore_urb(struct rt_urb *p_urb)
{
    DBG("RT-USBCORE: URB 0x%p: Unregister RT-Semaphore\n", p_urb);
    rtdm_sem_destroy(&p_urb->rt_sem);
    nrt_usb_free_ctrl_urb(p_urb);
}

/**
 * Erstellt einen blockierenden URB.
 * Zuerst wird ein URB mit Hilfe der Funktion nrt_usb_alloc_urb erstellt.
 * Abschliessend werden alle noetigen Daten im URB gespeichert und die
 * RT-Semaphore bei Xenomai registriert.
 * @return NULL, wenn kein Speicher mehr vorhanden oder Fehler beim Registrieren der RT-Semaphore.
 * @return Zeiger auf den erstellten URB
 */
struct rt_urb *nrt_usb_create_semaphore_urb(unsigned char *name,
                                            int64_t rt_sem_timeout)
{
    struct rt_urb *p_urb = nrt_usb_alloc_urb();
    if (!p_urb)
        return NULL;

    p_urb->rt_complete_fkt = NULL;
    p_urb->rt_sem_timeout = rt_sem_timeout;

    DBG("RT-USBCORE: URB 0x%p: Init RT-Semaphore (%s)\n", p_urb, name);
    rtdm_sem_init(&p_urb->rt_sem, 0);

    return p_urb;
}

/**
 * Loescht einen mit der Funktion nrt_usb_destroy_semaphore_urb erstellten URB.
 * @param p_urb Zeiger auf den zu loeschenden URB
 */
void nrt_usb_destroy_semaphore_urb(struct rt_urb *p_urb)
{
    DBG("RT-USBCORE: URB 0x%p: Unregister RT-Semaphore\n", p_urb);
    rtdm_sem_destroy(&p_urb->rt_sem);
    nrt_usb_free_urb(p_urb);
}

/**
 * Uebergibt einen URB an den RT-USB-Core.
 * Die Callback-Relevanten Daten des URBs werden ueberprueft und der URB an die
 * Funktion rt_intern_usb_submit_urb uebergeben. Die Funktion blockiert nicht,
 * sondern kehrt unmittelbar nach der Uebergabe des URBs an den Host-Controller
 * wieder zurueck. Wenn die Nachricht erfolgreich versendet wurde oder ein
 * Fehler auftrat, wird die Callback-Funktion aufgerufen.
 * @param p_urb Zeiger auf den zu versendenen URB
 * @return -EINVAL Ungltiger URB-Zeiger, keine Callback-Funktion angegeben oder Fehler in rt_intern_usb_submit_urb
 * @return 0, wenn erfolgreich
 */
int rt_usb_submit_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
    {
        ERR("[ERROR] Invalid URB-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    if (!p_urb->rt_complete_fkt)
    {
        ERR("[ERROR] Invalid Pointer to Callback-Function: 0x%p \n ", p_urb->rt_complete_fkt);
        return -EINVAL;
    }

    p_urb->core_in_time = rtdm_clock_read();
    return rt_intern_usb_submit_urb(p_urb, URB_CALLBACK);
}

/**
 * Uebergibt einen Control-URB an den RT-USB-Core.
 * Die Callback-Relevanten Daten des URBs werden ueberprft, der Control-Request
 * ausgefuellt und der URB an die Funktion rt_intern_usb_submit_urb uebergeben.
 * Die Funktion blockiert nicht, sondern kehrt unmittelbar nach der Uebergabe
 * des URBs an den Host-Controller wieder zurueck.
 * Wenn die Nachricht erfolgreich versendet wurde oder ein Fehler auftrat,
 * wird die Callback-Funktion aufgerufen.
 * @param p_urb Zeiger auf den zu versendenen Control-URB
 * @param request_type Request-Typ (z.B.  USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE), 8 Bit
 * @param request Request (z.B. USB_REQ_GET_DESCRIPTOR), 8 Bit
 * @param wValue Wert, 16 Bit
 * @param wIndex Index, 16 Bit
 * @param wLength Laenge der Puffer-Daten, 16 Bit
 * @param data Zeiger auf die Daten, die bertragen werden sollen
 * @return -EINVAL, wenn ungltiger URB-Zeiger, ungltiger Zeiger auf ein Control-Request, keine Callback-Funktion angegeben oder Fehler in rt_intern_usb_submit_urb
 * @return 0, wenn erfolgreich
 */
int rt_usb_submit_ctrl_urb(struct rt_urb *p_urb, __u8 request_type,
                           __u8 request, __u16 wValue, __u16 wIndex,
                           __u16 wLength, void *data)
{
    if (!p_urb)
    {
        ERR("[ERROR] Invalid URB-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    if (!p_urb->p_setup_packet)
    {
        ERR("[ERROR] Invalid Pointer to struct usb_ctrlrequest: 0x%p \n ", p_urb->p_setup_packet);
        return -EINVAL;
    }

    if (!p_urb->rt_complete_fkt)
    {
        ERR("[ERROR] Invalid Pointer to Callback-Function: 0x%p \n ", p_urb->rt_complete_fkt);
        return -EINVAL;
    }

    p_urb->core_in_time = rtdm_clock_read();

    p_urb->p_setup_packet->bRequestType = request_type;
    p_urb->p_setup_packet->bRequest     = request;
    p_urb->p_setup_packet->wValue       = cpu_to_le16(wValue);
    p_urb->p_setup_packet->wIndex       = cpu_to_le16(wIndex);
    p_urb->p_setup_packet->wLength      = cpu_to_le16(wLength);
    p_urb->transfer_buffer_length       = wLength;
    p_urb->p_transfer_buffer            = data;

    return rt_intern_usb_submit_urb(p_urb, URB_CALLBACK);
}

/**
 * Uebergibt einen URB an den RT-USB-Core.
 * Der URB wird an die Funktion rt_intern_usb_submit_urb bergeben.
 * Die Funktion BLOCKIERT solange, bis die Nachricht erfolgreich versendet wurde oder ein Fehler auftrat.
 * @param p_urb Zeiger auf den zu versendenen URB
 * @return -EINVAL, wenn ungltiger URB-Zeiger oder Fehler in rt_intern_usb_submit_urb
 * @return 0, wenn erfolgreich
 */
int rt_usb_send_urb_msg(struct rt_urb *p_urb)
{
    int ret;

    if (!p_urb)
    {
        ERR("[ERROR] Invalid URB-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    p_urb->core_in_time = rtdm_clock_read();
    ret = rt_intern_usb_submit_urb(p_urb, URB_WAIT_SEM);
    p_urb->core_exit_time = rtdm_clock_read();

    return ret;
}

/**
 * Uebergibt einen Control-URB an den RT-USB-Core.
 * Der Control-Request ausgefllt und der URB an die Funktion rt_intern_usb_submit_urb bergeben.
 * Die Funktion BLOCKIERT solange, bis die Nachricht erfolgreich versendet wurde oder ein Fehler auftrat.
 * @param p_urb Zeiger auf den zu versendenen Control-URB
 * @param request_type Request-Typ (z.B.  USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE), 8 Bit
 * @param request Request (z.B. USB_REQ_GET_DESCRIPTOR), 8 Bit
 * @param wValue Wert, 16 Bit
 * @param wIndex Index, 16 Bit
 * @param wLength Laenge der Puffer-Daten, 16 Bit
 * @param data Zeiger auf die Daten, die bertragen werden sollen
 * @return -EINVAL, wenn ungltiger Zeiger auf einen URB oder auf einen Control-Request
 * @return 0, wenn erfolgreich
 */
int rt_usb_send_ctrl_msg(struct rt_urb *p_urb, __u8 request_type, __u8 request,
                         __u16 wValue, __u16 wIndex, __u16 wLength, void *data)
{
    int ret;

    if (!p_urb)
    {
        ERR("[ERROR] Invalid URB-Pointer: 0x%p \n ", p_urb);
        return -EINVAL;
    }

    if (!p_urb->p_setup_packet)
    {
        ERR("[ERROR] Invalid Pointer to struct usb_ctrlrequest: 0x%p \n ", p_urb->p_setup_packet);
        return -EINVAL;
    }

    p_urb->core_in_time = rtdm_clock_read();

    p_urb->p_setup_packet->bRequestType = request_type;
    p_urb->p_setup_packet->bRequest     = request;
    p_urb->p_setup_packet->wValue       = cpu_to_le16(wValue);
    p_urb->p_setup_packet->wIndex       = cpu_to_le16(wIndex);
    p_urb->p_setup_packet->wLength      = cpu_to_le16( wLength);
    p_urb->transfer_buffer_length       = wLength;
    p_urb->p_transfer_buffer            = data;

    ret = rt_intern_usb_submit_urb(p_urb, URB_WAIT_SEM);

    p_urb->core_exit_time = rtdm_clock_read();

    return ret;
}

int nrt_usb_register_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
    {
        ERR("[ERROR] %s - Invalid URB-Pointer \n", __FUNCTION__);
        return -ENODEV;
    }

    if (!p_urb->p_hcd ||
        list_empty(&p_urb->p_hcd->usb_ctrl_list) ||
        !p_urb->p_hcd->p_hcd_fkt ||
        !p_urb->p_hcd->p_hcd_fkt->nrt_hcd_register_urb)
    {
        ERR("[ERROR] %s - Invalid Host-Controller-Pointer in URB 0x%p \n",
            __FUNCTION__, p_urb);
        return -ENODEV;
    }

    if (p_urb->max_packet_size <= 0 || p_urb->max_packet_size > 3072)
    {
        ERR("[ERROR] %s - Invalid Value for max_packet_size (%d Byte) in URB 0x%p \n",
            __FUNCTION__, p_urb->max_packet_size, p_urb);
        return -EINVAL;
    }

    if (p_urb->max_buffer_len < 0)
    {
        ERR("[ERROR] %s - Invalid Value for max_buffer_len (%d Byte) in URB 0x%p \n",
            __FUNCTION__, p_urb->max_buffer_len, p_urb);
        return -EINVAL;
    }

    // call host controller function
    return p_urb->p_hcd->p_hcd_fkt->nrt_hcd_register_urb(p_urb);
}

int nrt_usb_unregister_urb(struct rt_urb *p_urb)
{
    if (!p_urb)
    {
        ERR("[ERROR] %s - Invalid URB-Pointer \n", __FUNCTION__);
        return -ENODEV;
    }

    if (!p_urb->p_hcd ||
        list_empty(&p_urb->p_hcd->usb_ctrl_list) ||
        !p_urb->p_hcd->p_hcd_fkt ||
        !p_urb->p_hcd->p_hcd_fkt->nrt_hcd_unregister_urb)
    {
        ERR("[ERROR] %s - Invalid Host-Controller-Pointer in URB 0x%p \n",
            __FUNCTION__, p_urb);
        return -ENODEV;
    }
    return p_urb->p_hcd->p_hcd_fkt->nrt_hcd_unregister_urb(p_urb);
}

/******************************/
/* RT-CORE-API to the Drivers */
/******************************/

static int init_ctrl_urbs(void)
{
    p_ctrl_08_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_ctrl_08_urb)
        return -ENOMEM;


    p_ctrl_16_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_ctrl_16_urb)
    {
        nrt_usb_free_ctrl_urb(p_ctrl_08_urb);
        return -ENOMEM;
    }

    p_ctrl_32_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_ctrl_32_urb)
    {
        nrt_usb_free_ctrl_urb(p_ctrl_08_urb);
        nrt_usb_free_ctrl_urb(p_ctrl_16_urb);
        return -ENOMEM;
    }

    p_ctrl_64_urb = nrt_usb_alloc_ctrl_urb();
    if (!p_ctrl_64_urb)
    {
        nrt_usb_free_ctrl_urb(p_ctrl_08_urb);
        nrt_usb_free_ctrl_urb(p_ctrl_16_urb);
        nrt_usb_free_ctrl_urb(p_ctrl_32_urb);
        return -ENOMEM;
    }

    p_ctrl_08_urb->transfer_flags = URB_SHORT_NOT_OK;
    p_ctrl_08_urb->max_buffer_len = m_buff_size;
    p_ctrl_08_urb->max_packet_size = 8;
    p_ctrl_08_urb->transfer_dma = 0;
    p_ctrl_08_urb->setup_dma = 0;
    p_ctrl_08_urb->p_private = NULL;
    p_ctrl_08_urb->rt_complete_fkt = NULL;
    p_ctrl_08_urb->status = 0;
    p_ctrl_08_urb->actual_length = 0;

    p_ctrl_16_urb->transfer_flags = URB_SHORT_NOT_OK;
    p_ctrl_16_urb->max_buffer_len = m_buff_size;
    p_ctrl_16_urb->max_packet_size = 16;
    p_ctrl_16_urb->transfer_dma = 0;
    p_ctrl_16_urb->setup_dma = 0;
    p_ctrl_16_urb->p_private = NULL;
    p_ctrl_16_urb->rt_complete_fkt = NULL;
    p_ctrl_16_urb->status = 0;
    p_ctrl_16_urb->actual_length = 0;

    p_ctrl_32_urb->transfer_flags = URB_SHORT_NOT_OK;
    p_ctrl_32_urb->max_buffer_len = m_buff_size;
    p_ctrl_32_urb->max_packet_size = 32;
    p_ctrl_32_urb->transfer_dma = 0;
    p_ctrl_32_urb->setup_dma = 0;
    p_ctrl_32_urb->p_private = NULL;
    p_ctrl_32_urb->rt_complete_fkt = NULL;
    p_ctrl_32_urb->status = 0;
    p_ctrl_32_urb->actual_length = 0;

    p_ctrl_64_urb->transfer_flags = URB_SHORT_NOT_OK;
    p_ctrl_64_urb->max_buffer_len = m_buff_size;
    p_ctrl_64_urb->max_packet_size = 64;
    p_ctrl_64_urb->transfer_dma = 0;
    p_ctrl_64_urb->setup_dma = 0;
    p_ctrl_64_urb->p_private = NULL;
    p_ctrl_64_urb->rt_complete_fkt = NULL;
    p_ctrl_64_urb->status = 0;
    p_ctrl_64_urb->actual_length = 0;

    printk("RT-USBCORE: Control-URB @ 0x%p with  8 Byte Packet-Size\n",
           p_ctrl_08_urb);
    printk("RT-USBCORE: Control-URB @ 0x%p with 16 Byte Packet-Size\n",
           p_ctrl_16_urb);
    printk("RT-USBCORE: Control-URB @ 0x%p with 32 Byte Packet-Size\n",
           p_ctrl_32_urb);
    printk("RT-USBCORE: Control-URB @ 0x%p with 64 Byte Packet-Size\n",
           p_ctrl_64_urb);

    return 0;
}

void destroy_ctrl_urbs(void)
{
    nrt_usb_free_ctrl_urb(p_ctrl_08_urb);
    nrt_usb_free_ctrl_urb(p_ctrl_16_urb);
    nrt_usb_free_ctrl_urb(p_ctrl_32_urb);
    nrt_usb_free_ctrl_urb(p_ctrl_64_urb);
}

int __init mod_start(void)
{
    int i;
    int ret;

    PRNT(KERN_INFO "********** " DRIVER_DESC " " DRIVER_VERSION " ***********\n");

    PRNT("RT-USBCORE: Max %d Controller \n", MAX_CONTROLLER);
    PRNT("RT-USBCORE: Max %d USB-Devices \n", MAX_USB_DEV);

#ifdef CONFIG_USB4RT_DBG_COMMON
    PRNT("RT-USBCORE: Common debugging: enabled\n");
#else
    PRNT("RT-USBCORE: Common debugging: disabled\n");
#endif

#ifdef CONFIG_USB4RT_DBG_QH
    PRNT("RT-USBCORE: Queue head debugging: enabled\n");
#else
    PRNT("RT-USBCORE: Common debugging: disabled\n");
#endif

#ifdef CONFIG_USB4RT_DBG_TD
    PRNT("RT-USBCORE: Transfer descriptor debugging: enabled\n");
#else
    PRNT("RT-USBCORE: Common debugging: disabled\n");
#endif

#ifdef CONFIG_USB4RT_DBG_TIME
    PRNT("RT-USBCORE: Time debugging: enabled\n");
#else
    PRNT("RT-USBCORE: Common debugging: disabled\n");
#endif
    usb_anz_ctrl = 0;

    for (i=0;i < MAX_USB_DEV;i++)
    {
        usb_init_device(&usb_dev[i]);
        usb_dev[i].address = i;
    }

    ret = init_ctrl_urbs();
    if (ret)
    {
        return ret;
    }

    PRNT("RT-USBCORE: Initialize Hub-List \n");
    INIT_LIST_HEAD(&hub_list);

    PRNT("RT-USBCORE: Initialize Controller-List \n");
    INIT_LIST_HEAD(&usb_ctrl_list);

    PRNT("RT-USBCORE: Loading Completed (%d Byte allocated) \n", alloc_bytes);

    return 0;
}

void mod_exit(void)
{
    int i = 0;
    struct list_head  *p_list = NULL;
    struct list_head  *p_next = NULL;
    struct hub_device *p_hub  = NULL;

    destroy_ctrl_urbs();

    // Free Hub-List
    p_hub = NULL;
    p_list = hub_list.next;

    while (p_list != &hub_list)
    {
        p_next = p_list->next;
        p_hub = list_entry(p_list, struct hub_device, hub_list);
        kfree(p_hub);

        alloc_bytes -= sizeof(struct hub_device);
        DBG("RT-USBCORE: %d Byte allocated (%s %d Byte)\n",
            alloc_bytes, "-", sizeof(struct hub_device));

        p_list = p_next;
    }

    // Free Buffer for USB-Devices > 0
    for (i = 1; i<MAX_USB_DEV; i++)
    {
        usb_clear_device(&usb_dev[i]);
    }

    PRNT("RT-USBCORE: Unloading Completed (%d Byte allocated) \n",
         alloc_bytes);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_init (mod_start);
module_exit (mod_exit);

EXPORT_SYMBOL(rt_usb_get_device);
EXPORT_SYMBOL(rt_usb_put_device);

EXPORT_SYMBOL(rt_usb_send_urb_msg);
EXPORT_SYMBOL(rt_usb_send_ctrl_msg);
EXPORT_SYMBOL(rt_usb_submit_urb);
EXPORT_SYMBOL(rt_usb_submit_ctrl_urb);

EXPORT_SYMBOL(nrt_hcd_register_driver);
EXPORT_SYMBOL(nrt_hcd_unregister_driver);

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
