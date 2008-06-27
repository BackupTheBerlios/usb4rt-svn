/***************************************************************************
 *   Copyright (C) 2005,2006 by Jörg Langenberg                            *
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

#ifndef RT_USB_H
#define RT_USB_H

#include <linux/types.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#include <linux/usb_ch9.h>
#else
#include <linux/usb/ch9.h>
#endif

#include <rtdm/rtdm_driver.h>

/*--------------------------*
 *        Descriptors       *
 *--------------------------*/
typedef struct hub_descriptor // 9 Byte
{
    __u8 bLength;
    __u8 type;
    __u8 bNbrPorts;
    __u16 wHubCharacteristics;
    __u8 bPwrOn2PwrGood;
    __u8 bHubCntrCurrent;
    __u8 DeviceRemovable;   // assume bNbrPorts <=8
    __u8 PortPwrCntrMask;
} __attribute__((packed)) hub_descriptor_t;

typedef struct hid_descriptor // 6 Byte
{
    __u8  bLength;
    __u8  type;
    __u16 bcdHID;
    __u8  bCountryCode;
    __u8  bNumDescriptors;
}   __attribute__ ((packed)) hid_descriptor_t;

/*--------------------------*
 *        USB-Device        *
 *--------------------------*/
struct usb_device
{
    hub_descriptor_t *p_hub_desc;
    struct usb_config_descriptor *p_cfg_desc;
    struct usb_device_descriptor *p_dev_desc;
    enum usb_device_state state;
    enum usb_device_speed speed;

    __u8 in_use;
    spinlock_t lock;
    struct hc_device *p_hcd;

    __u16 vendor;
    __u16 product;

    __u8 rh_port;
    __u8 address;

    __u8 class;
    __u8 subclass;
    __u8 protocol;

    int epmaxpacketin[16];
    int epmaxpacketout[16];

    __u16 toggle_mask[2]; // 1 Bit per Endpoint; [0]=IN [1]=OUT
    __u16 ctrl_mask[2];   // 1 Bit per Endpoint; [0]=IN [1]=OUT
    __u16 bulk_mask[2];   // 1 Bit per Endpoint; [0]=IN [1]=OUT
    __u16 int_mask[2];    // 1 Bit per Endpoint; [0]=IN [1]=OUT
    __u16 iso_mask[2];    // 1 Bit per Endpoint; [0]=IN [1]=OUT

    void *p_private;      // internal core data
};

/*--------------------------*
 *   URB- Transfer-Flags    *
 *--------------------------*/
#define URB_SHORT_NOT_OK         0x0001  // report short reads as errors
//#define URB_ISO_ASAP           0x0002  // iso-only, urb->start_frame ignored
#define URB_NO_TRANSFER_DMA_MAP  0x0004  // urb->transfer_dma valid on submit
#define URB_NO_SETUP_DMA_MAP     0x0008  // urb->setup_dma valid on submit
//#define URB_NO_FSBR            0x0020  // UHCI-specific
//#define URB_ZERO_PACKET        0x0040  // Finish bulk OUTs with short packet
//#define URB_NO_INTERRUPT       0x0080  // HINT: no non-error interrupt needed
#define URB_TIMESTAMP_DATA       0x0100  // Bulk-only: Get Timestamp of first transferred TD

struct rt_urb;
struct pt_regs;

typedef void (*rt_usb_complete_t)(struct rt_urb *);

/*--------------------------*
 *       Realtime-URB       *
 *--------------------------*/
struct rt_urb
{
    // needed while registering the URB
    struct hc_device *p_hcd;                // pointer to the host controller
    int max_buffer_len;                     // (in) ALL max buffer-length
                                            // (needed by the hcd to create Buffers)
    int max_packet_size;                    // (in) ALL max packet-size
                                            // (needed by the hcd to create Buffers)

    // Needed while sending the URB
    struct usb_device *p_usbdev;            // (in) ALL pointer to associated device
    unsigned int pipe;                      // (in) ALL pipe information
    unsigned int transfer_flags;            // (in) ALL URB_SHORT_NOT_OK | ...
    int transfer_buffer_length;             // (in) data buffer length
    void *p_transfer_buffer;                // (in) ALL associated data buffer
    dma_addr_t transfer_dma;                // (in) IF  dma addr for transfer_buffer

    // needed while sending a control URB
    struct usb_ctrlrequest *p_setup_packet; // (in) CTRL setup packet
    dma_addr_t setup_dma;                   // (in) CTRL dma addr for setup_packet

    // callback or blocking
    rt_usb_complete_t rt_complete_fkt;      // (in) completion routine
    rtdm_sem_t rt_sem;                      // sleep on while waiting
    int64_t rt_sem_timeout;                 // semaphore timeout

    // Private and Context-Data
    void *p_private;                        // private core/HCD data
    void *p_context;                        // user data

    // return-values
    int status;                             // (return) non-ISO status
    int actual_length;                      // (return) actual transfer length

    uint64_t core_in_time;                  // core send time
    uint64_t core_exit_time;                // core finish time

    uint64_t schedule_time;                 // schedule time
    uint64_t unschedule_time;               // unschedule time

    uint64_t timestamp_first_data;          // timestamp of first transferred data package
};


/*-------------------------------*
 *     USB device locking        *
 *-------------------------------*/
extern struct usb_device *rt_usb_get_device(__u16 vendor,
                                            __u16 product ,
                                            struct usb_device *p_old_device);
extern void rt_usb_put_device(struct usb_device *p_usbdev);

/*-------------------------------*
 *      URB allocate-Helper      *
 *-------------------------------*/

// control callback URB
extern struct rt_urb *nrt_usb_create_ctrl_callback_urb(rt_usb_complete_t rt_callback_fkt);
extern void nrt_usb_destroy_ctrl_callback_urb(struct rt_urb *p_urb);

// callback URB
extern struct rt_urb *nrt_usb_create_callback_urb(rt_usb_complete_t rt_callback_fkt);
extern void nrt_usb_destroy_callback_urb(struct rt_urb *p_urb);

// control blocking URB
extern struct rt_urb *nrt_usb_create_ctrl_semaphore_urb(unsigned char *name,
                                                        int64_t rt_sem_timeout);
extern void nrt_usb_destroy_ctrl_semaphore_urb(struct rt_urb *p_urb);

// blocking URB
extern struct rt_urb *nrt_usb_create_semaphore_urb(unsigned char *name,
                                                   int64_t rt_sem_timeout);
extern void nrt_usb_destroy_semaphore_urb(struct rt_urb *p_urb);

// allocate/free URB
extern struct rt_urb *nrt_usb_alloc_urb(void);
extern void nrt_usb_free_urb(struct rt_urb *p_urb);

// allocate/free control URB
extern struct rt_urb *nrt_usb_alloc_ctrl_urb(void);
extern void nrt_usb_free_ctrl_urb(struct rt_urb *p_urb);

extern int nrt_usb_clear_stall(struct rt_urb *p_urb);

/*-------------------------------*
 *      URB registering          *
 *-------------------------------*/
extern int nrt_usb_register_urb(struct rt_urb *p_urb);
extern int nrt_usb_unregister_urb(struct rt_urb *p_urb);

/*--------------------------*
 *   synchronous messages   *
 *--------------------------*/
extern int rt_usb_send_urb_msg(struct rt_urb *p_urb);
extern int rt_usb_send_ctrl_msg(struct rt_urb *p_urb, __u8 request_type,
                                __u8 request, __u16 wValue, __u16 wIndex,
                                __u16 wLength , void *data);

/*--------------------------*/
/*   asynchronous messages  */
/*--------------------------*/
extern int rt_usb_submit_urb(struct rt_urb *p_urb);
extern int rt_usb_submit_ctrl_urb(struct rt_urb *p_urb, __u8 request_type,
                                  __u8 request, __u16 wValue, __u16 wIndex,
                                  __u16 wLength , void *data);

/*--------------------------*
 *       PIPES              *
 *--------------------------*/

/* Pipe : Bit 31                            0 */
/*        -------- -----EEE EDDDDDDD I-----TT */
/*  T = Type                                  */
/*  D = Device                                */
/*  E = Endpoint                              */
/*  I = Direction-IN                          */

#define PIPE_ENPOINT_SHIFT      15
#define PIPE_ENDPOINT_MASK      0xf
#define PIPE_DEVICE_SHIFT       8
#define PIPE_DEVICE_MASK        0x7f

#define usb_pipein(pipe)        ((pipe) & USB_DIR_IN)
#define usb_pipeout(pipe)       (!usb_pipein(pipe))
#define usb_pipedevice(pipe)    (((pipe) >> PIPE_DEVICE_SHIFT) & PIPE_DEVICE_MASK)
#define usb_pipeendpoint(pipe)  (((pipe) >> PIPE_ENPOINT_SHIFT) & PIPE_ENDPOINT_MASK)
#define usb_pipetype(pipe)      ((pipe) & USB_ENDPOINT_XFERTYPE_MASK)
#define usb_pipeisoc(pipe)      (usb_pipetype((pipe)) == USB_ENDPOINT_XFER_ISOC)
#define usb_pipeint(pipe)       (usb_pipetype((pipe)) == USB_ENDPOINT_XFER_INT)
#define usb_pipecontrol(pipe)   (usb_pipetype((pipe)) == USB_ENDPOINT_XFER_CONTROL)
#define usb_pipebulk(pipe)      (usb_pipetype((pipe)) == USB_ENDPOINT_XFER_BULK)

static inline unsigned int __create_pipe(struct usb_device *p_usbdev,
                                         unsigned int endpoint)
{
    return (p_usbdev->address << PIPE_DEVICE_SHIFT) | (endpoint << PIPE_ENPOINT_SHIFT);
}

/* Create various pipes... */
#define usb_sndctrlpipe(dev,endpoint)  (USB_ENDPOINT_XFER_CONTROL | __create_pipe(dev,endpoint))
#define usb_rcvctrlpipe(dev,endpoint)  (USB_ENDPOINT_XFER_CONTROL | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndisocpipe(dev,endpoint)  (USB_ENDPOINT_XFER_ISOC | __create_pipe(dev,endpoint))
#define usb_rcvisocpipe(dev,endpoint)  (USB_ENDPOINT_XFER_ISOC | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndbulkpipe(dev,endpoint)  (USB_ENDPOINT_XFER_BULK | __create_pipe(dev,endpoint))
#define usb_rcvbulkpipe(dev,endpoint)  (USB_ENDPOINT_XFER_BULK | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndintpipe(dev,endpoint)   (USB_ENDPOINT_XFER_INT  | __create_pipe(dev,endpoint))
#define usb_rcvintpipe(dev,endpoint)   (USB_ENDPOINT_XFER_INT  | __create_pipe(dev,endpoint) | USB_DIR_IN)

#define usb_maxpacket(dev, pipe)       (usb_pipeout(pipe) \
                                         ? (dev)->epmaxpacketout[usb_pipeendpoint(pipe)] \
                                         : (dev)->epmaxpacketin [usb_pipeendpoint(pipe)])

#define usb_get_data_toggle(dev, pipe) (usb_pipeout(pipe) \
                                         ? ((dev)->toggle_mask[1] & (1 << usb_pipeendpoint(pipe))) \
                                         : ((dev)->toggle_mask[0] & (1 << usb_pipeendpoint(pipe))))

#define usb_set_data_toggle(dev, pipe, value) ((value) \
                                                 ? \
                                                 (usb_pipeout(pipe) \
                                                   ? ((dev)->toggle_mask[1] |= (1 << usb_pipeendpoint(pipe))) \
                                                   : ((dev)->toggle_mask[0] |= (1 << usb_pipeendpoint(pipe)))) \
                                                 : \
                                                 (usb_pipeout(pipe) \
                                                   ? ((dev)->toggle_mask[1] &= ~(1 << usb_pipeendpoint(pipe))) \
                                                   : ((dev)->toggle_mask[0] &= ~(1 << usb_pipeendpoint(pipe)))) \
                                             )

#endif
