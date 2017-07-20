/*
 * Poplar USB device descriptors and Endpoints structure
 *
 * (C) Copyright 2017 Hisilicon Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __POPLAR_UDC_PRIV__
#define __POPLAR_UDC_PRIV__
#include <usb/poplar_dwc_regs.h>
#include <linux/usb/composite.h>

#ifndef readl
#define readl(addr) (*((volatile unsigned int *)(addr)))
#endif

#ifndef writel
#define writel(val, addr) (*(volatile unsigned int *)(addr) = (val))
#endif

#if DEBUG
#define HIUSB_PRINTF(fmt, args...) printf( fmt , ## args)
#else
#define HIUSB_PRINTF(fmt, args...)
#endif

#define UGETW(w) ((w)[0] | ((w)[1] << 8))


#define CLEAR_IN_EP_INTR(__epnum,__intr) \
do { \
		diepint_data_t diepint = {.d32=0}; \
		diepint.b.__intr = 1; \
		writel(diepint.d32,(REG_DIEPINT0 +__epnum*0x20)); \
} while (0)

#define CLEAR_OUT_EP_INTR(__epnum,__intr) \
do { \
		doepint_data_t doepint = {.d32=0}; \
		doepint.b.__intr = 1; \
		writel(doepint.d32,REG_DOEPINT0+__epnum*0x20); \
} while (0)


/** Maxpacket size for EP0 */
#define MAX_EP0_SIZE	64
/** Maxpacket size for any EP */
#define MAX_PACKET_SIZE 512
#define DATA_SIZE 32768

#define UE_DIR_IN	0x80
#define UE_DIR_OUT	0x00
#define UE_XFERTYPE	0x03
#define UE_BULK	0x02
#define UR_SET_ADDRESS		0x05
#define UR_GET_DESCRIPTOR	0x06


#define USB_STATUS_OK       0x01
#define USB_STATUS_ERROR    0x02
#define USB_STATUS_DATA     0x03
#define USB_STATUS_FIN      0x04

#define GET_STATUS           0
#define CLEAR_FEATURE        1
#define SET_FEATURE          3
#define SET_ADDRESS          5
#define GET_DESCRIPTOR       6
#define SET_DESCRIPTOR       7
#define GET_CONFIGURATION    8
#define SET_CONFIGURATION    9
#define GET_INTERFACE        10
#define SET_INTERFACE        11
#define SYNCH_FRAME          12

#define TYPE_DEVICE          1
#define TYPE_CONFIGURATION   2
#define TYPE_STRING          3
#define TYPE_INTERFACE       4
#define TYPE_ENDPOINT        5

#define DEVICE_READ          0x80
#define DEVICE_WRITE         0x00
#define INTERFACE_READ       0x81
#define INTERFACE_WRITE      0x01
#define ENDPOINT_READ        0x82
#define ENDPOINT_WRITE       0x02

#define USB_SPEED_UNKNOWN	0
#define USB_SPEED_LOW		1
#define USB_SPEED_FULL		2
#define USB_SPEED_HIGH		3
#define USB_SPEED_VARIABLE	4
#define USB_SPEED_SUPER		5


struct hiusb_request {
	void			*buf;
	unsigned		length;

	void			(*complete)(struct hiusb_request *req);
	int			status;
	unsigned		actual;
    unsigned    ret_status;
};


/**
 * States of EP0.
 */
typedef enum ep0_state {
	EP0_DISCONNECT,		/* no host */
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_STALL,
} ep0state_e;
#define UPACKED __attribute__((__packed__))

typedef unsigned char uByte;
typedef unsigned char uWord[2];
typedef unsigned char uDWord[4];

typedef struct {
	uByte		bmRequestType;
	uByte		bRequest;
	uWord		wValue;
	uWord		wIndex;
	uWord		wLength;
} UPACKED usb_device_request_t;

/** DWC_otg PCD Structure.
 * This structure encapsulates the data for the dwc_otg PCD.
 */
struct dwc_otg_pcd {

	/** State of EP0 */
	ep0state_e ep0state;

	/** SETUP packet for EP0
	 * This structure is allocated as a DMA buffer on PCD initialization
	 * with enough space for up to 3 setup packets.
	 */
	union {
		usb_device_request_t req;
		uint32_t d32[2];
	} *setup_pkt;

	dwc_ep_t ep0;
	dwc_ep_t in_ep;
	dwc_ep_t out_ep;
};

typedef struct dwc_otg_pcd dwc_otg_pcd_t;

#define DWC2_MAX_ENDPOINTS	3

enum ep_type {
	ep_control, ep_bulk_in, ep_bulk_out, ep_interrupt
};

struct dwc2_ep {
	struct usb_ep ep;
	struct hi_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;
	int len;
	void *dma_buf;

	u8 stopped;
	u8 bEndpointAddress;
	u8 bmAttributes;

	enum ep_type ep_type;
	int fifo_num;
};

struct hi_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

	struct dwc2_plat_otg_data *pdata;

	int ep0state;
	struct dwc2_ep ep[DWC2_MAX_ENDPOINTS];

	unsigned char usb_address;

	unsigned req_pending:1, req_std:1;
    unsigned			clocked:1;
};


static const unsigned short manufacturer_string[] = {
    (TYPE_STRING << 8) | (10 * 2),
    'H', 'i', 's', 'i', 'l', 'i', 'c', 'o', 'n',
};
static const unsigned short product_string[] = {
    (TYPE_STRING << 8) | (7 * 2),
    'P', 'o', 'p', 'l', 'a', 'r',
};

static const unsigned short default_string[] = {
    (TYPE_STRING << 8) | (8 * 2),
    'd', 'e', 'f', 'a', 'u', 'l', 't',
};

static const unsigned short language_table[] = {
    (TYPE_STRING << 8) | 4,
    0x0409, // LANGID for US English
};
/*Add serial NUM */
static const unsigned short serial_string[] = {
    (TYPE_STRING << 8) | (17 * 2),//0x322
    '0','1','2','3','4','5','6','7','8','9','P','O','P','L','A','R'
};

static const unsigned char device_desc[] = {
    18,              // length
    TYPE_DEVICE,     // type
    0x00, 0x02,      // usb spec rev 1.00
    0xFF,            // class
    0x00,            // subclass
    0x00,            // protocol
    0x40,            // max packet size
    0xD1, 0x18,      // vendor id (huawei 0x12d1)
    0x0d, 0xD0,      // product id (0xd001, (device chip, 001)
    0x00, 0x00,      // version 1.0
    0x01,            // manufacturer str idx
    0x02,            // product str idx
    0x03,            // serial number index
    0x01,            // number of configs,
};

static const unsigned char config_desc[] = {
// USB_CONFIG_DESCRIPTOR
    0x09,            // length
    TYPE_CONFIGURATION,
    0x20, 0x00,      // total length
    0x01,            // # interfaces
    0x01,            // config value
    0x00,            // config string
    0xC0,            // attributes:   bit 7 reserved and must be 1, bit 6 means self-powered.
    0x0,            // XXX max power (250ma)

// USB_INTERFACE_DESCRIPTOR
    0x09,            // length
    TYPE_INTERFACE,
    0x00,            // interface number
    0x00,            // alt number
    0x02,            // # endpoints
    0xFF,
    0x42,
    0x03,
    0x00,            // interface string

// USB_ENDPOINT_DESCRIPTOR (In Endpoint)
    0x07,            // length
    TYPE_ENDPOINT,
    0x81,            // in, #1 EndpointAddress
    0x02,            // bulk
    0x00, 0x02,      // max packet 512
    16,

// STATIC USB_ENDPOINT_DESCRIPTOR (Out Endpoint)
    0x07,            // length
    TYPE_ENDPOINT,
    0x02,            // out, #1
    0x02,            // bulk
    0x00, 0x02,      // max packet 512
    16,            // interval
};

static const unsigned char config_desc_fs[] = {
    0x09,            // length
    TYPE_CONFIGURATION,
    0x20, 0x00,      // total length
    0x01,            // # interfaces
    0x01,            // config value
    0x00,            // config string
    0xC0,            // attributes
    0x0,            // XXX max power (250ma)

    0x09,            // length
    TYPE_INTERFACE,
    0x00,            // interface number
    0x00,            // alt number
    0x02,            // # endpoints
    0xFF,
    0x42,
    0x03,
    0x00,            // interface string

    0x07,            // length
    TYPE_ENDPOINT,
    0x81,            // in, #1
    0x02,            // bulk
    0x40, 0x00,      // max packet 64
    16,            // interval

    0x07,            // length
    TYPE_ENDPOINT,
    0x02,            // out, #1
    0x02,            // bulk
    0x40, 0x00,      // max packet 64
    16,            // interval
};

typedef struct
{
    const void *data;
    unsigned short length;
    unsigned short id;
} dtable;

#define ID(type,num) ((type << 8) | num)

static const dtable descr_hs[] = {
    { device_desc, sizeof(device_desc), ID(TYPE_DEVICE, 0) },
    { config_desc, sizeof(config_desc), ID(TYPE_CONFIGURATION, 0) },
    { manufacturer_string, sizeof(manufacturer_string), ID(TYPE_STRING, 1) },
    { product_string, sizeof(product_string), ID(TYPE_STRING, 2) },
    { default_string, sizeof(default_string), ID(TYPE_STRING, 4) },
    { language_table, sizeof(language_table), ID(TYPE_STRING, 0) },
    { serial_string,  sizeof(serial_string),  ID(TYPE_STRING, 3) },
    { 0, 0, 0 },
};

static dtable descr_fs[] = {
    { device_desc, sizeof(device_desc), ID(TYPE_DEVICE, 0) },
    { config_desc_fs, sizeof(config_desc), ID(TYPE_CONFIGURATION, 0) },
    { manufacturer_string, sizeof(manufacturer_string), ID(TYPE_STRING, 1) },
    { product_string, sizeof(product_string), ID(TYPE_STRING, 2) },
    { default_string, sizeof(default_string), ID(TYPE_STRING, 4) },
    { language_table, sizeof(language_table), ID(TYPE_STRING, 0) },
    { serial_string,  sizeof(serial_string),  ID(TYPE_STRING, 3) },
    { 0, 0, 0 },
};



void hiusb_ep_queue(dwc_ep_t *ep,struct hiusb_request *req_handle);
void hiusb_ep_enable(int dir);
int dwc_get_device_speed(void);
void hiusb_irq(void * dev);
void udc_disconnect(void);
void hiusb_init(void);
void rx_cmd_and_data(void);
void rx_cmd(void);
void usb_tx_status_complete(struct hiusb_request *req);
void usb_rx_cmd_complete(struct hiusb_request *req);
static int do_set_interface(void);


int usb_gadget_handle_interrupts(int index);


struct f_fastboot {
	struct usb_function usb_function;

	/* IN/OUT EP's and corresponding requests */
	struct usb_ep *in_ep, *out_ep;
	struct usb_request *in_req, *out_req;
};

static inline struct f_fastboot *func_to_fastboot(struct usb_function *f);

#endif
