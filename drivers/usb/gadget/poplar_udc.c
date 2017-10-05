/*
 * Poplar USB device driver
 *
 * (C) Copyright 2017 Hisilicon Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <malloc.h>
#include <common.h>
#include <linux/errno.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/list.h>
#include <config.h>
#include "poplar_udc.h"

#include <linux/usb/composite.h>
#include <asm/arch/hi3798cv200.h>

static struct hiusb_request * ep0req ;
static struct hiusb_request * tx_req  ;
static struct hiusb_request * rx_req  ;
static struct dwc_otg_pcd * pcd ;
static struct hi_udc	   * hi_controller;
static unsigned char usb_config_value;

static char *cmdbuf;
typedef void (*COMPLETE)(struct usb_ep *ep,struct usb_request *req) ;

COMPLETE fastboot_rx_handler_command;
COMPLETE fastboot_tx_handler_complete;

struct f_fastboot *hif_fb;
#define FASTBOOT_COMMAND_MAX_LENGTH 64
char GetResponse[FASTBOOT_COMMAND_MAX_LENGTH + 1];


u32 usb_status;
static struct usb_request *usb_req;

extern void usb2_phy_init(void);

void HIUSB_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask, uint32_t set_mask)
{
	writel((readl(reg) & ~clear_mask) | set_mask, reg);
}
void usb_tx_status_complete(struct hiusb_request *req)
{
    struct usb_request usb_req;
	req->actual = 0;
	req->length = 0;
    usb_status = req->ret_status;
    usb_req.actual = 0;
    usb_req.length = 0;

    if(NULL != fastboot_tx_handler_complete)
        fastboot_tx_handler_complete(hif_fb->in_ep,&usb_req);
    fastboot_tx_handler_complete = NULL;
}

void tx_status(const char *status, unsigned ret_status)
{
	struct hiusb_request *req = tx_req;
	req->buf = (void*)status ;
    HIUSB_PRINTF("req->buf = %s\n",req->buf);
	req->length = strlen(req->buf);
	req->complete =usb_tx_status_complete;
	req->status = -EINPROGRESS;
	req->actual = 0;
    req->ret_status = ret_status;
	hiusb_ep_queue(&pcd->in_ep,req);
}

void usb_rx_cmd_complete(struct hiusb_request *req)
{
    usb_status = 0;
	if(req->status != 0) return;
    if(req->actual < req->length)
        cmdbuf[req->actual] = 0;
    usb_req->actual = req->actual;
    usb_req->buf    = req->buf;
    usb_req->status = req->status;
    usb_req->length = req->length;
    usb_req->complete(hif_fb->out_ep,usb_req);
	rx_cmd_and_data();
}

void rx_cmd(void)
{
	struct hiusb_request *req = rx_req;
	req->length = MAX_PACKET_SIZE;
	memset(req->buf, 0, MAX_PACKET_SIZE);
	req->complete = usb_rx_cmd_complete;
	req->status = -EINPROGRESS;
	req->actual = 0;
	hiusb_ep_queue(&pcd->out_ep,req);
}

void rx_cmd_and_data()
{
	struct hiusb_request *req = rx_req;
    req->length = DATA_SIZE;
    memset(req->buf, 0, DATA_SIZE);
    req->complete = usb_rx_cmd_complete;
	req->status = -EINPROGRESS;
	req->actual = 0;
	hiusb_ep_queue(&pcd->out_ep,req);
}

/*
 * Reset interface setting and re-init endpoint state (toggle etc).
 * Call with altsetting < 0 to disable the interface.  The only other
 * available altsetting is 0, which enables the interface.
 */
static int do_set_interface(void)
{
	int	rc = 0;
	struct hiusb_request *hiusb_req;

	hiusb_ep_enable(UE_DIR_IN);
	hiusb_ep_enable(UE_DIR_OUT);

	/* Allocate the requests */
	tx_req =	malloc(sizeof(*hiusb_req));
	memset(tx_req, 0, sizeof(*hiusb_req));
	tx_req->complete = 0;

	rx_req=malloc(sizeof(*hiusb_req));;
	memset(rx_req, 0, sizeof(*hiusb_req));
	rx_req->buf =malloc(DATA_SIZE) ;
	rx_req->complete = 0;
	cmdbuf = rx_req->buf;

	usb_req = calloc(1,sizeof(struct usb_request));
	if(!usb_req)
		return -1;
	return rc;
}

void setup(uint8_t * bytes)
{

    int speed = 0;
	struct usb_ctrlrequest *ctrl= (struct usb_ctrlrequest*)bytes;
	uint16_t	w_index = ctrl->wIndex;
	uint16_t	w_value = ctrl->wValue;
	uint16_t	w_length = ctrl->wLength;
	uint16_t	rc = 0;
	const dtable *d = descr_hs;
	ep0req->length = 0;
	ep0req->status = -EINPROGRESS;
	ep0req->actual = 0;

	speed = dwc_get_device_speed();
	if(speed == USB_SPEED_FULL)
        d = descr_fs;
	switch (ctrl->bRequest) {

	case GET_DESCRIPTOR:
        HIUSB_PRINTF("GET_DESCRIPTOR\n");
		while(d->data) {
			if(w_value == d->id) {

				unsigned len = d->length;
				if(len > w_length) len = w_length;

				rc =len;
				memcpy(ep0req->buf, d->data,len);
				break;
			}
			d++;
		}

		rc = min(rc, w_length);
		ep0req->length = rc;
		hiusb_ep_queue(&pcd->ep0,ep0req);
		break;

	/* One config, two speeds */
	case SET_CONFIGURATION:
        HIUSB_PRINTF("SET_CONFIGURATION\n");
		if (w_value == 1) {
			usb_config_value = w_value;
			do_set_interface();
			hiusb_ep_queue(&pcd->ep0,ep0req);
            int	result = -EINVAL;
            struct usb_configuration *c = NULL;
            struct usb_composite_dev *cdev = hi_controller->gadget.dev.driver_data;
            list_for_each_entry(c, &cdev->configs, list) {
                if (c->bConfigurationValue == w_value) {
                    result = 0;
                    HIUSB_PRINTF("Successful or not\n");
                    cdev->config = c;//so we build a relationship between cdev and config
                    break;
                    }
            }

            struct usb_function *f;
            f = c->interface[0];

            //we will get into set_alt() function,and in this function,f_fb->out_req->complete will be set as rx_handler_command .
            result = f->set_alt(f,0,0);
            if(0!=result)
                {
                    printf("set_alt Error! Poplar fastboot will not work.\n");
                }
            hif_fb = func_to_fastboot(f);

            usb_req->complete = hif_fb->out_req->complete;
			rx_cmd();
            printf("Poplar get into fastboot mode\n");
		}
		else
		{
            usb_config_value = 0;
        }

		break;
	case UR_SET_ADDRESS:
		{
            HIUSB_PRINTF("UR_SET_ADDRESS\n");
			dcfg_data_t dcfg;
			dcfg.d32= readl(REG_DCFG);
			dcfg.b.devaddr = UGETW(&w_value);
			writel(dcfg.d32, REG_DCFG);
			hiusb_ep_queue(&pcd->ep0,ep0req);
		}
		break;
    case GET_STATUS:
		{
            HIUSB_PRINTF("GET_STATUS\n");
			hiusb_ep_queue(&pcd->ep0,ep0req);
		}
		break;
	case GET_CONFIGURATION:
        HIUSB_PRINTF("GET_CONFIGURATION\n");
		if((w_value == 0) && (w_index == 0) && (w_length == 1))
		*(u8 *) ep0req->buf =usb_config_value;
		ep0req->length = 1;
		hiusb_ep_queue(&pcd->ep0,ep0req);
		break;
	default:
		break;
	}
}

static inline struct f_fastboot *func_to_fastboot(struct usb_function *f)
{
	return container_of(f, struct f_fastboot, usb_function);
}

/**
 * This function is being called from gadget
 * to enable PCD endpoint.
 */
void hiusb_ep_enable(int dir)
{
	dwc_ep_t *ep = NULL;
	depctl_data_t depctl;
	volatile uint32_t *addr;
	daint_data_t daintmsk = {.d32 = 0 };

	if (dir == UE_DIR_IN) {
		ep = &pcd->in_ep;
		addr = (uint32_t *)REG_DIEPCTL1;
		daintmsk.ep.in = 1 << ep->num;
	} else {
		ep = &pcd->out_ep;
		addr = (uint32_t *)REG_DOEPCTL2;
		daintmsk.ep.out = 1 << ep->num;
	}

	/* If the EP is already active don't change the EP Control register*/
	depctl.d32 = readl(addr);

	if (!depctl.b.usbactep) {
		depctl.b.mps = ep->maxpacket;
		depctl.b.eptype = ep->type;
		depctl.b.txfnum = ep->tx_fifo_num;
		depctl.b.setd0pid = 1;
		depctl.b.usbactep = 1;
		writel(depctl.d32,addr);
	}

	HIUSB_MODIFY_REG32((uint32_t *)REG_DAINTMSK, 0, daintmsk.d32);

}

/**
 * This function is called when dedicated Tx FIFO Empty interrupt occurs.
 * The active request is checked for the next packet to be loaded into
 * apropriate Tx FIFO.
 */
void hiusb_write_empty_tx_fifo( uint32_t epnum)
{
	dtxfsts_data_t txstatus = {.d32 = 0 };
	dwc_ep_t *ep = 0;
	uint32_t *fifo;
	uint32_t len = 0;
	int dwords;

	if (epnum == 0) {
		ep= &pcd->ep0;
		fifo = (uint32_t *)REG_DATAFIFO0;
	}
	else
	{
		ep=&pcd->in_ep;
		fifo = (uint32_t *)REG_DATAFIFO1;
	}

	len = ep->xfer_len - ep->xfer_count;
	dwords = (len + 3) / 4;

	/* While there is space in the queue and space in the FIFO and
	 * More data to tranfer, Write packets to the Tx FIFO */
	txstatus.d32 = readl(a32_to_ptr(REG_DTXFSTS0 + epnum * 0x20));

	while (ep->xfer_count < ep->xfer_len &&
	       txstatus.b.txfspcavail > dwords &&ep->xfer_len != 0) {
		/* Write the FIFO */
		uint32_t i;
		uint32_t byte_count;
		uint32_t dword_count;
		uint32_t *data_buff = (uint32_t *) ep->xfer_buff;

		/* Find the byte length of the packet either short packet or MPS */
		byte_count = ep->xfer_len - ep->xfer_count;
		dword_count = (byte_count + 3) / 4;

		for (i = 0; i < dword_count; i++, data_buff++)
		{
			writel(*data_buff,fifo);
		}
		ep->xfer_count += byte_count;
		ep->xfer_buff += byte_count;
		len = ep->xfer_len - ep->xfer_count;
		dwords = (len + 3) / 4;
		txstatus.d32 = readl(a32_to_ptr(REG_DTXFSTS0 + epnum * 0x20));
	}
}

int dwc_get_device_speed(void)
{
	dsts_data_t dsts;
	int speed = 0;
	dsts.d32 = readl(REG_DSTS);

	switch (dsts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
		speed = USB_SPEED_HIGH;
		HIUSB_PRINTF("high speed \n");
		break;
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		speed = USB_SPEED_FULL;
		HIUSB_PRINTF("full speed \n");
		break;
	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
		speed = USB_SPEED_LOW;
		HIUSB_PRINTF("low speed \n");
		break;
	}

	return speed;
}

/**
 * This function does the setup for a data transfer for an EP and
 * starts the transfer. For an IN transfer, the packets will be
 * loaded into the appropriate Tx FIFO in the ISR. For OUT transfers,
 * the packets are unloaded from the Rx FIFO in the ISR.  the ISR.
 *
 * @param ep The EP to start the transfer on.
 */
void hiusb_ep12_start_transfer(dwc_ep_t * ep)
{
	depctl_data_t depctl;
	deptsiz_data_t deptsiz;

	/* IN endpoint */
	if (ep->is_in == 1)
	{
		depctl.d32 = readl(REG_DIEPCTL1);
		deptsiz.d32 = readl(REG_DIEPTSIZ1);

		ep->xfer_len += ep->total_len - ep->xfer_len;

		/* Zero Length Packet? */
		if ((ep->xfer_len - ep->xfer_count) == 0) {
			deptsiz.b.xfersize = 0;
			deptsiz.b.pktcnt = 1;
		} else {
			deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
			deptsiz.b.pktcnt = (ep->xfer_len - ep->xfer_count - 1 +
			     ep->maxpacket) / ep->maxpacket;
		}
		writel(deptsiz.d32,REG_DIEPTSIZ1);

		/* Enable the Tx FIFO Empty Interrupt for this EP */
		if (ep->xfer_len > 0) {
			uint32_t fifoemptymsk = 0;
			fifoemptymsk = 1 << ep->num;
			HIUSB_MODIFY_REG32((uint32_t *)REG_DTKNQR4,0, fifoemptymsk);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		writel(depctl.d32,REG_DIEPCTL1);
	}
	else
	{

		depctl.d32 = readl(REG_DOEPCTL2);
		deptsiz.d32 = readl(REG_DOEPTSIZ2);

		ep->xfer_len += (ep->total_len - ep->xfer_len);

		if ((ep->xfer_len - ep->xfer_count) == 0) {
			/* Zero Length Packet */
			deptsiz.b.xfersize = ep->maxpacket;
			deptsiz.b.pktcnt = 1;
		} else {
			deptsiz.b.pktcnt =(ep->xfer_len - ep->xfer_count +
			     (ep->maxpacket - 1)) / ep->maxpacket;
			ep->xfer_len =deptsiz.b.pktcnt * ep->maxpacket + ep->xfer_count;
			deptsiz.b.xfersize = ep->xfer_len - ep->xfer_count;
		}
		writel(deptsiz.d32,REG_DOEPTSIZ2);

		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		writel(depctl.d32,REG_DOEPCTL2);
	}
}

/**
 * This function handles EP0 Control transfers.
 *
 * The state of the control tranfers are tracked in
 * <code>ep0state</code>.
 */
static void hiusb_complete_ep0(void )
{
	dwc_ep_t *ep0 = &pcd->ep0;
	deptsiz0_data_t deptsiz;
	depctl_data_t doepctl = {.d32 = 0 };

	switch (pcd->ep0state) {
	case EP0_DISCONNECT:
		break;

	case EP0_IDLE:
	{
        /* This branch has two functions:
		*	1: Set the next status of control transport;
		*	2: handle the setup packet.
		*/
		usb_device_request_t ctrl = pcd->setup_pkt->req;

		if (ctrl.bmRequestType & UE_DIR_IN)
		{
			ep0->is_in = 1;
			pcd->ep0state = EP0_IN_DATA_PHASE;
		}

		if (UGETW(ctrl.wLength) == 0)
		{
			ep0->is_in = 1;
			pcd->ep0state = EP0_IN_STATUS_PHASE;
		}

		setup((uint8_t *)&ctrl);
	}

	break;

	case EP0_IN_DATA_PHASE:
	//Control transport IN DATA PHASE
		deptsiz.d32 = readl(REG_DIEPTSIZ0);
		if (deptsiz.b.xfersize == 0) {

			ep0req->actual = ep0->xfer_count;

			pcd->ep0state = EP0_OUT_STATUS_PHASE;
			ep0->xfer_len = 0;
			ep0->total_len = 0;
			ep0->xfer_count = 0;
			ep0->is_in = 0;

			/* Prepare for more SETUP Packets */
			doepctl.d32 = readl(REG_DOEPCTL0);
			if (doepctl.b.epena) {
				return;
			}

			deptsiz.b.supcnt = 3;
			deptsiz.b.pktcnt = 1;
			deptsiz.b.xfersize = ep0->maxpacket;
			writel(deptsiz.d32,REG_DOEPTSIZ0);

			/** DOEPCTL0 Register write cnak will be set after setup interrupt */
			doepctl.b.epena = 1;
			doepctl.b.cnak = 1;
			writel(doepctl.d32, REG_DOEPCTL0);
		}
		break;

	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		ep0req->status = 0;
		ep0req->actual = ep0->xfer_count;
		ep0req->complete(ep0req);
		pcd->ep0state = EP0_IDLE;
		ep0->xfer_buff = 0;
		ep0->xfer_len = 0;
		ep0->is_in = 0;	/* OUT for next SETUP */
		break;
    default:
        break;
	}
}

/**
 * This function completes the request for the EP. If there are
 * additional requests for the EP in the queue they will be started.
 */
static void hiusb_complete_ep12(dwc_ep_t * ep)
{
	struct hiusb_request *req = 0;

	if (ep->is_in) {
		req = tx_req;
	} else {
		req = rx_req;
	}

	/* Complete the request */
	req->actual = ep->xfer_count;
	req->status = 0;
	req->complete(req);
	ep->xfer_len = 0;
	ep->xfer_count = 0;
}

void hiusb_ep_queue(dwc_ep_t *ep,struct hiusb_request *req_handle)
{
	/* EP0 Transfer */
	if (ep->num == 0) {
		depctl_data_t depctl;
		deptsiz0_data_t deptsiz;

		ep->xfer_buff = req_handle->buf;
		ep->xfer_len = req_handle->length;
		ep->total_len = req_handle->length;
		ep->xfer_count = 0;

		depctl.d32 = readl(REG_DIEPCTL0);
		if (depctl.b.epena)
			{
                HIUSB_PRINTF("Data is ok to tx, So return !\n");
                return;
                }

		depctl.d32 = readl(REG_DIEPCTL0);
		deptsiz.d32 = readl(REG_DIEPTSIZ0);
		deptsiz.b.xfersize = ep->xfer_len;//
		deptsiz.b.pktcnt = 1;
		writel(deptsiz.d32,REG_DIEPTSIZ0);

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
        /*
        For IN endpoints this bit indicates that the descriptor structure and data buffer
with data ready to transmit is setup.
        */
        depctl.b.epena = 1;

		writel(depctl.d32,REG_DIEPCTL0);

		/* Enable the Tx FIFO Empty Interrupt for this EP */
		if (ep->xfer_len > 0)
		{
			uint32_t fifoemptymsk = 0;
			fifoemptymsk |= 1 ;
            HIUSB_PRINTF("REG_DTKNQR4 = 0x%x\n",readl(REG_DTKNQR4));
			HIUSB_MODIFY_REG32((uint32_t *)REG_DTKNQR4,0, fifoemptymsk);
            HIUSB_PRINTF("REG_DTKNQR4 = 0x%x\n",readl(REG_DTKNQR4));
		}
	}		// non-ep0 endpoints
	else {
		/* Setup and start the Transfer */
		ep->xfer_buff = req_handle->buf;
		ep->xfer_len = 0;
		ep->xfer_count = 0;
		ep->total_len = req_handle->length;
		hiusb_ep12_start_transfer(ep);
	}
}




void hiusb_irq(void * _dev)
{
	gintsts_data_t gintsts= {.d32 = 0 };
	gusbcfg_data_t gusbcfg= {.d32 = 0 };
	deptsiz0_data_t doeptsize0 = {.d32 = 0 };
	depctl_data_t depctl = {.d32 = 0 };
	daint_data_t daintmsk = {.d32 = 0 };
	doepmsk_data_t doepmsk = {.d32 = 0 };
	diepmsk_data_t diepmsk = {.d32 = 0 };
	dcfg_data_t dcfg = {.d32 = 0 };
	dctl_data_t dctl = {.d32 = 0 };
	gintmsk_data_t gintmask = {.d32 = 0 };
	device_grxsts_data_t status= {.d32 = 0 };
	diepint_data_t diepint = {.d32 = 0 };
	doepint_data_t doepint = {.d32 = 0 };
	uint32_t ep_intr, empty_msk, msk;
	dwc_ep_t *ep;
	uint8_t  utmi8b;
	int speed;

    unsigned long flags = 0;
	spin_lock_irqsave(&dev->lock, flags);

	gintsts.d32 = 	(readl(REG_GINTSTS) &readl(REG_GINTMSK));

	/* reset interrupt */
	if (gintsts.b.usbreset) {

		HIUSB_PRINTF("USB RESET INT\n");
		daintmsk.b.inep0 = 1;
		daintmsk.b.outep0 = 1;
		doepmsk.b.setup = 1;
		doepmsk.b.xfercompl = 1;
		diepmsk.b.xfercompl = 1;
		writel(daintmsk.d32,REG_DAINTMSK);
		writel(doepmsk.d32,REG_DOEMSK);
		writel(diepmsk.d32,REG_DIEMSK);

		/* Reset Device Address */
		dcfg.d32 = readl(REG_DCFG);
		dcfg.b.devaddr = 0;
		writel(dcfg.d32,REG_DCFG);

		/* Clear interrupt */
		gintsts.b.usbreset = 1;
		writel(gintsts.d32,REG_GINTSTS);
	}


	/* speed enum interrupt*/
	if (gintsts.b.enumdone) {
		HIUSB_PRINTF("SPEED ENUM DONE\n");

		utmi8b = 9;

		/*ep0_activate*/
		depctl.d32 = readl(REG_DIEPCTL0);
		depctl.b.mps = DWC_DEP0CTL_MPS_64;// 0
		writel(depctl.d32,REG_DIEPCTL0);
		dctl.b.cgnpinnak = 1;
		HIUSB_MODIFY_REG32((uint32_t *)REG_DCTL, dctl.d32, dctl.d32);

		/*ep0 start out*/
		depctl.d32 = readl(REG_DOEPCTL0);
		if (depctl.b.epena) {
			return;
		}

		doeptsize0.b.supcnt = 3;
		doeptsize0.b.pktcnt = 1;
		doeptsize0.b.xfersize = 8 * 3;
		writel(doeptsize0.d32,REG_DOEPTSIZ0);

		/** DOEPCTL0 Register write cnak will be set after setup interrupt */
		depctl.b.epena = 1;
		writel(depctl.d32, REG_DOEPCTL0);

		pcd->ep0state = EP0_IDLE;

		/* UTMI+ interface */
		gusbcfg.d32 = readl(REG_GUSBCFG);
		speed = dwc_get_device_speed();

		if (speed == USB_SPEED_HIGH) {
			/* UTMI+ interface */
			gusbcfg.b.usbtrdtim = utmi8b;
		}
		else {
			/* Full or low speed */
			gusbcfg.b.usbtrdtim = 9;
		}
		writel(gusbcfg.d32,REG_GUSBCFG);

		/* Clear interrupt */
		gintsts.d32 = 0;
		gintsts.b.enumdone = 1;
		writel(gintsts.d32,REG_GINTSTS);
	}


	/* rxfifo full interrupt */
    //Indicates that there is at least one packet pending to be read from the RxFIFO
	if (gintsts.b.rxstsqlvl) {

		/* Disable the Rx Status Queue Level interrupt */
		gintmask.b.rxstsqlvl = 1;
		HIUSB_MODIFY_REG32((uint32_t *)REG_GINTMSK, gintmask.d32, 0);

		/* Get the Status from the top of the FIFO */
		status.d32 = readl(REG_GRXSTSP);

		/* Get pointer to EP structure */
		if (status.b.epnum == 0) {//ep0 Get the data
			ep = &pcd->ep0;
		} else
		{
			ep =  &pcd->out_ep;//EP2
		}

		switch (status.b.pktsts) {
			case DWC_STS_DATA_UPDT:// 0x2 :OUT Data Packet :data packet received
			if (status.b.bcnt && ep->xfer_buff)//ep->xfer_buff/** Number of bytes transferred. */
			{
				int i;
				int word_count = (status.b.bcnt + 3) / 4;
				volatile uint32_t *fifo = (uint32_t *)REG_DATAFIFO0;
				uint32_t *data_buff = (uint32_t *)ep->xfer_buff;

				for (i = 0; i < word_count; i++, data_buff++) {
					*data_buff = readl(fifo);//read data from RxFIFO
				}
				ep->xfer_count += status.b.bcnt;
				ep->xfer_buff += status.b.bcnt;
			}
			break;
			case DWC_DSTS_SETUP_UPDT:
                // 0x6 :SETUP Packet
				/* Pop 2 DWORDS off the receive data FIFO into memory */
				pcd->setup_pkt->d32[0] = readl(REG_DATAFIFO0);
				pcd->setup_pkt->d32[1] = readl(REG_DATAFIFO0);
				HIUSB_PRINTF("pcd->setup_pkt->d32[0] = 0x%x\n",pcd->setup_pkt->d32[0]);
                HIUSB_PRINTF("pcd->setup_pkt->d32[1] = 0x%x\n",pcd->setup_pkt->d32[1]);
				ep->xfer_count += status.b.bcnt;
			break;

			default:
			break;
		}

		/* Enable the Rx Status Queue Level interrupt */
		HIUSB_MODIFY_REG32((uint32_t *)REG_GINTMSK, 0, gintmask.d32);

		/* Clear interrupt */
		gintsts.d32 = 0;
		gintsts.b.rxstsqlvl = 1;
		writel(gintsts.d32,REG_GINTSTS);
	}


	/* in endpoint interrupt*/
	if (gintsts.b.inepint) {

		/* Read in the device interrupt bits */
		ep_intr = (readl(REG_DAINT) &readl(REG_DAINTMSK)& 0xffff);
		if (ep_intr & 0x1) {

			empty_msk =	readl(REG_DTKNQR4);
			msk = readl(REG_DIEMSK);
			msk |= (empty_msk & 0x1) << 7;
			diepint.d32 = readl(REG_DIEPINT0) & msk;

			/** IN EP Tx FIFO Empty Intr */
			if (diepint.b.emptyintr) {
				hiusb_write_empty_tx_fifo(0);//Reply data to Host
				CLEAR_IN_EP_INTR(0, emptyintr);
			}

			/* Transfer complete */
			if (diepint.b.xfercompl)
			{
				HIUSB_PRINTF("Transfer complete\n");
				/* Disable the Tx FIFO Empty Interrupt for this EP */
				HIUSB_MODIFY_REG32((uint32_t *)REG_DTKNQR4,0x01, 0);
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(0, xfercompl);
				hiusb_complete_ep0();
			}
		}

		if (ep_intr & 0x2) {

			empty_msk =	readl(REG_DTKNQR4);
			msk = readl(REG_DIEMSK);
			msk |= ((empty_msk >>1) & 0x1) << 7;
			diepint.d32 = readl(REG_DIEPINT1) & msk;

			/** IN EP Tx FIFO Empty Intr */
			if (diepint.b.emptyintr) {
				HIUSB_PRINTF("inep1_emptyintr\n");
				hiusb_write_empty_tx_fifo(1);
				CLEAR_IN_EP_INTR(1, emptyintr);
			}

			/* Transfer complete */
			if (diepint.b.xfercompl)
			{
				HIUSB_PRINTF("inep1_xfercompl\n");
				/* Disable the Tx FIFO Empty Interrupt for this EP */
				HIUSB_MODIFY_REG32((uint32_t *)REG_DTKNQR4,0x02, 0);
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(1, xfercompl);
				hiusb_complete_ep12(&pcd->in_ep);
				if(diepint.b.nak)
				{
				CLEAR_IN_EP_INTR(1, nak);
				}
			}
		}
	}


	/* out endpoint interrupt*/
	if (gintsts.b.outepintr) {

		/* Read in the device interrupt bits */
		ep_intr =  ((readl(REG_DAINT)&readl(REG_DAINTMSK) & 0xffff0000) >> 16);

		/* out0 intr */
		if (ep_intr & 0x1) {
			doepint.d32 = (readl(REG_DOEPINT0) &readl(REG_DOEMSK));

			/* Transfer complete */
            /*For OUT endpoint this field indicates that the requested data from the
internal FIFO is moved to external system memory. */
			if (doepint.b.xfercompl) {
                HIUSB_PRINTF("Transfer complete\n");
				/* Clear the bit in DOEPINTn for this interrupt */
				CLEAR_OUT_EP_INTR(0,xfercompl);

				doepint.d32 = readl(REG_DOEPINT0);
				if (pcd->ep0state == EP0_IDLE && doepint.b.sr) {
                    HIUSB_PRINTF("EP0_IDLE\n");
					CLEAR_OUT_EP_INTR(0,sr);
				}
				else{
					hiusb_complete_ep0();
				}
			}
/*Applies to control OUT endpoints only.
*Indicates that the SETUP phase for the control endpoint is complete and
*no more back-to-back SETUP packets were received for the current
*control transfer. On this interrupt, the application can decode the received
*SETUP data packet.
*/
/*
*Assertion of the DOEPINTn.SETUP Packet interrupt indicates that a valid SETUP packet has been
*transferred to the application and the data stage has started.
*/
			/* Setup Phase Done (contorl EPs) */
			if (doepint.b.setup) {
                HIUSB_PRINTF("Setup Phase Done\n");
				CLEAR_OUT_EP_INTR(0,setup);
				hiusb_complete_ep0();
			}
		}

		/* out2 inter */
		if (ep_intr & 0x4) {
            //We received data from endpoint2,OUT Endpoint.
			doepint.d32 = (readl(REG_DOEPINT2) &readl(REG_DOEMSK));

			if (doepint.b.xfercompl) {
				CLEAR_OUT_EP_INTR(2,xfercompl);
				hiusb_complete_ep12(&pcd->out_ep);
			}
		}
	}

    spin_unlock_irqrestore(&dev->lock, flags);
}

/**
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
void hiusb_core_reset(void)
{
	volatile grstctl_t greset = {.d32 = 0 };
	int count = 0;

	/* Wait for AHB master IDLE state. */
	do {
		udelay(10);
		greset.d32 = readl(REG_GRSTCTL);
		if (++count > 100000) {
			HIUSB_PRINTF("%s() HANG! AHB Idle GRSTCTL=%0x\n", __func__,
				 greset.d32);
			return;
		}
	} while (greset.b.ahbidle == 0);

	/* Core Soft Reset */
	count = 0;
	greset.b.csftrst = 1;
	writel( greset.d32,REG_GRSTCTL);
	do {
		greset.d32 = readl(REG_GRSTCTL);
		if (++count > 10000) {
			HIUSB_PRINTF("%s() HANG! Soft Reset GRSTCTL=%0x\n",
				 __func__, greset.d32);
			break;
		}
		udelay(1);
	}
	while (greset.b.csftrst == 1);

	udelay(100);
}
/*
 * Flush a Tx FIFO.
 * @param num Tx FIFO to flush.
 */
void hiusb_flush_tx_fifo(const int num)
{
	volatile grstctl_t greset = {.d32 = 0 };
	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	writel( greset.d32,REG_GRSTCTL);
	udelay(200);
}

/*
 * Flush Rx FIFO.
 */
void hiusb_flush_rx_fifo(void)
{
	volatile grstctl_t greset = {.d32 = 0 };
	greset.b.rxfflsh = 1;
	writel(greset.d32,REG_GRSTCTL);
	udelay(200);
}
static void hiusb_init_ep(dwc_ep_t * pcd_ep,uint32_t is_in,
	uint32_t ep_num,uint32_t tx_fifo_num,uint32_t type,uint32_t maxpacket)
{
	pcd_ep->is_in = is_in;
	pcd_ep->num = ep_num;
	pcd_ep->tx_fifo_num = tx_fifo_num;
	pcd_ep->type = type;
	pcd_ep->maxpacket = maxpacket;
	pcd_ep->xfer_buff = 0;
	pcd_ep->xfer_len = 0;
	pcd_ep->xfer_count = 0;
	pcd_ep->total_len = 0;
}
void ep0_complete( struct hiusb_request *req)
{
    //EP0 control transport finished
    HIUSB_PRINTF("\n&&&&&&&&&&&&&&&&&&&&&&&&Control Trans Finished&&&&&&&&&&&&&&&&&&&&&&&&\n");
}

void hiusb_init(void)
{
	gintmsk_data_t intr_mask = {.d32 = 0 };
	gahbcfg_data_t ahbcfg = {.d32 = 0 };
	gusbcfg_data_t gusbcfg = {.d32 = 0 };
	dcfg_data_t dcfg = {.d32 = 0 };
	grstctl_t resetctl = {.d32 = 0 };
	dctl_data_t dctl = {.d32 = 0 };
	diepmsk_data_t msk = {.d32 = 0 };
	depctl_data_t depctl= {.d32 = 0 };
	struct hiusb_request *usb_req;

	/* step 1: clock reset init *///PHY
    //usb2_phy_init(); This step has been done in board_usb_init() function.

	/*step 2: Disable interrupts */
	ahbcfg.b.glblintrmsk = 1;
	HIUSB_MODIFY_REG32((uint32_t *)REG_GAHBCFG, ahbcfg.d32, 0);

	/* setp 3 : force to device mode */
	gusbcfg.d32 =  readl(REG_GUSBCFG);
	gusbcfg.b.force_dev_mode = 1;
	gusbcfg.b.ulpi_utmi_sel = 0;    /* UTMI+ interface  16bit todo :asic need modify*/
	gusbcfg.b.phyif = 0;		   /* 8bit */
	writel(gusbcfg.d32,REG_GUSBCFG);

	/* setp 4 :  Reset after setting the PHY parameters */
	hiusb_core_reset();

	ahbcfg.b.nptxfemplvl_txfemplvl =	DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;//0
	ahbcfg.b.ptxfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;// 0 reserved for furture use
	ahbcfg.b.dmaenable = 0;
	writel(ahbcfg.d32,REG_GAHBCFG);

	/* setp 5 :  Restart the Phy Clock */
	writel(0,REG_PCGCCTL);

	/* setp 6 : Device configuration register */
	dcfg.d32 = readl(REG_DCFG);
	dcfg.b.descdma =0;
	dcfg.b.devspd = 0;
	dcfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;//0
	writel(dcfg.d32,REG_DCFG);

	/* setp 7 :Flush the FIFOs */
	hiusb_flush_tx_fifo(0x10);	/* all Tx FIFOs */
	hiusb_flush_rx_fifo();

	/* setp 8 :Flush the Learning Queue. */
	resetctl.b.intknqflsh = 1;
	writel(resetctl.d32,REG_GRSTCTL);


	/* setp 9 :Clear all pending Device Interrupts */
	writel(0,REG_DIEMSK);
	writel(0,REG_DOEMSK);
	writel(0xFFFFFFFF,REG_DAINT);
	writel(0,REG_DAINTMSK);

	/* setp 10 :Init endpoint 0(in/out) 1(in) 2(out) */
	depctl.d32 = readl(REG_DIEPCTL0);
	depctl.d32 = 0;
	depctl.b.snak = 1;
	writel(depctl.d32,REG_DIEPCTL0);
	writel(0,REG_DIEPTSIZ0);
	writel(0,REG_DIEPDMA0);
	writel(0xff,REG_DIEPINT0);

	depctl.d32 = readl(REG_DIEPCTL1);
	depctl.d32 = 0;
	depctl.b.snak = 1;
	writel(depctl.d32,REG_DIEPCTL1);
	writel(0,REG_DIEPTSIZ1);
	writel(0,REG_DIEPDMA1);
	writel(0xff,REG_DIEPINT1);

	depctl.d32 = readl(REG_DOEPCTL0);
	depctl.d32 = 0;
	depctl.b.snak = 1;
	writel(depctl.d32,REG_DOEPCTL0);
	writel(0,REG_DOEPTSIZ0);
	writel(0,REG_DOEPDMA0);
	writel(0xff,REG_DOEPINT0);

	depctl.d32 = readl(REG_DOEPCTL2);
	depctl.d32 = 0;
	depctl.b.snak = 1;
	writel(depctl.d32,REG_DOEPCTL2);
	writel(0,REG_DOEPTSIZ2);
	writel(0,REG_DOEPDMA2);
	writel(0xff,REG_DOEPINT2);

	/* setp 11 :enable interrupt */
	writel(0,REG_GINTMSK);
	writel(0xFFFFFFFF,REG_GINTSTS);
	writel(0xFFFFFFFF,REG_GOTGINT);
	intr_mask.b.rxstsqlvl = 1;
	intr_mask.b.usbreset = 1;
	intr_mask.b.enumdone = 1;
	intr_mask.b.inepintr = 1;
	intr_mask.b.outepintr = 1;
	HIUSB_MODIFY_REG32((uint32_t *)REG_GINTMSK, intr_mask.d32, intr_mask.d32);
	msk.b.txfifoundrn = 1;
	HIUSB_MODIFY_REG32((uint32_t *)REG_DIEMSK,msk.d32, msk.d32);

	/* setp 12 : soft disconnect clear */
	dctl.d32 = readl(REG_DCTL);
	dctl.b.sftdiscon = 0;
	writel(dctl.d32,REG_DCTL);

	/* setp 13 : init data struct */
	pcd = malloc(sizeof(dwc_otg_pcd_t));
	pcd->setup_pkt = malloc(sizeof(*pcd->setup_pkt) * 5);

	hiusb_init_ep(&pcd->ep0, 0, 0,0,0,MAX_EP0_SIZE);
	hiusb_init_ep( &pcd->in_ep, 1 /* IN */ , 1, 1,UE_BULK,MAX_PACKET_SIZE);
	hiusb_init_ep( &pcd->out_ep, 0 /* OUT */ , 2,0,UE_BULK,MAX_PACKET_SIZE);
	pcd->ep0state = EP0_DISCONNECT;
	ep0req =  malloc(sizeof(*usb_req));
	memset(ep0req, 0, sizeof(*usb_req));
	ep0req->buf = malloc(512);
	ep0req->complete = ep0_complete;

}

int usb_gadget_handle_interrupts(int index)
{
    hiusb_irq((void *)hi_controller);

	return 0;
}


/*
  Register entry point for the peripheral controller driver.
*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{

	int retval = 0;
	struct hi_udc *dev = hi_controller;

	if (!driver
	    || (driver->speed != USB_SPEED_FULL
		&& driver->speed != USB_SPEED_HIGH)
	    || !driver->bind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	dev->driver = driver;
	retval = driver->bind(&dev->gadget);//Call composite_bind() function to bind .

	if (retval) {
        HIUSB_PRINTF("%s: bind to driver --> error %d\n",
			    dev->gadget.name, retval);

		dev->driver = 0;
		return retval;
	}

	hiusb_init();
	return 0;
}

/*
 * Unregister entry point for the peripheral controller driver.
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{

	struct hi_udc *dev = hi_controller;
	unsigned long flags = 0;
    struct usb_composite_dev    *cdev;

	if (!dev)
		return -ENODEV;
	if (!driver || (driver != dev->driver))
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
    pcd->ep0state = EP0_DISCONNECT;
    hiusb_flush_tx_fifo(0x10);  /* all Tx FIFOs */
    hiusb_flush_rx_fifo();
	dev->driver = 0;
    cdev = dev->gadget.dev.driver_data;
    cdev->config =  NULL;
    hiusb_init_ep(&pcd->ep0, 0, 0,0,0,MAX_EP0_SIZE);
	hiusb_init_ep( &pcd->in_ep, 1 /* IN */ , 1, 1,UE_BULK,MAX_PACKET_SIZE);
	hiusb_init_ep( &pcd->out_ep, 0 /* OUT */ , 2,0,UE_BULK,MAX_PACKET_SIZE);

	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);

	return 0;
}

static const char driver_name[] = "hisilicon-udc";
static const char ep0name[] = "ep0-control";

/* Max packet size*/
//static unsigned int ep0_fifo_size = 64;
//static unsigned int ep_fifo_size =  512;
//static unsigned int ep_fifo_size2 = 1024;
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define EP_FIFO_SIZE2		1024

static int dwc2_ep_enable(struct usb_ep *_ep,
			 const struct usb_endpoint_descriptor *desc)
{
    return 0;
}
static int dwc2_ep_disable(struct usb_ep *ep)
{
    return 0;
}

struct dwc2_request {
	struct usb_request req;
	struct list_head queue;
};

static struct usb_request *dwc2_alloc_request(struct usb_ep *ep,
					     gfp_t gfp_flags)
{
    struct dwc2_request *req;
    req = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*req));
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}
static void dwc2_free_request(struct usb_ep *ep, struct usb_request * req)
{

}
#define ep_index(EP) ((EP)->bEndpointAddress&0xF)
static int dwc2_queue(struct usb_ep *_ep, struct usb_request *req, gfp_t gfp_flags)
{
    struct dwc2_ep *ep;
    u32 ep_num;
    ep = container_of(_ep, struct dwc2_ep, ep);
	ep_num = ep_index(ep);
    if(1==ep_num)
    {
        fastboot_tx_handler_complete = NULL;
        tx_req->buf = req->buf;
        tx_req->length = req->length;
        ((char*)tx_req->buf)[tx_req->length] = 0;
        if((req->length == strlen("OKAY")) && (!strncmp("OKAY",req->buf ,req->length))
            &&((strncmp("continue",cmdbuf, strlen("continue")))
                || (strncmp("boot",cmdbuf, strlen("boot")))))
        {
            /*In this situation,we must set the complete function here,therefor,
            we can call the corresponding complete function when the Tx transfor is completed.*/
            fastboot_tx_handler_complete = req->complete;//

        }

        tx_status(tx_req->buf, USB_STATUS_DATA);

    }
    return 0;

}
static int dwc2_dequeue(struct usb_ep *ep, struct usb_request *req)
{
    return 0;

}
static int dwc2_fifo_status(struct usb_ep *ep)
{
    return 0;

}
static void dwc2_fifo_flush(struct usb_ep *ep)
{

}
static int dwc2_udc_set_halt(struct usb_ep *_ep, int value)
{
    return 0;
}

static struct usb_ep_ops hi_ep_ops = {
	.enable = dwc2_ep_enable,
	.disable = dwc2_ep_disable,

	.alloc_request = dwc2_alloc_request,
	.free_request = dwc2_free_request,

	.queue = dwc2_queue,
	.dequeue = dwc2_dequeue,

	.set_halt = dwc2_udc_set_halt,
	.fifo_status = dwc2_fifo_status,
	.fifo_flush = dwc2_fifo_flush,
};



static const struct usb_gadget_ops hi_udc_ops = {
	/* current versions must always be self-powered */
};

static struct hi_udc memory = {
	.usb_address = 0,
	.gadget = {
		.ops = &hi_udc_ops,
		.ep0 = &memory.ep[0].ep,
		.name = driver_name,
	},

	/* control endpoint */
	.ep[0] = {
		.ep = {
			.name = ep0name,
			.ops = &hi_ep_ops,
			.maxpacket = EP0_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 0,
		.bmAttributes = 0,

		.ep_type = ep_control,
	},

	/* first group of endpoints */
	.ep[1] = {
		.ep = {
			.name = "ep1in-bulk",
			.ops = &hi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = USB_DIR_IN | 1,
		.fifo_num = 1,
	},

	.ep[2] = {
		.ep = {
			.name = "ep2out-bulk",
			.ops = &hi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = USB_DIR_OUT | 2,
		.fifo_num = 2,
	},
};

/*
 *	probe - binds to the platform device
 */

int poplar_udc_probe(struct dwc2_plat_otg_data *pdata)
{
    struct hi_udc *dev = &memory;
	int retval = 0;
    dev->pdata = pdata;
    unsigned int i;
    dev->gadget.is_dualspeed = 1;	/* Hack only*/
	dev->gadget.is_otg = 0;
	dev->gadget.is_a_peripheral = 0;
	dev->gadget.b_hnp_enable = 0;
	dev->gadget.a_hnp_support = 0;
	dev->gadget.a_alt_hnp_support = 0;

    hi_controller = dev;

    INIT_LIST_HEAD(&dev->gadget.ep_list);
    INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
    /* basic endpoint records init */
    for (i = 0; i < DWC2_MAX_ENDPOINTS; i++) {
        struct dwc2_ep *ep = &dev->ep[i];

        if (i != 0)
            list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

        ep->desc = 0;
        ep->stopped = 0;
        INIT_LIST_HEAD(&ep->queue);
        ep->pio_irqs = 0;
    }

    return retval;
}
