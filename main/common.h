
#pragma once
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "hal/usbh_ll.h"
#include "hcd.h"
#include "esp_log.h"
#include "hcd.h"

#define USB_WEAK        __attribute__((weak))


#define EVENT_QUEUE_LEN         10
#define NUM_XFER_REQS           3
#define XFER_DATA_MAX_LEN       256     //Just assume that will only IN/OUT 256 bytes for now
#define PORT_NUM                1
#define DEVICE_ADDR             1


hcd_pipe_handle_t ctrl_pipe_hdl;
hcd_xfer_req_handle_t req_hdls[NUM_XFER_REQS];
uint8_t *data_buffers[NUM_XFER_REQS];
usb_irp_t *irps[NUM_XFER_REQS];

#define USB_CTRL_REQ_INIT_GET_STRING(ctrl_req_ptr, lang, desc_index, len) ({ \
    (ctrl_req_ptr)->bRequestType = USB_B_REQUEST_TYPE_DIR_IN | USB_B_REQUEST_TYPE_TYPE_STANDARD | USB_B_REQUEST_TYPE_RECIP_DEVICE;   \
    (ctrl_req_ptr)->bRequest = USB_B_REQUEST_GET_DESCRIPTOR;   \
    (ctrl_req_ptr)->wValue = (USB_W_VALUE_DT_STRING << 8) | ((desc_index) & 0xFF); \
    (ctrl_req_ptr)->wIndex = (lang);    \
    (ctrl_req_ptr)->wLength = (len);  \
})

typedef struct {
    hcd_port_handle_t port_hdl;
    hcd_port_event_t port_event;
} port_event_msg_t;


QueueHandle_t pipe_evt_queue;
QueueHandle_t port_evt_queue;
uint8_t bMaxPacketSize0;
uint8_t conf_num;
hcd_pipe_handle_t ctrl_pipe_hdl;
hcd_port_handle_t port_hdl;
bool isConnected;
bool recoveryPort;

void pipe_event_task(void* p);
void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len);

void wait_for_connection();
void on_connected();
USB_WEAK void cdc_create_pipe(usb_desc_ep_t* ep);
void port_event_task(void* p);
void recovery_port();
bool port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr);
void phy_force_conn_state(bool connected, TickType_t delay_ticks);


void alloc_pipe_and_xfer_reqs_ctrl(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     hcd_xfer_req_handle_t *req_hdls,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers);


