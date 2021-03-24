
#pragma once
#include "hcd.h"

#define USB_CTRL_REQ_INIT_GET_STRING(ctrl_req_ptr, lang, desc_index, len) ({ \
    (ctrl_req_ptr)->bRequestType = USB_B_REQUEST_TYPE_DIR_IN | USB_B_REQUEST_TYPE_TYPE_STANDARD | USB_B_REQUEST_TYPE_RECIP_DEVICE;   \
    (ctrl_req_ptr)->bRequest = USB_B_REQUEST_GET_DESCRIPTOR;   \
    (ctrl_req_ptr)->wValue = (USB_W_VALUE_DT_STRING << 8) | ((desc_index) & 0xFF); \
    (ctrl_req_ptr)->wIndex = (lang);    \
    (ctrl_req_ptr)->wLength = (len);  \
})

typedef struct {
    hcd_pipe_handle_t pipe_hdl;
    hcd_pipe_event_t pipe_event;
} pipe_event_msg_t;

typedef void (*ctrl_pipe_cb_t)(pipe_event_msg_t msg, hcd_xfer_req_handle_t req_hdl);
void register_ctrl_pipe_callback(ctrl_pipe_cb_t);

void ctrl_pipe_event_task(void* p);
void alloc_pipe_and_xfer_reqs_ctrl(hcd_port_handle_t port_hdl, hcd_pipe_handle_t* handle);
void free_pipe_and_xfer_reqs_ctrl(hcd_pipe_handle_t pipe_hdl);


void xfer_get_device_desc();
void xfer_set_address(uint8_t addr);
void xfer_get_current_config();
void xfer_set_configuration(uint8_t);
void xfer_get_desc();
void xfer_get_string(uint8_t);
