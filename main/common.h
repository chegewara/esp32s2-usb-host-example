
#include "hcd.h"


#define EVENT_QUEUE_LEN         10
#define NUM_XFER_REQS           5
#define XFER_DATA_MAX_LEN       256     //Just assume that will only IN/OUT 256 bytes for now
#define PORT_NUM                1
#define DEVICE_ADDR             2

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

typedef struct {
    hcd_pipe_handle_t pipe_hdl;
    hcd_pipe_event_t pipe_event;
} pipe_event_msg_t;


QueueHandle_t pipe_evt_queue;
QueueHandle_t port_evt_queue;
uint8_t bMaxPacketSize0;
uint8_t conf_num;
bool pipe_done;
hcd_pipe_handle_t ctrl_pipe_hdl;


void pipe_event_task(void* p);
void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len);

void alloc_pipe_and_xfer_reqs_ctrl(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     hcd_xfer_req_handle_t *req_hdls,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers);

void alloc_pipe_and_xfer_reqs_bulk(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     hcd_xfer_req_handle_t *req_hdls,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers);

void free_pipe_and_xfer_reqs(hcd_pipe_handle_t pipe_hdl,
                                    hcd_xfer_req_handle_t *req_hdls,
                                    uint8_t **data_buffers,
                                    usb_irp_t **irps,
                                    int num_xfers);



