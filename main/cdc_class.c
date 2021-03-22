#include <stdio.h>
#include <string.h>
#include "common.h"
#include "cdc_class.h"
#include "pipes.h"

#define MAX_NUM_ENDP    3
#define EP1             0
#define EP2             1
#define EP3             2


hcd_pipe_handle_t cdc_ep_pipe_hdl[MAX_NUM_ENDP];
hcd_xfer_req_handle_t cdc_ep_req_hdls[MAX_NUM_ENDP][1];
uint8_t *cdc_data_buffers[MAX_NUM_ENDP][1];
usb_irp_t *cdc_ep_irps[MAX_NUM_ENDP][1];
usb_desc_ep_t endpoints[MAX_NUM_ENDP];

static bool cdc_pipe_callback(hcd_pipe_handle_t pipe_hdl, hcd_pipe_event_t pipe_event, void *user_arg, bool in_isr)
{
    QueueHandle_t pipe_evt_queue = (QueueHandle_t)user_arg;
    pipe_event_msg_t msg = {
        .pipe_hdl = pipe_hdl,
        .pipe_event = pipe_event,
    };
    if (in_isr) {
        BaseType_t xTaskWoken = pdFALSE;
        xQueueSendFromISR(pipe_evt_queue, &msg, &xTaskWoken);
        return (xTaskWoken == pdTRUE);
    } else {
        xQueueSend(pipe_evt_queue, &msg, portMAX_DELAY);
        return false;
    }
}

static void alloc_pipe_and_xfer_reqs_cdc(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     hcd_xfer_req_handle_t *req_hdls,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers,
                                     usb_desc_ep_t* ep)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){}

    //Create default pipe
    // printf("Creating default pipe\n");
    hcd_pipe_config_t config = {
        .callback = cdc_pipe_callback,
        .callback_arg = (void *)pipe_evt_queue,
        .context = NULL,
        .ep_desc = ep,
        .dev_addr = DEVICE_ADDR,
        .dev_speed = port_speed,
    };
    if(ESP_OK == hcd_pipe_alloc(port_hdl, &config, pipe_hdl)) {}
    if(NULL == pipe_hdl) {
        ESP_LOGE("", "NULL == pipe_hdl");
    }
    //Create transfer requests (and other required objects such as IRPs and data buffers)
    printf("Creating transfer requests\n");
    for (int i = 0; i < num_xfers; i++) {
        //Allocate transfer request object
        req_hdls[i] = hcd_xfer_req_alloc();
        if(NULL == req_hdls[i]) ESP_LOGE("", "err 4");
        //Allocate data buffers
        data_buffers[i] = heap_caps_malloc(sizeof(usb_ctrl_req_t) + XFER_DATA_MAX_LEN, MALLOC_CAP_DMA);
        if(NULL == data_buffers[i]) ESP_LOGE("", "err 5");
        //Allocate IRP object
        irps[i] = heap_caps_malloc(sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
        if(NULL == irps[i]) ESP_LOGE("", "err 6");
        //Set the transfer request's target
        hcd_xfer_req_set_target(req_hdls[i], *pipe_hdl, irps[i], (void*)i);
    }
}

void cdc_create_pipe(usb_desc_ep_t* ep)
{
    if(USB_DESC_EP_GET_XFERTYPE(ep) == USB_XFER_TYPE_INTR){
        memcpy(&endpoints[EP1], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, pipe_evt_queue, &cdc_ep_pipe_hdl[EP1], &cdc_ep_req_hdls[EP1][0], &cdc_data_buffers[EP1][0], &cdc_ep_irps[EP1][0], 1, ep);
    } else if(USB_DESC_EP_GET_XFERTYPE(ep) == USB_XFER_TYPE_BULK && USB_DESC_EP_GET_EP_DIR(ep)){
        memcpy(&endpoints[EP2], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, pipe_evt_queue, &cdc_ep_pipe_hdl[EP2], &cdc_ep_req_hdls[EP2][0], &cdc_data_buffers[EP2][0], &cdc_ep_irps[EP2][0], 1, ep);                
    } else {
        memcpy(&endpoints[EP3], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, pipe_evt_queue, &cdc_ep_pipe_hdl[EP3], &cdc_ep_req_hdls[EP3][0], &cdc_data_buffers[EP3][0], &cdc_ep_irps[EP3][0], 1, ep);                
    }
}

void delete_pipes()
{
    for (size_t i = 0; i < MAX_NUM_ENDP; i++)
    {
        if(cdc_ep_pipe_hdl[i] == NULL) continue;
        if (HCD_PIPE_STATE_INVALID == hcd_pipe_get_state(cdc_ep_pipe_hdl[i]))
        {                
            ESP_LOGD("", "pipe state: %d", hcd_pipe_get_state(cdc_ep_pipe_hdl[i]));
            free_pipe_and_xfer_reqs( cdc_ep_pipe_hdl[i], &cdc_ep_req_hdls[i][0], &cdc_data_buffers[i][0], &cdc_ep_irps[i][0], 1);
            cdc_ep_pipe_hdl[i] = NULL;
        }
    }
}

void xfer_set_line_coding(uint32_t bitrate)
{
    USB_CTRL_REQ_CDC_SET_LINE_CODING((cdc_ctrl_line_t *) data_buffers[0], 0, bitrate, 0, 0, 5);

    irps[0]->num_bytes = bMaxPacketSize0;
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 7; // bytes in data packet (excluding 8 control bytes)


    esp_err_t err;
    if(ESP_OK == (err = hcd_xfer_req_enqueue(req_hdls[0]))) {
        ESP_LOGI("xfer", "set line codding");
    } else {
        ESP_LOGI("xfer", "set line codding: 0x%x", err);
    }
}

void xfer_get_line_coding()
{
    USB_CTRL_REQ_CDC_GET_LINE_CODING((usb_ctrl_req_t *) data_buffers[0], 0);

    irps[0]->num_bytes = bMaxPacketSize0;
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 7;

    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("xfer", "Get line coding");
    }
}

void xfer_set_control_line(bool dtr, bool rts)
{
    USB_CTRL_REQ_CDC_SET_CONTROL_LINE_STATE((usb_ctrl_req_t *) data_buffers[0], 0, dtr, rts);

    irps[0]->num_bytes = bMaxPacketSize0;
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 0;
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("xfer", "Set control line");
    }
}


// ENDPOINTS
void xfer_intr_data()
{
    cdc_ep_irps[EP1][0]->num_bytes = 8;    //1 worst case MPS
    cdc_ep_irps[EP1][0]->data_buffer = cdc_data_buffers[EP1][0];
    cdc_ep_irps[EP1][0]->num_iso_packets = 0;
    cdc_ep_irps[EP1][0]->num_bytes = 8;

    esp_err_t err;
    if(ESP_OK == (err = hcd_xfer_req_enqueue(cdc_ep_req_hdls[EP1][0]))) {
        ESP_LOGD("", "INT ");
    } else {
        ESP_LOGE("", "INT err: 0x%02x", err);
    }
}

void xfer_in_data()
{
    ESP_LOGD("", "EP: 0x%02x", USB_DESC_EP_GET_ADDRESS(&endpoints[EP2]));
    memset(cdc_data_buffers[EP2][0], 0x0, 64);
    cdc_ep_irps[EP2][0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    cdc_ep_irps[EP2][0]->data_buffer = cdc_data_buffers[EP2][0];
    cdc_ep_irps[EP2][0]->num_iso_packets = 0;
    cdc_ep_irps[EP2][0]->num_bytes = 64;

    if(ESP_OK == hcd_xfer_req_enqueue(cdc_ep_req_hdls[EP2][0])) {
        ESP_LOGD("", "BULK %s, dir: %d", "IN", USB_DESC_EP_GET_EP_DIR(&endpoints[EP2]));
    }

}

void xfer_out_data(uint8_t* data, size_t len)
{
    ESP_LOGD("", "EP: 0x%02x", USB_DESC_EP_GET_ADDRESS(&endpoints[EP3]));
    memcpy(cdc_data_buffers[EP3][0], data, len);
    cdc_ep_irps[EP3][0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    cdc_ep_irps[EP3][0]->data_buffer = cdc_data_buffers[2][0];
    cdc_ep_irps[EP3][0]->num_iso_packets = 0;
    cdc_ep_irps[EP3][0]->num_bytes = len;

    if(ESP_OK == hcd_xfer_req_enqueue(cdc_ep_req_hdls[EP3][0])) {
        ESP_LOGD("", "BULK %s, dir: %d", "OUT", USB_DESC_EP_GET_EP_DIR(&endpoints[EP3]));
    }
}
