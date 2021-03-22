
#include <stdio.h>
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
#include "common.h"
#include "pipes.h"

static bool ctrl_pipe_callback(hcd_pipe_handle_t pipe_hdl, hcd_pipe_event_t pipe_event, void *user_arg, bool in_isr)
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

void free_pipe_and_xfer_reqs(hcd_pipe_handle_t pipe_hdl,
                                    hcd_xfer_req_handle_t *req_hdls,
                                    uint8_t **data_buffers,
                                    usb_irp_t **irps,
                                    int num_xfers)
{
    //Dequeue transfer requests
    do{
        hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(pipe_hdl);
        if(req_hdl == NULL) break;
    }while(1);

    ESP_LOGD("", "Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    for (int i = 0; i < num_xfers; i++) {
        heap_caps_free(irps[i]);
        heap_caps_free(data_buffers[i]);
        hcd_xfer_req_free(req_hdls[i]);
    }
    ESP_LOGD("", "Freeing pipe\n");
    //Delete the pipe
    if(ESP_OK != hcd_pipe_free(pipe_hdl)) {
        ESP_LOGE("", "err to free pipes");
    }
}

void alloc_pipe_and_xfer_reqs_ctrl(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     hcd_xfer_req_handle_t *req_hdls,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){}

    //Create default pipe
    // printf("Creating default pipe\n");
    hcd_pipe_config_t config = {
        .callback = ctrl_pipe_callback,
        .callback_arg = (void *)pipe_evt_queue,
        .context = NULL,
        .ep_desc = USB_XFER_TYPE_CTRL,    //NULL EP descriptor to create a default pipe
        .dev_addr = 0,
        .dev_speed = port_speed,
    };
    if(ESP_OK == hcd_pipe_alloc(port_hdl, &config, pipe_hdl)) {}
    if(NULL == pipe_hdl) {
        ESP_LOGE("", "NULL == pipe_hdl");
    }
    //Create transfer requests (and other required objects such as IRPs and data buffers)
    // printf("Creating transfer requests\n");
    for (int i = 0; i < num_xfers; i++) {
        //Allocate transfer request object
        req_hdls[i] = hcd_xfer_req_alloc();
        if(NULL == req_hdls[i]) ESP_LOGE("", "err 4");
        //Allocate data buffers
        data_buffers[i] = heap_caps_calloc(1, sizeof(usb_ctrl_req_t) + XFER_DATA_MAX_LEN, MALLOC_CAP_DMA);
        if(NULL == data_buffers[i]) ESP_LOGE("", "err 5");
        //Allocate IRP object
        irps[i] = heap_caps_malloc(sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
        if(NULL == irps[i]) ESP_LOGE("", "err 6");
        //Set the transfer request's target
        hcd_xfer_req_set_target(req_hdls[i], *pipe_hdl, irps[i], (void*)i);
    }
}

void pipe_event_task(void* p)
{
    printf("start pipe event task\n");
    pipe_event_msg_t msg;
    while(1){
        xQueueReceive(pipe_evt_queue, &msg, portMAX_DELAY);
        hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(msg.pipe_hdl);
        hcd_pipe_handle_t pipe_hdl;
        usb_irp_t *irp;
        void *context;
        if(req_hdl == NULL) continue;
        hcd_xfer_req_get_target(req_hdl, &pipe_hdl, &irp, &context);
        // ESP_LOGW("", "\t-> Pipe [%d] event: %d\n", (uint8_t)context, msg.pipe_event);

        switch (msg.pipe_event)
        {
            case HCD_PIPE_EVENT_NONE:
                break;

            case HCD_PIPE_EVENT_XFER_REQ_DONE:
                // ESP_LOGI("Pipe: ", "XFER status: %d, num bytes: %d, actual bytes: %d", irp->status, irp->num_bytes, irp->actual_num_bytes);
                if(0 == irp->num_bytes) break;
                if(msg.pipe_hdl == ctrl_pipe_hdl){
                    ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", irp->data_buffer, 8, ESP_LOG_DEBUG);
                    ESP_LOG_BUFFER_HEX_LEVEL("Actual data", irp->data_buffer + 8, irp->actual_num_bytes, ESP_LOG_DEBUG);

                    usb_ctrl_req_t* ctrl = irp->data_buffer;
                    if(ctrl->bRequest == USB_B_REQUEST_GET_DESCRIPTOR){
                        parse_cfg_descriptor((irp->data_buffer + 8), irp->status, irp->actual_num_bytes);
                    } else if(ctrl->bRequest == USB_B_REQUEST_GET_CONFIGURATION) {
                        ESP_LOGW("", "current configuration: %d", irp->data_buffer[9]);
                    } else {
                        // ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", irp->data_buffer, 8, ESP_LOG_INFO);
                        // ESP_LOG_BUFFER_HEX_LEVEL("Actual data", irp->data_buffer + 8, irp->actual_num_bytes, ESP_LOG_INFO);
                    }
                } else {
                    ESP_LOGI("", "BULK msg done: %s", (irp->data_buffer));
                }
                break;

            case HCD_PIPE_EVENT_ERROR_XFER:
                ESP_LOGW("", "XFER error: %d", irp->status);
                hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
                break;
            
            case HCD_PIPE_EVENT_ERROR_STALL:
                ESP_LOGW("", "Device stalled: %s pipe, state: %d", msg.pipe_hdl == ctrl_pipe_hdl?"CTRL":"BULK", hcd_pipe_get_state(msg.pipe_hdl));
                hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
                break;
            
            default:
                ESP_LOGW("", "some pipe event");
                break;
        }
    }
}

void xfer_get_device_desc()
{
    USB_CTRL_REQ_INIT_GET_DEVC_DESC((usb_ctrl_req_t *) data_buffers[0]);

    irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;


    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("", "Get device desc");
    }
}

void xfer_set_address(uint8_t addr)
{
    USB_CTRL_REQ_INIT_SET_ADDR((usb_ctrl_req_t *) data_buffers[0], addr);

    irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 0;


    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("", "Set address");
    }
}

void xfer_get_current_config()
{
    USB_CTRL_REQ_INIT_GET_CONFIG((usb_ctrl_req_t *) data_buffers[0]);
    irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 0;


    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("", "Get current config");
    }
}

void xfer_set_configuration()
{
    USB_CTRL_REQ_INIT_SET_CONFIG((usb_ctrl_req_t *) data_buffers[0], 1);
    irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;
    irps[0]->num_bytes = 0;


    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("", "Get current config");
    }
}

void xfer_get_desc()
{
    USB_CTRL_REQ_INIT_GET_CFG_DESC((usb_ctrl_req_t *) data_buffers[0], 1, XFER_DATA_MAX_LEN);
    //important!! if is shorter than buffer and descriptor is longer than num_bytes, then it will stuck here
    // so its best if both values are equal
    irps[0]->num_bytes = XFER_DATA_MAX_LEN;
    irps[0]->data_buffer = data_buffers[0];
    irps[0]->num_iso_packets = 0;



    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[0])) {
        ESP_LOGD("", "Get config desc");
    }
}
