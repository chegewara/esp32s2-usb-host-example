
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
#include "common.h"

static bool pipe_callback(hcd_pipe_handle_t pipe_hdl, hcd_pipe_event_t pipe_event, void *user_arg, bool in_isr)
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

void alloc_pipe_and_xfer_reqs(hcd_port_handle_t port_hdl,
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
        .callback = pipe_callback,
        .callback_arg = (void *)pipe_evt_queue,
        .context = NULL,
        .ep_desc = NULL,    //NULL EP descriptor to create a default pipe
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
        data_buffers[i] = heap_caps_malloc(sizeof(usb_ctrl_req_t) + XFER_DATA_MAX_LEN, MALLOC_CAP_DMA);
        if(NULL == data_buffers[i]) ESP_LOGE("", "err 5");
        //Allocate IRP object
        irps[i] = heap_caps_malloc(sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
        if(NULL == irps[i]) ESP_LOGE("", "err 6");
        //Set the transfer request's target
        hcd_xfer_req_set_target(req_hdls[i], *pipe_hdl, irps[i], i);
    }
}

void free_pipe_and_xfer_reqs(hcd_pipe_handle_t pipe_hdl,
                                    hcd_xfer_req_handle_t *req_hdls,
                                    uint8_t **data_buffers,
                                    usb_irp_t **irps,
                                    int num_xfers)
{
    printf("Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    for (int i = 0; i < num_xfers; i++) {
        heap_caps_free(irps[i]);
        heap_caps_free(data_buffers[i]);
        hcd_xfer_req_free(req_hdls[i]);
    }
    printf("Freeing default pipe\n");
    //Delete the pipe
    if(ESP_OK == hcd_pipe_free(pipe_hdl)) {}
}

void pipe_event_task(void* p)
{
    // printf("start pipe event task\n");
    pipe_event_msg_t msg;
    while(1){
        xQueueReceive(pipe_evt_queue, &msg, portMAX_DELAY);
        hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(msg.pipe_hdl);
        hcd_pipe_handle_t pipe_hdl;
        usb_irp_t *irp;
        void *context;
        if(req_hdl == NULL) continue;
        hcd_xfer_req_get_target(req_hdl, &pipe_hdl, &irp, &context);
        ESP_LOGD("", "\t-> Pipe [%d] event: %d\n", (uint8_t)context, msg.pipe_event);

        switch (msg.pipe_event)
        {
            case HCD_PIPE_EVENT_NONE:
                break;

            case HCD_PIPE_EVENT_XFER_REQ_DONE:
                ESP_LOGV("Pipe: ", "XFER status: %d, num bytes: %d, actual bytes: %d", irp->status, irp->num_bytes, irp->actual_num_bytes);
                if(0 == irp->num_bytes) break;
                ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", irp->data_buffer, 8, ESP_LOG_DEBUG);
                ESP_LOG_BUFFER_HEX_LEVEL("Actual data", irp->data_buffer + 8, irp->actual_num_bytes, ESP_LOG_DEBUG);

                usb_ctrl_req_t* ctrl = irp->data_buffer;
                if(ctrl->bRequest == USB_B_REQUEST_GET_DESCRIPTOR){
                    parse_cfg_descriptor((irp->data_buffer + 8), irp->status, irp->actual_num_bytes);
                } else if(ctrl->bRequest == USB_B_REQUEST_GET_CONFIGURATION) {
                    ESP_LOGD("", "current configuration: %d", irp->data_buffer[9]);
                }
                break;

            case HCD_PIPE_EVENT_ERROR_XFER:
                ESP_LOGW("", "XFER error: %d", irp->status);
                hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
                break;
            
            case HCD_PIPE_EVENT_ERROR_STALL:
                ESP_LOGW("", "Device stalled");
                hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_CLEAR);
                break;
            
            default:
                ESP_LOGW("", "some pipe event");
                break;
        }
        pipe_done = true;
    }
}
