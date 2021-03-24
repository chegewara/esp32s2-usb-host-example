
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
#include "ctrl_pipe.h"

#define EVENT_QUEUE_LEN         10
#define NUM_XFER_REQS           4
#define XFER_DATA_MAX_LEN       64     //Just assume that will only IN/OUT 64 bytes for now
#define PORT_NUM                1


static hcd_pipe_handle_t ctrl_pipe_hdl;
static hcd_xfer_req_handle_t req_hdls[NUM_XFER_REQS];
static uint8_t *data_buffers[NUM_XFER_REQS];
static usb_irp_t *irps[NUM_XFER_REQS];
static QueueHandle_t ctrl_pipe_evt_queue;
static ctrl_pipe_cb_t ctrl_pipe_cb;
uint8_t bMaxPacketSize0;

void register_ctrl_pipe_callback(ctrl_pipe_cb_t cb)
{
    ctrl_pipe_cb = cb;
}

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

void free_pipe_and_xfer_reqs_ctrl(hcd_pipe_handle_t pipe_hdl)
{
    //Dequeue transfer requests
    do{
        hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(pipe_hdl);
        if(req_hdl == NULL) break;
    }while(1);

    ESP_LOGD("", "Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    for (int i = 0; i < NUM_XFER_REQS; i++) {
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

void alloc_pipe_and_xfer_reqs_ctrl(hcd_port_handle_t port_hdl, hcd_pipe_handle_t* handle)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){}

    //Create default pipe
    // printf("Creating default pipe\n");
    hcd_pipe_config_t config = {
        .callback = ctrl_pipe_callback,
        .callback_arg = (void *)ctrl_pipe_evt_queue,
        .context = NULL,
        .ep_desc = USB_XFER_TYPE_CTRL,    //NULL EP descriptor to create a default pipe
        .dev_addr = 0,
        .dev_speed = port_speed,
    };
    if(ESP_OK != hcd_pipe_alloc(port_hdl, &config, &ctrl_pipe_hdl)) ESP_LOGE("", "cant alloc pipe");
    if(NULL == ctrl_pipe_hdl) {
        ESP_LOGE("", "NULL == pipe_hdl");
        return;
    }
    //Create transfer requests (and other required objects such as IRPs and data buffers)
    // printf("Creating transfer requests\n");
    for (int i = 0; i < NUM_XFER_REQS; i++) {
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
        hcd_xfer_req_set_target(req_hdls[i], ctrl_pipe_hdl, irps[i], (void*)i);
    }
    *handle = ctrl_pipe_hdl;
}

void ctrl_pipe_event_task(void* p)
{
    printf("start pipe event task\n");
    pipe_event_msg_t msg;
    ctrl_pipe_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(pipe_event_msg_t));

    while(1){
        xQueueReceive(ctrl_pipe_evt_queue, &msg, portMAX_DELAY);
        hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(msg.pipe_hdl);
        if(req_hdl == NULL) continue;
        if (ctrl_pipe_cb != NULL)
        {
            ctrl_pipe_cb(msg, req_hdl);
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

void xfer_set_configuration(uint8_t num)
{
    USB_CTRL_REQ_INIT_SET_CONFIG((usb_ctrl_req_t *) data_buffers[2], num);
    irps[2]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    irps[2]->data_buffer = data_buffers[2];
    irps[2]->num_iso_packets = 0;
    irps[2]->num_bytes = 0;


    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[2])) {
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

void xfer_get_string(uint8_t num)
{
    USB_CTRL_REQ_INIT_GET_STRING((usb_ctrl_req_t *) data_buffers[num], 0, num, XFER_DATA_MAX_LEN);
    irps[num]->num_bytes = XFER_DATA_MAX_LEN;
    irps[num]->data_buffer = data_buffers[num];
    irps[num]->num_iso_packets = 0;

    //Enqueue those transfer requests
    if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[num])) {
        ESP_LOGD("", "Get string: %d", num);
    }
}
