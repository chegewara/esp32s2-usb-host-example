
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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

QueueHandle_t port_evt_queue;
QueueHandle_t pipe_evt_queue;
bool isConnected = false;
uint8_t conf_num = 0;

uint8_t bMaxPacketSize0 = 64;
hcd_port_handle_t port_hdl;

hcd_pipe_handle_t default_pipe;
hcd_xfer_req_handle_t req_hdls[NUM_XFER_REQS];
uint8_t *data_buffers[NUM_XFER_REQS];
usb_irp_t *irps[NUM_XFER_REQS];

static void wait_for_connection();

// -------------------------------------------------- PHY Control ------------------------------------------------------

static void phy_force_conn_state(bool connected, TickType_t delay_ticks)
{
    vTaskDelay(delay_ticks);
    usb_wrap_dev_t *wrap = &USB_WRAP;
    if (connected) {
        //Swap back to internal PHY that is connected to a devicee
        wrap->otg_conf.phy_sel = 0;
    } else {
        //Set externa PHY input signals to fixed voltage levels mimicing a disconnected state
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_EXTPHY_VP_IDX, false);
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_EXTPHY_VM_IDX, false);
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_EXTPHY_RCV_IDX, false);
        //Swap to the external PHY
        wrap->otg_conf.phy_sel = 1;
    }
}

// ------------------------------------------------ Helper Functions ---------------------------------------------------

static bool port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr)
{
    QueueHandle_t port_evt_queue = (QueueHandle_t)user_arg;
    // TEST_ASSERT(in_isr);    //Current HCD implementation should never call a port callback in a task context
    port_event_msg_t msg = {
        .port_hdl = port_hdl,
        .port_event = port_event,
    };

    BaseType_t xTaskWoken = pdFALSE;
    xQueueSendFromISR(port_evt_queue, &msg, &xTaskWoken);
    return (xTaskWoken == pdTRUE);
}

static void port_event_task(void* p)
{
    // printf("start port event task\n");
    port_event_msg_t msg;
    hcd_port_event_t event;
    hcd_port_state_t state;
    while(1){
        xQueueReceive(port_evt_queue, &msg, portMAX_DELAY);
        ESP_LOGD("", "port event: %d", msg.port_event);

        switch (msg.port_event)
        {
            case HCD_PORT_EVENT_CONNECTION:
                ESP_LOGI("", "HCD_PORT_EVENT_CONNECTION");
                isConnected = true;
                hcd_port_handle_event(msg.port_hdl);
                break;

            case HCD_PORT_EVENT_DISCONNECTION:
                break;

            case HCD_PORT_EVENT_ERROR:
                break;

            case HCD_PORT_EVENT_OVERCURRENT:
                break;

            case HCD_PORT_EVENT_SUDDEN_DISCONN:{
                state = hcd_port_get_state(port_hdl);
                ESP_LOGI("", "hcd_port_state_t: %d", state);
                phy_force_conn_state(false, 0);    //Force disconnected state on PHY
                if(ESP_OK == hcd_port_command(port_hdl, HCD_PORT_CMD_POWER_OFF)) ESP_LOGI("", "Port powered OFF");
                //Dequeue transfer requests
                for (int i = 0; i < NUM_XFER_REQS; i++) {
                    hcd_xfer_req_handle_t req_hdl = hcd_xfer_req_dequeue(default_pipe);
                    hcd_pipe_handle_t pipe_hdl;
                    usb_irp_t *irp;
                    void *context;
                    if(req_hdl == NULL) continue;

                    hcd_xfer_req_get_target(req_hdl, &pipe_hdl, &irp, &context);
                }
                hcd_port_recover(port_hdl);
                //Free transfer requests
                free_pipe_and_xfer_reqs(default_pipe, req_hdls, data_buffers, irps, NUM_XFER_REQS);

                isConnected = false;
                wait_for_connection();                
                break;
            }
            
            default:
                ESP_LOGE("", "port event: %d", msg.port_event);
                break;
        }
    }
}

/**
 * @brief Creates port and pipe event queues. Sets up the HCD, and initializes a port.
 *
 * @param[out] port_evt_queue Port event queue
 * @param[out] pipe_evt_queue Pipe event queue
 * @param[out] port_hdl Port handle
 */
static bool setup()
{
    port_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(port_event_msg_t));
    pipe_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(pipe_event_msg_t));

    if(port_evt_queue) xTaskCreate(port_event_task, "port_task", 4*1024, NULL, 10, NULL);
    if(pipe_evt_queue) xTaskCreate(pipe_event_task, "pipe_task", 4*1024, NULL, 10, NULL);

    //Install HCD
    hcd_config_t config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    if(hcd_install(&config) == ESP_OK) {
        //Initialize a port
        hcd_port_config_t port_config = {
            .callback = port_callback,
            .callback_arg = (void *)port_evt_queue,
            .context = NULL,
        };
        esp_err_t err;
        if(ESP_OK == (err = hcd_port_init(PORT_NUM, &port_config, &port_hdl))){
            if(HCD_PORT_STATE_NOT_POWERED == hcd_port_get_state(port_hdl)) ESP_LOGI("", "USB host setup properly");
        
            phy_force_conn_state(false, 0);    //Force disconnected state on PHY
            return true;
        } else {
            ESP_LOGE("", "Error to init port: %d!!!", err);
        }
    } else {
        ESP_LOGE("", "Error to install HCD!!!");
    }
    return false;
}

/**
 * @brief Powers ON a port and waits for a connection, then resets the connected device
 *
 * @param port_hdl Port handle
 * @param port_evt_queue Port event queue
 */
static void wait_for_connection()
{
    hcd_port_state_t state = hcd_port_get_state(port_hdl);
    //Power ON the port
    if(ESP_OK == hcd_port_command(port_hdl, HCD_PORT_CMD_POWER_ON)) ESP_LOGI("", "Port powered ON");
    state = hcd_port_get_state(port_hdl);
    ESP_LOGD("", "hcd_port_state_t: %d", state);
    //Wait for connection event
    printf("Waiting for conenction\n");
    phy_force_conn_state(true, pdMS_TO_TICKS(100));     //Allow for connected state on PHY
    hcd_port_event_t event;

    while (!isConnected)
    {
        vTaskDelay(1);
    }
    
    if(HCD_PORT_STATE_DISABLED == hcd_port_get_state(port_hdl)) ESP_LOGI("", "HCD_PORT_STATE_DISABLED");
    //Reset newly connected device
    printf("Resetting\n");
    if(ESP_OK == hcd_port_command(port_hdl, HCD_PORT_CMD_RESET)) ESP_LOGI("", "USB device reseted");
    if(HCD_PORT_STATE_ENABLED == hcd_port_get_state(port_hdl)) ESP_LOGI("", "HCD_PORT_STATE_ENABLED");
    //Get speed of conencted
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){
        if (port_speed == USB_SPEED_FULL) {
            printf("Full speed enabled\n");
        } else {
            printf("Low speed enabled\n");
        }
    }
}

static void xfer_get_device_desc()
{
    USB_CTRL_REQ_INIT_GET_DEVC_DESC((usb_ctrl_req_t *) data_buffers[0]);

    for (int i = 0; i < NUM_XFER_REQS; i++) {
        irps[i]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
        irps[i]->data_buffer = data_buffers[i];
        irps[i]->num_iso_packets = 0;
    }

    //Enqueue those transfer requests
    for (int i = 0; i < 1; i++) {
        if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[i])) {
            ESP_LOGD("xfer", "Get dev desc");
        }
    }
}

static void xfer_set_address(uint8_t addr)
{
    USB_CTRL_REQ_INIT_SET_ADDR((usb_ctrl_req_t *) data_buffers[4], addr);

    for (int i = 4; i < 5; i++) {
        irps[i]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
        irps[i]->data_buffer = data_buffers[i];
        irps[i]->num_iso_packets = 0;
    }
    irps[4]->num_bytes = 0;

    //Enqueue those transfer requests
    for (int i = 4; i < 5; i++) {
        if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[i])) {
            ESP_LOGD("", "Set address");
        }
    }
}

static void xfer_get_current_config()
{
    // USB_CTRL_REQ_INIT_GET_CONFIG((usb_ctrl_req_t *) data_buffers[0]);
}

static void xfer_get_desc()
{
    for (int i = 0; i < conf_num; i++) {
        USB_CTRL_REQ_INIT_GET_CFG_DESC((usb_ctrl_req_t *) data_buffers[i], i, XFER_DATA_MAX_LEN);

        irps[i]->num_bytes = bMaxPacketSize0;
        irps[i]->data_buffer = data_buffers[i];
        irps[i]->num_iso_packets = 0;
    }

    //Enqueue those transfer requests
    for (int i = 0; i < conf_num; i++) {
        if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[i])) {
            ESP_LOGD("xfer", "Pipe: %d", i);
        }
    }
}

static void xfer_get_strings()
{
    // we are assuming the device has all 3 base strings (manufacturer, product and S/N)
    uint8_t str_num = 4;
    for (int i = 1; i < str_num; i++) {
        USB_CTRL_REQ_INIT_GET_STRING((usb_ctrl_req_t *) data_buffers[i], 0, i, XFER_DATA_MAX_LEN);
        irps[i]->num_bytes = bMaxPacketSize0;
        irps[i]->data_buffer = data_buffers[i];
        irps[i]->num_iso_packets = 0;
    }

    //Enqueue those transfer requests
    for (int i = 1; i < str_num; i++) {
        if(ESP_OK == hcd_xfer_req_enqueue(req_hdls[i])) {
            ESP_LOGD("xfer", "Pipe: %d", i);
        }
    }
}

void app_main(void)
{
    printf("Hello world USB host!\n");

    if(setup()){
        wait_for_connection();
        // device connected, pipes and xfers can be allocated
        alloc_pipe_and_xfer_reqs(port_hdl, pipe_evt_queue, &default_pipe, req_hdls, data_buffers, irps, NUM_XFER_REQS);

        // get device descriptor on EP0
        xfer_get_device_desc();
        while(!pipe_done){
            ets_delay_us(10);
        }
        pipe_done = false;

        // set new address
        xfer_set_address(DEVICE_ADDR);

        while(!pipe_done){
            ets_delay_us(10);
        }

        // here device should be ready to use new address, we can switch to that address too
        hcd_pipe_update(default_pipe, DEVICE_ADDR, bMaxPacketSize0);
        vTaskDelay(10);
        // now we can get configuration descriptors and optionally strings
        xfer_get_desc();
        xfer_get_strings();
    }

    while(1){
        vTaskDelay(1000);
    }
}
