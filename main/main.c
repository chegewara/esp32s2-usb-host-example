
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
#include "cdc_class.h"

QueueHandle_t port_evt_queue;
QueueHandle_t pipe_evt_queue;
hcd_port_handle_t port_hdl;
hcd_pipe_handle_t ctrl_pipe_hdl;

bool isConnected = false;
bool recoveryPort = false;

uint8_t bMaxPacketSize0 = 64;
uint8_t conf_num = 0;



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


void app_main(void)
{
    printf("Hello world USB host!\n");

    if(setup()){
        wait_for_connection();
    }

    vTaskDelay(10);
    xfer_set_control_line(1, 1);
    wait();
    xfer_set_line_coding(115200);
    wait();
    xfer_get_line_coding();
    while(1){
        if(recoveryPort) recovery_port();

        if(1){
            xfer_in_data();
        }
        vTaskDelay(10);
    }
}
