
#include "common.h"
#include "pipes.h"
#include "cdc_class.h"

// -------------------------------------------------- PHY Control ------------------------------------------------------

void phy_force_conn_state(bool connected, TickType_t delay_ticks)
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

bool port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr)
{
    QueueHandle_t port_evt_queue = (QueueHandle_t)user_arg;
    port_event_msg_t msg = {
        .port_hdl = port_hdl,
        .port_event = port_event,
    };

    BaseType_t xTaskWoken = pdFALSE;
    xQueueSendFromISR(port_evt_queue, &msg, &xTaskWoken);
    return (xTaskWoken == pdTRUE);
}

void port_event_task(void* p)
{
    // printf("start port event task\n");
    port_event_msg_t msg;
    hcd_port_event_t event;
    hcd_port_state_t state;
    while(1){
        xQueueReceive(port_evt_queue, &msg, portMAX_DELAY);
        ESP_LOGD("", "port event: %d", msg.port_event);
        hcd_port_handle_event(msg.port_hdl);

        switch (msg.port_event)
        {
            case HCD_PORT_EVENT_CONNECTION:
                ESP_LOGI("", "HCD_PORT_EVENT_CONNECTION");
                isConnected = true;
                break;

            // case HCD_PORT_EVENT_DISCONNECTION:
            //     break;

            // case HCD_PORT_EVENT_ERROR:
            //     break;

            // case HCD_PORT_EVENT_OVERCURRENT:
            //     break;

            case HCD_PORT_EVENT_SUDDEN_DISCONN:{
                recoveryPort = true;
                break;
            }
            
            default:
                ESP_LOGE("", "port event: %d", msg.port_event);
                break;
        }
    }
}

/**
 * @brief Powers ON a port and waits for a connection, then resets the connected device
 *
 * @param port_hdl Port handle
 * @param port_evt_queue Port event queue
 */
void wait_for_connection()
{
    hcd_port_state_t state = hcd_port_get_state(port_hdl);
    //Power ON the port
    if(ESP_OK == hcd_port_command(port_hdl, HCD_PORT_CMD_POWER_ON)) ESP_LOGI("", "Port powered ON");
    state = hcd_port_get_state(port_hdl);
    ESP_LOGD("", "hcd_port_state_t: %d", state);
    //Wait for connection event
    printf("Waiting for conenction\n");
    phy_force_conn_state(true, pdMS_TO_TICKS(10));     //Allow for connected state on PHY
    hcd_port_event_t event;

    while (!isConnected)
    {
        vTaskDelay(1);
    }
    
    if(HCD_PORT_STATE_DISABLED == hcd_port_get_state(port_hdl)) ESP_LOGI("", "HCD_PORT_STATE_DISABLED");
    //Reset newly connected device
    printf("Resetting\n");
    if(ESP_OK == hcd_port_command(port_hdl, HCD_PORT_CMD_RESET)) ESP_LOGI("", "USB device reseted");
    while(HCD_PORT_STATE_ENABLED != hcd_port_get_state(port_hdl));
    ESP_LOGI("", "HCD_PORT_STATE_ENABLED");
    //Get speed of conencted
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){
        if (port_speed == USB_SPEED_FULL) {
            printf("Full speed enabled\n");
        } else {
            printf("Low speed enabled\n");
        }
    }
    on_connected();
}

void on_connected()
{
    alloc_pipe_and_xfer_reqs_ctrl(port_hdl, pipe_evt_queue, &ctrl_pipe_hdl, req_hdls, data_buffers, irps, NUM_XFER_REQS);

    // get device descriptor on EP0
    xfer_get_device_desc();
    if(!wait()) return;
    
    // set new address
    xfer_set_address(DEVICE_ADDR);

    if(!wait()) return;
    // here device should be ready to use new address, we can switch to that address too
    hcd_pipe_update(ctrl_pipe_hdl, DEVICE_ADDR, bMaxPacketSize0);
    vTaskDelay(10);
    // now we can get configuration descriptors and optionally strings
    xfer_get_desc();
    // xfer_get_strings(1);

    if(!wait()) return;
 
    vTaskDelay(30);
    xfer_set_configuration();

    if(!wait()) return;
 
    vTaskDelay(30);
}

void recovery_port()
{
    recoveryPort = false;
    isConnected = false;
    hcd_pipe_update(ctrl_pipe_hdl, 0, bMaxPacketSize0);

    hcd_port_state_t state;

    hcd_port_command(port_hdl, HCD_PORT_CMD_RESET);
    if (HCD_PIPE_STATE_INVALID == hcd_pipe_get_state(ctrl_pipe_hdl))
    {                
        ESP_LOGW("", "pipe state: %d", hcd_pipe_get_state(ctrl_pipe_hdl));
        //Free transfer requests
        free_pipe_and_xfer_reqs(ctrl_pipe_hdl, req_hdls, data_buffers, irps, NUM_XFER_REQS);
        ctrl_pipe_hdl = NULL;
        delete_pipes();

        esp_err_t err;
        if(HCD_PORT_STATE_RECOVERY == (state = hcd_port_get_state(port_hdl))){
            if(ESP_OK != (err = hcd_port_recover(port_hdl))) ESP_LOGE("recovery", "should be not powered state %d => (%d)", state, err);
        } else {
            ESP_LOGE("", "hcd_port_state_t: %d", state);
        }

        wait_for_connection();
    }
}
