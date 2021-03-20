
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

// void parse_configuration(uint8_t* data_buffer, usb_transfer_status_t status)
// {
//     // uint8_t bLength;
//     // uint8_t bDescriptorType;
//     // uint16_t wTotalLength;
//     // uint8_t bNumInterfaces;
//     // uint8_t bConfigurationValue;
//     // uint8_t iConfiguration;
//     // uint8_t bmAttributes;
//     // uint8_t bMaxPower;
//     if(status == USB_TRANSFER_STATUS_COMPLETED){
//         usb_desc_cfg_t* cfg = (usb_desc_cfg_t*)&data_buffer[8];
//         printf("Config num: %d\n", cfg->bConfigurationValue);
//         printf("Number of intf: %d\n", cfg->bNumInterfaces);
//         printf("Attributes: 0x%02x\n", cfg->bmAttributes);
//         printf("Max power: %d mA\n", cfg->bMaxPower * 2);
//     } else {
//         ESP_LOGW("", "status: %d", status);
//     }
// }

static inline int bcd_to_decimal(unsigned char x) {
    return x - 6 * (x >> 4);
}

static void utf16_to_utf8(char* in, char* out, uint8_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        out[i/2] = in[i];
        i++;
    }
}

static void parse_device_descriptor(uint8_t* data_buffer, usb_transfer_status_t status)
{
    if(status == USB_TRANSFER_STATUS_COMPLETED){
        usb_desc_devc_t* desc = (usb_desc_devc_t*)data_buffer;
        bMaxPacketSize0 = desc->bMaxPacketSize0;

        printf("Length: %d\n", desc->bLength);
        printf("Descriptor type: %d\n", desc->bLength);
        printf("USB version: %d.%02d\n", bcd_to_decimal(desc->bcdUSB >> 8), bcd_to_decimal(desc->bcdUSB & 0xff));
        printf("Device class: 0x%02x\n", desc->bDeviceClass);
        printf("Device subclass: 0x%02x\n", desc->bDeviceSubClass);
        printf("Device protocol: 0x%02x\n", desc->bDeviceProtocol);
        printf("EP0 max packet size: %d\n", desc->bMaxPacketSize0);
        printf("VID: 0x%04x\n", desc->idVendor);
        printf("PID: 0x%04x\n", desc->idProduct);
        printf("Revision number: %d.%02d\n", bcd_to_decimal(desc->bcdDevice >> 8), bcd_to_decimal(desc->bcdDevice & 0xff));
        printf("Manufacturer id: %d\n", desc->iManufacturer);
        printf("Product id: %d\n", desc->iProduct);
        printf("Serial id: %d\n", desc->iSerialNumber);
        printf("Configurations num: %d\n", desc->bNumConfigurations);
        conf_num = desc->bNumConfigurations;
    } else {
        ESP_LOGW("", "status: %d", status);
    }
}

void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len)
{
    if(!len) return;
    if(status == USB_TRANSFER_STATUS_COMPLETED){
        uint8_t offset = 0;
        uint8_t type = *(&data_buffer[0] + offset + 1);
        do{
            ESP_LOGD("", "type: %d\n", type);
            switch (type)
            {
                case USB_W_VALUE_DT_DEVICE:
                    parse_device_descriptor(data_buffer, status);
                    offset += len;
                    break;

                case USB_W_VALUE_DT_CONFIG:{
                    usb_desc_cfg_t* data = (usb_desc_cfg_t*)(data_buffer + offset);
                    printf("Number of Interfaces: %d\n", data->bNumInterfaces);
                    // printf("type: %d\n", data->bConfigurationValue);
                    // printf("type: %d\n", data->iConfiguration);
                    printf("Attributes: 0x%02x\n", data->bmAttributes);
                    printf("Max power: %d mA\n", data->bMaxPower * 2);
                    offset += 9;
                    break;
                }
                case USB_W_VALUE_DT_STRING:{
                    usb_desc_str_t* data = (usb_desc_str_t*)(data_buffer + offset);
                    uint8_t len = 0;
                    len = data->val[0];
                    offset += len;
                    char* str = (char*)calloc(1, len);
                    utf16_to_utf8((char*)&data->val[2], str, len);
                    printf("strings: %s\n", str);
                    free(str);
                    break;
                }
                case USB_W_VALUE_DT_INTERFACE:{
                    usb_desc_intf_t* data = (usb_desc_intf_t*)(data_buffer + offset);
                    offset += 9;
                    printf("bInterfaceNumber: %d\n", data->bInterfaceNumber);
                    printf("bAlternateSetting: %d\n", data->bAlternateSetting);
                    printf("bNumEndpoints: %d\n", data->bNumEndpoints);
                    printf("bInterfaceClass: 0x%02x\n", data->bInterfaceClass);
                    printf("bInterfaceSubClass: 0x%02x\n", data->bInterfaceSubClass);
                    printf("bInterfaceProtocol: 0x%02x\n", data->bInterfaceProtocol);
                    break;
                }            
                case USB_W_VALUE_DT_ENDPOINT:{
                    usb_desc_ep_t* data = (usb_desc_ep_t*)(data_buffer + offset);
                    offset += 7;
                    printf("bEndpointAddress: 0x%02x\n", data->bEndpointAddress);
                    printf("bmAttributes: 0x%02x\n", data->bmAttributes);
                    printf("bDescriptorType: %d\n", data->bDescriptorType);
                    printf("wMaxPacketSize: %d\n", data->wMaxPacketSize);
                    printf("bInterval: %d ms\n", data->bInterval);
                    break;
                }            
                default:
                    ESP_LOGI("", "unknown descriptor: %d", type);
                    offset += data_buffer[offset];
                    break;
            }
            if(offset >= len) break;
            type = *(data_buffer + offset + 1);
        }while(1);
    } else {
        ESP_LOGW("", "status: %d", (uint8_t)status);
    }
}
