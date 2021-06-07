## Simple usb host example 
Since usb host support is still alpha or beta stage and include file is in private_include i copied component to this example. 
Code has been refactored to use events in client code. 

## Example logs:

### Before connecting device
```
Hello world USB host!
I (627) : USB host setup properly
I (627) : Port powered ON
Waiting for conenction
```

### After connecting device
```
I (892) : HCD_PORT_EVENT_CONNECTION
I (892) : HCD_PORT_STATE_DISABLED
I (952) : USB device reset
I (952) : HCD_PORT_STATE_ENABLED
...
I (974) : address set: 1
I (978) : set current configuration: 1
```

### Example reading device descriptors and strings
- device descriptor on EP0
```
Device descriptor:
Length: 18
Descriptor type: 18
USB version: 2.10
Device class: 0x00 (>ifc)
Device subclass: 0x00
Device protocol: 0x00
EP0 max packet size: 64
VID: 0x0781
PID: 0x5597
Revision number: 1.00
Manufacturer id: 1
Product id: 2
Serial id: 3
Configurations num: 1
```

- configuration descriptor
```
Config:
Number of Interfaces: 1
Attributes: 0x80
Max power: 224 mA

Interface:
bInterfaceNumber: 0
bAlternateSetting: 0
bNumEndpoints: 2
bInterfaceClass: 0x08 (Mass Storage)
bInterfaceSubClass: 0x06
bInterfaceProtocol: 0x50

Endpoint:
bEndpointAddress: 0x81
bmAttributes: 0x02
bDescriptorType: 5
wMaxPacketSize: 64
bInterval: 0 ms

Endpoint:
bEndpointAddress: 0x02
bmAttributes: 0x02
bDescriptorType: 5
wMaxPacketSize: 64
bInterval: 0 ms
```

- manufacturer, product and serial strings
```
strings: SanDisk
strings: Cruzer Glide 3.0
strings: 4C530000240507207073
```

##Changes
- update usb host component to current master
- change the way IRPs are allocated to use dynamic allocation


Have a nice play.
