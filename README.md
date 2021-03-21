## Simple usb host example 
Since usb host support is still alpha or beta stage and include file is in private_include i copied component to this example. It is really very simple example, without handling errors, and even not all devices will be handled. For now only sandisk pendrive works without issue, and CP210X completely is not working. 

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
I (727) : HCD_PORT_EVENT_CONNECTION
Resetting
I (1037) : USB device reseted
I (1037) : HCD_PORT_STATE_ENABLED
Full speed enabled
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
I (71201) Pipe: : XFER status: 0, num bytes: 64, actual bytes: 16
strings: SanDisk
I (71209) Pipe: : XFER status: 0, num bytes: 64, actual bytes: 34
strings: Cruzer Glide 3.0
I (71219) Pipe: : XFER status: 0, num bytes: 64, actual bytes: 42
strings: 4C530000240507207073
```


Have a nice play.
