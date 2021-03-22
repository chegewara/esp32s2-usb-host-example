### Description
It is not completed yet, no handling INTR endpoint, USB CDC class.

### API
Working api:
- void xfer_set_line_coding(uint32_t); - control bitrate
- void xfer_set_control_line(bool dtr, bool rts); - control DTR and RTS signals
- void xfer_get_line_coding(); - get bitrate and other serial params
- void xfer_in_data(); - check for data sent from device to host
- void xfer_out_data(); -  send data from host to device

# TODO
It would be good to add option to handle get line coding, maybe callback?
Received data are only printed in logs, here also would be good to add some handling option.
