
#pragma once
#include "common.h"

typedef struct {
    hcd_pipe_handle_t pipe_hdl;
    hcd_pipe_event_t pipe_event;
} pipe_event_msg_t;

// since we are using the same XFER queue in ctrl pipe(there is 3 configured), then we have to wait until it will be dequeued before we can send another ctrl request
static inline bool wait()
{
    hcd_pipe_event_t state;
    do{
        state = hcd_pipe_get_event(ctrl_pipe_hdl);
        ets_delay_us(10);
        if(state == HCD_PIPE_EVENT_ERROR_XFER) return false;
    }while(state != HCD_PIPE_EVENT_XFER_REQ_DONE);
    return true;
}

void free_pipe_and_xfer_reqs(hcd_pipe_handle_t pipe_hdl,
                                    hcd_xfer_req_handle_t *req_hdls,
                                    uint8_t **data_buffers,
                                    usb_irp_t **irps,
                                    int num_xfers);
void xfer_get_device_desc();
void xfer_set_address(uint8_t addr);
void xfer_get_current_config();
void xfer_set_configuration();
void xfer_get_desc();
