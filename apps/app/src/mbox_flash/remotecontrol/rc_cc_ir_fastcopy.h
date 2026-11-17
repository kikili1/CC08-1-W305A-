#ifndef RC_CC_IR_FASTCOPY_H
#define RC_CC_IR_FASTCOPY_H

#include "typedef.h"

enum {
    IR_COPY_MASTER = 1,
    IR_COPY_SLAVE = 2,
};
enum {
    IR_COPY_KEY_OTHER = 1,
    IR_COPY_KEY_OK = 2,
    IR_COPY_KEY_BACK = 4,
};

u8 rc_cc_ir_fastcopy_active(void);
u8 rc_cc_ir_fastcopy_blocks_keys(void);
u8 rc_cc_ir_fastcopy_led_on(void);
void rc_cc_ir_fastcopy_enter_request(u8 role);
void rc_cc_ir_fastcopy_key_request(u8 key);
void rc_cc_ir_fastcopy_keys_released(void);
void rc_cc_ir_fastcopy_tick(void);
void rc_cc_ir_fastcopy_process(void);
void rc_cc_ir_fastcopy_rx_pulse(u8 mark, u32 duration_us);

/* Hardware stays with the existing IR driver and its timer/PWM ownership. */
void rc_cc_ir_fastcopy_hw_receive(u8 enable);
u8 rc_cc_ir_fastcopy_hw_send(const u8 *record);
u8 rc_cc_ir_fastcopy_hw_sending(void);
u32 rc_cc_ir_fastcopy_hw_sent_at(void);
void rc_cc_ir_fastcopy_hw_stop(void);

#endif
