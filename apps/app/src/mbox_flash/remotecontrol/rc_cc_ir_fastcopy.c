#include "typedef.h"
#include "app_config.h"
#include "rc_ir.h"
#include "rc_cc_ir_fastcopy.h"
#include "sys_timer.h"
#include "config.h"
#include "msg.h"
#include "app_power_mg.h"
#include <string.h>

#if TCFG_IR_ENABLE && TCFG_IR_STUDY_ENABLE

#define COPY_MASTER_MS (30UL * 60 * 1000)
#define COPY_SLAVE_MS 8000
#define COPY_KEY_GAP_MS 300
#define COPY_QUIET_MS 200
#define COPY_SECOND_WAIT_MS 400
#define COPY_RAW_MAX (3 * 76)

/* Four distinct marks cannot be a complete three-cycle playback of a learned key. */
static const union_xdata_ir empty_slot = { .ir_buf = {
    .ir_study_use = {0, 3, 4, 0},
    .ir_study_h = {2400, 4800, 1200, 3600},
    .ir_study_l = {3300, 1700, 2700},
    .ir_study_buf = {0x00, 0x11, 0x22, 0x30},
} };

enum { COPY_WAIT, COPY_TX_FIRST, COPY_TX_GAP, COPY_TX_SECOND };
static volatile u8 copy_role, enter_request, key_request, key_guard, keys_released;
static u8 copy_phase, rx_slot;
static volatile u32 copy_activity;
static u32 copy_gap_start;
static union_xdata_ir copy_records[2];
static u8 rx_pulses[COPY_RAW_MAX];
static volatile u16 rx_count;
static volatile u8 rx_active, rx_failed;
static volatile u32 rx_activity;

static u8 rc_cc_ir_fastcopy_elapsed(u32 since, u32 ms)
{
    return SYS_TIMER_JIFFIES - since >= sys_timer_msecs_to_jiffies(ms);
}

static u8 rc_cc_ir_fastcopy_record_valid(const union_xdata_ir *record, u16 len)
{
    const struct _IR_STUDY_STRUCT *r = &record->ir_buf;
    const u8 *h = r->ir_study_use;
    if(len < 54 || len > sizeof(*record) || h[0] != 0xab || len != 53 + h[1] ||
       !h[3] || h[2] + h[3] > h[1] + 1)
        return 0;
    for(u8 i = 0; i < h[2] + h[3]; i++) {
        u8 high = r->ir_study_buf[i] >> 4;
        u8 low = r->ir_study_buf[i] & 15;
        if(high >= HL_LONG || low >= HL_LONG || r->ir_study_h[high] <= 40 ||
           r->ir_study_l[low] <= 40 || r->ir_study_h[high] >= COPY_QUIET_MS * 1000 ||
           r->ir_study_l[low] >= COPY_QUIET_MS * 1000)
            return 0;
    }
    /* shortcut: 300 ms separation requires code-internal gaps below 200 ms. */
    return 1;
}

static void rc_cc_ir_fastcopy_rx_reset(void)
{
    rx_count = rx_active = 0;
    rx_activity = SYS_TIMER_JIFFIES;
    memset(&copy_records[rx_slot], 0, sizeof(copy_records[rx_slot]));
}

static void rc_cc_ir_fastcopy_exit(void)
{
    rc_cc_ir_fastcopy_hw_stop();
    copy_role = 0;
    copy_phase = COPY_WAIT;
    key_request = enter_request = 0;
    rx_active = 0;
    /* Suppress the held exit key until the whole matrix is released. */
    key_guard = !keys_released;
}

static void rc_cc_ir_fastcopy_wait(void)
{
    rc_cc_ir_fastcopy_hw_stop();
    copy_phase = COPY_WAIT;
    rx_slot = rx_failed = rx_active = 0;
    if(copy_role == IR_COPY_SLAVE) {
        copy_activity = SYS_TIMER_JIFFIES;
        rc_cc_ir_fastcopy_rx_reset();
        rc_cc_ir_fastcopy_hw_receive(1);
    }
}

u8 rc_cc_ir_fastcopy_active(void) { return copy_role; }
u8 rc_cc_ir_fastcopy_blocks_keys(void) { return copy_role || key_guard || enter_request; }

u8 rc_cc_ir_fastcopy_led_on(void)
{
    u8 busy = copy_role == IR_COPY_MASTER ? copy_phase != COPY_WAIT :
              !rx_failed && (rx_active || rx_slot);
    return copy_role && (!busy || ((SYS_TIMER_JIFFIES /
           sys_timer_msecs_to_jiffies(200)) & 1));
}

void rc_cc_ir_fastcopy_enter_request(u8 role)
{
    if(!copy_role && (role == IR_COPY_MASTER || role == IR_COPY_SLAVE)) {
        enter_request = role;
        key_guard = 1;
        keys_released = 0;
    }
}

void rc_cc_ir_fastcopy_key_request(u8 key)
{
    keys_released = 0;
    if(copy_role)
        key_request |= key;
}

void rc_cc_ir_fastcopy_keys_released(void)
{
    keys_released = 1;
    if(!enter_request)
        key_guard = 0;
}

void rc_cc_ir_fastcopy_tick(void)
{
    if(copy_role || enter_request)
        post_msg(1, MSG_MATRIX_KEY);
}

static void rc_cc_ir_fastcopy_master_send(void)
{
    u8 mask = 0;
    for(u8 slot = 0; slot < 2; slot++) {
        u8 header[2];
        u16 id = CFG_IR_KEY_NUM_BEGIN + slot + 1;
        if(syscfg_read(id, header, 2) != 2 || header[0] != 0xab) {
            if(ir_valid_key & BIT(slot + 1))
                return;
            continue;
        }
        u16 len = 53 + header[1];
        if(len > sizeof(copy_records[slot]) ||
           syscfg_read(id, &copy_records[slot], len) != len ||
           !rc_cc_ir_fastcopy_record_valid(&copy_records[slot], len))
            return;
        mask |= BIT(slot);
    }
    if(!mask)
        return;
    if(!(mask & 1))
        copy_records[0] = empty_slot;
    if(!(mask & 2))
        copy_records[1] = empty_slot;
    if(rc_cc_ir_fastcopy_hw_send(copy_records[0].xdata_ir_buf))
        copy_phase = COPY_TX_FIRST;
}

static u8 rc_cc_ir_fastcopy_pulse_index(u32 *times, u32 duration)
{
    if(duration <= 40 || duration >= COPY_QUIET_MS * 1000)
        return 15;
    for(u8 i = 0; i < HL_LONG; i++) {
        if(!times[i]) {
            times[i] = duration;
            return i;
        }
        u32 difference = times[i] > duration ? times[i] - duration : duration - times[i];
        if(difference < duration / 6) {
            times[i] = (times[i] + duration) / 2;
            return i;
        }
    }
    return 15;
}

/* Envelope edges are supplied by the existing raw-carrier capture ISR. */
void rc_cc_ir_fastcopy_rx_pulse(u8 mark, u32 duration)
{
    if(copy_role != IR_COPY_SLAVE)
        return;
    if(mark) {
        rx_active = 1;
    } else if(!rx_active)
        return; /* The space before this segment's first mark belongs to the separator. */
    rx_activity = SYS_TIMER_JIFFIES;
    copy_activity = rx_activity;
    if(rx_failed)
        return; /* Drain both segments of a failed round before accepting another round. */
    struct _IR_STUDY_STRUCT *r = &copy_records[rx_slot].ir_buf;
    u8 index = rc_cc_ir_fastcopy_pulse_index(mark ? r->ir_study_h : r->ir_study_l, duration);
    if(index == 15 || (mark && rx_count == COPY_RAW_MAX)) {
        rx_failed = 1;
        return;
    }
    if(mark)
        rx_pulses[rx_count++] = (index << 4) | 15;
    else
        rx_pulses[rx_count - 1] = (rx_pulses[rx_count - 1] & 0xf0) | index;
}

static u8 rc_cc_ir_fastcopy_is_empty(void)
{
    const struct _IR_STUDY_STRUCT *marker = &empty_slot.ir_buf;
    const struct _IR_STUDY_STRUCT *r = &copy_records[rx_slot].ir_buf;
    if(rx_count != 4 || (rx_pulses[3] & 15) != 15)
        return 0;
    for(u8 i = 0; i < 4; i++) {
        u8 high = rx_pulses[i] >> 4;
        u8 low = rx_pulses[i] & 15;
        u32 mark = marker->ir_study_h[i];
        if(high >= HL_LONG || r->ir_study_h[high] + 180 < mark ||
           r->ir_study_h[high] > mark + 180)
            return 0;
        if(i < 3) {
            u32 space = marker->ir_study_l[i];
            if(low >= HL_LONG || r->ir_study_l[low] + 180 < space ||
               r->ir_study_l[low] > space + 180)
                return 0;
        }
    }
    return 1;
}

/* Keep the first-frame prefix and one repeat block in the normal 128-byte format. */
static u8 rc_cc_ir_fastcopy_rx_pack(void)
{
    struct _IR_STUDY_STRUCT *r = &copy_records[rx_slot].ir_buf;
    if(rc_cc_ir_fastcopy_is_empty())
        return 1; /* A recognized empty marker leaves the staged record invalid. */
    for(u16 repeat = 1; repeat <= rx_count / 2 && repeat <= sizeof(r->ir_study_buf); repeat++) {
        u16 prefix = rx_count - 2 * repeat;
        u8 gap = rx_pulses[rx_count - repeat - 1] & 15;
        if(!prefix || gap >= HL_LONG || r->ir_study_l[gap] < 6000)
            continue;
        u16 i;
        for(i = 0; i < repeat; i++) {
            u8 first = rx_pulses[prefix + i];
            u8 second = rx_pulses[prefix + repeat + i];
            if((first & 15) >= HL_LONG || (first >> 4) != (second >> 4) ||
               (i + 1 < repeat && (first & 15) != (second & 15)) ||
               r->ir_study_l[first & 15] > r->ir_study_l[gap] * 7 / 6)
                break;
        }
        if(i != repeat)
            continue;
        if(prefix == repeat && !memcmp(rx_pulses, rx_pulses + repeat, repeat))
            prefix = 0; /* Three identical full frames need only one stored repeat frame. */
        u16 used = prefix + repeat;
        if(used > sizeof(r->ir_study_buf))
            continue;
        memcpy(r->ir_study_buf, rx_pulses, used);
        r->ir_study_use[0] = 0xab;
        r->ir_study_use[1] = used - 1;
        r->ir_study_use[2] = prefix;
        r->ir_study_use[3] = repeat;
        return rc_cc_ir_fastcopy_record_valid(&copy_records[rx_slot], 52 + used);
    }
    return 0;
}

static u8 rc_cc_ir_fastcopy_slave_save(void)
{
    u16 voltage = app_power_get_vbat_now();
    if(voltage && voltage < 2100)
        return 0;
    if(!copy_records[0].xdata_ir_buf[0] && !copy_records[1].xdata_ir_buf[0])
        return 0;
    for(u8 slot = 0; slot < 2; slot++) {
        if(!copy_records[slot].xdata_ir_buf[0])
            continue;
        u16 len = 53 + copy_records[slot].xdata_ir_buf[1];
        if(!rc_cc_ir_fastcopy_record_valid(&copy_records[slot], len))
            return 0;
    }
    for(u8 slot = 0; slot < 2; slot++) {
        if(!copy_records[slot].xdata_ir_buf[0])
            continue;
        u16 len = 53 + copy_records[slot].xdata_ir_buf[1];
        if(syscfg_write(CFG_IR_KEY_NUM_BEGIN + slot + 1, &copy_records[slot], len) != len) {
            system_ir_vm_initial();
            return 0;
        }
    }
    for(u8 slot = 0; slot < 2; slot++) {
        if(!copy_records[slot].xdata_ir_buf[0] &&
           syscfg_write(CFG_IR_KEY_NUM_BEGIN + slot + 1, &copy_records[slot], 1) != 1) {
            system_ir_vm_initial();
            return 0;
        }
    }
    system_ir_vm_initial();
    return 1;
}

void rc_cc_ir_fastcopy_process(void)
{
    u8 enter, keys;
    local_irq_disable();
    enter = enter_request;
    enter_request = 0;
    keys = key_request;
    key_request = 0;
    local_irq_enable();
    if(enter && !copy_role) {
        copy_role = enter;
        copy_activity = SYS_TIMER_JIFFIES;
        rc_cc_ir_fastcopy_wait();
    }
    if(!copy_role)
        return;
    if(keys & IR_COPY_KEY_BACK) {
        rc_cc_ir_fastcopy_exit();
        return;
    }
    if(keys)
        copy_activity = SYS_TIMER_JIFFIES;
    if(copy_role == IR_COPY_MASTER && (keys & IR_COPY_KEY_OK) &&
       !key_guard && copy_phase == COPY_WAIT)
        rc_cc_ir_fastcopy_master_send();
    if(keys_released)
        key_guard = 0;
    if(copy_role == IR_COPY_MASTER) {
        if(rc_cc_ir_fastcopy_elapsed(copy_activity, COPY_MASTER_MS)) {
            rc_cc_ir_fastcopy_exit();
            return;
        }
        if(copy_phase == COPY_TX_FIRST && !rc_cc_ir_fastcopy_hw_sending()) {
            copy_gap_start = rc_cc_ir_fastcopy_hw_sent_at();
            rc_cc_ir_fastcopy_hw_stop();
            copy_phase = COPY_TX_GAP;
        }
        if(copy_phase == COPY_TX_GAP && rc_cc_ir_fastcopy_elapsed(copy_gap_start, COPY_KEY_GAP_MS)) {
            if(rc_cc_ir_fastcopy_hw_send(copy_records[1].xdata_ir_buf))
                copy_phase = COPY_TX_SECOND;
            else
                rc_cc_ir_fastcopy_wait();
        }
        if(copy_phase == COPY_TX_SECOND && !rc_cc_ir_fastcopy_hw_sending())
            rc_cc_ir_fastcopy_wait();
        return;
    }
    if(rx_active) {
        if(rc_cc_ir_fastcopy_elapsed(rx_activity, COPY_QUIET_MS)) {
            rc_cc_ir_fastcopy_hw_receive(0);
            if(!rx_failed && !rc_cc_ir_fastcopy_rx_pack())
                rx_failed = 1;
            if(++rx_slot == 2) {
                if(!rx_failed && rc_cc_ir_fastcopy_slave_save())
                    rc_cc_ir_fastcopy_exit();
                else
                    rc_cc_ir_fastcopy_wait();
            } else {
                rc_cc_ir_fastcopy_rx_reset();
                rc_cc_ir_fastcopy_hw_receive(1);
            }
        }
    } else if(rx_slot && rc_cc_ir_fastcopy_elapsed(rx_activity, COPY_SECOND_WAIT_MS))
        rc_cc_ir_fastcopy_wait();
    else if(rc_cc_ir_fastcopy_elapsed(copy_activity, COPY_SLAVE_MS))
        rc_cc_ir_fastcopy_exit();
}

#endif
