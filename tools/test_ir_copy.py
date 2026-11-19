"""Run the production waveform-copy state machine and sender with mocked SDK I/O.

Only SDK includes are replaced; C function bodies are compiled unchanged.
Run: python tools/test_ir_copy.py --cc path/to/gcc
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REMOTE = ROOT / "apps/app/src/mbox_flash/remotecontrol"
PRELUDE = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define TCFG_IR_ENABLE 1
#define TCFG_IR_STUDY_ENABLE 1
#define CFG_IR_KEY_NUM_BEGIN 100
#define MSG_MATRIX_KEY 1
#define SYS_TIMER_JIFFIES ticks
#define sys_timer_msecs_to_jiffies(n) ((n) / 2)
static u32 ticks;
static void local_irq_disable(void) {}
static void local_irq_enable(void) {}
static int post_msg(int count, int msg) { (void)count; (void)msg; return 0; }
"""
CORE_MOCKS = r"""
static union_xdata_ir vm[3], wire[2];
static u16 vm_len[3], voltage;
static int sending, receiving, tx_count, writes, fail_write, fail_read, fail_send;
static u32 sent_at;
u16 ir_valid_key;
static int syscfg_read(u16 id, void *buf, u16 len) {
    int key = id - CFG_IR_KEY_NUM_BEGIN;
    if(fail_read == key || len > vm_len[key]) return -1;
    memcpy(buf, &vm[key], len);
    return len;
}
static int syscfg_write(u16 id, const void *buf, u16 len) {
    int key = id - CFG_IR_KEY_NUM_BEGIN;
    writes++;
    if(writes == fail_write) return -1;
    memcpy(&vm[key], buf, len);
    vm_len[key] = len;
    return len;
}
void system_ir_vm_initial(void) {
    ir_valid_key = 0;
    for(int k = 1; k <= 2; k++)
        if(vm_len[k] >= 2 && vm[k].xdata_ir_buf[0] == 0xab) ir_valid_key |= BIT(k);
}
static u16 app_power_get_vbat_now(void) { return voltage; }
void rc_cc_ir_fastcopy_hw_receive(u8 on) { receiving = on; }
void rc_cc_ir_fastcopy_hw_stop(void) { sending = receiving = 0; }
u8 rc_cc_ir_fastcopy_hw_sending(void) { return sending; }
u32 rc_cc_ir_fastcopy_hw_sent_at(void) { return sent_at; }
u8 rc_cc_ir_fastcopy_hw_send(const u8 *record) {
    if(fail_send) return 0;
    assert(!sending);
    memcpy(&wire[tx_count % 2], record, 128);
    sending = 1;
    tx_count++;
    return 1;
}
"""
LED_MOCKS = r"""
#define TCFG_RGB_ENABLE 0
static u8 ir_copy_combo, led_output;
static u16 ir_copy_hold;
static u8 fir_study, firk_learn, flow_pow, fled_pow, discon_key_led_phase, discon_key_led_on;
static u8 fir_led, fir_led_ir, fled_pair, fgyro_calib, fkey_pair, firk_clrr;
static u8 Connect_led_set(u8 en) { led_output = en; return en; }
"""
FIXTURES = r"""
static void make_record(union_xdata_ir *record, int kind, u32 code) {
    memset(record, 0, sizeof(*record));
    struct _IR_STUDY_STRUCT *r = &record->ir_buf;
    r->ir_study_use[0] = 0xab;
    r->ir_study_h[0] = 9000; r->ir_study_h[1] = 560;
    u32 spaces[] = {4500, 560, 1690, 40000, 2250, 96000};
    memcpy(r->ir_study_l, spaces, sizeof(spaces));
    r->ir_study_buf[0] = 0;
    for(int i = 1; i <= 32; i++) r->ir_study_buf[i] = 0x11 + !!(code & BIT(i-1));
    r->ir_study_buf[33] = 0x13;
    if(kind == 0) { /* NEC-style full first frame and short repeats. */
        r->ir_study_use[1] = 35; r->ir_study_use[2] = 34; r->ir_study_use[3] = 2;
        r->ir_study_buf[34] = 0x04; r->ir_study_buf[35] = 0x15;
    } else if(kind == 1) { /* Repeat the entire frame, not the short NEC repeat. */
        r->ir_study_use[1] = 67; r->ir_study_use[2] = 34; r->ir_study_use[3] = 34;
        memcpy(r->ir_study_buf + 34, r->ir_study_buf, 34);
    } else { /* Largest stored frame: exactly 76 pulse pairs. */
        r->ir_study_use[1] = 75; r->ir_study_use[2] = 0; r->ir_study_use[3] = 76;
        for(int i = 1; i < 75; i++) r->ir_study_buf[i] = 0x11 + (i & 1);
        r->ir_study_buf[75] = 0x13;
    }
}
static int sequence(const union_xdata_ir *record, u32 *out) {
    const struct _IR_STUDY_STRUCT *r = &record->ir_buf;
    int count = 0;
    for(int cycle = 0; cycle < 3; cycle++) {
        int start = cycle == 0 ? 0 : r->ir_study_use[2];
        int len = cycle == 0 && r->ir_study_use[2] ? r->ir_study_use[2] : r->ir_study_use[3];
        for(int i = start; i < start + len; i++) {
            u8 pulse = r->ir_study_buf[i];
            out[count++] = r->ir_study_h[pulse >> 4];
            if(cycle != 2 || i + 1 != start + len)
                out[count++] = r->ir_study_l[pulse & 15];
        }
    }
    return count;
}
"""
CORE_TESTS = r"""
static u32 clock_us;
static void advance(u32 ms) { ticks += ms / 2; clock_us = ticks * 2000; rc_cc_ir_fastcopy_process(); }
static void start(u8 role) {
    rc_cc_ir_fastcopy_enter_request(role); rc_cc_ir_fastcopy_process();
    assert(rc_cc_ir_fastcopy_active() == role && rc_cc_ir_fastcopy_led_on());
    cover_led_high_low(); assert(led_output);
    rc_cc_ir_fastcopy_keys_released(); rc_cc_ir_fastcopy_process();
}
static void press(u8 key) {
    rc_cc_ir_fastcopy_key_request(key); rc_cc_ir_fastcopy_process();
    rc_cc_ir_fastcopy_keys_released(); rc_cc_ir_fastcopy_process();
}
static void reset(void) {
    rc_cc_ir_fastcopy_exit(); rc_cc_ir_fastcopy_keys_released();
    memset(vm, 0, sizeof(vm)); memset(vm_len, 0, sizeof(vm_len));
    ticks = clock_us = tx_count = writes = fail_write = fail_read = fail_send = 0;
    voltage = 3000; ir_valid_key = 0;
    ir_copy_combo = discon_key_led_phase = discon_key_led_on = 0;
}
static void store(int key, const union_xdata_ir *record) {
    vm[key] = *record; vm_len[key] = 53 + record->xdata_ir_buf[1];
    system_ir_vm_initial();
}
static void tx_done(void) {
    sending = 0; sent_at = ticks; rc_cc_ir_fastcopy_process();
}
static void feed(const union_xdata_ir *record, int bad, int jitter) {
    u32 pulse[456]; int count = sequence(record, pulse);
    assert(receiving);
    for(int i = 0; i < count; i++) {
        clock_us += pulse[i]; ticks = clock_us / 2000;
        rc_cc_ir_fastcopy_rx_pulse(!(i & 1), bad && i == 0 ? 10 : pulse[i] + jitter);
        rc_cc_ir_fastcopy_process();
    }
}
static void same_playback(const union_xdata_ir *a, const union_xdata_ir *b) {
    u32 left[456], right[456];
    int count = sequence(a, left);
    assert(sequence(b, right) == count);
    for(int i = 0; i < count; i++) {
        u32 difference = left[i] > right[i] ? left[i] - right[i] : right[i] - left[i];
        assert(difference <= left[i] / 6);
    }
}
static void roundtrip(int mask, int first_kind, int second_kind, int jitter) {
    union_xdata_ir first, second, sent[2];
    make_record(&first, first_kind, 0x12ab34cd);
    make_record(&second, second_kind, 0xa5a55a5a);
    reset();
    if(mask & 1) store(1, &first);
    if(mask & 2) store(2, &second);
    start(IR_COPY_MASTER); press(IR_COPY_KEY_OK);
    assert(tx_count == 1 && sending && !receiving);
    sent[0] = wire[0];
    same_playback(&sent[0], mask & 1 ? &first : &second);
    tx_done(); assert(copy_phase == COPY_TX_GAP);
    advance(298); assert(tx_count == 1);
    advance(2); assert(tx_count == 2 && copy_phase == COPY_TX_SECOND);
    sent[1] = wire[1];
    same_playback(&sent[1], mask & 2 ? &second : &first);
    tx_done(); assert(copy_phase == COPY_WAIT && rc_cc_ir_fastcopy_led_on() && rc_cc_ir_fastcopy_active());
    reset(); store(1, &second); store(2, &first);
    start(IR_COPY_SLAVE);
    feed(&sent[0], 0, jitter);
    assert(!writes && rx_slot == 0);
    advance(198); assert(rx_slot == 0);
    advance(2); assert(rx_slot == 1 && !writes && receiving);
    advance(100); feed(&sent[1], 0, jitter);
    assert(!writes);
    advance(200);
    assert(!rc_cc_ir_fastcopy_active() && !receiving && !rc_cc_ir_fastcopy_led_on() && writes == 2);
    assert(ir_valid_key == 6);
    same_playback(&sent[0], &vm[1]); same_playback(&sent[1], &vm[2]);
    system_ir_vm_initial(); assert(ir_valid_key == 6);
}
static void failed_round(int bad_slot, int write_failure, int low_battery) {
    union_xdata_ir source, old[3];
    make_record(&source, 0, 0xa55aa55a);
    reset(); store(1, &source); store(2, &source);
    memcpy(old, vm, sizeof(vm)); fail_write = write_failure;
    if(low_battery) voltage = 2000;
    start(IR_COPY_SLAVE);
    feed(&source, bad_slot == 1, 0); advance(200);
    assert(!writes && rx_slot == 1);
    advance(100); feed(&source, bad_slot == 2, 0); advance(200);
    assert(rc_cc_ir_fastcopy_active() == IR_COPY_SLAVE && rc_cc_ir_fastcopy_led_on() && receiving && rx_slot == 0);
    if(bad_slot || low_battery || write_failure == 1)
        assert(!memcmp(old, vm, sizeof(vm)));
    advance(7998); assert(rc_cc_ir_fastcopy_active()); advance(2); assert(!rc_cc_ir_fastcopy_active());
}
int main(void) {
    for(int mask = 1; mask <= 3; mask++)
        for(int kind = 0; kind <= 2; kind++) roundtrip(mask, kind, (kind + 1) % 3, 0);
    roundtrip(3, 0, 1, -25); roundtrip(3, 1, 0, 25);
    for(int bad = 1; bad <= 2; bad++) failed_round(bad, 0, 0);
    failed_round(0, 1, 0); failed_round(0, 2, 0); failed_round(0, 0, 1);

    reset(); start(IR_COPY_MASTER); press(IR_COPY_KEY_OK);
    assert(!tx_count && rc_cc_ir_fastcopy_led_on());
    union_xdata_ir source; make_record(&source, 0, 0x12345678); store(1, &source);
    fail_read = 1; press(IR_COPY_KEY_OK); assert(!tx_count);
    fail_read = 0; fail_send = 1; press(IR_COPY_KEY_OK); assert(!tx_count && copy_phase == COPY_WAIT);
    fail_send = 0;
    vm[1].ir_buf.ir_study_l[5] = 200000; press(IR_COPY_KEY_OK); assert(!tx_count);
    store(1, &source); vm[1].ir_buf.ir_study_buf[0] = 0xf0;
    press(IR_COPY_KEY_OK); assert(!tx_count);

    /* Losing the complete first frame must not learn two short repeat codes as a key. */
    reset(); start(IR_COPY_SLAVE);
    for(int repeat = 0; repeat < 2; repeat++) {
        rc_cc_ir_fastcopy_rx_pulse(1, 9000); rc_cc_ir_fastcopy_rx_pulse(0, 2250);
        rc_cc_ir_fastcopy_rx_pulse(1, 560);
        if(repeat == 0) rc_cc_ir_fastcopy_rx_pulse(0, 96000);
    }
    advance(200); assert(rx_failed && rx_slot == 1 && !writes);

    /* An incomplete burst with a missing space must fail without indexing table[15]. */
    reset(); start(IR_COPY_SLAVE);
    rc_cc_ir_fastcopy_rx_pulse(1, 560); rc_cc_ir_fastcopy_rx_pulse(1, 560);
    rc_cc_ir_fastcopy_rx_pulse(0, 10000); rc_cc_ir_fastcopy_rx_pulse(1, 560);
    rc_cc_ir_fastcopy_rx_pulse(0, 10000); rc_cc_ir_fastcopy_rx_pulse(1, 560);
    advance(200); assert(rx_failed && !writes);

    /* A new carrier starts before 200 ms: do not time out while its mark is still arriving. */
    reset(); start(IR_COPY_SLAVE); rc_cc_ir_fastcopy_rx_pulse(1, 560);
    advance(198); rc_cc_ir_fastcopy_rx_pulse(0, 198000);
    advance(4); assert(rx_slot == 0 && rx_active && !writes);

    /* The initial frame's gap may be longer than the repeat frame's gap. */
    reset(); start(IR_COPY_SLAVE);
    source.ir_buf.ir_study_l[3] = 150000;
    feed(&source, 0, 0); advance(200);
    assert(!rx_failed && rx_slot == 1);
    make_record(&source, 0, 0x12345678);

    reset(); store(1, &source); rc_cc_ir_fastcopy_enter_request(IR_COPY_MASTER); rc_cc_ir_fastcopy_process();
    rc_cc_ir_fastcopy_key_request(IR_COPY_KEY_OK); rc_cc_ir_fastcopy_process(); assert(!tx_count);
    rc_cc_ir_fastcopy_keys_released(); press(IR_COPY_KEY_OK); assert(tx_count == 1);
    press(IR_COPY_KEY_OK); assert(tx_count == 1); /* A running round cannot be restarted. */
    tx_done(); press(IR_COPY_KEY_BACK); assert(!rc_cc_ir_fastcopy_active() && !sending);

    reset(); start(IR_COPY_MASTER); advance(1799998); assert(rc_cc_ir_fastcopy_active());
    press(IR_COPY_KEY_OTHER); advance(1799998); assert(rc_cc_ir_fastcopy_active());
    advance(2); assert(!rc_cc_ir_fastcopy_active());
    reset(); ticks = UINT32_MAX - 100; start(IR_COPY_MASTER);
    advance(1800000); assert(!rc_cc_ir_fastcopy_active());
    reset(); start(IR_COPY_SLAVE); advance(7998); press(IR_COPY_KEY_OTHER);
    advance(2); assert(rc_cc_ir_fastcopy_active());
    advance(7996); assert(rc_cc_ir_fastcopy_active());
    advance(2); assert(!rc_cc_ir_fastcopy_active());

    reset(); start(IR_COPY_SLAVE);
    for(int key = IR_COPY_KEY_OTHER; key <= IR_COPY_KEY_OK; key++) {
        advance(7000); press(key); assert(rc_cc_ir_fastcopy_active());
    }
    advance(7998); assert(rc_cc_ir_fastcopy_active() && !tx_count && !writes);
    advance(2); assert(!rc_cc_ir_fastcopy_active());

    /* A release while another key is held also restarts the child idle timer. */
    reset(); start(IR_COPY_SLAVE); advance(7000);
    matrix_activity(3, 1); rc_cc_ir_fastcopy_process();
    advance(7998); assert(rc_cc_ir_fastcopy_active());
    matrix_activity(1, 1); rc_cc_ir_fastcopy_process(); /* Unchanged scan is not activity. */
    advance(2); assert(!rc_cc_ir_fastcopy_active());

    /* Polling an older RX timestamp must not undo a more recent key event. */
    reset(); start(IR_COPY_SLAVE); rc_cc_ir_fastcopy_rx_pulse(1, 560);
    advance(60); press(IR_COPY_KEY_OTHER);
    u32 key_time = ticks;
    advance(60); assert(copy_activity == key_time);
    rc_cc_ir_fastcopy_rx_pulse(0, 1000); assert(copy_activity == ticks);

    reset(); start(IR_COPY_SLAVE); feed(&source, 0, 0); advance(200);
    advance(8000); assert(rc_cc_ir_fastcopy_active() && rx_slot == 0 && !writes && rc_cc_ir_fastcopy_led_on());
    advance(8000); assert(!rc_cc_ir_fastcopy_active());
    reset(); start(IR_COPY_SLAVE);
    for(int i = 0; i <= COPY_RAW_MAX; i++) {
        rc_cc_ir_fastcopy_rx_pulse(1, 560); rc_cc_ir_fastcopy_rx_pulse(0, 560);
    }
    assert(rx_failed && rx_count == COPY_RAW_MAX && !writes && rc_cc_ir_fastcopy_led_on());
    for(int role = 1; role <= 2; role++) {
        reset(); ir_copy_combo = role; ir_copy_hold = 0;
        int threshold = role == IR_COPY_MASTER ? 300 : 250;
        for(int scan = 0; scan < threshold - 1; scan++) entry_scan();
        assert(!enter_request && !rc_cc_ir_fastcopy_active());
        entry_scan(); assert(enter_request == role && ir_copy_hold == threshold);
        rc_cc_ir_fastcopy_process(); assert(rc_cc_ir_fastcopy_active() == role);
        reset(); ir_copy_combo = role; discon_key_led_phase = 4;
        cover_led_high_low(); assert(!led_output); /* Held combo no longer overrides the normal LED. */
        discon_key_led_phase = 0; start(role);
        for(int step = 0; step < 4; step++) {
            advance(100); cover_led_high_low(); assert(led_output); /* Enter ready mode without a flash. */
        }
        ir_copy_combo = 0;
        reset(); start(role); rc_cc_ir_fastcopy_key_request(IR_COPY_KEY_BACK); rc_cc_ir_fastcopy_process();
        assert(!rc_cc_ir_fastcopy_active() && rc_cc_ir_fastcopy_blocks_keys() && !receiving);
        rc_cc_ir_fastcopy_keys_released(); assert(!rc_cc_ir_fastcopy_blocks_keys());
    }
    puts("PASS: waveform roundtrips, single-key duplication, 300 ms separation, staging, failures and timeouts");
    return 0;
}
"""
TX_MOCKS = r"""
struct gptimer_config { u32 resolution_us; void (*irq_cb)(int); int tid; u16 irq_priority; };
static u8 fir_star, fir_study_star, ir_learn_rx_bound;
static int pwm_tid = 3, pwm, duration, paused, released;
static void ir_learn_stop_capture(void) {}
static void app_irflt_config(void) {}
static void unrequest_irq(int irq) { (void)irq; }
static void gpio_disable_function(int pin, int fn) { (void)pin; (void)fn; }
#define IRQ_TIME1_IDX 1
#define IO_PORT_SPILT(pin) pin
#define IR_RX_PIN 5
#define PORT_FUNC_TIMER1_CAPTURE 1
static int gptimer_init(const struct gptimer_config *cfg) { (void)cfg; return 4; }
static void gptimer_start(int tid) { (void)tid; paused = 0; }
static void gptimer_pause(int tid) { (void)tid; paused = 1; }
static void gptimer_deinit(int tid) { (void)tid; released++; }
static void gptimer_pwm_enable(int tid) { (void)tid; pwm = 1; }
static void gptimer_pwm_disable(int tid) { (void)tid; pwm = 0; }
static void gptimer_set_resolution(int tid, u32 us) { (void)tid; duration = us; }
static void gptimer_set_count(int tid, int n) { (void)tid; (void)n; }
"""
TX_TESTS = r"""
int main(void) {
    for(int kind = 0; kind < 3; kind++) {
        union_xdata_ir source; u32 expected[456];
        make_record(&source, kind, 0x12ab34cd);
        int count = sequence(&source, expected), old_released = released;
        assert(rc_cc_ir_fastcopy_hw_send(source.xdata_ir_buf) && rc_cc_ir_fastcopy_hw_sending());
        for(int i = 0; i < count; i++) {
            rc_cc_ir_fastcopy_tx_irq(4);
            assert(pwm == !(i & 1) && duration == (int)expected[i] - 40);
        }
        ticks = 1234;
        rc_cc_ir_fastcopy_tx_irq(4); /* End at the last mark, without transmitting its trailing space. */
        assert(!pwm && paused && !rc_cc_ir_fastcopy_hw_sending() && rc_cc_ir_fastcopy_hw_sent_at() == 1234);
        rc_cc_ir_fastcopy_hw_stop(); assert(released == old_released + 1 && copy_tx_tid == -1);
    }
    fir_star = 1; union_xdata_ir source; make_record(&source, 0, 0);
    assert(!rc_cc_ir_fastcopy_hw_send(source.xdata_ir_buf));
    puts("PASS: production sender emits one first frame plus two repeats, full-frame repeats, complete stop and cleanup");
    return 0;
}
"""


def without_includes(source):
    return re.sub(r"^#include[^\n]*", "", source, flags=re.MULTILINE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("gcc"))
    args = parser.parse_args()
    if not args.cc:
        parser.error("GCC is required; specify --cc")
    header = without_includes((REMOTE / "rc_ir.h").read_text(encoding="latin1"))
    header += without_includes((REMOTE / "rc_cc_ir_fastcopy.h").read_text())
    core = without_includes((REMOTE / "rc_cc_ir_fastcopy.c").read_text())
    driver = (REMOTE / "rc_ir.c").read_text(encoding="latin1")
    sender = driver.split("static volatile u8 copy_tx_busy;", 1)[1].split(
        "___interrupt\nvoid timer_ir_rx_isr", 1)[0]
    sender = "static volatile u8 copy_tx_busy;" + sender
    key_source = (REMOTE / "rc_key.c").read_text(encoding="latin1")
    threshold = re.search(r"u16 copy_hold_ticks =[^;]+;", key_source).group()
    entry = re.search(r"if\(ir_copy_combo &&[^\n]+\)\s*rc_cc_ir_fastcopy_enter_request\(ir_copy_combo\);", key_source).group()
    entry = "static void entry_scan(void) {" + threshold + entry + "}"
    activity = re.search(r"if\(rc_cc_ir_fastcopy_active\(\) == IR_COPY_SLAVE &&\s*memcmp\(previous_keys, PRE_BUFF, sizeof\(previous_keys\)\)\)\s*rc_cc_ir_fastcopy_key_request\(IR_COPY_KEY_OTHER\);", key_source).group()
    activity = "static void matrix_activity(u8 previous, u8 current) {u8 previous_keys[1] = {previous}, PRE_BUFF[1] = {current};" + activity + "}"
    led_source = (REMOTE / "rc_led.c").read_text(encoding="latin1")
    cover = led_source.split("void cover_led_high_low(void)", 1)[1].split("u8 rcled_idle_query", 1)[0]
    cover = "void cover_led_high_low(void)" + cover
    cases = {
        "core": PRELUDE + header + CORE_MOCKS + core + LED_MOCKS + cover + entry + activity + FIXTURES + CORE_TESTS,
        "sender": PRELUDE + header + TX_MOCKS + sender + FIXTURES + TX_TESTS,
    }
    with tempfile.TemporaryDirectory(prefix="cc08_ir_copy_") as tmp:
        for name, source in cases.items():
            c = Path(tmp) / f"{name}.c"
            exe = Path(tmp) / f"{name}.exe"
            c.write_text(source, encoding="latin1")
            subprocess.run([args.cc, "-std=c99", "-O1", "-Wall", "-Wextra",
                            "-Wno-unused-function", "-Wno-unused-variable",
                            "-Wno-unused-but-set-variable",
                            "-Werror", str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
