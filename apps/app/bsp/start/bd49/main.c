/*********************************************************************************************
    *   Filename        : main.c

    *   Description     :

    *   Author          :

    *   Email           :

    *   Last modifiled  :

    *   Copyright:(c)JIELI  2011-2017  @ , All Rights Reserved.
*********************************************************************************************/

#pragma bss_seg(".main.data.bss")
#pragma data_seg(".main.data")
#pragma const_seg(".main.text.const")
#pragma code_seg(".main.text")
#pragma str_literal_override(".main.text.const")

#include "config.h"
#include "common.h"
/* #include "gpio.h" */
#include "clock.h"
#include "maskrom.h"
/* #include "asm/power/p33.h" */
#include "app_config.h"
#include "init.h"
/* #include "init_app.h" */
#include "app.h"
/* #include "msg.h" */
#include "device.h"
/* #include "flash_init.h" */
/* #include "asm/power_interface.h" */
/* #include "power_api.h" */
/* #include "asm/debug.h" */
#include "wdt.h"
#include "efuse.h"
#include "rtc.h"
/* #include "vm.h" */
#include "power_interface.h"
/* #include "power_api.h" */

#include "my_malloc.h"
#include "sys_timer.h"
#include "app_reg_include.h"

#define LOG_TAG_CONST       MAIN
#define LOG_TAG             "[main]"
#include "log.h"

#include "msg.h"

#include "crc16.h"

u32 get_app_bin_crc()
{
    //把app ld.c 的app..code / data / debug data 区域的起始地址 和末尾地址extern出来
    extern const u8 app_code_real_begin[];
    extern const u8 app_code_real_end[];
    extern const u8 data_buf_start[];
    extern const u8 data_buf_end[];
    extern const u8 debug_buf_start[];
    extern const u8 debug_buf_end[];

    u32 app_code_begin = (u32) app_code_real_begin;
    u32 app_code_end  = (u32)app_code_real_end;
    u32 data_begin = (u32)data_buf_start;
    u32 data_end = (u32)data_buf_end;
    u32 debug_data_begin = (u32) debug_buf_start;
    u32 debug_data_end  = (u32)debug_buf_end;
    
    u32 app_code_len = app_code_end-app_code_begin;//app code区域的大小
    u32 data_len = data_end-data_begin;//data区域的大小
    u32 debug_data_len = debug_data_end - debug_data_begin;//.debug_data区域的大小

    //计算anpcade区域的CRC值
    u32 app_code_crc  = CRC16((void *)app_code_begin, app_code_len);
    //计真 appcode 和 data 区域的CRC值
    u32 app_bin_crcl = CRC16_with_initval((void *)data_begin,data_len,app_code_crc);
    //计算app_code和data 和 debug_data 区域的CRC值(app_bin_crc2 即最终app.bin文件的crc值)
    u32 app_bin_crc2 = CRC16_with_initval((void *)debug_data_begin,debug_data_len,app_bin_crcl);
    return app_bin_crc2;
}

extern const u8 config_no_osc_enable;
__attribute__((noreturn))

//void audio_adc_demo(void);
u32 app_bin_crc = 0;
void c_main(int cfg_addr)
{
    // app_bin_crc = get_app_bin_crc();
    log_init(TCFG_UART_BAUDRATE);

    log_info("1 hello word bd49\n");
    _system_bool_byte = 0;
    _ir_bool_byte = 0;
    _rf_bool_byte = 0;
    // _iic_bool_byte = 0;
    _gy_bool_byte = 0;
    _key_bool_byte = 0;
    _led_bool_byte = 0;

    struct maskrom_argv mask_argv = {0};
    mask_argv.pchar = (void *)putchar;
    mask_argv.exp_hook = (void *)exception_analyze;
    mask_argv.local_irq_enable = NULL;
    mask_argv.local_irq_disable = NULL;
    mask_argv.flt = NULL;

    HWI_Install(1, (u32)exception_irq_handler, 7) ;
    mask_init(&mask_argv);
    emu_init();
    wdt_init(WDT_8S);
    efuse_init();
    /* clk_out0(6,CLK_OUT_PLL_96M); */
    /* clk_out0(6,CLK_OUT_STD_12M); */
    /* clk_out2(6,CLK_OUT2_STD_48M,1); */
    /* clk_out2(6,CLK_OUT2_SYSPLL_D2P0,1); */
    clk_voltage_init(CLOCK_MODE_ADAPTIVE, DVDD_VOL_129V);
    if (config_no_osc_enable) {
        clk_early_init(PLL_REF_LRC, 200000, 480000000);
    } else {
        clk_early_init(PLL_REF_XOSC_DIFF, 24000000, 480000000);
    }
    clock_dump();
    clk_set("sys", TCFG_SYS_PLL_CLK);
    app_bin_crc = get_app_bin_crc();

    /* while(1); */

    //test: pmu_oe
    JL_WLA->WLA_CON10 |= BIT(15);
    //reinit uart io
    /*JL_OMAP->PA5_OUT = 0;*/
    /*JL_PORTA->DIR &= ~BIT(6);*/
    /*JL_OMAP->PA6_OUT = FO_UART1_TX;*/

    //==============TODO: reinit uart
    //port_init();

    log_info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    log_info("       bd49 setup %s-%s", __DATE__, __TIME__);
    log_info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    efuse_dump();

    my_malloc_init();

#if SYS_TIMER_EN
    sys_timer_init();
#endif

    power_early_flowing();

    printf("PWR_WK_REASON_PORT_EDGE:%d\n",is_wakeup_source(PWR_WK_REASON_PORT_EDGE));
    if(is_wakeup_source(PWR_WK_REASON_PORT_EDGE))  //IO唤醒
    {
        printf("PWR_WK_REASON_PORT_EDGE:%d\n",is_wakeup_source(PWR_WK_REASON_PORT_EDGE));
        fsys_powup = 1;
    }

    board_power_init();
    power_io_wakeup_init();
    log_info("app_bin_crc:%x\n",app_bin_crc);

    /* test_soft(); */
    /* test_apa(); */
    /* boot_info_test(); */
    /* my_malloc_init(); */
    /* vfs_demo_sydfs(); */
    system_init();

    power_later_init();

    //fat测试
    /* void fat_demo(void); */
    /* fat_demo(); */


    //设备升级测试
    /* y_printf("\n >>>[test]:func = %s,line= %d\n",__FUNCTION__, __LINE__); */
    /* delay_10ms(100); */
    /* u16 dev_update_check(char *logo); */
    /* int err = dev_update_check("sd0"); */
    /* y_printf(">>>[test]:check err??????????? err = 0x%x\n", err); */


    app();
    //audio_adc_demo();
    /* decoder_func(); */
    u32 i = 0;
    while (1) {
        if (i > 0xfffff) {
            log_char('.');
            i = 0;
        }
        i++;
    }
#if 0
    wdt_close();

    mask_init_for_app();
    irq_init();
    local_irq_disable();
    local_irq_enable();

    //上电初始化所有IO
    port_init();

    log_init(TCFG_UART_BAUDRATE);

    pll_sel(TCFG_PLL_SEL, TCFG_PLL_DIV, TCFG_HSB_DIV);

    dump_clock_info();

    debug_init();

    wdt_init(WDT_8S);

    P3_PINR_CON &= ~BIT(0); //关闭长按复位

    /* gpio_clk_out(IO_PORTC_00, CLK_OUT_HSB); */

    log_info("time & date %s %s \n  OTP c_main\n", __TIME__, __DATE__);

    power_reset_source_dump();
    power_wakeup_reason_dump();
    sys_power_init();

    system_init();
    app();
    while (1) {
        wdt_clear();
    }
#endif
}


