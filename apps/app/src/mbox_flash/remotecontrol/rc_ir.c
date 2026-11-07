/*********************************************************************************************
    *   Filename        : rc_ir.c

    *   Description     :

    *   Author          :

    *   Email           :

    *   Last modifiled  : 2024-02-21 11:21

    *   Copyright:(c)XingWei  2011-2019  @ , All Rights Reserved.
*********************************************************************************************/
#include "typedef.h"
#include "rc_ir.h"
#include "msg.h"
#include "app_config.h"
#include "rc_app.h"
#include "config.h"
#include "gpio.h"
#include "vble_complete.h"
#include "bt_ble.h"

#include "gptimer.h"
#include "sys_timer.h"
#include "hw_timer.h"
#include "cpu.h"

#include "app_reg_include.h"


#if(TCFG_IR_ENABLE)
#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[RC_IR]"
#include "log.h"


static volatile u8 is_ir_active = 0;

_bool_int  _ir_bool_bit;

u8 ir_led_times;
u8	IR_KEY_NUM;
static u8 ir_learn_key;
u16 IR_STEP;
u32 max_time;
u32 dat_Period;
u16 retran_long;
u16 ir_valid_key;
u8 timer_isr_str;
static u16 ir_learn_idle_timer;
static u16 ir_learn_frame_timer;
static u16 ir_learn_feedback_timer;
static volatile u16 ir_learn_led_edges;
static u16 ir_study_timer;

#define IR_LEARN_IDLE_MS     20000
#define IR_LEARN_FLASH_ON_MS   100
#define IR_LEARN_FLASH_OFF_MS  200

static u8  high_num,low_num,max_num,max_l_ct;
static u8  IR_STUDY_CON[3],comp_buf[3];

union_xdata_ir	_struct_ir_xdata;
#define IR_STUDY_ARRAY _struct_ir_xdata.xdata_ir_buf
#define IR_STUDY_BUF _struct_ir_xdata.ir_buf.ir_study_buf
#define IR_STUDY_H _struct_ir_xdata.ir_buf.ir_study_h
#define IR_STUDY_L _struct_ir_xdata.ir_buf.ir_study_l
#define study_buf_ct _struct_ir_xdata.ir_buf.ir_study_use[1]
#define con_num _struct_ir_xdata.ir_buf.ir_study_use[2]
#define IR_con_long _struct_ir_xdata.ir_buf.ir_study_use[3]

#define PWM_ON   {gpio_set_pull_down(IR_TX_PIN,1);JL_TIMER4->CNT = 0;JL_TIMER4->CON |= (BIT(0));}
#define PWM_OFF  {gpio_set_pull_down(IR_TX_PIN,0);gpio_direction_output(IR_TX_PIN,0);JL_TIMER4->CON &=~(BIT(0));}

#define src_clk	24000000
#define	prd_1us 1//(src_clk / (8 * 1000000))

#define NEC_FRE_38K		38000
#define NEC_DUTY_30		3000
static struct gptimer_pwm_config *pwm_config = NULL;
static struct gptimer_pwm_config _pwm_config;
static int pwm_tid;
static struct gptimer_config *timer_config = NULL;
static struct gptimer_config _timer_config;
static int timer_tid;

void timer5_ir_init(void)
{
    timer_config = &_timer_config;
    timer_config->resolution_us = 563;
    timer_config->irq_cb = system_ir_tran;
    timer_config->tid = -1;
    timer_config->irq_priority = 7; //使用不可屏蔽中断
    timer_tid = gptimer_init(timer_config);
    //log_info("timer_config tid = %d\n", timer_tid);
}


void system_ir_initial(void)
{
    log_info("func:%s(), line:%d\n", __func__, __LINE__);

    pwm_config = &_pwm_config;
    pwm_config->port = IR_TX_PIN / IO_GROUP_NUM; //PORTA;
    pwm_config->pin = IR_TX_PIN % IO_GROUP_NUM; //3;
    pwm_config->freq = NEC_FRE_38K;
    pwm_config->pwm_duty_X10000 = NEC_DUTY_30;
    pwm_config->tid = -1;
    pwm_tid = gptimer_pwm_init(pwm_config);
	// gptimer_pwm_flip(pwm_tid);
    log_info("pwm_config tid = %d\n", pwm_tid);
    gpio_set_mode(IO_PORT_SPILT(IR_TX_PIN), 0);

    gptimer_start(pwm_tid);
    gptimer_pwm_disable(pwm_tid);
	//timer5_ir_init();

	is_ir_active =0;

	IR_KEY_NUM =0;
}
void system_ir_vm_initial(void)
{
	u8 i,array[2];
	ir_valid_key =0;

	for(i=0;i<TOTAL_IR_KEY;i++)
	{
		array[0]=0;
		syscfg_read(CFG_IR_KEY_NUM_BEGIN+i,array,2);
		if(array[0]==0xab)//�����Ѿ��洢������Ч
			ir_valid_key |=1<<i;
	}

}

//-------------------------------------------------------
//Func  : ��EEPROM�����������
//Desc	:
//Input	:
//Output:
//Return:
//Author: Owen
//Date	: 2018/06/30
//-------------------------------------------------------
void system_ir_clrr_key(void)
{
	u8 i,array[2];

	for(i=0;i<TOTAL_IR_KEY;i++)
	{
		array[0] =0;
		syscfg_write(CFG_IR_KEY_NUM_BEGIN+i,array,1);
	}

 	ir_valid_key =0;
}
void ir_tran_PWM_OFF(void)
{
	log_info("ir_tran_PWM_OFF\n");
	gptimer_pwm_disable(pwm_tid);
	gptimer_deinit(timer_tid);
    //gptimer_pause(timer_tid);
	fpwr_debug_pre = 1;
  	fir_star =0;
  	fir_retran =0;
	fir_study_send = 0;
  	fir_send =0;
  	is_ir_active =0;
}
//-------------------------------------------------------
//Func  : ��EEPROM����ѹ�����Ͻ���
//Desc	:
//Input	:
//Output:
//Return:
//Author: Owen
//Date	: 2018/06/30
//-------------------------------------------------------
void system_ir_check(void)
{
  u8 array[2],i;
  if(fir_study)
	 return;
  if(fir_send)
  {
  	if(!fir_star)
	{
		syscfg_read(CFG_IR_KEY_NUM_BEGIN+IR_KEY_NUM,array,2);
		if(array[0]==0xab)//�����Ѿ��洢������Ч
		{
			log_info("ir id =ab\n");
			syscfg_read(CFG_IR_KEY_NUM_BEGIN+IR_KEY_NUM,IR_STUDY_ARRAY,array[1]+4+24+24+1);

			IR_STEP =0;
			fir_star =1;
			is_ir_active =1;

			timer5_ir_init();
			gptimer_set_resolution(timer_tid,5000);
			gptimer_start(timer_tid);
		}
		else fir_study_send = 0;
		//else system_ir_only_send(0x01,0xfe,0x40);
	}
  }
}

void IR_STUDY_BUF_set(u8 r,u8 buf)
{
  u8 i;

  for(i=0;i<8;i++)
  	{
	  if((buf&0x1)==0)
		{
		 dat_Period +=(1125-560);
		 IR_STUDY_BUF[r+i] =0x11;
		}
	  else {
	  	 IR_STUDY_BUF[r+i]=0x12;
		 dat_Period +=(2250-560);
	  	}
	  buf>>=1;
  	}
}

u8 system_ir_only_send(u8 add1,u8 add2,u8 IR_DAT)
{
 if(IR_DAT==0xfe)
	return 0xfe;
 if(fir_study || fir_star)//????????????????????????????????????????????????
	{
		return 0xfe;
	}
 study_buf_ct=35;//�ܳ���
 con_num=34;//������ʼλ��

 IR_STUDY_L[0]=4500;
 IR_STUDY_L[1]=1125-560;
 IR_STUDY_L[2]=2250-560;
 //IR_STUDY_L[3]=0x285f;
 IR_STUDY_L[4]=2250;
 IR_STUDY_L[5]=96190;
 IR_STUDY_H[0]=9000;
 IR_STUDY_H[1]=560;

 IR_STUDY_BUF[0]=0x00;
 dat_Period =9000+4500+(560*33);//(9+4.5+0.56*33)

 IR_STUDY_BUF_set(1,add1);//addr1
 IR_STUDY_BUF_set(1+8,add2);//addr2
 IR_STUDY_BUF_set(1+8+8,IR_DAT);
 IR_STUDY_BUF_set(1+8+8+8,~IR_DAT);

 IR_STUDY_L[3]=108000 -dat_Period;//108000

 IR_STUDY_BUF[33+0] =0x13;
 IR_STUDY_BUF[33+1] =0x04;
 IR_STUDY_BUF[33+2] =0x15;

 IR_con_long=2;//���볤��
log_info("system_ir_only_send\n");
 IR_STEP =0;
 fir_star =1;
 is_ir_active =1;
 timer5_ir_init();
 gptimer_set_resolution(timer_tid,5000);
 gptimer_start(timer_tid);
 return 0;
}


static void system_ir_rece(void *priv);

static void ir_learn_stop_capture(void)
{
	fir_study_star = 0;
	JL_TIMER1->CON &= ~(0b11 << 0);
	JL_TIMER2->CON = 0;
	if(ir_learn_frame_timer)
		sys_timeout_del(ir_learn_frame_timer);
	ir_learn_frame_timer = 0;
}

static void ir_learn_idle_timeout(void *priv)
{
	ir_learn_idle_timer = 0;
	user_ir_learn_timeout();
}

static void ir_learn_refresh_timeout(void)
{
	if(ir_learn_idle_timer)
		sys_timer_modify(ir_learn_idle_timer, IR_LEARN_IDLE_MS);
	else
		ir_learn_idle_timer = sys_timeout_add(NULL, ir_learn_idle_timeout, IR_LEARN_IDLE_MS);
}

void user_ir_learn_start(void)
{
	fir_study = 1;
	fir_send = 0;
	ir_led_times = 0;
	fir_led_ir = 1;
	ir_learn_refresh_timeout();
}

static void ir_learn_led_step(void *priv)
{
	ir_learn_feedback_timer = 0;
	if(!fir_study || !ir_learn_led_edges)
		return;

	fir_led_ir = !fir_led_ir;
	if(--ir_learn_led_edges)
	{
		u16 delay_ms = fir_led_ir ? IR_LEARN_FLASH_ON_MS : IR_LEARN_FLASH_OFF_MS;
		ir_learn_feedback_timer = sys_timeout_add(NULL, ir_learn_led_step, delay_ms);
	}
}

static void ir_learn_blink(u8 flashes)
{
	/* Finish the current indication if a result arrives during the selection flash. */
	local_irq_disable();
	ir_learn_led_edges += 2 * flashes;
	u8 start = !ir_learn_feedback_timer;
	local_irq_enable();
	if(start)
		ir_learn_led_step(NULL);
}

void user_ir_learn_select(u8 key)
{
	if(!fir_study || key < 1 || key >= TOTAL_IR_KEY)
		return;

	ir_learn_refresh_timeout();
	ir_learn_stop_capture();
	fir_learn_finish = 0;
	if(ir_study_timer)
		sys_timeout_del(ir_study_timer);
	ir_learn_key = key;
	ir_learn_blink(1);
	system_ir_rece(NULL);
}

/* Called by MSG_MATRIX_KEY so flash writes stay outside the timer ISR. */
void user_ir_learn_process(u8 battery_critical)
{
	if(!fir_study || !fir_learn_finish || !ir_learn_key)
		return;

	fir_learn_finish = 0;
	u8 key = ir_learn_key;
	int len = 4 + 24 + 24 + study_buf_ct + 1;
	IR_STUDY_ARRAY[0] = 0xab;
	IR_con_long = comp_buf[0];
	if(fir_learn_fail || battery_critical || syscfg_write(CFG_IR_KEY_NUM_BEGIN + key, IR_STUDY_ARRAY, len) != len)
	{
		if(fir_study)
			ir_study_timer = sys_timeout_add(NULL, system_ir_rece, 1200);
		return;
	}

	ir_valid_key |= BIT(key);
	if(!fir_study)
		return;
	ir_learn_key = 0;
	ir_learn_refresh_timeout();
	ir_learn_blink(2);
}

static void user_ir_learn_out(void)
{
	ir_learn_stop_capture();
	fir_learn_finish = 1;
	post_msg(1, MSG_MATRIX_KEY);
}

static void user_ir_learn_finish(void *priv)
{
	u8 i;
	ir_learn_frame_timer = 0;
	ir_learn_stop_capture();
	fir_learn_fail = 1;

	if(con_num==2)
	   {
		  con_num =3;
		  IR_STUDY_CON[2] =study_buf_ct;
		  IR_STUDY_BUF[study_buf_ct] |=IR_STUDY_BUF[IR_STUDY_CON[0]]&0x0f;
		  if(ir_set_continue())
		  	{
			  if(comp_buf[0] )
			  {
				fir_learn_fail = 0;
			  }
		  	}
	   }
	else if(con_num==1 && IR_STUDY_CON[0] >= 33){
	  IR_STUDY_BUF[study_buf_ct] |=0xf;//��ʱ
	  if((abs(IR_STUDY_H[0]/prd_1us-9000)<1000)&&(abs(IR_STUDY_L[0]/prd_1us-4500)<500))//9ms
		 {
 		  con_num =34;
		  study_buf_ct =35;
		  comp_buf[0] =2;
		  IR_STUDY_BUF[33+0] =0x13;
		  IR_STUDY_BUF[33+1] =0x04;
		  IR_STUDY_BUF[33+2] =0x15;

		  IR_STUDY_L[4]=2250*prd_1us;//2250
		  IR_STUDY_L[5]=96190*prd_1us;//96190
		  dat_Period =9000+4500+(560*33);//(9+4.5+0.56*33)
		  for(i=1;i<=32;i++)
			 {
			  if((IR_STUDY_BUF[i]&0x0f)==1)
				 dat_Period +=(2250-560);
			  else dat_Period +=(1125-560);
			 }
		  IR_STUDY_L[3]=(108000 -dat_Period)*prd_1us; 
		  log_info("ir_con==%d\n",study_buf_ct);
		  
		  fir_learn_fail = 0;
		 }
	 }
	user_ir_learn_out();
}

void ir_timer2_inti(void)
{
	JL_TIMER2->CON = TIMER_PCLR;  //清PND, 清寄存器
    JL_TIMER2->CNT = 0; //清除计数器
    JL_TIMER2->PRD = 0xffffffff;  //设置计数值，最大计时时间 = 1/(时钟源/分频系数)*65535，单位:s
    JL_TIMER2->CON = (TIMER_SSEL_STD_24M | TIMER_PSET_64); //24M时钟源，8分频
    JL_TIMER2->CON |= (TIMER_MODE_TIME); //计时模式启动
}

void user_ir_learn_timeout(void)
{
	if(!fir_study)
		return;

	ir_learn_stop_capture();
	if(ir_learn_idle_timer)
		sys_timeout_del(ir_learn_idle_timer);
	if(ir_study_timer)
		sys_timeout_del(ir_study_timer);
	ir_learn_led_edges = 0;
	if(ir_learn_feedback_timer)
		sys_timeout_del(ir_learn_feedback_timer);
	ir_learn_idle_timer = 0;
	ir_study_timer = 0;
	ir_learn_feedback_timer = 0;
	ir_learn_key = 0;
	fir_learn_finish = 0;
	fir_led_ir = 0;
	unrequest_irq(IRQ_TIME1_IDX);
	gpio_disable_function(IO_PORT_SPILT(IR_RX_PIN), PORT_FUNC_TIMER1_CAPTURE);
	JL_TIMER1->CON = 0;
	fir_study = 0;
}

static void app_irflt_config(void)
{
	void timer_ir_rx_isr(void);

	fir_learn_parity = 0;
	high_num = 0;
	low_num = 0;
	timer_isr_str = 0;

	gpio_set_mode(IO_PORT_SPILT(IR_RX_PIN), PORT_INPUT_PULLUP_10K);
	gpio_set_function(IO_PORT_SPILT(IR_RX_PIN), PORT_FUNC_TIMER1_CAPTURE);
	JL_TIMER1->CON = TIMER_PCLR;
	JL_TIMER1->CNT = 0;
	JL_TIMER1->PRD = 0;
	request_irq(IRQ_TIME1_IDX, 7, timer_ir_rx_isr, 0);
	JL_TIMER1->CON = TIMER_CESL_IO | TIMER_SSEL_STD_24M | TIMER_PSET_64 | TIMER_MODE_FALL;
}


//-------------------------------------------------------
//Func  : ѹ���յ����δ洢��EEPROM
//Desc	:
//Input	:
//Output:
//Return:
//Author: Owen
//Date	: 2018/06/30
//-------------------------------------------------------
static void system_ir_rece(void *priv)
{
	ir_study_timer = 0;
	if(!fir_study || !ir_learn_key)
		return;
	if(fir_star)
	{
		ir_study_timer = sys_timeout_add(NULL, system_ir_rece, 20);
		return;
	}

	max_time = 0;
	study_buf_ct = 0;
	con_num = 0;
	IR_STUDY_CON[0] = 0;
	for(u8 i = 0; i < HL_LONG; i++)
	{
		IR_STUDY_H[i] = 0;
		IR_STUDY_L[i] = 0;
	}
	fir_study_star = 1;
	app_irflt_config();
}

u8 ir_set_continue(void)
{
	comp_buf[0] =IR_STUDY_CON[1]-IR_STUDY_CON[0];
	comp_buf[1] =IR_STUDY_CON[2]-IR_STUDY_CON[1];
	if(comp_buf[0]>1)
	 {
		if(ir_compare_data(2,0))
		   {
			if(ir_compare_data(1,0))
			   {
				con_num =IR_STUDY_CON[0]+1;
				study_buf_ct = IR_STUDY_CON[1];
				//comp_buf[0]	=comp_buf[0]	;
				con_num =con_num -comp_buf[0];
				study_buf_ct -=comp_buf[0];
			   }
			else {
				con_num =IR_STUDY_CON[1]+1;
				study_buf_ct = IR_STUDY_CON[2];
				comp_buf[0] +=comp_buf[1];
				con_num =con_num - comp_buf[0];
				study_buf_ct -=comp_buf[1];
			   }
		   }
		else {
			 comp_buf[0] =0;
			 con_num =0;
			}
		return 1;
	 }
	return 0;
}

//-------------------------------------------------------
//Func  : �Ƚ����������м�������Ϣ�����Ƿ�һ������Ϣ������һ�·���1������Ϊ0
//Desc	:
//Input	:
//Output:
//Return:
//Author: Owen
//Date	: 2018/06/30
//-------------------------------------------------------
bool ir_compare_data(u8 a,u8 b)
{
   u8 i;
   bool flag;

     flag =1;
	 for(i=0;i<comp_buf[b];i++)
		{
		 if(IR_STUDY_BUF[IR_STUDY_CON[a]-i]^IR_STUDY_BUF[IR_STUDY_CON[b]-i])
			{
			 flag =0;
			 break;
			}
		}
	 return flag;
}

void system_ir_tran(void)
{
  static u32  time;
  static u8  buf,dat;

  if((buf==0xf))
	 {
		ir_tran_PWM_OFF();
		return;
	 }
  if(!fir_send && !IR_STEP)
	 {
		ir_tran_PWM_OFF();
		return;
	 }
  if(fir_retran)
	{
	 buf =con_num+IR_STEP/2;
	 dat =IR_STEP;
	}
  else {
	 buf =IR_STEP/2;
	 dat =IR_STEP;
	}
  if(dat&0x1)
	{
	 buf=IR_STUDY_BUF[buf]&0x0f;
	 if(IR_STUDY_L[buf] <= 20000)//20ms
	 	fpwr_debug = 1;
	 else fpwr_debug = 0;

	 gptimer_set_resolution(timer_tid,IR_STUDY_L[buf] - 40);
	 gptimer_set_resolution(timer_tid,IR_STUDY_L[buf] - 40);
	 if(gptimer_get_resolution(timer_tid)^(IR_STUDY_L[buf] - 40))
	 {
		gptimer_set_resolution(timer_tid,IR_STUDY_L[buf] - 40);
	 }
	}
  else {
  	 buf=IR_STUDY_BUF[buf]>>4;
	 gptimer_set_resolution(timer_tid,IR_STUDY_H[buf] - 40);
	 gptimer_set_resolution(timer_tid,IR_STUDY_H[buf] - 40);
	 if(gptimer_get_resolution(timer_tid)^(IR_STUDY_H[buf] - 40))
	 {
		gptimer_set_resolution(timer_tid,IR_STUDY_H[buf] - 40);
	 }
  	}
  gptimer_set_count(timer_tid,0);
  if(!fir_retran)
	 {
	  if(IR_STEP&0x1)
		  gptimer_pwm_disable(pwm_tid);
	  else gptimer_pwm_enable(pwm_tid);
	  
	  IR_STEP++;
	  
	  if(con_num)
		time =(u16)(con_num*2);
	  else time =(u16)(IR_con_long*2);

	  if(IR_STEP>=time)
		 {
		  fir_retran =1;
		  IR_STEP =0;
		 }
	 }
  else {
	 if(IR_con_long)
		{
		 if(IR_STEP&0x1)
			gptimer_pwm_disable(pwm_tid);
		 else gptimer_pwm_enable(pwm_tid);
		 IR_STEP++;
		 if(IR_STEP>=(u16)(IR_con_long*2))
			IR_STEP =0;
		}
	}

}

___interrupt
void timer_ir_rx_isr(void)
{
 u8 i;
 u32 time,t2_ct;

 JL_TIMER1->CON |= TIMER_PCLR;   //清PND
 if(!fir_study_star)
	return;
 
 if(!fir_learn_parity)
 	{
 	 time =(JL_TIMER1->PRD )/3 *8;//低电平持续时间
 	 JL_TIMER1->CNT =0;
 	 JL_TIMER1->CON &= ~0b01;	//上升沿
 	 fir_learn_parity =1;
	 ir_timer2_inti();
 	}
 else {
 	 t2_ct =(JL_TIMER2->CNT)/3 *8;//载波持续时间 
	 time =JL_TIMER1->PRD /3 *8;

	 JL_TIMER1->CNT =0;
	 while(1)
	 	{
	 	 if(JL_TIMER1->CNT>(26*2*prd_1us))
		 	break;
		 if(!gpio_read(IR_RX_PIN))
			return;	
	 	}

 	 JL_TIMER1->CON |= TIMER_MODE_FALL;	//下降沿
	 fir_learn_parity =0;
 	}

 
 if(!fir_learn_parity)
 	{
	 for(i=0;i<HL_LONG;i++)
		{
		 if(abs(IR_STUDY_H[i]-t2_ct)<t2_ct/6)
			{
			 IR_STUDY_H[i] =(IR_STUDY_H[i]+t2_ct)/2;
			 break;
			}
		}
	 if(i==HL_LONG)
	   {
	    if(high_num>=HL_LONG)
			{
			 log_info("high_num>=HL_LONG\n");
			 fir_learn_fail =1;
			 user_ir_learn_out();
			 return;
			}
		IR_STUDY_H[high_num] =t2_ct;

		i  =high_num;
		high_num++;
		
	   }
	 IR_STUDY_BUF[study_buf_ct] =i<<4;

 	}
 else {
 	 if(!timer_isr_str)
	 	{
	 	 timer_isr_str =1;
		 return;
	 	}
	 fir_max_valid =0;

	 for(i=0;i<HL_LONG;i++)
		{
		 if(abs(IR_STUDY_L[i]-time)<time/6)
			{
			 IR_STUDY_L[i] =(IR_STUDY_L[i]+time)/2;
			 break;
			}
		}
	 if(i==HL_LONG)
	   {
		if(low_num>=HL_LONG)
		 {
		  log_info("low_num>=HL_LONG\n");
		  fir_learn_fail =1;
		  user_ir_learn_out();
		  return;
		 }

		IR_STUDY_L[low_num] =time;

		if(max_time<time)
		   {
			fir_max_valid =1;
			max_time=time;
		   }
		i =low_num;
		low_num++;
	   }
	 IR_STUDY_BUF[study_buf_ct] |=i&0xf;
	 if(fir_max_valid)
	  {
	   max_num =study_buf_ct;
	   max_l_ct=i;
	   con_num=0;
	  }
	 if(max_time>6000*prd_1us)//6ms
	  {
	   for(i=max_num;i<study_buf_ct && con_num<3;i++)
		  {
		   if((IR_STUDY_BUF[i]&0x0f)==max_l_ct)
			  {
			   IR_STUDY_CON[con_num] =i;
			   max_num =i+1;
			   con_num++;
			  }
		  }
	  }
	 if(con_num>=3)
	  {
	   if(ir_set_continue())
	   	{
		   if(!comp_buf[0])
			  {
			   fir_learn_fail =1;
			   user_ir_learn_out();
			   return;
			  }
		   else {
				fir_learn_fail =0;//succ
			    user_ir_learn_out();
				return;
			   }
	   	}
	   else {
		   log_info("learn fail con err\n");
		   fir_learn_fail =1;
		   user_ir_learn_out();
		   return;

	   	}
	  }

	 if(!ir_learn_frame_timer)
	   ir_learn_frame_timer = sys_timeout_add(NULL, user_ir_learn_finish, 200);
	 else sys_timer_modify(ir_learn_frame_timer, 200);

	 if(++study_buf_ct >= sizeof(IR_STUDY_BUF))
	 {
		 fir_learn_fail = 1;
		 user_ir_learn_out();
	 }
 	}
}

u8 ir_idle_query(void)
{
    return !(is_ir_active || fir_study);
}
REGISTER_LP_TARGET(ir_lp_target) = {
    .name = "rcir",
    .is_idle = ir_idle_query,
};

#endif

