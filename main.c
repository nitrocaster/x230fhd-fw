#include "common.h"
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_adc1.h"
#include <iostm8s003f3.h>
#include <math.h>

#define DP_ACTIVITY_THRESHOLD 81
#define PWM_FRQ_DIVIDER 0x2000
#define PWM_DUTY 0x1000
#define BL_TIMEOUT 200
#define BL_OVERRIDE 0
#define BL_DIM_TEST 0

#define set_tout(ms) do { tim4_tout = ms; } while (0)
#define is_tout (!tim4_tout)
#define tim1_reset TIM1_DeInit
#define tim2_reset TIM2_DeInit
#define PWMCAP_CURRENT_STATE (PC_IDR_bit.IDR6 ? 1 : 0)
#define led_enable(on) (PB_ODR_ODR5 = !(on))

static u8 dp_active = 0;
static u16 dp_dtct_lag = 0;
static u16 current_uduty = 0xffff;
static u8 sys_bl_on = 0;
static u8 ext_bl_on = 0;
static float fduty = 0.0f;
static u8 prev_en = 0;
static u8 bl_defer = 0;
static u16 bl_pwm_blink = 0;
static u8 compat_mode = 0;

static void spin_sleep(u16 q)
{
    while (--q)
    {
        for (u16 i = 0; i<29000; i++)
            __no_operation();
    }
}

static void spin_sleep_fast(u16 q)
{
    while (--q)
    {
        for (u16 i = 0; i<128; i++)
            __no_operation();
    }
}

// init sysclk to run at 16MHz using the internal oscillator
static void sysclk_init(void)
{
    CLK_ICKR = 0; // reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1; // Enable the HSI
    CLK_ECKR = 0; // disable the external clock
    while (!CLK_ICKR_HSIRDY); // wait for the HSI to be ready for use
    CLK_CKDIVR_bit.HSIDIV = 1; // F(hsi) = F(cpu)/2
    CLK_CKDIVR_bit.CPUDIV = 0; // ensure the clocks are running at full speed
    CLK_PCKENR1 = 0xFF; // enable all peripheral clocks
    CLK_PCKENR2 = 0xFF; // ditto
    CLK_CCOR = 0; // turn off CCO
    CLK_HSITRIMR = 0; // turn off any HSIU trimming
    CLK_SWIMCCR = 0; // set SWIM to run at clock/2
    CLK_SWR = 0xE1; // use HSI as the clock source
    CLK_SWCR = 0; // reset the clock switch control register
    CLK_SWCR_SWEN = 1; // enable switching
    while (CLK_SWCR_SWBSY); // pause while the clock switch is busy
}

// TIM1_CH1 pwm output
static void tim1_setup_pwm_gen(u16 prescaler, u16 frq_div, u16 duty)
{
    TIM1_PSCRH = prescaler >> 8;
    TIM1_PSCRL = prescaler & 0xFF;
    TIM1_ARRH = frq_div >> 8; // High byte of freq_div
    TIM1_ARRL = frq_div & 0xFF; // Low byte of freq_div
#if 1 // channel 1
    TIM1_CCR1H = duty >> 8; // High byte of duty
    TIM1_CCR1L = duty & 0xFF; // Low byte of duty
    TIM1_CCER1_CC1P = 0; // Active high
    TIM1_CCER1_CC1E = 1; // Enable compare mode for channel 1
    // PWM Mode 1 - active if counter < CCR1, inactive otherwise
    TIM1_CCMR1_OC1M = 6;
#else // channel 2
    TIM1_CCR2H = duty >> 8; // High byte of duty
    TIM1_CCR2L = duty & 0xFF; // Low byte of duty
    TIM1_CCER1_CC2P = 0; // Active high
    TIM1_CCER1_CC2E = 1; // Enable compare mode for channel 2
    // PWM Mode 2 - active if counter < CCR1, inactive otherwise
    TIM1_CCMR2_OC2M = 6;
#endif
    TIM1_Cmd(ENABLE); // finally enable the timer
}

// TIM2_CH2 pwm capture
void tim2_pwmgen_init(u8 prescaler, u16 frq_div, u16 duty)
{
    TIM2_PSCR = prescaler; // Prescaler = 2^0 = 1
    TIM2_ARRH = frq_div >> 8; // High byte of freq_div
    TIM2_ARRL = frq_div & 0xFF; // Low byte of freq_div
#if 0 // channel 1
    TIM2_CCR1H = duty >> 8; // High byte of duty
    TIM2_CCR1L = duty & 0xFF; // Low byte of duty
    TIM2_CCER1_CC1P = 0; // Active high
    TIM2_CCER1_CC1E = 1; // Enable compare mode for channel 1
    // PWM Mode 1 - active if counter < CCR1, inactive otherwise
    TIM2_CCMR1_OC1M = 6;
#else // channel 2
    TIM2_CCR2H = duty >> 8; // High byte of duty
    TIM2_CCR2L = duty & 0xFF; // Low byte of duty
    TIM2_CCER1_CC2P = 0; // Active high
    TIM2_CCER1_CC2E = 1; // Enable compare mode for channel 2
    // PWM Mode 2 - active if counter < CCR1, inactive otherwise
    TIM2_CCMR2_OC2M = 6;
#endif
    TIM2_CR1_bit.UDIS = 0;
    TIM2_IER_bit.UIE = 1; // enable interrupts
    TIM2_Cmd(ENABLE); // finally enable the timer
}

static void tim2_adjust_pwm_gen(u16 duty)
{
    tim2_dc = duty;
}

static void tim1_pwmcap_init(void)
{
    TIM1_CR1_bit.DIR = 0; // count up
    TIM1_CCMR1_bit.CC1S = 1; // [01] set TI2FP1 as an input for CCR1
    TIM1_CCER1_bit.CC1P = 0; // TI2FP1 is active on rising edge
    TIM1_CCMR2_bit.CC2S = 2; // [10] set TI1FP2 as an input for CCR2
    TIM1_CCER1_bit.CC2P = 1; // TI2FP2 is active on falling edge
    TIM1_SMCR_bit.TS = 5; // [101] select TI1FP1 as trigger input
    TIM1_SMCR_bit.SMS = 4; // [100]
    TIM1_CCER1_bit.CC1E = 1; // enable the captures
    TIM1_CCER1_bit.CC2E = 1;
    TIM1_IER_bit.CC1IE = 1; // allow CC1 and CC2 capture interrupt
    TIM1_IER_bit.CC2IE = 1;
    TIM1_CR1_bit.CEN = 1; // run TIM1
}

static void tim2_setup_pwm_cap(void)
{
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    TIM2_PWMIConfig(
        TIM2_CHANNEL_2,
        TIM2_ICPOLARITY_RISING,
        TIM2_ICSELECTION_DIRECTTI,
        TIM2_ICPSC_DIV8,
        0); // input capture filter (skip cycles)
    TIM2_ITConfig(TIM2_IT_CC2, ENABLE);
    TIM2_Cmd(ENABLE);
}

// init timer for 1ms interrupts
static void tim4_tout_init(void)
{
    TIM4->ARR = 130; // 255 - 125
    TIM4->PSCR = 6; // 8MHz / 64 = 125KHz => 125 updates per millisecond
    TIM4_IER_bit.UIE = 1;
    TIM4_CR1_bit.CEN = 1;
}

/*
    D3 : OUT  = BL_CTL          =>      TIM2_CH2
    A3 : OUT  = BL_ENABLE       
    C6 : IN   = BL_CTL_SYS      =>      TIM1_CH1
    D6 : IN   = BL_ENABLE_SYS   
    C4 : ADC  = DP_ACTIVITY     =>      Main loop
    .
    C7 = OPT1 (hook pullup)
    D4 = OPT2 (drive low and read C7 to detect jumper)
*/

static void compat_init(void)
{
    // OPT1
    PC_DDR_bit.DDR7 = 0; // input
    PC_CR1_bit.C17 = 1; // wpu
    // OPT2
    PD_DDR_bit.DDR4 = 1; // 1=output
    PD_CR1_bit.C14 = 1; // 1=push-pull
    // detect
    PD_ODR_bit.ODR4 = 0; // drive OPT2 low
    u8 opt1 = PC_IDR_bit.IDR7; // read OPT1
    // low level means OPT1 and OPT2 are bridged
    compat_mode = !opt1;
}

// [D6] BL_ENABLE_SYS
static u8 bl_enable_read(void)
{ return PD_IDR_bit.IDR6; }
// [A3] BL_ENABLE
static void bl_enable_write(u8 enable)
{ PA_ODR_bit.ODR3 = enable; }

static void dp_activity_init()
{
    ADC1_DeInit();
    ADC_CSR_bit.CH = 2; // AIN2
    ADC_CR3_bit.DBUF = 0;
    ADC_CR2_bit.ALIGN = 1; // right aligned data
    ADC_CSR_bit.EOCIE = 0; // no interrupt after conversion
    ADC_CR1_bit.ADON = 1; // enable ADC
}

static u16 dp_activity_read()
{
    ADC_CR1_bit.ADON = 1; // read C4 - AIN2
    while (!ADC_CSR_bit.EOC)
        __no_operation();
    u16 drl = ADC_DRL; // read low byte first (right alignment)
    u16 drh = ADC_DRH;
    ADC_CSR_bit.EOC = 0;
    return drl | (drh<<8);
}

static void bl_adjust(u8 en, u8 sys_bl, float fdc)
{
#if BL_OVERRIDE
    bl_enable_write(1);
    static u8 dc_set = 0;
    if (!dc_set)
    {
        tim2_adjust_pwm_gen(0xffff);
        dc_set = 1;
    }
#else // !BL_OVERRIDE
    // if there was no sys_bl_on, and ext_bl_on was set by dp_activity,
    // don't apply new params before BL_TIMEOUT is exceeded
    if (!compat_mode && prev_en && !en && !sys_bl)
    {
        // setup timeout if it wasn't set before
        if (!bl_defer)
        {
            set_tout(BL_TIMEOUT);
            bl_defer = 1;
        }
        if (!is_tout)
            return;
    }
    static u16 dc = 0;
    dc = (u16)ceil(fdc*PWM_FRQ_DIVIDER);
    bl_defer = 0;
    if (prev_en!=en)
    {
        prev_en = en;
        bl_enable_write(en);
    }
#if !BL_DIM_TEST
    if (!compat_mode && current_uduty)
    {
        if (bl_pwm_blink<128 && !dc)
        {
            bl_pwm_blink++;
            return;
        }
        bl_pwm_blink = 0;
    }
    current_uduty = dc;
    tim2_adjust_pwm_gen(dc);
#endif
#endif // BL_OVERRIDE
}

static void gpio_init(void)
{
    // setup all pins as inputs with wpu
    PA_DDR = 0x00; // input=0
    PA_CR1_bit.C10 = 1; // wpu
    PA_CR1_bit.C11 = 1; // wpu
    PA_CR1_bit.C12 = 1; // wpu
    PA_CR1_bit.C13 = 0; // A3: no wpu
    PA_CR1_bit.C14 = 1; // wpu
    PA_CR1_bit.C15 = 1; // wpu
    PA_CR1_bit.C16 = 1; // wpu
    PA_CR1_bit.C17 = 1; // wpu
    PB_DDR = 0x00; // input=0
    PB_CR1 = 0xff; // wpu
    PC_DDR = 0x00; // input=0
    PC_CR1_bit.C10 = 1; // wpu
    PC_CR1_bit.C11 = 1; // wpu
    PC_CR1_bit.C12 = 1; // wpu
    PC_CR1_bit.C13 = 1; // wpu
    PC_CR1_bit.C14 = 0; // C4:
    PC_CR1_bit.C15 = 1; // wpu
    PC_CR1_bit.C16 = 0; // C6:
    PC_CR1_bit.C17 = 1; // wpu
    PD_DDR = 0x00; // input=0
    PD_CR1_bit.C10 = 1; // wpu
    PD_CR1_bit.C11 = 1; // wpu
    PD_CR1_bit.C12 = 1; // wpu
    PD_CR1_bit.C13 = 0; // D3:
    PD_CR1_bit.C14 = 1; // wpu
    PD_CR1_bit.C15 = 1; // wpu
    PD_CR1_bit.C16 = 0; // D6:
    PD_CR1_bit.C17 = 1; // wpu
    // setup backlight pins
    // D3: backlight control (output)
    PD_DDR_bit.DDR3 = 1; // 1=output
    PD_CR1_bit.C13 = 1; // 1=push-pull
    PD_CR2_bit.C23 = 1; // high-speed output
    // A3: backlight enable (output)
    PA_DDR_bit.DDR3 = 1; // 1=output
    PA_CR1_bit.C13 = 1; // 1=push-pull 0=open-drain
    PA_ODR_bit.ODR3 = 0; // disable by default
    // C6: system backlight control (input) [TIM1_CH1]
    PC_DDR_bit.DDR6 = 0; // 0=input
    PC_CR1_bit.C16 = 0; // 0=no_wpu
    // D6: system backlight enable (input)
    PD_DDR_bit.DDR6 = 0; // 0=input
    PD_CR1_bit.C16 = 0; // 0=no_wpu
    // C4: DP activity (input)
    PC_DDR_bit.DDR4 = 0; // 0=input
    PC_CR1_bit.C14 = 0; // 0=no_wpu
}

void main(void)
{
    __disable_interrupt();
    compat_init();
    gpio_init();
    sysclk_init();
    tim1_reset();
    tim1_pwmcap_init();
    tim2_reset();
    tim2_pwmgen_init(0, PWM_FRQ_DIVIDER, 0); // init with zero duty cycle
    tim4_tout_init();
    dp_activity_init();
    __enable_interrupt();
    while (1)
    {
        spin_sleep_fast(1);
        // 1] Read BL_CTL_SYS
        if (pwmcap_alive>PWMCAP_TTL)
        {
#ifndef DISABLE_MAX_PWM
            pwmcap_st = 0;
            pwmcap_dc_num = PWMCAP_CURRENT_STATE;
            pwmcap_dc_denom = 1;
#endif
        }
        else
            ++pwmcap_alive;
        fduty = pwmcap_dc_num*1.0f / pwmcap_dc_denom;
        if (fduty>1.0f)
        {
            fduty = 1.0f;
            continue;
        }
        // 2] Read BL_ENABLE_SYS
        sys_bl_on = bl_enable_read();
        // 3] If BL_ENABLE_SYS is low, then check DP_ACTIVITY
        if (!compat_mode)
        {
            if (!sys_bl_on)
            {
                const u16 dp_lvl = dp_activity_read();
                dp_active = dp_lvl>DP_ACTIVITY_THRESHOLD;
                // 3.1] If DP_ACTIVITY is present, set PWM duty cycle to 100%
                //      and drive BL_ENABLE high
                if (dp_active)
                {
                    if (dp_dtct_lag<2000)
                    {
                        ++dp_dtct_lag;
                        dp_active = 0;
                    }
                    else
                        fduty = 1.0f;
                }
                else
                    dp_dtct_lag = 0;
                ext_bl_on = dp_active;
            }
            else
                ext_bl_on = 1;
        }
        else
            ext_bl_on = sys_bl_on;
        bl_adjust(ext_bl_on, sys_bl_on, fduty);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line)
{
    while (1);
}
#endif
