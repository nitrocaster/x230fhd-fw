#include "stm8s_it.h"
#include "common.h"
#include <iostm8s003f3.h>
#include "stm8s_tim1.h"

#ifdef _COSMIC_
// Dummy Interrupt routine
INTERRUPT_HANDLER(NonHandledInterrupt, 25)
{}
#endif

// TRAP Interrupt routine
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{}

// Top Level Interrupt routine
INTERRUPT_HANDLER(TLI_IRQHandler, 0)
{}

// @brief Auto Wake Up Interrupt routine
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{}

// Clock Controller Interrupt routine
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{}

// External Interrupt PORTA Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{}

// External Interrupt PORTB Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{}

// External Interrupt PORTC Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{}

// External Interrupt PORTD Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{}

// External Interrupt PORTE Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{}

#if defined (STM8S903) || defined (STM8AF622x) 
// External Interrupt PORTF Interrupt routine
INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{}
#endif

#if defined (STM8S208) || defined (STM8AF52Ax)
// CAN RX Interrupt routine
INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{}

// CAN TX Interrupt routine
INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{}
#endif

// SPI Interrupt routine
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{}

// Timer1 Update/Overflow/Trigger/Break Interrupt routine
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{}

static volatile u16 pwmcap_ic1 = 0;
static volatile u16 pwmcap_ic2 = 0;
volatile u16 pwmcap_dc_num = 0;
volatile u16 pwmcap_dc_denom = 1;
volatile u16 pwmcap_alive = 0;
volatile u16 tim2_dc = 0x2000;
volatile u8 pwmcap_st = 0;
#define PWMCAP_PERIOD_THRESHOLD 1000
#define PWMCAP_ON_THRESHOLD 1000
#define PWMCAP_STATUS_PERIOD 1
#define PWMCAP_STATUS_ONTIME 2
#define PWMCAP_STATUS_OVERFLOW 4

// Timer1 Capture/Compare Interrupt routine
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
    if (TIM1_GetITStatus(TIM1_IT_CC1))
    {
        const u16 cc1 = TIM1_GetCapture1();
        const u16 diff = cc1>pwmcap_ic1 ? cc1-pwmcap_ic1 : pwmcap_ic1-cc1;
        if (diff>PWMCAP_PERIOD_THRESHOLD)
            pwmcap_st |= PWMCAP_STATUS_OVERFLOW;
        pwmcap_ic1 = cc1;
        pwmcap_st |= PWMCAP_STATUS_PERIOD;
    }
    if (TIM1_GetITStatus(TIM1_IT_CC2))
    {
        const u16 cc2 = TIM1_GetCapture2();
        const u16 diff = cc2>pwmcap_ic2 ? cc2-pwmcap_ic2 : pwmcap_ic2-cc2;
        if (diff>PWMCAP_ON_THRESHOLD)
            pwmcap_st |= PWMCAP_STATUS_OVERFLOW;
        pwmcap_ic2 = cc2;
        pwmcap_st |= PWMCAP_STATUS_ONTIME;
    }
    TIM1_ClearFlag(TIM1_FLAG_TRIGGER);
    if (pwmcap_st & (PWMCAP_STATUS_PERIOD|PWMCAP_STATUS_ONTIME))
    {
        if (!(pwmcap_st & PWMCAP_STATUS_OVERFLOW))
        {
            pwmcap_dc_num = pwmcap_ic2;
            pwmcap_dc_denom = pwmcap_ic1;
        }
        pwmcap_st = 0;
        pwmcap_alive = 0;
    }
}

#if defined (STM8S903) || defined (STM8AF622x)
// Timer5 Update/Overflow/Break/Trigger Interrupt routine
INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{}
 
// Timer5 Capture/Compare Interrupt routine
INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{}

#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */
// Timer2 Update/Overflow/Break Interrupt routine
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
   const u16 dc = tim2_dc;
   TIM2_CCR2H = dc>>8;
   TIM2_CCR2L = dc&0xFF;
   TIM2_SR1_bit.UIF = 0;
}

// Timer2 Capture/Compare Interrupt routine
INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{}
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8AF626x)
// Timer3 Update/Overflow/Break Interrupt routine
INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{}

// Timer3 Capture/Compare Interrupt routine
INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{}
#endif /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
// UART1 TX Interrupt routine
INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{}

// UART1 RX Interrupt routine
INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{}
#endif /* (STM8S208) || (STM8S207) || (STM8S103) || (STM8S903) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8AF622x)
// UART4 TX Interrupt routine
INTERRUPT_HANDLER(UART4_TX_IRQHandler, 17)
{}

// UART4 RX Interrupt routine
INTERRUPT_HANDLER(UART4_RX_IRQHandler, 18)
{}
#endif /* (STM8AF622x) */

// I2C Interrupt routine
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{}

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
// UART2 TX interrupt routine
INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{}

// UART2 RX interrupt routine
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{}
#endif /* (STM8S105) || (STM8AF626x) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
// UART3 TX interrupt routine
INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{}

// UART3 RX interrupt routine
INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{}
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
// ADC2 interrupt routine
INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{}
#else /* STM8S105 or STM8S103 or STM8S903 or STM8AF626x or STM8AF622x */
// ADC1 interrupt routine
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{}
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8S903) || defined (STM8AF622x)
// Timer6 Update/Overflow/Trigger Interrupt routine
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{}
#else /* STM8S208 or STM8S207 or STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */

volatile u16 tim4_tout = 0;
// Timer4 Update/Overflow Interrupt routine
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
    TIM4->SR1 = 0;
    if (tim4_tout)
       --tim4_tout;
}
#endif /* (STM8S903) || (STM8AF622x)*/

// Eeprom EEC Interrupt routine
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{}
