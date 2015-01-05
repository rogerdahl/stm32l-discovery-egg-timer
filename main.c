// SIMPLE COUNT DOWN THEN COUNT UP TIMER.

/* base headers */
#include "stdint.h"

#include "stm32l1xx_gpio.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_lcd.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_dbgmcu.h"

#include "misc.h"


// Touch
// #include "tsl.h"


/* lcd wrapper routines header */
#include "stm32l_discovery_lcd.h"

/* boot mode */

#define CONFIG_BOOT_SRAM 0
#define CONFIG_BOOT_FLASH 1


/* gpios
   refer to CD00277537.pdf, APB memory space.
   refer to CD00240193.pdf, GPIO.
*/

#define GPIOA_MODER (GPIOA_BASE + 0x00)
#define GPIOA_ODR (GPIOA_BASE + 0x14)

#define GPIOB_MODER (GPIOB_BASE + 0x00)
#define GPIOB_ODR (GPIOB_BASE + 0x14)

#define GPIOC_MODER (GPIOC_BASE + 0x00)
#define GPIOC_ODR (GPIOC_BASE + 0x14)


/* leds */

#define LED_BLUE (1 << 6) /* port B, pin 6 */
#define LED_GREEN (1 << 7) /* port B, pin 7 */

static inline void setup_leds(void)
{
    /* configure port 6 and 7 as output */
    *(volatile uint32_t*)GPIOB_MODER |= (1 << (7 * 2)) | (1 << (6 * 2));
}

void green_off(void) {
    GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
}

void green_on(void) {
    GPIO_HIGH(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
}

void blue_on(void) {
    GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
}

void blue_off(void) {
    GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
}

//static inline void switch_leds_on(void)
//{
//    *(volatile uint32_t*)GPIOB_ODR = LED_BLUE | LED_GREEN;
//}
//
//static inline void switch_leds_off(void)
//{
//    *(volatile uint32_t*)GPIOB_ODR = 0;
//}

void display_time(void);

#define delay()						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < 300000; ++i)				\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)


#if CONFIG_BOOT_SRAM

extern uint32_t _fstack;

static inline void setup_stack(void)
{
    /* setup the stack to point to _fstack (refer to ld script) */

    static const uint32_t fstack = (uint32_t)&_fstack;

    __asm__ __volatile__
    (
        "ldr sp, %0\n\t"
        :
        : "m"(fstack)
        : "sp"
    );
}

#endif /* CONFIG_BOOT_SRAM */



/**
 * @brief  NVIC definitions
 */
#define TIMER_DELAY_PREEMPTION_PRIORITY						2
#define TIMER_DELAY_SUB_PRIORITY									2
#define TIMER_DELAY_IRQ_CHANNEL										TIM2_IRQn


//void RTC_Configuration(void);
void NVIC_Configuration(void);
//void TAMPER_STAMP_IRQHandler(void);
void RTC_Timer(void);


void RTC_TamperConfiguration(void);


/* application related setup */

static void RCC_Configuration(void)
{
    /* Enable HSI Clock */
    RCC_HSICmd(ENABLE);

    /*!< Wait till HSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

    /* Set HSI as sys clock*/
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    /* Set MSI clock range to ~4.194MHz*/
    RCC_MSIRangeConfig(RCC_MSIRange_6);

    /* Enable the GPIOs clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);

    /* Enable comparator, LCD and PWR mngt clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR,ENABLE);

    /* Enable ADC & SYSCFG clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG , ENABLE);

    /* Allow access to the RTC */
    PWR_RTCAccessCmd(ENABLE);

    /* Reset RTC Backup Domain */
    RCC_RTCResetCmd(ENABLE);
    RCC_RTCResetCmd(DISABLE);

    /* LSE Enable */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait until LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

    /* RTC Clock Source Selection */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC */
    RCC_RTCCLKCmd(ENABLE);

    /*Disable HSE*/
    RCC_HSEConfig(RCC_HSE_OFF);
    if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
    {
        /* Stay in infinite loop if HSE is not disabled*/
        while(1);
    }
}

static void Init_GPIOs(void)
{
////#if 0 /* fixme: GPIO_Init raises a bug in some gcc toolchains */

    /* GPIO, EXTI and NVIC Init structure declaration */
    GPIO_InitTypeDef GPIO_InitStructure;

////#if 0
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
////#endif

////#if 0
    /* Configure User Button pin as input */
    GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);
////#endif

////#if 0
    /* Select User Button pin as input source for EXTI Line */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

    /* Configure EXT1 Line 0 in interrupt mode trigged on Rising edge */
    // The EXT1 Line 0 interrupt is handled by void EXTI0_IRQHandler(void).
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
////#endif

#if 0
    /* Configure the LED_pin as output push-pull for LD3 & LD4 usage*/
    GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);

    /* Force a low level on LEDs*/
    GPIO_LOW(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
    GPIO_LOW(LD_GPIO_PORT,LD_BLUE_GPIO_PIN);

    /* Counter enable: GPIO set in output for enable the counter */
    GPIO_InitStructure.GPIO_Pin = CTN_CNTEN_GPIO_PIN;
    GPIO_Init( CTN_GPIO_PORT, &GPIO_InitStructure);

    /* To prepare to start counter */
    GPIO_HIGH(CTN_GPIO_PORT,CTN_CNTEN_GPIO_PIN);

    /* Configure Port A LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    /* Select LCD alternate function for Port A LCD Output pins */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_LCD) ;

    /* Configure Port B LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 \
                                  | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_LCD) ;
#endif

#if 0
    /* Configure Port C LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
                                  | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init( GPIOC, &GPIO_InitStructure);

    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource0,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource2,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_LCD) ;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_LCD) ;
#endif

#if 0
    /* Configure ADC (IDD_MEASURE) pin as Analogue */
    GPIO_InitStructure.GPIO_Pin = IDD_MEASURE  ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_Init( IDD_MEASURE_PORT, &GPIO_InitStructure);
#endif

//#else /* fixme */

    ///* set every port in digital output mode  */
    //
    ///* PA[1:3,8:10,15] */
    //*(volatile uint32_t*)GPIOA_MODER |=
    //  (GPIO_Mode_AF << (1 * 2)) |
    //  (GPIO_Mode_AF << (2 * 2)) |
    //  (GPIO_Mode_AF << (3 * 2)) |
    //  (GPIO_Mode_AF << (8 * 2)) |
    //  (GPIO_Mode_AF << (9 * 2)) |
    //  (GPIO_Mode_AF << (10 * 2)) |
    //  (GPIO_Mode_AF << (15 * 2));
    //
    ///* PB[3:5,8:15] */
    //*(volatile uint32_t*)GPIOB_MODER |=
    //  (GPIO_Mode_AF << (3 * 2)) |
    //  (GPIO_Mode_AF << (4 * 2)) |
    //  (GPIO_Mode_AF << (5 * 2)) |
    //  (GPIO_Mode_AF << (8 * 2)) |
    //  (GPIO_Mode_AF << (9 * 2)) |
    //  (GPIO_Mode_AF << (10 * 2)) |
    //  (GPIO_Mode_AF << (11 * 2)) |
    //  (GPIO_Mode_AF << (12 * 2)) |
    //  (GPIO_Mode_AF << (13 * 2)) |
    //  (GPIO_Mode_AF << (14 * 2)) |
    //  (GPIO_Mode_AF << (15 * 2));
    //
    ///* PC[0:3,6:11] */
    //*(volatile uint32_t*)GPIOC_MODER |=
    //  (GPIO_Mode_AF << (0 * 2)) |
    //  (GPIO_Mode_AF << (1 * 2)) |
    //  (GPIO_Mode_AF << (2 * 2)) |
    //  (GPIO_Mode_AF << (3 * 2)) |
    //  (GPIO_Mode_AF << (6 * 2)) |
    //  (GPIO_Mode_AF << (7 * 2)) |
    //  (GPIO_Mode_AF << (8 * 2)) |
    //  (GPIO_Mode_AF << (9 * 2)) |
    //  (GPIO_Mode_AF << (10 * 2)) |
    //  (GPIO_Mode_AF << (11 * 2));

////#endif /* fixme */
}

/**
* @brief  RTC Tamper Configuration..
* @param  None
* @retval None
*/
void RTC_TamperConfiguration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // EXTI configuration
    EXTI_ClearITPendingBit(EXTI_Line19);
    EXTI_InitStructure.EXTI_Line = EXTI_Line19;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable RTC_TAMP_STAMP_IRQn
    NVIC_InitStructure.NVIC_IRQChannel = TAMPER_STAMP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // determines the number of active pulse for the specific level
    RTC_TamperFilterConfig(RTC_TamperFilter_2Sample);

    // Determines the frequency at which each of the tamper inputs are sampled
    RTC_TamperSamplingFreqConfig(RTC_TamperSamplingFreq_RTCCLK_Div32768);

    RTC_TamperPullUpCmd(DISABLE);

    // Select the tamper 1 with High level
    RTC_TamperTriggerConfig(RTC_Tamper_2, RTC_TamperTrigger_HighLevel );

    // Clear tamper 1 flag
    RTC_ClearFlag(RTC_FLAG_TAMP2F);
}


/**
  * @brief  The Low Speed External (LSE) clock is used as RTC clock source.
  * The RTC clock can be output on the Tamper pin (PC.13). To enable this functionality,
  * uncomment the corresponding line: #define RTCClockOutput_Enable.
  *
  * The RTC is in the backup (BKP) domain, still powered by VBAT when VDD is switched off,
  * so the RTC configuration is not lost if a battery is connected to the VBAT pin.
  * A key value is written in backup data register1 (BKP_DR1) to indicate if the RTC
  * is already configured.
  *
  * @param  None
  * @retval None
  */

void RTC_Timer(void)
{
    /* Initialize LED which connected on PB8, Enable the Clock*/
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_LED pin */
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    //USART_InitStructure.USART_BaudRate = 115200;
    //USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    //USART_InitStructure.USART_StopBits = USART_StopBits_1;
    //USART_InitStructure.USART_Parity = USART_Parity_No;
    //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //STM_COMInit(COM1, &USART_InitStructure);


    /* NVIC configuration */
    NVIC_Configuration();
    //if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    //{
    //    /* Backup data register value is not correct or not yet programmed (when
    //       the first time the program is executed) */
    //
    //    // printf("\r\n\n RTC not yet configured....");
    //
    //    /* RTC Configuration */
    //    RTC_Configuration();
    //
    //    // printf("\r\n RTC configured....");
    //
    //    /* Adjust time by values entred by the user on the hyperterminal */
    //    //Time_Adjust();
    //
    //    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    //}
    //else
    //{
    //    /* Check if the Power On Reset flag is set */
    //    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    //    {
    //        // printf("\r\n\n Power On Reset occurred....");
    //    }
    //    /* Check if the Pin Reset flag is set */
    //    else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    //    {
    //        // printf("\r\n\n External Reset occurred....");
    //    }
    //
    //RTC_Configuration();
    // printf("\r\n No need to configure RTC....");
    /* Wait for RTC registers synchronization */
    //    RTC_WaitForSynchro();
    /* Enable the RTC Second */
    ///////////RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_ITConfig(RTC_IT_TS, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    ///RTC_WaitForLastTask(); ////////////////////////////////////////////////////////////////////////////
////}


////////////////////////////////////////////////////////////////////////////////////////
//#ifdef RTCClockOutput_Enable
//    /* Enable PWR and BKP clocks */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//
//    /* Allow access to BKP Domain */
//    PWR_BackupAccessCmd(ENABLE);
//
//    /* Disable the Tamper Pin */
//    BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
//                                 functionality must be disabled */
//
//    /* Enable RTC Clock Output on Tamper Pin */
//    BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
//#endif

    /* Clear reset flags */
    RCC_ClearFlag();

    /* Display time in infinite loop */
    //Time_Show();
}


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    ////////////////////////////////////////////////////////////////////////////
    //NVIC_InitTypeDef NVIC_InitStructure;
    //
    ///* Configure one bit for preemption priority */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //
    ///* Enable the RTC Interrupt */
    //NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
}

// Number of seconds until LEDs start flashing.
volatile int sec = 1 * 60 * 60;
// Number of seconds to add to timeout when USER button is pressed.
const int bump_sec = 15 * 60;


// User button
void EXTI0_IRQHandler(void)
{
  green_off();
  blue_off();
  sec += bump_sec;
  display_time();
  EXTI_ClearITPendingBit(EXTI_Line0);
}

// TAMPER_STAMP_IRQHandler (void)
// This function handles Tamper&Stamp pin interrupt request.
//void TAMPER_STAMP_IRQHandler(void)
//{
//    ++c;
//
//    if (RTC_GetITStatus(RTC_IT_TS) != RESET) ///////////////////////////////////////
//    {
//        RTC_ClearITPendingBit(RTC_IT_TS);
//
//        ++c;
//
//        /* Wait until last write operation on RTC registers has finished */
//        /////RTC_WaitForLastTask(); ///////////////////////////////////////////////////////////////////////////
//    }
//}


/**
  * @brief  Configures the RTC peripheral and select the clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
    RTC_InitTypeDef RTC_InitStructure;
    RTC_TimeTypeDef  RTC_TimeStruct;

    /* Enable the PWR clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    /* Allow access to RTC */
    PWR_RTCAccessCmd(ENABLE);

    /* Reset RTC Domain */
    RCC_RTCResetCmd(ENABLE);
    RCC_RTCResetCmd(DISABLE);

    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    /* Set the time to 00h 00mn 00s AM */
    RTC_TimeStruct.RTC_H12     = RTC_H12_AM;
    RTC_TimeStruct.RTC_Hours   = 0x00;
    RTC_TimeStruct.RTC_Minutes = 0x00;
    RTC_TimeStruct.RTC_Seconds = 0x00;
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
}

/**
  * @brief  Configures the RTC Alarm.
  * @param  None
  * @retval None
  */
void RTC_AlarmConfig(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* EXTI configuration */
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    RTC_AlarmStructInit(&RTC_AlarmStructure);

    /* Set the alarmA Masks */
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All;
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    /* Set AlarmA subseconds and enable SubSec Alarm : generate 8 interripts per Second */
    //RTC_AlarmSubSecondConfig(RTC_Alarm_A, 0xFF, RTC_AlarmSubSecondMask_SS14_5);

    /* Enable AlarmA interrupt */
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    /* Enable the alarmA */
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE); //////////////// WAS SET TO DISABLE???
}

void itoa(char* a, int i, int n) {
    for (int j = n - 1; j >= 0; --j) {
        if (i) {
            a[j] = (i % 10) + '0';
            i /= 10;
        }
        else {
            a[j] = '0';
        }
    }
}

void display_time(void) {
    int s = sec;
    if (s < 0) {
        s = -s;
    }

    int h = s / (60 * 60);
    int m = s / 60 % 60;
    s = s % 60;
    
    char v[7];
    itoa(v, h, 2);
    itoa(v + 2, m, 2);
    itoa(v + 4, s, 2);
    v[6] = '\0';

    LCD_GLASS_DisplayString((uint8_t*)v);
}

void RTC_Alarm_IRQHandler(void)
{
    --sec;

    display_time();

    /* Clear RTC AlarmA Flags */
    // both must be cleared. why?
    RTC_ClearITPendingBit(RTC_IT_ALRA);

    /* Clear the EXTIL line 17 */
    // If this is not cleared, things freeze.
    EXTI_ClearITPendingBit(EXTI_Line17);
}


///**
//  * @brief  This function handles External lines 15 to 10 interrupt request.
//  * @param  None
//  * @retval None
//  */
//void EXTI15_10_IRQHandler(void)
//{
//    ++c;
//    /* Clear the EXTI Line 11 */
//    EXTI_ClearITPendingBit(EXTI_Line11);
//}




//static void __attribute__((naked)) __attribute__((used)) main(void)
int main(void)
{
#if CONFIG_BOOT_SRAM
    /* do not use previsouly setup stack, if any */
    setup_stack();
#endif /* CONFIG_BOOT_SRAM */

    RCC_Configuration();
    Init_GPIOs();

    LCD_GLASS_Configure_GPIO();
    LCD_GLASS_Init();
    LCD_GLASS_Clear();

    // Have to do RCC_Configuration both before and after setting up the LCD...???
    RCC_Configuration();

    //RTC_TamperConfiguration();
    //RTC_Config();

    setup_leds();

    /* Configure the RTC peripheral by selecting the clock source.*/
    RTC_Config();

    /* Configure RTC AlarmA register to generate one interupt per second */
    RTC_AlarmConfig();

    int toggle = 0;

    green_off();
    blue_off();

    while (1)
    {
        if (sec > 0) {
            continue;
        }

        if (toggle) {
            toggle = 0;
            green_on();
            blue_off();
        }
        else {
            toggle = 1;
            green_off();
            blue_on();
        }

        delay();
    }

    // Will never get here.
    return 0;
}
