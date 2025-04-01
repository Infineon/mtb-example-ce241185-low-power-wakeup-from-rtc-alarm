/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the low power wakeup
*              from RTC alarm example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include <time.h>
#include "cy_sysint.h"
#include <stdio.h>
#include "cy_retarget_io.h"
#include "mtb_hal.h"

/*******************************************************************************
* Macros
********************************************************************************/

#define RTC_INTERRUPT_PRIORITY      3u

#define STRING_BUFFER_SIZE          80u

/* Constants to define LONG and SHORT presses on User Button (x10 = ms) */
#define SHORT_PRESS_COUNT           10u     /* 100 ms < press < 2 sec */
#define LONG_PRESS_COUNT            200u    /* press > 2 sec */

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS       10u     /* in ms */
#define LONG_GLITCH_DELAY_MS        100u    /* in ms */

typedef enum
{
    SWITCH_NO_EVENT     = 0u,
    SWITCH_SHORT_PRESS  = 1u,
    SWITCH_LONG_PRESS   = 2u,
} en_switch_event_t;

#define _RTC_TM_YEAR_BASE           1900

/*******************************************************************************
* Global Variables
*******************************************************************************/
const cy_stc_sysint_t intrCfg =
{
    .intrSrc = ((NvicMux3_IRQn << CY_SYSINT_INTRSRC_MUXIRQ_SHIFT) | srss_interrupt_backup_IRQn),
    .intrPriority = RTC_INTERRUPT_PRIORITY
};

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    UART_context;           /** UART context */
static mtb_hal_uart_t               UART_hal_obj;           /** Debug UART HAL object  */

/* Variable for storing character read from terminal */
uint8_t uart_read_value = 0UL;

/*****************************************************************************
* Function Prototypes
********************************************************************************/
void set_rtc_alarm_date_time(void);
en_switch_event_t get_switch_event(void);
void debug_printf(const char *str);
void handle_error(void);
static void RTC_Handler(void);
void rtc_from_pdl_time(cy_stc_rtc_config_t *pdlTime, const int year, struct tm *time);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the MCU. It does...
*    1. Initialize the UART and RTC blocks.
*    2. Check the reset reason, if it is wakeup from Hibernate power mode, then
*       set RTC initial time and date.
*    Do Forever loop:
*    3. Check if User button was pressed and for how long.
*    4. If short pressed, set the RTC alarm and then go to DeepSleep mode.
*    5. If long pressed, set the RTC alarm and then go to Hibernate mode.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    SCB_DisableICache();
    SCB_DisableDCache();

    /* Check the IO status. If current status is frozen, unfreeze the system. */
    if (Cy_SysPm_GetIoFreezeStatus())
    {
        /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
    }

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* set interrupt*/
    Cy_SysInt_Init(&intrCfg,RTC_Handler);
    NVIC_ClearPendingIRQ((IRQn_Type)intrCfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type) NvicMux3_IRQn);

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Enable(UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&UART_hal_obj, &UART_hal_config, &UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&UART_hal_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("PDL: Low power wakeup from RTC alarm example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press 'D' key to DeepSleep mode, Press 'H' key to Hibernate mode.\r\n\r\n");

    /* Check the reset reason */
    if(CY_SYSLIB_RESET_HIB_WAKEUP == (Cy_SysLib_GetResetReason() & CY_SYSLIB_RESET_HIB_WAKEUP))
    {
        /* The reset has occurred on a wakeup from Hibernate power mode */
        debug_printf("Wakeup from the Hibernate mode\r\n");
    }
    else
    {
        /* Initialize RTC */
        Cy_RTC_SelectClockSource((cy_en_rtc_clk_select_sources_t)CY_SYSCLK_BAK_IN_CLKLF);
        Cy_RTC_Init(&RTCALM_config);
    }
    
    /* Print the current date and time by UART */
    debug_printf("Current date and time.\r\n");

    /* Enable alarm interrupt */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_CENTURY);

    for (;;)
    {
        uart_read_value = Cy_SCB_UART_Get(UART_HW);

        /* Go to Deep sleep mode */
        if (uart_read_value == 'd')
        {
            debug_printf("Go to DeepSleep mode\r\n");

            /* Set the RTC generate alarm after 10 seconds */
            set_rtc_alarm_date_time();
            Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);

            /* Go to deep sleep */
            Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
            debug_printf("Wakeup from DeepSleep mode\r\n");
        }

        /* Go to Deep Hibenate mode */
        if (uart_read_value == 'h')
        {
            debug_printf("Go to Hibernate mode\r\n");

            /* Set the RTC generate alarm after 10 seconds */
            set_rtc_alarm_date_time();
            Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);

            /* Go to hibernate and configure the RTC alarm as wakeup source */
            Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_RTC_ALARM);
            Cy_SysPm_SystemEnterHibernate();
        }  
    }
}

/*******************************************************************************
* Function Name: set_rtc_alarm_date_time
********************************************************************************
* Summary:
*  This functions sets the RTC alarm date and time.
*
* Parameter:
*  void
*
*******************************************************************************/
void set_rtc_alarm_date_time(void)
{
    /* Print the RTC alarm time by UART */
    debug_printf("RTC alarm will be generated after 10 seconds\r\n");

    /* Set the RTC alarm for the specified number of seconds in the future */
    cy_stc_rtc_config_t currentTime;
    cy_stc_rtc_alarm_t alarmTime;

    uint32_t savedIntrStatus = Cy_SysLib_EnterCriticalSection();
    Cy_RTC_GetDateAndTime(&currentTime);
    Cy_SysLib_ExitCriticalSection(savedIntrStatus);

    int newSec = currentTime.sec + 10;
    int overflowMin = newSec / 60;
    newSec = newSec % 60;

    /* configure alarm*/
    alarmTime.sec = newSec;
    alarmTime.secEn = CY_RTC_ALARM_ENABLE;
    alarmTime.min = currentTime.min + overflowMin;
    alarmTime.minEn = CY_RTC_ALARM_DISABLE;
    alarmTime.hour = currentTime.hour;
    alarmTime.hourEn = CY_RTC_ALARM_DISABLE;
    alarmTime.dayOfWeek = currentTime.dayOfWeek;
    alarmTime.dayOfWeekEn = CY_RTC_ALARM_DISABLE;
    alarmTime.date = currentTime.date;
    alarmTime.dateEn = CY_RTC_ALARM_DISABLE;
    alarmTime.month = currentTime.month;
    alarmTime.monthEn = CY_RTC_ALARM_DISABLE;
    alarmTime.almEn = CY_RTC_ALARM_ENABLE;

    while(Cy_RTC_SetAlarmDateAndTime(&alarmTime,CY_RTC_ALARM_1) != CY_RET_SUCCESS);
}

/*******************************************************************************
* Function Name: debug_printf
********************************************************************************
* Summary:
* This function print out the current date time and user string.
*
* Parameters:
*  str      Point to the user print string.
*
* Return:
*  void
*
*******************************************************************************/
void debug_printf(const char *str)
{
    struct tm currentDateTime = {0};
    char buffer[STRING_BUFFER_SIZE];

    /* Get the current time and date from the RTC peripheral */
    cy_stc_rtc_config_t dateTime = { .hrFormat = CY_RTC_24_HOURS };
    Cy_RTC_GetDateAndTime(&dateTime);
    const int year = (int)(dateTime.year + 2000u);
    rtc_from_pdl_time(&dateTime, year, &currentDateTime);

    /* strftime() is a C library function which is used to format date and time into string.
     * It comes under the header file "time.h" which is included by HAL (See
     * http://www.cplusplus.com/reference/ctime/strftime/)
     */
    strftime(buffer, sizeof(buffer), "%X %F", &currentDateTime);
    /* Print the the current date and time and user string */
    printf("%s: %s", buffer, str);
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/** Wrapper around the PDL RTC interrupt handler to adapt the function signature */
static void RTC_Handler(void)
{
    cy_stc_rtc_dst_t const *dstTime = 0;
    Cy_RTC_Interrupt(dstTime, false);
}
void rtc_from_pdl_time(cy_stc_rtc_config_t *pdlTime, const int year, struct tm *time)
{
    /* The number of days that precede each month of the year, not including Feb 29 */
    static const uint16_t CUMULATIVE_DAYS[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    time->tm_sec = (int)pdlTime->sec;
    time->tm_min = (int)pdlTime->min;
    time->tm_hour = (int)pdlTime->hour;
    time->tm_mday = (int)pdlTime->date;
    time->tm_mon = (int)(pdlTime->month - 1u);
    time->tm_year = (int)(year - _RTC_TM_YEAR_BASE);
    time->tm_wday = (int)(pdlTime->dayOfWeek - 1u);
    time->tm_yday = (int)CUMULATIVE_DAYS[time->tm_mon] + (int)pdlTime->date - 1 +
        (((int)(pdlTime->month) >= 3 && (int)(Cy_RTC_IsLeapYear((uint32_t)year) ? 1u : 0u)));
    time->tm_isdst = -1;
}

/* [] END OF FILE */
