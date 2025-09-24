/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cybsp.h"
#ifdef COMPONENT_MTB_HAL
#include "mtb_hal.h"
#include "cy_time.h"
#include "retarget_io_init.h"
#else
#include "cyhal.h"
#endif
#include <cybsp_wifi.h>
#include <FreeRTOS.h>
#include <task.h>
#include "cyabs_rtos.h"
#include <stdio.h>
#include <lwip/tcpip.h>
#include <lwip/api.h>
#include "command_console.h"
#include "iperf_utility.h"
#include "wifi_utility.h"
#include "cy_wcm.h"
#if (defined COMPONENT_WICED_BLE)
#include "bt_utility.h"
#include "bt_cfg.h"
#endif

#include "ent_sec_utility.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define BLINKY_LED_DELAY_MSEC       (1000U)

/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

typedef mtb_hal_rtc_t rtc_type;
extern const cy_stc_rtc_config_t CYBSP_RTC_config;

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;

/* wcm parameters */
static cy_wcm_config_t wcm_config;
static cy_wcm_connect_params_t conn_params;

cy_rslt_t ConnectWifi();

#define CONSOLE_COMMAND_MAX_PARAMS     (32)
#define CONSOLE_COMMAND_MAX_LENGTH     (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH (10)

const char* console_delimiter_string = " ";

static char command_buffer[CONSOLE_COMMAND_MAX_LENGTH];
static char command_history_buffer[CONSOLE_COMMAND_MAX_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

#define WIFI_SSID                        "WIFI_SSID"
#define WIFI_KEY                         "WIFI_PASSWORD"
#define WIFI_BAND                        CY_WCM_WIFI_BAND_ANY
#define CMD_CONSOLE_MAX_WIFI_RETRY_COUNT 15
#define IP_STR_LEN                       16

#define CY_RSLT_ERROR                    ( -1 )

/* UART HAL object used by BSP for Debug UART port */
#if defined(COMPONENT_MTB_HAL)
extern mtb_hal_uart_t *CYBSP_DEBUG_UART_hal_obj;
#else
extern cyhal_uart_t cy_retarget_io_uart_obj;
#endif

extern cy_stc_sd_host_context_t sdhc_host_context;

cy_stc_syspm_callback_params_t sdcardDSParams =
{   
    CYBSP_WIFI_SDIO_HW,
    &sdhc_host_context
};

cy_stc_syspm_callback_t sdDeepSleepCallbackHandler =
{
    Cy_SD_Host_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &sdcardDSParams,
    NULL,
    NULL,
    0
};

cy_rslt_t command_console_add_command();

/*******************************************************************************
* LP Timer Function Prototypes
*******************************************************************************/
#if (MTB_HAL_DRIVER_AVAILABLE_LPTIMER)
#include "cycfg_peripherals.h"
mtb_hal_lptimer_t lptimer_obj_overall;

//Only MCWDT_b still defines this value so in case we don't have it defined we use the old presets
#if !defined _MTB_HAL_LPTIMER_RESET_TIME_US
#if defined (CY_IP_M0S8WCO)
#define _MTB_HAL_LPTIMER_RESET_TIME_US      (180)
#else
#define _MTB_HAL_LPTIMER_RESET_TIME_US       (62)
#endif
#endif

#if (defined (CY_IP_MXS40SRSS) && (CY_IP_MXS40SRSS_VERSION >= 2)) || ((SRSS_NUM_MCWDT_B) > 0)

#define  _Cy_MCWDT_Enable(base)     Cy_MCWDT_Unlock(base); \
                                    Cy_MCWDT_Enable(base, (CY_MCWDT_CTR1 | CY_MCWDT_CTR2), _MTB_HAL_LPTIMER_RESET_TIME_US); \
                                    Cy_MCWDT_Lock(base);
#else
#define _TEST_LPTIMER_CTRL          (CY_MCWDT_CTR0 | CY_MCWDT_CTR1 | CY_MCWDT_CTR2) /* _MTB_HAL_LPTIMER_CTRL */
#define  _Cy_MCWDT_Enable(base)     Cy_MCWDT_Enable(base, _TEST_LPTIMER_CTRL, _MTB_HAL_LPTIMER_RESET_TIME_US);
#endif

/* In design.modus we should setup an lptimer and call it lptimer_0.
However for Explorer devices each LPTimer instance is dedicated to each core (mcwdt[0] - cm33, mcwdt[1] - cm55)
therefore we can't use one for both tests.
For this reason in design.modus we initialize both and give them specific names.
The following defines and pointers are referencing the correct generated structures and are then used
throughout the test code. */
#if defined(CY_DEVICE_PSE84) || defined(CY_DEVICE_PSE84_A0)
#if (CY_CPU_CORTEX_M55)
#define lptimer_IRQ CYBSP_CM55_LPTIMER_1_IRQ
const mtb_hal_lptimer_configurator_t* lptimer_hal_config_overall =  (const mtb_hal_lptimer_configurator_t*)&CYBSP_CM55_LPTIMER_1_hal_config;
const cy_stc_mcwdt_config_t* lptimer_config_overall  = (const cy_stc_mcwdt_config_t*)&CYBSP_CM55_LPTIMER_1_config;
#else
#define lptimer_IRQ CYBSP_CM33_LPTIMER_0_IRQ
const mtb_hal_lptimer_configurator_t* lptimer_hal_config_overall = (const mtb_hal_lptimer_configurator_t*) &CYBSP_CM33_LPTIMER_0_hal_config;
const cy_stc_mcwdt_config_t* lptimer_config_overall  = (const cy_stc_mcwdt_config_t*)&CYBSP_CM33_LPTIMER_0_config;
#endif
#else
#define lptimer_IRQ lptimer_0_IRQ
const mtb_hal_lptimer_configurator_t* lptimer_hal_config_overall = (const mtb_hal_lptimer_configurator_t*) &lptimer_0_hal_config;
const cy_stc_mcwdt_config_t* lptimer_config_overall  = (const cy_stc_mcwdt_config_t*)&lptimer_0_config;
#endif

static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj_overall);
}

cy_rslt_t init_lptimer(mtb_hal_lptimer_t* obj)
{
    cy_rslt_t result = (cy_rslt_t) Cy_MCWDT_Init(lptimer_hal_config_overall->base, lptimer_config_overall);
    if(CY_RSLT_SUCCESS != result)
    {
        return -1;
    }
    _Cy_MCWDT_Enable(lptimer_hal_config_overall->base);

    result = mtb_hal_lptimer_setup(&lptimer_obj_overall, lptimer_hal_config_overall);
    if(CY_RSLT_SUCCESS != result)
    {
        return -1;
    }

    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = lptimer_IRQ,
        .intrPriority = 7u
    };

    Cy_SysInt_Init(&lptimer_intr_cfg, lptimer_interrupt_handler);
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    return CY_RSLT_SUCCESS;
}
#endif

static void get_ip_string(char* buffer, uint32_t ip)
{
    sprintf(buffer, "%lu.%lu.%lu.%lu",
            (unsigned long)(ip      ) & 0xFF,
            (unsigned long)(ip >>  8) & 0xFF,
            (unsigned long)(ip >> 16) & 0xFF,
            (unsigned long)(ip >> 24) & 0xFF);
}

void registerSDCardDSCallback(void)
{
    Cy_SysPm_RegisterCallback(&sdDeepSleepCallbackHandler);
}

#if MTB_HAL_DRIVER_AVAILABLE_SDIO
mtb_hal_sdio_t sdio_instance;
cy_stc_sd_host_context_t sdhc_host_context;
#define SDHC_SDIO_64B_BLOCK                 (64U)

void sdio_interrupt_handler(void)
{
    mtb_hal_sdio_process_interrupt(&sdio_instance);
}

void app_sdio_init(void)
{
    mtb_hal_sdio_cfg_t hal_cfg;

    mtb_hal_sdio_setup(&sdio_instance, &CYBSP_WIFI_SDIO_sdio_hal_config, NULL, &sdhc_host_context);

    Cy_SD_Host_Enable(sdio_instance.sdxx.base);
    Cy_SD_Host_Init(sdio_instance.sdxx.base, CYBSP_WIFI_SDIO_sdio_hal_config.host_config, &sdhc_host_context);
    Cy_SD_Host_SetHostBusWidth(sdio_instance.sdxx.base, CY_SD_HOST_BUS_WIDTH_4_BIT);
    cy_stc_sysint_t intr_cfg_1 = {.intrSrc = CYBSP_WIFI_SDIO_IRQ, .intrPriority = 7u};
    Cy_SysInt_Init(&intr_cfg_1, sdio_interrupt_handler);
    NVIC_EnableIRQ(CYBSP_WIFI_SDIO_IRQ);

    hal_cfg.frequencyhal_hz =  25000000;
    hal_cfg.block_size = SDHC_SDIO_64B_BLOCK;
    mtb_hal_sdio_configure(&sdio_instance, &hal_cfg);

    mtb_hal_gpio_setup(&wcm_config.wifi_wl_pin, CYBSP_WIFI_WL_REG_ON_PORT_NUM, CYBSP_WIFI_WL_REG_ON_PIN);
    mtb_hal_gpio_setup(&wcm_config.wifi_host_wake_pin, CYBSP_WIFI_HOST_WAKE_PORT_NUM, CYBSP_WIFI_HOST_WAKE_PIN);
}
#endif

cy_rslt_t ConnectWifi()
{
    cy_rslt_t res ;

    const char *ssid = WIFI_SSID ;
    const char *key = WIFI_KEY ;
    cy_wcm_wifi_band_t band = WIFI_BAND;
    int retry_count = 0;
    cy_wcm_ip_address_t ip_addr;
    char ipstr[IP_STR_LEN];

    memset(&conn_params, 0, sizeof(cy_wcm_connect_params_t));

    while (1)
    {
        /*
        * Join to WIFI AP
        */
        memcpy(&conn_params.ap_credentials.SSID, ssid, strlen(ssid) + 1);
        memcpy(&conn_params.ap_credentials.password, key, strlen(key) + 1);
        conn_params.ap_credentials.security = CY_WCM_SECURITY_WPA2_AES_PSK;
        conn_params.band = band;

        res = cy_wcm_connect_ap(&conn_params, &ip_addr);
        cy_rtos_delay_milliseconds(500);

        if(res != CY_RSLT_SUCCESS)
        {
            retry_count++;
            if (retry_count >= CMD_CONSOLE_MAX_WIFI_RETRY_COUNT)
            {
                printf("Exceeded max WiFi connection attempts\n");
                return CY_RSLT_ERROR;
            }
            printf("Connection to WiFi network failed. Retrying...\n");
            continue;
        }
        else
        {
            printf("Successfully joined wifi network '%s , result = %ld'\n", ssid, (long)res);
            get_ip_string(ipstr, ip_addr.ip.v4);
            printf("IP Address %s assigned\n", ipstr);
            break;
        }
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t command_console_add_command(void) {

    cy_command_console_cfg_t console_cfg;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    printf( "executing command_console_add_remove_command \n");
#if defined(COMPONENT_MTB_HAL)
    console_cfg.serial             = (void *)CYBSP_DEBUG_UART_hal_obj;
#else
    console_cfg.serial             = (void *)&cy_retarget_io_uart_obj;
#endif
    console_cfg.line_len           = sizeof(command_buffer);
    console_cfg.buffer             = command_buffer;
    console_cfg.history_len        = CONSOLE_COMMAND_HISTORY_LENGTH;
    console_cfg.history_buffer_ptr = command_history_buffer;
    console_cfg.delimiter_string   = console_delimiter_string;
    console_cfg.params_num         = CONSOLE_COMMAND_MAX_PARAMS;
    console_cfg.thread_priority    = CY_RTOS_PRIORITY_NORMAL;
    console_cfg.delimiter_string   = " ";

    /* Initialize command console library */
    result = cy_command_console_init(&console_cfg);
    if ( result != CY_RSLT_SUCCESS )
    {
        printf ("Error in initializing command console library : %ld \n", (long)result);
        goto error;
    }

    /* Initialize Wi-Fi utility and add Wi-Fi commands */
    result = wifi_utility_init();
    if ( result != CY_RSLT_SUCCESS )
    {
        printf ("Error in initializing command console library : %ld \n", (long)result);
        goto error;
    }

    /* Initialize IPERF utility and add IPERF commands */
    iperf_utility_init(&wcm_config.interface);

#if (defined COMPONENT_WICED_BLE)
    /* Initialize Bluetooth utility and add BT commands */
    bt_utility_init();
#endif

    /* Initialize Enterprise Security utility commands */
    ent_utility_init();

    return CY_RSLT_SUCCESS;

error:
    return CY_RSLT_ERROR;

}

static void console_task(void *arg)
{
    cy_rslt_t res;

    printf(" CY_SRAM_SIZE:%ld\n", (long)CY_SRAM_SIZE);
    printf(" Heap size:%d\n", configTOTAL_HEAP_SIZE);
    printf(" SystemCoreClock:%ld\n", (long)SystemCoreClock);
    printf("==============================================\n");

    wcm_config.interface = CY_WCM_INTERFACE_TYPE_AP_STA;
    wcm_config.wifi_interface_instance = &sdio_instance;
    res = cy_wcm_init(&wcm_config);
    if(res != CY_RSLT_SUCCESS)
    {
        printf("Error: \n------------------------------------------------------------------------------\n"
           "Wi-Fi Connection Manager initialization failed!\n"
           "This version of the code example only supports EVK versions REV *C and above that contain the latest revision of CYW55513 device.\n"
           "The revision of CYW55513 is printed in the messages above and it should be 'chip rev: 1' for this code example to work.\n"
           "Verify your EVK version otherwise contact Infineon representative to get the latest hardware.\n"
           "It could be hardware issue if you are using the latest hardware and still see this error.\n"
           "-------------------------------------------------------------------------------\n");
        CY_ASSERT(0);
    }
    printf("WCM Initialized\n");

    command_console_add_command();

    while(1)
    {
        cy_rtos_delay_milliseconds(500);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM55 application. 
* 
* CM33 application enables the CM55 CPU and then the CM55 application enters 
* deep sleep.
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
    rtc_type obj;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    mtb_clib_support_init(&obj);

#if (MTB_HAL_DRIVER_AVAILABLE_LPTIMER)
    result = init_lptimer(&lptimer_obj_overall);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }
    cyabs_rtos_set_lptimer(&lptimer_obj_overall);
#endif
    app_sdio_init();
    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* Register for deepsleep callback */
    registerSDCardDSCallback();
    printf("Command console Enterprise-security Application !!!\r\n\n");

    xTaskCreate(console_task, "console", 1024*10, "console", 1, NULL) ;
    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    return 0;
}

/* [] END OF FILE */
