//
// Copyleft  F.Chaxel 2017
//
/**** From STM32 *****************/
/*                          77 STM32 specific? 
* #include <stdbool.h>  
* #include <stdint.h>
*********************************/
// #include "hardware.h"  //also STM32 specific? 
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/sys/mstimer.h"
#include "rs485.h"
#include "led.h"
#include "bacnet.h"
/*******************************/
// #define BACDL_MSTP

#include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event_loop.h"
#include "nvs_flash.h"

// #include "driver/gpio.h"

// #include "lwip/sockets.h"
// #include "lwip/netdb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// #include "bacnet/wp.h"

char *BACnet_Version = "1.0";
#define PRINT_ENABLED

void app_main(void) 
{
    // struct mstimer Blink_Timer;

   // xTaskCreate(mstimer_init, "mstimer_init",512,NULL,5,NULL);
    // mstimer_init();
    led_init();
    bacnet_init();
    // mstimer_set(&Blink_Timer, 125);

    xTaskCreate(bacnet_task, /* Function to implement the task */
        "BACnetTask", /* Name of the task */
        10000, /* Stack size in words */
        NULL, /* Task input parameter */
        5, /* Priority of the task */
        NULL); /* Task handle. */


   // wp_encode_apdu()
}
