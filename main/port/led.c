/**************************************************************************
 *
 * Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *********************************************************************/
#include <stdint.h>
/**** From STM32 *****************
 * #include "stm32f4xx.h"
*********************************/

/**** Import ESP32 modules ****/
 /** Free RTOS **/
 
 /** Other **/
  #include "driver/gpio.h"
  #include "esp_log.h"
/***************************************/
#include "bacnet/basic/sys/mstimer.h"
#include "led.h"

#define LD1 21      //0    
#define LD2 22      //1
#define LD3 23      //2
#define LD4 02      //3 (RS485-LED) 


static struct mstimer Off_Delay_Timer[LED_MAX];
static bool LED_State[LED_MAX];

/*************************************************************************
 * Description: Activate the LED
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void led_on(unsigned index)
{
    switch (index) {
        case LED_LD1:
            gpio_set_level(LD1, 1);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = true;
            break;
        case LED_LD2:
            gpio_set_level(LD2, 1);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = true;
            break;
        case LED_LD3:
            gpio_set_level(LD3, 1);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = true;
            break;
        case LED_RS485:
            gpio_set_level(LD4, 1);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = true;
            break;
        default:
            break;
    }
}

/*************************************************************************
 * Description: Deactivate the LED
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void led_off(unsigned index)
{
    switch (index) {
        case LED_LD1:
            gpio_set_level(LD1, 0);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = false;
            break;
        case LED_LD2:
            gpio_set_level(LD2, 0);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = false;
            break;
        case LED_LD3:
            gpio_set_level(LD3, 0);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = false;
            break;
        case LED_RS485:
            gpio_set_level(LD4, 0);
            mstimer_set(&Off_Delay_Timer[index], 0);
            LED_State[index] = false;
            break;
        default:
            break;
    }
}

/*************************************************************************
 * Description: Get the state of the LED
 * Returns: true if on, false if off.
 * Notes: none
 *************************************************************************/
bool led_state(unsigned index)
{
    bool state = false;

    if (index < LED_MAX) {
        if (LED_State[index]) {
            state = true;
        }
    }

    return state;
}

/*************************************************************************
 * Description: Toggle the state of the LED
 * Returns: none
 * Notes: none
 *************************************************************************/
void led_toggle(unsigned index)
{
    if (led_state(index)) {
        led_off(index);
    } else {
        led_on(index);
    }
}

/*************************************************************************
 * Description: Delay before going off to give minimum brightness.
 * Returns: none
 * Notes: none
 *************************************************************************/
void led_off_delay(unsigned index, uint32_t delay_ms)
{
    if (index < LED_MAX) {
        mstimer_set(&Off_Delay_Timer[index], delay_ms);
    }
}

/*************************************************************************
 * Description: Turn on, and delay before going off.
 * Returns: none
 * Notes: none
 *************************************************************************/
void led_on_interval(unsigned index, uint16_t interval_ms)
{
    if (index < LED_MAX) {
        led_on(index);
        mstimer_set(&Off_Delay_Timer[index], interval_ms);
    }
}

/*************************************************************************
 * Description: Task for blinking LED
 * Returns: none
 * Notes: none
 *************************************************************************/
void led_task(void)
{
    unsigned index = 0;

    for (index = 0; index < LED_MAX; index++) {
        if (mstimer_expired(&Off_Delay_Timer[index])) {
            led_off(index);
        }
    }
}

/*************************************************************************
 * Description: Initialize the LED hardware
 * Returns: none
 * Notes: none
 *************************************************************************/
void led_init(void)
{
    /* Configure the Receive LED on MS/TP board */
    gpio_pad_select_gpio(LD1);
    gpio_set_direction(LD1, GPIO_MODE_OUTPUT);
    /* Configure the Transmit LED on MS/TP board */
    gpio_pad_select_gpio(LD2);
    gpio_set_direction(LD2, GPIO_MODE_OUTPUT);
    /* Configure the LD3 on Discovery board */
    gpio_pad_select_gpio(LD3);
    gpio_set_direction(LD3, GPIO_MODE_OUTPUT);
    /* Configure the LD4 on Discovery board */
    gpio_pad_select_gpio(LD4);
    gpio_set_direction(LD4, GPIO_MODE_OUTPUT);

    led_off(LED_LD1);
    led_off(LED_LD2);
    led_off(LED_LD3);
    led_off(LED_RS485);
}

    // /*** STM32 Init LED ***
    // GPIO_InitTypeDef GPIO_InitStructure;

    // /* NUCLEO board user LEDs */
    // /* Enable GPIOx clock */
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // /* start with fresh structure */
    // GPIO_StructInit(&GPIO_InitStructure);
    // /* Configure the LED on NUCLEO board */
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);

    // /* RS485 shield user LED */
    // /* Enable GPIOx clock */
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // /* start with fresh structure */
    // GPIO_StructInit(&GPIO_InitStructure);
    // /* Configure the LED on NUCLEO board */
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);