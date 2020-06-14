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
 *
 * Module Description:
 * Handle the configuration and operation of the RS485 bus.
 **************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
// #include "hardware.h" //STM 32 specifc
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/bits.h"
#include "bacnet/basic/sys/fifo.h"
#include "led.h"
#include "rs485.h"

/* This files has been added for use with ESP32 */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


static const char *TAG = "RS485";

/* Define pins for Uart1 */
//Define pins vor UART1
#define UART0_TXD           (17)
#define UART0_RXD           (16)
#define UART0_RTS           (18)

#define BUF_SIZE            (1024)
#define RD_BUF_SIZE         (BUF_SIZE)
#define PACKET_READ_TICS    (500 / portTICK_RATE_MS) 
static QueueHandle_t uart_queue;

/* buffer for storing received bytes - size must be power of two */
static uint8_t Receive_Buffer_Data[512];
static FIFO_BUFFER Receive_Buffer;
/* amount of silence on the wire */
static struct mstimer Silence_Timer;
/* baud rate */
static uint32_t Baud_Rate = 38400;

/* The minimum time after the end of the stop bit of the final octet of a */
/* received frame before a node may enable its EIA-485 driver: 40 bit times. */
/* At 9600 baud, 40 bit times would be about 4.166 milliseconds */
/* At 19200 baud, 40 bit times would be about 2.083 milliseconds */
/* At 38400 baud, 40 bit times would be about 1.041 milliseconds */
/* At 57600 baud, 40 bit times would be about 0.694 milliseconds */
/* At 76800 baud, 40 bit times would be about 0.520 milliseconds */
/* At 115200 baud, 40 bit times would be about 0.347 milliseconds */
/* 40 bits is 4 octets including a start and stop bit with each octet */
#define Tturnaround (40UL)

/*************************************************************************
 * Description: Reset the silence on the wire timer.
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_silence_reset(void)
{
    mstimer_set(&Silence_Timer, 0);
}

/*************************************************************************
 * Description: Determine the amount of silence on the wire from the timer.
 * Returns: true if the amount of time has elapsed
 * Notes: none
 **************************************************************************/
bool rs485_silence_elapsed(uint32_t interval)
{
  
    return (mstimer_elapsed(&Silence_Timer) > interval);
}

/*************************************************************************
 * Description: Baud rate determines turnaround time.
 * Returns: amount of milliseconds
 * Notes: none
 **************************************************************************/
static uint16_t rs485_turnaround_time(void)
{
    ESP_LOGI(TAG,"rs485_turnaround_time");
    /* delay after reception before transmitting - per MS/TP spec */
    /* wait a minimum  40 bit times since reception */
    /* at least 2 ms for errors: rounding, clock tick */
    if (Baud_Rate) {
        return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
    } else {
        return 2;
    }
}

/*************************************************************************
 * Description: Use the silence timer to determine turnaround time.
 * Returns: true if turnaround time has expired.
 * Notes: none
 **************************************************************************/
bool rs485_turnaround_elapsed(void)
{
    ESP_LOGI(TAG,"rs485_turnaround_elapsed");
    return (mstimer_elapsed(&Silence_Timer) > rs485_turnaround_time());
}

/*************************************************************************
 * Description: Determines if an error occured while receiving
 * Returns: true an error occurred.
 * Notes: none
 **************************************************************************/
bool rs485_receive_error(void)
{
    // ESP_LOGI(TAG,"rs485_receive_error: false");
    return false;
}

/*********************************************************************/

/*********************************************************************
* @brief
*USARTx
*interrupt
*handler
*sub-routine
* @param[in]
*None
* @return
*None
**********************************************************************/
// /****STM32 Specific*********/ //Changed for ES32 void USART2_IRQHandler(void)
// void Receive_Task(void)
// {
//    uint8_t data_byte;
//    ESP_LOGI(TAG,"looking for uart darta");
//    uart_read_bytes(UART_NUM_1, &data_byte, 1, 100 / portTICK_RATE_MS); 
//    (void)FIFO_Put(&Receive_Buffer, data_byte);

//     // if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
//     //     /* Read one byte from the receive data register */
//     //     data_byte = USART_ReceiveData(USART2);
//     //     (void)FIFO_Put(&Receive_Buffer, data_byte);
//     // }
// }
/*************************************************************************
 * DESCRIPTION: Reads the uart bytes
 * RETURN:      true if a byte is available, with the byte in the parameter
 * NOTES:       none
 **************************************************************************/
void uart_event_task()
{
uart_event_t event;
    // size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
     uint8_t data_byte;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGV(TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGV(TAG, "[UART DATA]: %d", event.size);
                    const uint8_t len =  uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                   // memcpy(data_byte, dtmp, event.size);
                    // data_byte = dtmp;
                    ESP_LOG_BUFFER_HEX(TAG, dtmp, event.size);
                    for (int i = 0; i < len; i++) {
                    data_byte = dtmp[i];
                    (void)FIFO_Put(&Receive_Buffer, data_byte);
                    }
                     
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGV(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGV(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGV(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGV(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGV(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGV(TAG, "detect pattern. No pattern set");
                    break;
                //Others
                default:
                    ESP_LOGV(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}



/*************************************************************************
 * DESCRIPTION: Return true if a byte is available
 * RETURN:      true if a byte is available, with the byte in the parameter
 * NOTES:       none
 **************************************************************************/
bool rs485_byte_available(uint8_t *data_register)
{
    // int8_t *data = (uint8_t *) malloc(8);
    
    bool data_available = false; /* return value */
    ESP_LOGI(TAG, "check if Bytes aviable");

    if (!FIFO_Empty(&Receive_Buffer)) {
        if (data_register) {
            *data_register = FIFO_Get(&Receive_Buffer);
        }
        rs485_silence_reset();
        data_available = true;
        //led_rx_on_interval(10);
    }




//          const int len =  uart_read_bytes(UART_NUM_1,data_register,1,PACKET_READ_TICS);
//             if ( len > 0)
//                 {
//                 ESP_LOGI(TAG, "Received %u bytes:", len);
//                 // printf("[ ");
                    
//                     ESP_LOG_BUFFER_HEX(TAG,data_register,1);
//                     // printf("0x%.2X ", (uint8_t)data);
//                     // *data_register = (uint8_t)data;
//                     // ESP_LOG_BUFFER_HEX(TAG,data_register,1);

// /* 
//                  for (int i = 0; i < len; i++) {
//                     printf("0x%.2X ", (uint8_t)data[i]);
//                     data_register[1] = (uint8_t)data[i];
//                     // printf("0x%.2X ", (uint8_t)data_register[1]);
                
//                  } 
//                  */

//                 // printf("] \n");
//                 // data[len]= '\0';
//                  fflush(stdout);
                                
//                 rs485_silence_reset();
//                 led_rx_on_interval(10);
//                 data_available = true;
                
                    
//                 }
           
    


/*     
    ESP_LOGI(TAG,"RS485_DataAvailable: Look if data is aviable:%d", data_available);
    const int len = uart_read_bytes(UART_NUM_1, data, 8, 100 / portTICK_RATE_MS);
        if ( len > 0)
             {
                // data[len]= '\0';
                ESP_LOGI(TAG, "Received %u bytes:" ,len);
                for (int i = 0; i < len; i++) {
                 printf("0x%.2X ", (uint8_t)data[1]);
                }
                 fflush(stdout);
                 
                 rs485_silence_reset();
                 data_available = true;
                 led_rx_on_interval(10);
                 
             }

 */
/*    
     if (!FIFO_Empty(&Receive_Buffer)) {
        if (data_register) {
            *data_register = FIFO_Get(&Receive_Buffer);
            ESP_LOGI(TAG,"rs485_byte_available");
        }
        rs485_silence_reset();
        data_available = true;
        led_rx_on_interval(10);
    }
 */
    return data_available;
}

/*************************************************************************
 * DESCRIPTION: Sends a byte of data
 * RETURN:      nothing
 * NOTES:       none
 **************************************************************************/
void rs485_byte_send(uint8_t tx_byte)
{
  //   ESP_LOG_BUFFER_HEX(TAG, tx_byte, 1);
    ESP_LOGI(TAG,"byte_send");
    led_tx_on_interval(10);
    uart_write_bytes(UART_NUM_1, (const char*)&tx_byte, 1);
    rs485_silence_reset();
}

/*************************************************************************
 * Description: Determines if a byte in the USART has been shifted from
 *   register
 * Returns: true if the USART register is empty
 * Notes: none
 **************************************************************************/
bool rs485_byte_sent(void)
{
    ESP_LOGI(TAG,"byte_sent");
     esp_err_t err = uart_wait_tx_done(UART_NUM_1,100);
    if (err == ESP_OK)
    {
        return true;
    } else {
        return false;
     } 
// return uart_wait_tx_done(UART_NUM_1,100);
}

/*************************************************************************
 * Description: Determines if the entire frame is sent from USART FIFO
 * Returns: true if the USART FIFO is empty
 * Notes: none
 **************************************************************************/
bool rs485_frame_sent(void)
{
    ESP_LOGI(TAG,"rs485_frame_sent");
     esp_err_t err = uart_wait_tx_done(UART_NUM_1,100);
    if (err == ESP_OK)
    {
        return true;
    } else {
        return false;
     }
 
    // return uart_wait_tx_done(UART_NUM_1,100); //alternative ?? 
    // return USART_GetFlagStatus(USART2, USART_FLAG_TC);
}

/*************************************************************************
 * DESCRIPTION: Send some data and wait until it is sent
 * RETURN:      true if a collision or timeout occurred
 * NOTES:       none
 **************************************************************************/
void rs485_bytes_send(uint8_t *buffer, /* data to send */
    uint16_t nbytes)
{ /* number of bytes of data */
    uint8_t tx_byte;
    ESP_LOGI(TAG, "rs485_bytes_send");
    while (nbytes) {
        /* Send the data byte */
        tx_byte = *buffer;
        /* Send one byte */
        uart_write_bytes(UART_NUM_1, (const char*)&tx_byte, 1);
        // uart_write_bytes(UART_NUM_1, (const char*)&buffer[i], 1); //Original for comparison
        while (!rs485_byte_sent()) {
            /* do nothing - wait until Tx buffer is empty */
        }
        buffer++;
        nbytes--;
    }
    /* was the frame sent? */
    while (!rs485_frame_sent()) {
        /* do nothing - wait until the entire frame in the
           Transmit Shift Register has been shifted out */
    }
    rs485_silence_reset();

    return;
}

/*************************************************************************
 * Description: Configures the baud rate of the USART
 * Returns: nothing
 * Notes: none
 **************************************************************************/
static void rs485_baud_rate_configure(void)
{
    uart_config_t uart0_config = {
        .baud_rate = Baud_Rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    ESP_LOGI(TAG, "Start Modem application test and configure UART.");
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 20, &uart_queue, 0);
    // Configure UART parameters
    uart_param_config(UART_NUM_1, &uart0_config);
    uart_set_pin(UART_NUM_1, UART0_TXD, UART0_RXD, UART0_RTS, UART_PIN_NO_CHANGE);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);

    //Reads uart and puts data to FIFO Buffer
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
  
}

/*************************************************************************
 * Description: Sets the baud rate to non-volatile storeage and configures USART
 * Returns: true if a value baud rate was saved
 * Notes: none
 **************************************************************************/
bool rs485_baud_rate_set(uint32_t baud)
{
    bool valid = true;

    switch (baud) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 76800:
        case 115200:
            Baud_Rate = baud;
            rs485_baud_rate_configure();
            break;
        default:
            valid = false;
            break;
    }

    return valid;
}

/*************************************************************************
 * Description: Determines the baud rate in bps
 * Returns: baud rate in bps
 * Notes: none
 **************************************************************************/
uint32_t rs485_baud_rate(void)
{
    return Baud_Rate;
}

/*************************************************************************
 * Description: Enable the Request To Send (RTS) aka Transmit Enable pin
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_rts_enable(bool enable)
{
    if (enable) {
        ESP_LOGI(TAG, "RTS_Enable");
        ESP_ERROR_CHECK(uart_set_rts(UART_NUM_1,1));
        
    } else {
        ESP_LOGI(TAG, "RTS_Disable");
        ESP_ERROR_CHECK(uart_set_rts(UART_NUM_1,0));
        
    }
}

/*************************************************************************
 * Description: Initialize the room network USART
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_init(void)
{
    rs485_baud_rate_set(Baud_Rate);
    //  uint8_t tx_byte = 0X11;
    // ESP_LOGI(TAG,"write test byte to uart");
     uart_write_bytes(UART_NUM_1, "TEST BYtes", 20); 
    // USART_Cmd(USART2, ENABLE);
    vTaskDelay(2000/portTICK_PERIOD_MS); 
    //xTaskCreate(Receive_Task, "Receive", 512, NULL, 3,NULL);
    //Receive_Task();


     
        FIFO_Init(&Receive_Buffer, &Receive_Buffer_Data[0],
        (unsigned)sizeof(Receive_Buffer_Data));
    

    rs485_silence_reset();
}
