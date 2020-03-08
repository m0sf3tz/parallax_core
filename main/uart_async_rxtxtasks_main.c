/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "parallax.h"


//static const int RX_BUF_SIZE = 1024;




int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}


extern void resetDevice();
static void tx_task(void *arg)
{
	
 	 resetDevice();

   //matchFingerPrintToId();

   deleteAllUsers(); 
   //addUser(13); 
   //fetchNumberOfUsers(); 
   while (1) {
   }
}


/*
void gpio_init()
{
  //
  // SET UP RESET PIN
  //
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = PIN;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
   
  gpio_set_level(22, 1);

  //
  // SET UP PROXIMITY GPIO
  //
  //interrupt of LOW level
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  //bit mask of the pins, use GPIO5
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode    
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

}

*/


void app_main(void)
{
    
	xTaskCreate(parallax_thread, "parallax_core", 4096, NULL, configMAX_PRIORITIES-1, NULL);

  while(parallaxCoreReady == 0)
  {
   portYIELD();
  }


   commandQ_parallax_t qCommand;
   memset(&qCommand, 0, sizeof(qCommand));
    
   //qCommand.command = PARALLAX_ADD_USER;
   //qCommand.id      = 12;
   //parallax_thread_gate(&qCommand);

   qCommand.command = PARALLAX_CMP_USER;
   int ret = parallax_thread_gate(&qCommand);
   printf("%d \n", ret);

}
