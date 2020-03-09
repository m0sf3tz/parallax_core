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

void app_main(void)
{
    
	xTaskCreate(parallax_thread, "parallax_core", 4096, NULL, configMAX_PRIORITIES-1, NULL);

  while(parallaxCoreReady == 0)
  {
   portYIELD();
  }

   commandQ_parallax_t qCommand;
   memset(&qCommand, 0, sizeof(qCommand));

/*
   qCommand.command = PARALLAX_DLT_ALL;
   qCommand.id      = 12;
   parallax_thread_gate(&qCommand);
*/


   qCommand.command = PARALLAX_ADD_USER;
   qCommand.id      = 11;
   int x = parallax_thread_gate(&qCommand);
   printf("Response = %d\n", x);
}
