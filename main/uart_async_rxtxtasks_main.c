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

#define PIN (1 << 22)
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define GPIO_INPUT_IO_1     36
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_1)
#define ESP_INTR_FLAG_DEFAULT 0


static xQueueHandle gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    gpio_intr_disable(gpio_num);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            vTaskDelay(1000 / portTICK_RATE_MS);
            printf("Enabling interrupt again..\n");
            gpio_intr_enable(io_num);
        }
    }
}

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

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


  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  //start gpio task
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

}




void app_main(void)
{
    init();
    gpio_init();

    
		//while(1);
		xTaskCreate(tx_task, "RESET", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
