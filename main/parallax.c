#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "parallax.h"

// Globals 
int parallaxCoreReady;

static QueueHandle_t parallaxCommandQ;
static QueueHandle_t  parallaxCommandQ_res;

static SemaphoreHandle_t parallaxCommandMutex;
static xQueueHandle gpio_evt_queue;

// Debug tag
static const char TAG[30] = "PARALLAX_CORE";

// ISR tied to proximity GPIO
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    gpio_intr_disable(gpio_num);
}

// Put parallax in reset
// reset = 0 ---> take out of reset
// reset = 1 ---> put in reset
static void parallax_reset(int reset)
{
  if (reset != 0 && reset != 1)
  {
    ESP_LOGE(TAG, "INCORRECT VALUE TO PARALLAX RESET");
    return; 
  }
  gpio_set_level(RESET_PIN, reset);
}

static void decorate_function(const char * str)
{
  printf("\n");
  printf("%s\n", str);
  printf("\n");
}

char checksum(char * packet)
{
  // Calculate the checksum
  return (packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]);
}

// Checks a packet has the correct response 
// assumes response is the 5th byte
static char getResponse(char * packet)
{
  return(packet[RESPONSE_BYTE]);
}

// Prints out a packet
static void printPacket(char * command, const char * start_message)
{
  int i;
  
  printf("%s", start_message);
  for (i = 0; i < 8; i++){
    printf("0x%02x ", command[i]);
  }
  printf("\n");
}

// Sanitize response from device, verify checksum + len
static int validateResponse(int bytes_len, char * packet)
{
  if (bytes_len != RESPONSE_LEN)
  {
    printf("Faled to validate response len, expected %d - got %d \n", RESPONSE_LEN, bytes_len);
    goto reset;
  }

  // calculate checksum and compare to what 
  // we read from the device 
  if (checksum(packet) != packet[CHECKSUM_BYTE])
  {
    printf("Failed to valididate response checksum, calculated %d, actuall %d\n", checksum(packet), packet[CHECKSUM_BYTE]);
    goto reset;
  }
  
  return true;

reset:
  //give some time for the printf to drain before resetting
  puts("");
  printPacket(packet, "FAILED PACKET:   ");
  printf("...resetting fingerprint device...\n");
  reset_device();
  return false;
}

static char add_user(commandQ_parallax_t * cmd)
{
  decorate_function("starting to add a user");

  BaseType_t xStatus;
  int ret;
  int i;
  int io_drain;

  // Drain Q, should be empty anyway..
  do{
    xStatus = xQueueReceive(gpio_evt_queue, &io_drain, 0);
  }while(xStatus == pdPASS);

  gpio_intr_enable(GPIO_INPUT_IO_1);

  for(i = 0; i < 3; i++)
  {
    xStatus = xQueueReceive(gpio_evt_queue, &io_drain, pdMS_TO_TICKS(5000));
    if(xStatus != pdTRUE)
    {
      puts("timed out waiting for proximity ISR!");
      return TIMED_OUT_GETTING_PRINT;
    }
    
    // Debounce switch, re-enable interrupt (was dissabled in ISR)
    vTaskDelay(250 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_IO_1);

    ret = add_user_state_machine(cmd->id, i);
    if (ret != ACK_SUCCESS)
    {
      gpio_intr_disable(GPIO_INPUT_IO_1);
      return ret;
    }
  }

  gpio_intr_disable(GPIO_INPUT_IO_1);
  return ADDED_USER;
}

// This function breaks down the 3 commands we need
// to send to the device into 3 distinct commads,
// to pick a command, pass the state to it,
// so to add user ID 0, make the following calls
//   add_user_state_machine(0x0,0);
//   add_user_state_machine(0x0,1);
//   add_user_state_machine(0x0,2);
static int add_user_state_machine(uint16_t userId, int state)
{
  if ( state < 0 || state > 3)
  {
    return 1;
  }

  char userIdLow, userIdHigh;
  userIdLow = 0xFF & userId;
  userIdHigh = (userId >> 8) & 0xFF;

  //we need to send the following combination of commands to add a user
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_1,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_2,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_3,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // Where CMD_ADD_FINGERPRINT_(1/2/3) are equal to 0x1,0x2,0x3.
  // We also need a small wait in between sending the commands

  char command[8];
  char rx_buff[8];
  memset(command, 0, sizeof(command));
  memset(rx_buff, 0, sizeof(rx_buff));

  command[0] = CMD_GUARD;
  command[1] = (state + 1); //CMD_ADD_FINGERPRINT_1 starts at 1, state starts at 0
  command[2] = userIdHigh;
  command[3] = userIdLow;
  command[4] = USR_PRIV;
  command[5] = 0;
  command[6] = checksum(command);
  command[7] = CMD_GUARD;

  printPacket(command, "TX:  ");

  // we sleep for about a second between writes, if we can not
  // write the entire message everytime, this means that the buffer inside
  // the uart-core is not getting drained 
  // we will simply restart if we every get to this positon - recovering is too hard
  int txBytes = uart_write_bytes(UART_NUM_1, command, CMD_LEN); 
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  //read the response and check the status 
  int rxBytes = uart_read_bytes(UART_NUM_1, (uint8_t*)rx_buff, CMD_LEN, 2500 / portTICK_PERIOD_MS); // For some reason the write API use char
                                                                            // and the read api uses uint8_t... fix it here    
                                                                                                  
  // Validate both len and checksum
  if (!validateResponse(rxBytes, rx_buff))
  {
    return FAILED_ADD_FINGERPRINT;
  }
  
  printPacket(rx_buff, "RX:  ");
  // Validate we got an ACK back, anything else, reset
  char response = getResponse(rx_buff);
  if(response != ACK_SUCCESS)
  {
    puts("resetting...");
    reset_device();
  }

  printf("Response == %d \n", response); 
  return (getResponse(rx_buff));
}


// Delete _ALL_ users
static int delete_all_users()
{
  decorate_function("Starting to delete all users");

  char command[8];
  char rx_buff[8];
  memset(command, 0, sizeof(command));
  memset(rx_buff, 0, sizeof(rx_buff));

  command[0] = CMD_GUARD;
  command[1] = CMD_DELETE_ALL_USERS;
  command[2] = 0;
  command[3] = 0;
  command[4] = 0;
  command[5] = 0;
  command[6] = checksum(command);
  command[7] = CMD_GUARD;

  printPacket(command, "TX:  ");

  int txBytes = uart_write_bytes(UART_NUM_1, command, CMD_LEN); 

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // We know that the chechsum is correct (since we computed it locally)
  // We will just use this function for checking we sent out all our command
  validateResponse(txBytes, command);

  //read the response and check the status 
  int rxBytes = uart_read_bytes(UART_NUM_1, (uint8_t*)rx_buff, CMD_LEN, 1000 / portTICK_PERIOD_MS); // For some reason the write API use char
                                                                                                    // and the read api uses uint8_t... fix it here    
                                                                            
  // Validate both len and checksum
  validateResponse(rxBytes, rx_buff);
  printPacket(rx_buff, "RX:  ");

  if (getResponse(rx_buff) != ACK_SUCCESS)
  {
    char response = getResponse(rx_buff);
    printf("Response == %d\n", response);
  }

  decorate_function("done deleting all users");
  
  return getResponse(rx_buff);
}

// compare 1:1 (check who is trying to identifiy) 
uint16_t match_finger_print_to_id()
{
  decorate_function("Starting 1:N compare");

  char command[8];
  char rx_buff[8];
  memset(command, 0, sizeof(command));
  memset(rx_buff, 0, sizeof(rx_buff));

  command[0] = CMD_GUARD;
  command[1] = CMD_SCAN_COMPARE_1_TO_N;
  command[2] = 0;
  command[3] = 0;
  command[4] = 0;
  command[5] = 0;
  command[6] = checksum(command);
  command[7] = CMD_GUARD;

  printPacket(command, "TX:  ");

  int txBytes = uart_write_bytes(UART_NUM_1, command, CMD_LEN); 

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // We know that the chechsum is correct (since we computed it locally)
  // We will just use this function for checking we sent out all our command
  validateResponse(txBytes, command);

  //read the response and check the status 
  int rxBytes = uart_read_bytes(UART_NUM_1, (uint8_t*)rx_buff, CMD_LEN, 1000 / portTICK_PERIOD_MS); // For some reason the write API use char
                                                                                                    // and the read api uses uint8_t... fix it here    
                                                                            
  // Validate both len and checksum
  validateResponse(rxBytes, rx_buff);
  printPacket(rx_buff, "RX:  ");
  printf("\nuserdId = %d\n", rx_buff[2] << 8 |   rx_buff[3]);
  
  /*
  if (getResponse(rx_buff) != ACK_SUCCESS)
  {
    char response = getResponse(rx_buff);
    printf("Response == %d\n", response);
  }
*/
  decorate_function("done 1:N compare");
  
  return getResponse(rx_buff);
}

// Fetch how many users we have
uint16_t fetchNumberOfUsers()
{
  return 0;;
}

void reset_device()
{
   parallax_reset(PARALLAX_PUT_IN_REST);
	 vTaskDelay(pdMS_TO_TICKS(1000));
   parallax_reset(PARALLAX_TAKE_OUT_RESET);
   vTaskDelay(pdMS_TO_TICKS(1000)); //give time for the device to boot
   // Yikes.. fingerprint module seems to send out a dummy byte on reset (power slurp being down TX?)
   // Drain RX buf...
   char byteBlackHole[BLACK_HOLE_BYTES];
   uart_read_bytes(UART_NUM_1, (uint8_t*)byteBlackHole, BLACK_HOLE_BYTES, 0);  
}

static void init_gpio()
{
  // ********************* SET UP RESET PIN ********************** 

  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(gpio_config_t));

  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = RESET_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
  //pull device out of reset 
  gpio_set_level(RESET_PIN, 1);

  // ******************* SET UP PROXIMITY GPIO *******************
 
  memset(&io_conf, 0, sizeof(gpio_config_t));
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

  // ********************** SETUP ISR *** ***********************

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
  //start with the interrupt dissabled
  //gpio_intr_disable(GPIO_INPUT_IO_1);

  // ********************* SET UP UART ***************************

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

int parallax_thread_gate(commandQ_parallax_t * cmd)
{
  int ret;
  xSemaphoreTake(parallaxCommandMutex, portMAX_DELAY );  
  xQueueSend(parallaxCommandQ, cmd, 0);                     // Send the command
  xQueueReceive(parallaxCommandQ_res, &ret, portMAX_DELAY); // Wait for the response
  xSemaphoreGive(parallaxCommandMutex);
  return ret;
}

void parallax_thread(const void * ptr)
{
  int ret;
  commandQ_parallax_t commandQ_cmd; 
  parallaxCommandQ     = xQueueCreate(1 , sizeof(commandQ_parallax_t));
  parallaxCommandQ_res = xQueueCreate(1 , sizeof(int32_t));
  parallaxCommandMutex =  xSemaphoreCreateMutex();
  gpio_evt_queue = xQueueCreate(3, sizeof(uint32_t));
  init_gpio();
  
  //let the rest of the system know we parallax-core is ready
  parallaxCoreReady = 1;

  for(;;)
  {   
    // Wait for command
    BaseType_t xStatus = xQueueReceive(parallaxCommandQ, &commandQ_cmd,portMAX_DELAY);
    if(xStatus == NULL){
      ESP_LOGE(TAG, "Failed to create Queue..");
    }

    switch (commandQ_cmd.command)
    {
      case PARALLAX_ADD_USER:
        ret = add_user(&commandQ_cmd);
        xQueueSend(parallaxCommandQ_res, &ret, 0);
        break;
      case PARALLAX_CMP_USER:
        ret = match_finger_print_to_id();
        xQueueSend(parallaxCommandQ_res, &ret, 0);
        break;
      case PARALLAX_DLT_ALL:
        ret = delete_all_users();
        xQueueSend(parallaxCommandQ_res, &ret, 0);
        break;
    }
  }
}
