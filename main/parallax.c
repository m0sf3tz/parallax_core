#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "parallax.h"

#define DEBUG_MESSAGE

static char rx_buff[RX_BUF_LEN];

static void decorateFunctions(const char * str)
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

static void printPacket(char * command, const char * start_message)
{
  int x;
  
  printf("%s", start_message);
  for (x = 0; x < 8; x++){
    printf("0x%02x ",command[x]);
  }
  printf("\n");
}

// Sanitize response from device, verify checksum + len
// Fail hard in error condition
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
  printPacket(packet, "FAILED PACKET:   ");
  printf("...resetting fingerprint device...\n");
  resetDevice();
  return false;
}


char addUser(uint16_t userId)
{
  decorateFunctions("starting to add a user");

  int i;
  char userIdLow, userIdHigh;
  userIdLow = 0xFF & userId;
  userIdHigh = (userId >> 8) & 0xFF;

  //we need to send the following combination of commands to add a user
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_1,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_2,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // [ CMD_GUARD ,CMD_ADD_FINGERPRINT_3,USERID_HIGH, USERID_LOW, USER_PRIV, 0, CHECKSUM, CMD_GUARD ]
  // Where CMD_ADD_FINGERPRINT_(1/2/3) are equal to 0x1,0x2,0x3.
  // We also need a small wait in between sending the commands


  for(i = 0; i < 3; i++)
  {
    char command[8];
    char rx_buff[8];
    memset(command, 0, sizeof(command));
    memset(rx_buff, 0, sizeof(rx_buff));

    command[0] = CMD_GUARD;
    command[1] = (i + 1); //CMD_ADD_FINGERPRINT_1 starts at 1, I starts at 1
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
    int rxBytes = uart_read_bytes(UART_NUM_1, (uint8_t*)rx_buff, CMD_LEN, 1000 / portTICK_PERIOD_MS); // For some reason the write API use char
                                                                              // and the read api uses uint8_t... fix it here    
                                                                              
    // Validate both len and checksum
    if (!validateResponse(rxBytes, rx_buff))
    {
      return false;
    }
    
    printPacket(rx_buff, "RX:  ");

    if (getResponse(rx_buff) != ACK_SUCCESS)
    {
      char response = getResponse(rx_buff);
      printf("Response == %d \n", response);
    }
  }
  decorateFunctions("Done adding user");
  
  // final call holds pass/fail information about adding fingerprint
  return getResponse(rx_buff);
}

// Delete _ALL_ users
char deleteAllUsers()
{
  decorateFunctions("Starting to delete all users");

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

  decorateFunctions("done deleting all users");
  
  return getResponse(rx_buff);
}

// compare 1:1 (check who is trying to identifiy) 
uint16_t matchFingerPrintToId()
{
  decorateFunctions("Starting 1:N compare");

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
  decorateFunctions("done 1:N compare");
  
  return getResponse(rx_buff);

}

// Fetch how many users we have
uint16_t fetchNumberOfUsers()
{
  return 0;;
}

void resetDevice()
{
	 gpio_set_level(22, 0 ) ;
	 vTaskDelay(pdMS_TO_TICKS(3000));
   gpio_set_level(22, 1 ) ;	
   vTaskDelay(pdMS_TO_TICKS(1000)); //give time for the device to boot
   // Yikes.. fingerprint module seems to send out a dummy byte on reset (power slurp being down TX?)
   // Drain RX buf...
   char byteBlackHole[BLACK_HOLE_BYTES];
   uart_read_bytes(UART_NUM_1, (uint8_t*)byteBlackHole, BLACK_HOLE_BYTES, 0);  
}
