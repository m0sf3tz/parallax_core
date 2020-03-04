#define RESPONSE_BYTE                     0x04 
#define CHECKSUM_BYTE                     0x06
#define USR_PRIV                          0x01 // Every user has the same privilage of 1
#define CMD_GUARD                         0xF5 // Start and end of a command
#define CMD_LEN                           0x08 
#define RESPONSE_LEN                      (CMD_LEN)


#define ACK_SUCCESS                       0x00 // Operation successfully
#define ACK_FAIL                          0x01 // Operation failed
#define ACK_FULL                          0x04 // Fingerprint database is full
#define ACK_NOUSER                        0x05 // No such user
#define ACK_USER_EXISTS                   0x07 // Already exists
#define ACK_TIMEOUT                       0x08 // Acquisition timeout


#define CMD_SLEEP                         0x2C // Puts the device to sleep
#define CMD_SET_MODE                      0x2D
#define CMD_ADD_FINGERPRINT_1             0x01
#define CMD_ADD_FINGERPRINT_2             0x02
#define CMD_ADD_FINGERPRINT_3             0x03
#define CMD_DELETE_USER                   0x04
#define CMD_DELETE_ALL_USERS              0x05
#define CMD_GET_USERS_COUNT               0x09
#define CMD_SCAN_COMPARE_1_TO_1           0x0B
#define CMD_SCAN_COMPARE_1_TO_N           0x0C
#define CMD_READ_USER_PRIVLAGE            0x0A
#define CMD_SENSITIVITY                   0x28
#define CMD_SCAN_GET_IMAGE                0x24
#define CMD_SCAN_GET_EIGENVALS            0x23
#define CMD_SCAN_PUT_EIGENVALS            0x44
#define CMD_PUT_EIGENVALS_COMPARE_1_TO_1  0x42
#define CMD_PUT_EIGENVALS_COMPARE_1_TO_N  0x43
#define CMD_GET_USER_EIGENVALS            0x31
#define CMD_PUT_USER_EIGENVALS            0x41
#define CMD_GET_USERS_INFO                0x2B
#define CMD_SET_SCAN_TIMEOUT              0x2E // Set the timeout, multiples of ~.25 seconds

#define RX_BUF_LEN (8)

char addUser(uint16_t userID);
char deleteAllUsers();
uint16_t fetchNumberOfUsers();
