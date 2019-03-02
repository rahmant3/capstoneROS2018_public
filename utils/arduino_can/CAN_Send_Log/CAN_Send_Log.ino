// Description:
//  CAN playback tool, which takes an input CAN log and sends the messages on the CAN bus. To be 
//  used with Arduino Uno and the Sparkfun CAN Shield board. The output CAN messages are printed
//  to the screen in a CSV format, readable by CANView software 
//  (see: http://www.yachtd.com/products/can_view.html).
//
//   The input CAN log must be named "CAN_LOG.TXT", with entries conforming to the CSV format:
//    "Time,CAN,Dir,Bit,ID(hex),DLC,D0,D1,D2,D3,D4,D5,D6,D7"
//    e.g. "0:00:34.621,0,RX,11,310,8,0,0,4,81,99,99,0,FF"
//
//  CAN messages are sent immediately from the first valid CAN entry, and then with appropriate
//  delays in between. 
//
//  On the CAN shield, if LED D7 is blinking, then there was an issue with initializing the
//  system. Failing to insert an SD card will cause an initialization error.
//
//  If LED D7 is a solid color, then the system was initialized successfully. 
//
//  LED D8 is turned ON during each CAN tx.
//
//  Depends on the following libraries:
//    - Sparkfun provided libraries: https://github.com/sparkfun/SparkFun_CAN-Bus_Arduino_Library
//    - CAN library for MCP2515 by Cory J. Fowler: https://github.com/coryjfowler/MCP_CAN_lib
//
//  Note, the memory requirements are very tight. Changes to this software that use
//  additional memory may cause the SD card to fail to initialize or other undefined 
//  behavior, so take care when making changes to this file.
//
// History:
//  2018-12-24 by Tamkin Rahman
//  - Created.

// -----------------------------------------------------------------------------------------------
//  INCLUDES
// -----------------------------------------------------------------------------------------------
#include <mcp_can.h>
#include <SD.h>
#include <SPI.h>

// -----------------------------------------------------------------------------------------------
//  DEFINES
// -----------------------------------------------------------------------------------------------
#define MAX_TOKENS 14
#define MAX_TOKENS_SIZE 12

#define MIN_CANLOG_TOKENS 6
#define MAX_CANLOG_TOKENS 14

#define TIMESTAMP_TOKENS_LEN 3

#define CAN_LOG_TIMESTAMP_IX 0
#define CAN_LOG_ID_LEN_IX    3
#define CAN_LOG_ID_IX        4
#define CAN_LOG_DATA_LEN_IX  5
#define CAN_LOG_DATA_START   6

#define EXTENDED_ID_LEN   29

#define INIT_SUCCESS_LED  7
#define CAN_TX_LED        8

#define CAN0_INT      2   // CAN Interrupt pin
#define SD_CS         9   // SD Card chip select pin
#define CAN0_CS       10  // CAN chip select pin

// -----------------------------------------------------------------------------------------------
//  STRUCTS AND STRUCT TYPEDEFS
// -----------------------------------------------------------------------------------------------
typedef struct {
  unsigned long timestamp;
  unsigned int id;
  bool extended;
  int datalen;
  byte data[8];
} CANLogMessage;

// -----------------------------------------------------------------------------------------------
//  CONSTANTS
// -----------------------------------------------------------------------------------------------
const char * CAN_LOG_FILE = "CAN_LOG.TXT";

// -----------------------------------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------------------------------
MCP_CAN CAN0(CAN0_CS);

char line[80];      // Buffer for reading a line off the SD card.
CANLogMessage msg;  // CANLogMessage object containing the current CAN msg to send.

// -----------------------------------------------------------------------------------------------
//  PROTOTYPES
// -----------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------
// Splits a given input string based on a given delimiter.
//
// Returns:
//  The number of strings in the output array.
// -----------------------------------------------------------------------------------------------
int string_split(
  const char * input,                      // The input string to split.
  const char * delim,                      // The delimiter to split on (e.g. ",")
  char output[MAX_TOKENS][MAX_TOKENS_SIZE] // The array of strings containing the output.
);

// -----------------------------------------------------------------------------------------------
// Converts the given string entry into a CAN message.
//
// Returns:
//  True on success, False on failure.
// -----------------------------------------------------------------------------------------------
bool entry_to_CAN_message(
  const char * line,       // The input string containing the CAN message.
  CANLogMessage * message  // The CANLogMessage object containing the output.
);

// -----------------------------------------------------------------------------------------------
// Initialize the CAN configuration.
//
// Returns:
//  True on success, False on failure.
// -----------------------------------------------------------------------------------------------
bool can_init();

// -----------------------------------------------------------------------------------------------
// Initialize the SD card configuration.
//
// Returns:
//  True on success, False on failure.
// -----------------------------------------------------------------------------------------------
bool sd_init();

// -----------------------------------------------------------------------------------------------
//  FUNCTIONS
// -----------------------------------------------------------------------------------------------
int string_split(const char* input, const char * delim, char output[MAX_TOKENS][MAX_TOKENS_SIZE])
{
  char line[80];
  char * token;
  
  int ix = 0;
  
  strcpy(line, input);
  token = strtok(line, delim);
  
  while ((token != NULL) && (ix < MAX_TOKENS))
  {
    strcpy(output[ix], token);
    token = strtok(NULL, delim);

    ix++;
  }

  return ix;
}

// -----------------------------------------------------------------------------------------------
bool entry_to_CAN_message(const char * line, CANLogMessage * message)
{
  int timestamp_len;
  unsigned long timestamp_hrs;
  unsigned long timestamp_min;
  double timestamp_s;
  
  bool rc = false;
  char csv_split_output[MAX_TOKENS][MAX_TOKENS_SIZE];
  char timestamp_split[MAX_TOKENS][MAX_TOKENS_SIZE];

  int len = string_split(line, ",", csv_split_output);

  if ((len >= MIN_CANLOG_TOKENS) && (len <= MAX_CANLOG_TOKENS))
  {
    timestamp_len = string_split(csv_split_output[CAN_LOG_TIMESTAMP_IX], ":", timestamp_split);

    if (timestamp_len == TIMESTAMP_TOKENS_LEN)
    {   
      timestamp_hrs = strtol(timestamp_split[0], NULL, 10);
      timestamp_min = strtol(timestamp_split[1], NULL, 10);
      timestamp_s = atof(timestamp_split[2]);

      message->timestamp = timestamp_hrs * 3600 * 1000 + timestamp_min * 60 * 1000 + (unsigned long)(timestamp_s * 1000);
      
      message->id = strtol(csv_split_output[CAN_LOG_ID_IX], NULL, 16);  // Note that the ID is in HEX.
      message->extended = (strtol(csv_split_output[CAN_LOG_ID_LEN_IX], NULL, 10) == EXTENDED_ID_LEN);
      message->datalen = strtol(csv_split_output[CAN_LOG_DATA_LEN_IX], NULL, 10);
  
      for (int ix = 0; ix < message->datalen; ix++)
      {
        message->data[ix] = strtol(csv_split_output[CAN_LOG_DATA_START + ix], NULL, 16);
      }

      rc = true;
    }
  }

  return rc;
}

// -----------------------------------------------------------------------------------------------
void setup()
{
  bool init_success = false;
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // Initialize status LEDs
  pinMode(INIT_SUCCESS_LED, OUTPUT);
  pinMode(CAN_TX_LED, OUTPUT);

  digitalWrite(INIT_SUCCESS_LED, LOW);
  digitalWrite(CAN_TX_LED, LOW);
  
  if(can_init())
  {
    Serial.println("MCP2515 initialized successfully!");

    if(sd_init())
    {
      Serial.println("SD card initialized successfully!");
      init_success = true;
    }
    else
    {
      Serial.println("Error initializing SD card...");
    }
  }
  else
  {
    Serial.println("Error initializing MCP2515...");
  }

  if (init_success)
  {      
    if (!SD.exists(CAN_LOG_FILE))
    {
      init_success = false;
      Serial.println("Failed to find the input CAN log file...");
    }
  }
  
  if(!init_success)
  {
    Serial.println("Failed to initialize the system.");
    while(1) // Loop forever.
    {
      digitalWrite(INIT_SUCCESS_LED, HIGH);
      delay(1000);
      digitalWrite(INIT_SUCCESS_LED, LOW);
      delay(1000);
    }
  }
  else
  {
    digitalWrite(INIT_SUCCESS_LED, HIGH);
    #ifdef DEBUG
      Serial.println("Starting system.");
    #endif
  }
  
}

// -----------------------------------------------------------------------------------------------
bool can_init()
{
  bool rc = false;
  // Initialize MCP2515 running at 16 MHz with a baudrate of 250kb/s and the masks and filters disabled.
  //if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK)
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
  {
    rc = true;
    
    CAN0.setMode(MCP_NORMAL);                     
  }
  else
  {
    rc = false;
  }

  return rc;
}

// -----------------------------------------------------------------------------------------------
bool sd_init()
{
  bool rc = false;
  pinMode(SD_CS, OUTPUT);

  if (SD.begin(SD_CS)) 
  {
    rc = true;
  }
  
  return rc;
}

// -----------------------------------------------------------------------------------------------
int read_line(File dataFile, char * output, int max_size)
{
  int len = 0;
  char in = '\0';
  
  while (dataFile.available() && (len < (max_size - 1)) && (in != '\n'))
  {
    in = dataFile.read();

    if (in != '\n')
    {
      output[len] = in;
      len++;
    }
  }

  output[len] = '\0';

  return len;
}

// -----------------------------------------------------------------------------------------------
void loop()
{
  bool started = false;
  bool finished = false;
  unsigned long offset = 0;
  unsigned long current_time = 0;
  unsigned long delta = 0;
  
  File dataFile;
  
  dataFile = SD.open(CAN_LOG_FILE, FILE_READ);
  if (!dataFile)
  {
    finished = true;
    Serial.println("Error, failed to open CAN log file.");
  }
  
  while (!finished)
  {
    if (read_line(dataFile, line, 80) > 0)
    {
      Serial.println(line);
      
      if (entry_to_CAN_message(line, &msg))
      {
        if (!started)
        {
          offset = msg.timestamp;
          started = true;
        }

        current_time = millis() + offset;
        delta = 0;
        if (current_time < msg.timestamp)
        {
          delta = msg.timestamp - current_time;
        }
        if (delta > 0)
        {
          #ifdef DEBUG
            Serial.print("Delaying for: ");
            Serial.print(delta);
            Serial.println(" ms");
          #endif
          delay(delta);
        }
        
        digitalWrite(CAN_TX_LED, HIGH);
        byte sndStat = CAN0.sendMsgBuf(msg.id, msg.extended, msg.datalen, msg.data);
        if(sndStat != CAN_OK)
        {
          Serial.println("Error Sending Message...");
        }
        digitalWrite(CAN_TX_LED, LOW);
      }
    }
    else
    {
      finished = true;
      dataFile.close();
    }
  }

  while(1);
}
