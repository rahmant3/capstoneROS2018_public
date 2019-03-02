// Description:
//  CAN sniffing tool, to be used with Arduino Uno and the Sparkfun CAN Shield board. The CAN 
//  messages are printed to the screen and SD card in a CSV format, readable by CANView software
//  (see: http://www.yachtd.com/products/can_view.html).
//
//  On the CAN shield, if LED D7 is blinking, then there was an issue with initializing the
//  system. Failing to insert an SD card will cause an initialization error.
//
//  If LED D7 is a solid color, then the system was initialized successfully. 
//
//  LED D8 is turned ON during each CAN rx.
//
//  Depends on the following libraries:
//      - Sparkfun provided libraries: https://github.com/sparkfun/SparkFun_CAN-Bus_Arduino_Library
//      - CAN library for MCP2515 by Cory J. Fowler: https://github.com/coryjfowler/MCP_CAN_lib
//
// History:
//  2018-12-06 by Tamkin Rahman
//  - Created.
//  2018-12-11 by Tamkin Rahman
//  - Added LED feedback. 
//  - Remove joystick control.
//  - Remove "set_millis" function. On startup, simply start with the default (zero).

// -----------------------------------------------------------------------------------------------
//  INCLUDES
// -----------------------------------------------------------------------------------------------
#include <mcp_can.h>
#include <SD.h>
#include <SPI.h>

// -----------------------------------------------------------------------------------------------
//  DEFINES
// -----------------------------------------------------------------------------------------------
#define DEBUG // Leave defined for debug messages on serial output.

#define INIT_SUCCESS_LED  7
#define CAN_RX_LED        8

#define CAN0_INT      2   // CAN Interrupt pin
#define SD_CS         9   // SD Card chip select pin
#define CAN0_CS       10  // CAN chip select pin

#define MAX_MSG_SIZE  80

// -----------------------------------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------------------------------
const char * CAN_LOG_FILE = "can_log.txt";
const char * HEADER = "Time,CAN,Dir,Bit,ID(hex),DLC,D0,D1,D2,D3,D4,D5,D6,D7";

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[MAX_MSG_SIZE];                        // Array to store serial string

MCP_CAN CAN0(CAN0_CS);

// -----------------------------------------------------------------------------------------------
//  PROTOTYPES
// -----------------------------------------------------------------------------------------------

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
// Writes the given data to the file.
//
// Returns:
//  True on success, False on failure.
// -----------------------------------------------------------------------------------------------
bool write_to_file(
    const char* filename, // The file to write to.
    const char* data      // The data to write.
);

// -----------------------------------------------------------------------------------------------
//  FUNCTIONS
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
  pinMode(CAN_RX_LED, OUTPUT);

  digitalWrite(INIT_SUCCESS_LED, LOW);
  digitalWrite(CAN_RX_LED, LOW);
  
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
    if(!write_to_file(CAN_LOG_FILE, HEADER))
    {
      init_success = false;
      Serial.println("Failed to write header to log file.");
    }
    else
    {
      #ifdef DEBUG
        Serial.println(HEADER);
      #endif
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

    // Configuring pin for /INT input
    pinMode(CAN0_INT, INPUT);                            
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

  if (SD.begin(SD_CS)) {
    rc = true;
  }
  
  return rc;
}

// -----------------------------------------------------------------------------------------------
bool write_to_file(const char* filename, const char* data)
{
  bool rc = false;
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile)
  {
    dataFile.println(data);
    rc = true;
    dataFile.close();   //close file
  }

  return rc;
}

// -----------------------------------------------------------------------------------------------
bool get_can_string(char output[MAX_MSG_SIZE])
{
  bool rc = false;
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    // Calculate timestamp:
    unsigned long now   = millis();
    unsigned long now_s = now / 1000;
    int milliseconds    = now % 1000;
    int seconds         = now_s % 60;
    int minutes         = (now_s/60) % 60;
    int hours           = (now_s/3600) % 24;

    int id_len = 11;
    int id = 0;
    // Read the message:
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if (len > 0)
    {
      if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      {
        id_len = 29;
        id = rxId & 0x1FFFFFFF;
      }
      else
      {
        id_len = 11;
        id = rxId & 0x7FF;
      }
  
      // CSV file headers are: Time (in H:M:S),CAN,Dir,Bit,ID(hex),DLC,D0,D1,D2,D3,D4,D5,D6,D7
      sprintf(output, "%d:%02d:%02d.%03d,0,RX,%d,%X,%d", hours, minutes, seconds, milliseconds, id_len, id, len);

      for (int ix = 0; ix < len; ix++)
      {
        sprintf(output, "%s,%X", output, rxBuf[ix]);
      }
   
      rc = true;
    }
  }

  return rc;
}

// -----------------------------------------------------------------------------------------------
void loop()
{     
  if (get_can_string(msgString))
  {
    digitalWrite(CAN_RX_LED, HIGH);
    if (!write_to_file(CAN_LOG_FILE, msgString))
    {
      #ifdef DEBUG
        Serial.println("Failed to write to SD card!");
      #endif
    }
    #ifdef DEBUG
      Serial.println(msgString);
    #endif
    digitalWrite(CAN_RX_LED, LOW);
  }
  
}
