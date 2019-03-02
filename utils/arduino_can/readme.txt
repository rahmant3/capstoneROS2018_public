Overview:
This folder contains the following software:
- CAN sniffing tool, that reads data from the CAN bus and saves it to an SD card.
- CAN playback tool, that reads data from a file on the SD card and writes the CAN messages to the CAN bus.

Hardware:
The following hardware is required:
- Arduino Uno
- Sparkfun CAN bus shield (for the Arduino Uno). See: https://www.sparkfun.com/products/13262

Software:
The following libraries are required:
- Sparkfun provided libraries: https://github.com/sparkfun/SparkFun_CAN-Bus_Arduino_Library
- CAN library for MCP2515 by Cory J. Fowler: https://github.com/coryjfowler/MCP_CAN_lib

CAN sniffing tool:
The CAN messages are printed to the screen and SD card in a CSV format, readable by CANView software (see: http://www.yachtd.com/products/can_view.html).

The status LEDs on the CAN shield behave as follows:
- If LED D7 is blinking, then there was an issue with initializing the system. For example, failing to insert an SD card before power-on will cause an initialization error.
- If LED D7 is a solid color, then the system was initialized successfully. 
- LED D8 is turned ON during each CAN rx.

CAN playback tool:
The CAN messages must be on the SD card in the file "CAN_LOG.TXT", with messages in a CSV format readable by CANView software (see: http://www.yachtd.com/products/can_view.html).

CAN Messages are sent immediately from the first valid entry, and then with appropriate delays in between (based on the timestamp).

The status LEDS on the CAN shield behave as follows:
- If LED D7 is blinking, then there was an issue with initializing the system. For example, failing to insert an SD card before power-on will cause an initialization error.
- If LED D7 is a solid color, then the system was initialized successfully. 
- LED D8 is turned ON during each CAN tx.

