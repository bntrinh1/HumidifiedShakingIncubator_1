#include <Wire.h>
#include <stdarg.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <Arduino.h>
#include <DHT.h>

#define CFA_634_I2C_ADDRESS  (42)

#define USE_PRINTF //Saves ~1600 bytes




//============================================================================
// ref http://playground.arduino.cc/Main/Printf
void SerPrintFF(const __FlashStringHelper *fmt, ... )
  {
  char
    tmp[128]; // resulting string limited to 128 chars
  va_list
    args;
  va_start(args, fmt );
  vsnprintf_P(tmp, 128, (const char *)fmt, args);
  va_end (args);
  Serial.print(tmp);
  }
//----------------------------------------------------------------------------
// ref http://scott.dd.com.au/wiki/Arduino_Static_Strings
void SerialPrint_P(const char flash_string[])
  {
  uint8_t
    c;
  for(;0x00 != (c = pgm_read_byte(flash_string)); flash_string++)
    {
    Serial.write(c);
    }
  }
//============================================================================
int freeRam(void)
  {
  extern int
    __heap_start;
  extern int
    *__brkval;
  int
    v;
  return((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
  }
//============================================================================
// Waits until a switch is closed before booting (debug)
#define led_pin 3
#define switch_pin 2
//----------------------------------------------------------------------------

//============================================================================
typedef struct
  {
  uint8_t   command;
  uint8_t   length;
  uint8_t   data[24];
  uint16_t  crc;
  } CFPacket_t;
//----------------------------------------------------------------------------
class CrystalfontzI2CPacketLCD
  {
  public:
    //vars
    //functions
    CrystalfontzI2CPacketLCD(uint8_t address);
    uint8_t sendPacket_getReply(
              CFPacket_t *packet_sent,
              CFPacket_t *packet_received,
              uint8_t print_errors);
    void dumpPacket(CFPacket_t *packet_to_send);
    uint8_t Search_I2C_Adresses(void);
    void Set_I2C_Adress(uint8_t new_address);
    void Change_Module_Address(uint8_t new_address);
    void writeText(uint8_t x, uint8_t y, const char *text, uint8_t length);
    void writeFlashText(uint8_t x, uint8_t y, const __FlashStringHelper *flash_string, uint8_t length);
    void clearScreen(void);
    uint8_t getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases);
    void setUpBar(uint8_t spec_char,uint8_t vert_mask);
    void drawBar(uint8_t col,uint8_t row,uint8_t chars_wide,
                 uint8_t px_length,uint8_t spec_char_solid,
                 uint8_t spec_char_variable,uint8_t vert_mask);
  private:
    //vars
    uint8_t  i2c_address;
    //functions
    uint16_t CRC(uint8_t *ptr, uint16_t len);
  };
//----------------------------------------------------------------------------
CrystalfontzI2CPacketLCD::CrystalfontzI2CPacketLCD(uint8_t address)
  {
  i2c_address=address;
  }
//----------------------------------------------------------------------------
uint16_t CrystalfontzI2CPacketLCD::CRC(uint8_t *data, uint16_t length)
  {
  //calculate the CRC for the packet data
  uint16_t crc = 0xFFFF;
  while(length--)
    crc = _crc_ccitt_update(crc, *data++);
  return ~crc;
  }
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::sendPacket_getReply(
        CFPacket_t *packet_to_send,
        CFPacket_t *packet_received,
        uint8_t print_errors)
  {
  uint8_t
    bytes_received;
  uint8_t
    i;
  //Valid commands are from 0-35.
  //The maximum received length is known for each packet from the data sheet,
  //so this table will allow us to optimize read performance/minimize I2C
  //traffic by reading only the number of bytes that are significant.
  //0xFF is a magic value for invalid commands
  //Storing data in flash in the Arduino is really obtuse.
  static const uint8_t receive_packet_length[36] PROGMEM= {
    1+1+16+2,  //  0 = Ping Command (variable, 16 is max)
    1+1+16+2,  //  1 = Get Hardware & Firmware Version
    1+1+ 0+2,  //  2 = Write User Flash Area
    1+1+16+2,  //  3 = Read User Flash Area
    1+1+ 0+2,  //  4 = Store Current State As Boot State
    1+1+ 0+2,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
    1+1+ 0+2,  //  6 = Clear LCD Screen
    1+1+ 0+2,  //  7 = Set LCD Contents, Line 1
    1+1+ 0+2,  //  8 = Set LCD Contents, Line 2
    1+1+ 0+2,  //  9 = Set LCD Special Character Data
    1+1+ 9+2,  // 10 = Read 8 Bytes of LCD Memory
    1+1+ 0+2,  // 11 = Set LCD Cursor Position
    1+1+ 0+2,  // 12 = Set LCD Cursor Style
    1+1+ 0+2,  // 13 = Set LCD Contrast
    1+1+ 0+2,  // 14 = Set LCD & Keypad Backlight
    1+1+ 4+2,  // 15 = Read Temperature
    0xFF,      // 16 = (reserved)
    0xFF,      // 17 = (reserved)
    1+1+ 9+2,  // 18 = Read DOW Device Information
    0xFF,      // 19 = (reserved)
    1+1+16+2,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
    1+1+ 7+2,  // 21 = Setup Live Temperature Display (2 or max of 7)
    1+1+ 0+2,  // 22 = Send Command Directly to the LCD Controller
    0xFF,      // 23 = (reserved)
    1+1+ 3+2,  // 24 = Read Keypad, Polled Mode
    0xFF,      // 25 = (reserved)
    0xFF,      // 26 = (reserved)
    0xFF,      // 27 = (reserved)
    1+1+ 0+2,  // 28 = Set ATX Switch Functionality
    1+1+ 0+2,  // 29 = Enable/Feed Host Watchdog Reset
    1+1+15+2,  // 30 = Read Reporting/ATX/Watchdog (debug)
    1+1+ 0+2,  // 31 = Send data to LCD
    1+1+ 1+2,  // 33 = Set I2C slave address
    1+1+ 0+2,  // 34 = Set/Configure GPIO
    1+1+ 4+2}; // 35 = Read GPIO & Configuration

  //Table of times to delay in order to assure that the reply is valid.
  //These cheat/optimize a bit from the data sheet, since I have access
  //to the module firmware ;)
  static const uint16_t command_execution_delay[36] PROGMEM = {
       3,  //  0 = Ping Command (variable, 16 is max)
       2,  //  1 = Get Hardware & Firmware Version
      20,  //  2 = Write User Flash Area
       2,  //  3 = Read User Flash Area
      30,  //  4 = Store Current State As Boot State
    1500,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
       2,  //  6 = Clear LCD Screen
       3,  //  7 = Set LCD Contents, Line 1
       3,  //  8 = Set LCD Contents, Line 2
       2,  //  9 = Set LCD Special Character Data
       2,  // 10 = Read 8 Bytes of LCD Memory
       1,  // 11 = Set LCD Cursor Position
       1,  // 12 = Set LCD Cursor Style
       1,  // 13 = Set LCD Contrast
      50,  // 14 = Set LCD & Keypad Backlight
       2,  // 15 = Read Temperature
       0,  // 16 = (reserved)
       0,  // 17 = (reserved)
       2,  // 18 = Read DOW Device Information
       0,  // 19 = (reserved)
      50,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
       3,  // 21 = Setup Live Temperature Display (2 or max of 7)
       2,  // 22 = Send Command Directly to the LCD Controller
       0,  // 23 = (reserved)
       2,  // 24 = Read Keypad, Polled Mode
       0,  // 25 = (reserved)
       0,  // 26 = (reserved)
       0,  // 27 = (reserved)
       2,  // 28 = Set ATX Switch Functionality
       2,  // 29 = Enable/Feed Host Watchdog Reset
       3,  // 30 = Read Reporting/ATX/Watchdog (debug)
       4,  // 31 = Send data to LCD
       2,  // 33 = Set I2C slave address
       2,  // 34 = Set/Configure GPIO
       3}; // 35 = Read GPIO & Configuration

  //Validate the command
  if(35 < packet_to_send->command)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->command out of range %d requested, max is 35\n"),
                 packet_to_send->command);
      }
    return(1);
    }
  if(0xFF == receive_packet_length[packet_to_send->command])
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->command number %d is invalid/reserved\n"),
                 packet_to_send->command);
      }
    return(2);
    }
  //Validate the data length
  if(18 < packet_to_send->length)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: packet_to_send->length out of range %d requested, max is 18\n"),
                 packet_to_send->length);
      }
    return(3);
    }
  //Start the I2C transaction
  Wire.beginTransmission(i2c_address);
  //Send the command byte
  Wire.write(packet_to_send->command);
  //Send the significant data length (will match the I2C since we are in
  //control of it here)
  Wire.write(packet_to_send->length);
  //Send the data[]
  for(i=0;i<packet_to_send->length;i++)
    {
    Wire.write(packet_to_send->data[i]);
    }
  //Calculate the crc
  packet_to_send->crc = CRC((uint8_t*)packet_to_send, packet_to_send->length+2);
  //Low byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[0]))));
  //High byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[1]))));
  //Stop the I2C transaction
  Wire.endTransmission();

  //Now we need to wait for the command to complete, based on the
  //delay table.
  //Even with all the crazy cariable type macros above, this still does not work:
  //delay(command_execution_delay[packet_to_send->command]);
#define EXECUTION_DELAY (pgm_read_word_near(&command_execution_delay[packet_to_send->command]))
  delay(EXECUTION_DELAY);

#define EXPECTED_BYTES (pgm_read_byte_near(&receive_packet_length[packet_to_send->command]))
  //Now it is safe to read the response packet back from the CFA533.
  bytes_received=Wire.requestFrom(i2c_address,EXPECTED_BYTES);

  //If the bytes received does not agree, throw a warning.
  if(bytes_received != EXPECTED_BYTES)
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: Wire.requestFrom fail. Expected %d bytes got %d bytes\n"),
                 EXPECTED_BYTES,
                 bytes_received);
      }
    }

  if(1<=bytes_received)
    {
    //Get the command byte of the respose
    packet_received->command=Wire.read();

    //Verify the low 6 bits of the Rx vs Tx command. They should match.
    if((packet_received->command & 0x3F ) != (packet_to_send->command & 0x3F ))
      {
      if(print_errors)
        {
        SerPrintFF(F("sendPacket_getReply: Received unexpected response of %3d (0x%02X) to command %3d (0x%02X).\n"),
                  packet_received->command&0x3F,packet_received->command&0x3F,
                  packet_to_send->command&0x3F, packet_to_send->command&0x3F);
        }
      }
    //Verify the top 2 bits of the Rx command.
    if((packet_received->command & 0xC0 ) != (0x40))
      {
      if(print_errors)
        {
        SerPrintFF(F("Received unexpected response type of 0x%02X to command %3d (0x%02X). Expect 0x40.\n"),
                  packet_received->command & 0xC0,
                  packet_to_send->command&0x3F,   packet_to_send->command&0x3F);
        }
      }
    }
  else
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: No command byte returned.\n"));
      }
    return(4);
    }

  if(2<=bytes_received)
    {
    //Find out how may bytes of the transfer the CFA533 thinks are significant.
    packet_received->length=Wire.read();

    //Range check the length. There should not be more than 18 (max data) + 2 (crc), and
    //we should have at least response->length still available.
    if(((18 + 2) < packet_received->length)||
       (Wire.available() < packet_received->length))
      {
      if(print_errors)
        {
        SerPrintFF(F("Invalid length of %d in response to command %d. Truncating to %d.\n"),
                   packet_received->length,packet_to_send->command,Wire.available());
        }
      //Attempt to gracefully continue: Override the length
      packet_received->length = Wire.available();
      }
    }
  else
    {
    if(print_errors)
      {
      SerPrintFF(F("sendPacket_getReply: No length byte returned.\n"));
      }
    return(5);
    }

  //Transfer over the data
  for(i=0;i<packet_received->length;i++)
    {
    packet_received->data[i]=Wire.read();
    }

  //Check the CRC of the incoming packet.
  uint16_t
    calculated_crc;
  calculated_crc = CRC((uint8_t*)packet_received, packet_received->length+2);

  //Low byte of CRC
#define LOW_CRC  (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[0]))))
  //High byte of CRC
#define HIGH_CRC (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[1]))))

  uint8_t
    crc_low;
  uint8_t
    crc_high;
  crc_low=Wire.read();
  crc_high=Wire.read();

  if((crc_low != LOW_CRC) || (crc_high != HIGH_CRC))
    {
    if(print_errors)
      {
      SerPrintFF(F("calculated CRC hi:low (0x%02X:%02X) %3d : %3d\n"),
                 HIGH_CRC,LOW_CRC,
                 HIGH_CRC,LOW_CRC);
      SerPrintFF(F("received CRC hi:low (0x%02X:%02X) %3d : %3d\n"),
                 crc_high,crc_low,
                 crc_high,crc_low);
      }
    return(6);
    }
  //All good.
  return(0);
  }
//----------------------------------------------------------------------------
  const char cmd_Str_00[] PROGMEM = " 0 = Ping Command";
  const char cmd_Str_01[] PROGMEM = " 1 = Get Hardware & Firmware Version";
  const char cmd_Str_02[] PROGMEM = " 2 = Write User Flash Area";
  const char cmd_Str_03[] PROGMEM = " 3 = Read User Flash Area";
  const char cmd_Str_04[] PROGMEM = " 4 = Store Current State As Boot State";
  const char cmd_Str_05[] PROGMEM = " 5 = Reboot CFA-533, Reset Host, or Power Off Host";
  const char cmd_Str_06[] PROGMEM = " 6 = Clear LCD Screen";
  const char cmd_Str_07[] PROGMEM = " 7 = Set LCD Contents, Line 1";
  const char cmd_Str_08[] PROGMEM = " 8 = Set LCD Contents, Line 2";
  const char cmd_Str_09[] PROGMEM = " 9 = Set LCD Special Character Data";
  const char cmd_Str_10[] PROGMEM = "10 = Read 8 Bytes of LCD Memory";
  const char cmd_Str_11[] PROGMEM = "11 = Set LCD Cursor Position";
  const char cmd_Str_12[] PROGMEM = "12 = Set LCD Cursor Style";
  const char cmd_Str_13[] PROGMEM = "13 = Set LCD Contrast";
  const char cmd_Str_14[] PROGMEM = "14 = Set LCD & Keypad Backlight";
  const char cmd_Str_15[] PROGMEM = "15 = Read Temperature";
  const char cmd_Str_16[] PROGMEM = "16 = (reserved)";
  const char cmd_Str_17[] PROGMEM = "17 = (reserved)";
  const char cmd_Str_18[] PROGMEM = "18 = Read DOW Device Information";
  const char cmd_Str_19[] PROGMEM = "19 = (reserved)";
  const char cmd_Str_20[] PROGMEM = "20 = Arbitrary DOW Transaction (variable, 16 is max)";
  const char cmd_Str_21[] PROGMEM = "21 = Setup Live Temperature Display (2 or max of 7)";
  const char cmd_Str_22[] PROGMEM = "22 = Send Command Directly to the LCD Controller";
  const char cmd_Str_23[] PROGMEM = "23 = (reserved)";
  const char cmd_Str_24[] PROGMEM = "24 = Read Keypad, Polled Mode";
  const char cmd_Str_25[] PROGMEM = "25 = (reserved)";
  const char cmd_Str_26[] PROGMEM = "26 = (reserved)";
  const char cmd_Str_27[] PROGMEM = "27 = (reserved)";
  const char cmd_Str_28[] PROGMEM = "28 = Set ATX Switch Functionality";
  const char cmd_Str_29[] PROGMEM = "29 = Enable/Feed Host Watchdog Reset";
  const char cmd_Str_30[] PROGMEM = "30 = Read Reporting/ATX/Watchdog (debug)";
  const char cmd_Str_31[] PROGMEM = "31 = Send data to LCD";
  const char cmd_Str_32[] PROGMEM = "32 = (reserved)";
  const char cmd_Str_33[] PROGMEM = "33 = Set I2C slave address";
  const char cmd_Str_34[] PROGMEM = "34 = Set/Configure GPIO";
  const char cmd_Str_35[] PROGMEM = "35 = Read GPIO & Configuration";
  //Should be able to put this array into PROGMEM, but it breaks:
  //    SerialPrint_P(Command_Strings[packet->command]);
  //even thought it works for
  //    SerialPrint_P(Command_Strings[3]);
  //weird as hell.
  //const PROGMEM char * const PROGMEM Command_Strings[36] = {
  const char *  Command_Strings[36] = {
    cmd_Str_00,cmd_Str_01,cmd_Str_02,cmd_Str_03,cmd_Str_04,cmd_Str_05,cmd_Str_06,
    cmd_Str_07,cmd_Str_08,cmd_Str_09,cmd_Str_10,cmd_Str_11,cmd_Str_12,cmd_Str_13,
    cmd_Str_14,cmd_Str_15,cmd_Str_16,cmd_Str_17,cmd_Str_18,cmd_Str_19,cmd_Str_20,
    cmd_Str_21,cmd_Str_22,cmd_Str_23,cmd_Str_24,cmd_Str_25,cmd_Str_26,cmd_Str_27,
    cmd_Str_28,cmd_Str_29,cmd_Str_30,cmd_Str_31,cmd_Str_32,cmd_Str_33,cmd_Str_34,
    cmd_Str_35};

void CrystalfontzI2CPacketLCD::dumpPacket(CFPacket_t *packet)
  {
  SerPrintFF(F("Decode %s%s packet: %2d ( 0x%02x ): \""),
             packet->command&0x80?"ERROR ":"",
             packet->command&0x40?"response":"command",
             packet->command&0x3F,
             packet->command&0x3F);
  SerialPrint_P(Command_Strings[(packet->command)&0x3F]);
  SerPrintFF(F("\"\n"));
  SerPrintFF(F("  Length: %2d ( 0x%02x )"),
             packet->length,
             packet->length);

  //Special decode for some packets
  if(0x01 == (packet->command&0x3F))
    {
    // 1 = Get Hardware & Firmware Version
    SerPrintFF(F(" Data: \""));
    uint8_t
      i;
    for(i=0;i<packet->length;i++)
      {
      SerPrintFF(F("%c"),isprint(packet->data[i])?packet->data[i]:' ');
      }
    SerPrintFF(F("\" "));
    }
  else
    {
    // default byte-by byte dump
    SerPrintFF(F("\n"));
    uint8_t
      i;
    for(i=0;i<packet->length;i++)
      {
      SerPrintFF(F("    Data[%2d]: %3d ( 0x%02x ) \'%c\'\n"),
                 i,
                 packet->data[i],
                 packet->data[i],
                 isprint(packet->data[i])?packet->data[i]:' ');
      }
    }
  SerPrintFF(F("  CRC: 0x%04X\n"),
             packet->crc,
             packet->length);
  }
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::Search_I2C_Adresses(void)
  {
  CFPacket_t
    address_command;
  CFPacket_t
    address_response;
  uint8_t
    original_address;
  uint8_t
    device_found_at_address;

  original_address=i2c_address;
  device_found_at_address=0xFF;

  //Set up the packet
  address_command.command = 1;
  address_command.length = 0;

  for(i2c_address=0;i2c_address<=127;i2c_address++)
    {
    address_response.command = 0xFF;
    address_response.length = 0;
    uint8_t
       response;
    response=sendPacket_getReply(&address_command,&address_response,0);

    if(0x00 == response)
      {
      if(0xFF == device_found_at_address)
        {
        device_found_at_address=i2c_address;
        }
      SerPrintFF(F("Device found at addreess %3d\n"),i2c_address);
      dumpPacket(&address_response);
      }
    }
 
    
  i2c_address=original_address;
  return(device_found_at_address);
  }

//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::Set_I2C_Adress(uint8_t new_address)
  {
  
  i2c_address=new_address;
  
  }

//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::Change_Module_Address(uint8_t new_address)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  SerPrintFF(F("Change module I2C address: from %d to %d\n"),
               i2c_address,new_address);
  //Set up the packet to change the module's address.
  command.command = 33;
  command.length = 1;
  command.data[0] = new_address;
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);

  //Now the module address has changed. Change ours to match.
  Set_I2C_Adress(new_address);
  
  //Do not know why this delay is needed. I suspect there is a timeout
  //in the wire library -- but not sure.
  delay(300);
  
  //Set up the packet to save the module's new address as part of
  //its boot state.
  command.command = 4;
  command.length = 0;
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::writeText(uint8_t x, uint8_t y, const char *text, uint8_t length)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 31;
  command.length = length + 2;
  command.data[0] = x;
  command.data[1] = y;
  memcpy(command.data + 2, text, length);

  //send the packet
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

void CrystalfontzI2CPacketLCD::writeFlashText(uint8_t x, uint8_t y, const __FlashStringHelper *flash_string, uint8_t length)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 31;
  command.length = length + 2;
  command.data[0] = x;
  command.data[1] = y;
  memcpy_P(command.data + 2, flash_string, length);

  //send the packet
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::clearScreen(void)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 6;
  command.length = 0;

  //send the packet
  sendPacket_getReply(&command,&response,0);
  //dumpPacket(&command);
  //dumpPacket(&response);
  }
//----------------------------------------------------------------------------
#define KP_UP     0x01
#define KP_ENTER  0x02
#define KP_CANCEL 0x04
#define KP_LEFT   0x08
#define KP_RIGHT  0x10
#define KP_DOWN   0x20
uint8_t CrystalfontzI2CPacketLCD::getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 24; //24 (0x18): Read Keypad, Polled Mode
  command.length = 0;
  //Send the packet, get the response
  sendPacket_getReply(&command,&response,0);
  // type: 0x40 | 0x18 = 0x58 = 8810
  // data_length: 3
  // data[0] = bit mask showing the keys currently pressed
  // data[1] = bit mask showing the keys that have been pressed since the last poll
  // data[2] = bit mask showing the keys that have been released since the last poll
  //Pull the goodies out of the response, into the users varaibles.
  *down=response.data[0];
  *presses=response.data[1];
  *releases=response.data[2];

  if(*down || *presses || *releases)
    {
  //  dumpPacket(&command);
  //  dumpPacket(&response);
    return(1);
    }
  return(0);
  }
//----------------------------------------------------------------------------
//============================================================================
//============================================================================
//============================================================================
//============================================================================
//============================================================================
//============================================================================
//This does not depend on printf so it is small
void I2C_String_XY(uint8_t col, uint8_t row,
                   const __FlashStringHelper *flash_string )
  {
   //Start the I2C transaction
  Wire.beginTransmission(CFA_634_I2C_ADDRESS);
  //Move the cursor to col,row
  Wire.write(17);
  Wire.write(col);
  Wire.write(row);
  //Pipe out the string, up to but not incuding the null terminator
  uint8_t
    this_character;
  //Get an editable copy of the flash string pointer
  const char
    *ptr;
  ptr=(const char *)flash_string;
  //Grab the next character from the flash string. If it is not
  //null, send it out.
  while(0 != (this_character=pgm_read_byte(ptr)))
    {
    Wire.write(this_character);
    //Point to the next character in RAM
    ptr++;
    }    
  //Stop the I2C transaction
  Wire.endTransmission();
  //Give the display a little time to deal with its new life situation.
  delay(3);
  }
//============================================================================
void I2C_show_dec3_XY(uint8_t col, uint8_t row,uint8_t input)
  {
  //Start the I2C transaction
  Wire.beginTransmission(CFA_634_I2C_ADDRESS);
  //Move the cursor to col,row
  Wire.write(17);
  Wire.write(col);
  Wire.write(row);

  uint8_t
    digit;
  uint8_t
    no_blank;
  digit=input/100U;
  if(digit != 0U)
    {
    Wire.write(digit+(uint8_t)'0');
    no_blank=1U;
    }
  else
    {
    Wire.write((uint8_t)' ');
    no_blank=0U;
    }
  input%=100U;
  /*lint --e(1960)*/
  digit=input/10U;
  if((digit|no_blank) != 0U)
    {
    Wire.write(digit+(uint8_t)'0');
    }
  else
    {
    Wire.write((uint8_t)' ');
    }
  Wire.write((input%10U) + (uint8_t)'0');
  //Stop the I2C transaction
  Wire.endTransmission();
  //Give the display some time to process the command
  delay(1);
  }
//============================================================================
#ifdef USE_PRINTF
// ref http://playground.arduino.cc/Main/Printf
void I2CPrintFFXY(uint8_t col, uint8_t row,
                  const __FlashStringHelper *fmt, ... )
  {
  //Use variable arguments to format the string
  //this includes printf lib which is big
  char
    tmp[40]; // resulting string limited to 128 chars
  va_list
    args;
  va_start(args, fmt );
  vsnprintf_P(tmp, 40, (const char *)fmt, args);
  va_end (args);
 
  //Send the formatted string to the display
  //Start the I2C transaction
  Wire.beginTransmission(CFA_634_I2C_ADDRESS);
  //Move the cursor to col,row
  Wire.write(17);
  Wire.write(col);
  Wire.write(row);
  //Pipe out the string, up to but not incuding the null terminator
  uint8_t
    this_character;
  //Get a pointer into the formatted string on the stack
  char
    *ptr;
  ptr=&tmp[0];
  //Grab the next character from the flash string. If it is not
  //null, send it out.
  while(0 != (this_character=*ptr))
    {
    Wire.write(this_character);
    //Point to the next character in RAM
    ptr++;
    }    
  //Stop the I2C transaction
  Wire.endTransmission();
  //Give the display some time to process the command
  delay(1);
  }
#endif  
//============================================================================

//-------------------------------Define DHT Temperature Sensor Pins---------------------------//
#define DHTPIN1 6
#define DHTPIN2 7
#define DHTPIN3 4
#define DHTPIN4 5
#define DHTTYPE DHT22  //AM2302

DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);
DHT dht4(DHTPIN4, DHTTYPE);

//-------------------------------Initialize Relays and assign pins---------------------------//
int Relay1 = 23;
int Relay2 = 25;
int Relay3 = 27;
int Relay4 = 29;
int Relay5 = 31;
int Relay6 = 33;
int Relay7 = 35;
int Relay8 = 37;


//----------------------------Set Swing Values?---------------------------------------------//
float tempgoal; //oven temperature setting in F
float humidgoal; //oven humidity goal in %
float RPMgoal;
float tempswing = 1; // +/-C for heater and cooling fan.
float humidswing = 1; // +/-%humidity
int D = 2000; //delay time (microseconds?)
//char status = 'off';
   //Initialize average temperature and average humidity variables
float avg_oventemp;
float avg_humidity;
float oventemp;
float humidity;


int i = 30;
int i2;
int i3 = 100; 
const int pinPwm = 3;
const int pinDir = 2;
static int iSpeed;
static int iAcc = 1;


//============================================================================
//============================================================================
//============================================================================
//============================================================================
CrystalfontzI2CPacketLCD   *cfPacket;
//============================================================================
//============================================================================
uint8_t SensorDisplay()
{


//Read Temperature from Sensor 1 and Add value to avg_temp
  oventemp = dht1.readTemperature();  //true for F, blank for C
  avg_oventemp = oventemp;
  //Read Humidity for Sensor 1
  humidity = dht1.readHumidity();
  avg_humidity = humidity;

//    Serial.print("dht1 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht1 Humidity \t");
//    Serial.print(humidity);
 
  //Read Temperature from Sensor 2 and add to average
    oventemp = dht2.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read Humidity from Sensor 2
    humidity = dht2.readHumidity();
    avg_humidity = avg_humidity + humidity;

//    Serial.print("\t dht2 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht2 Humidity \t");
//    Serial.print(humidity);
    
    // Read Temperature from Sensor 3 and add to average
    oventemp = dht3.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read humidity
    humidity = dht3.readHumidity();
    avg_humidity = avg_humidity + humidity;
   
//    Serial.print("\t dht3 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht3 Humidity \t");
//    Serial.print(humidity);

    // Read Temperature from Sensor 4 and add to average
    oventemp = dht4.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read Humidity from Sensor 4
    humidity = dht4.readHumidity();
    avg_humidity = avg_humidity + humidity;
    
//    Serial.print("\t dht4 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht4 Humidity \t");
//    Serial.print(humidity);

  avg_oventemp = avg_oventemp/4;
  avg_humidity = avg_humidity/4;

  

  
  //                  "01234567890123456789"
  I2C_String_XY(5,0,F("LIVE FEED:"));
  I2C_String_XY(0,1,F("Temperature:       C"));
  I2C_show_dec3_XY(15,1,avg_oventemp);  
  I2C_String_XY(0,2,F("Humidity:          %"));  
  I2C_show_dec3_XY(16,2,avg_humidity);
  I2C_String_XY(18,1,F("\x1E\x01\x80")); //Displays Degree Sign
 
  //Relay 3 (Humidifier) & 4 (blower) Control for Oven Humidity
  if(humidgoal + humidswing <= avg_humidity) 
  {
   digitalWrite(Relay3,HIGH);
   digitalWrite(Relay4,HIGH);
  }
  else if(humidgoal - humidswing >= avg_humidity)
  {
   digitalWrite(Relay3,LOW);
   digitalWrite(Relay4,LOW);
   digitalWrite(Relay8,LOW);
  }

//Relay 1 & 2 Control for Heaters - first stage heating
  if (tempgoal + tempswing <= avg_oventemp) 
  {
    digitalWrite(Relay1,HIGH);
    //digitalWrite(Relay2,HIGH); //commented out for 2 stage heating
    //digitalWrite(Relay5,HIGH); // keep fan on
    //digitalWrite(Relay6,HIGH); //commented out for 2 stage heating
  }
  else if (tempgoal - tempswing >= avg_oventemp)
  {
    digitalWrite(Relay1,LOW);
    //digitalWrite(Relay2,LOW);
    digitalWrite(Relay5,LOW);
    //digitalWrite(Relay6,LOW);
    digitalWrite(Relay8,LOW);
  }


  if (tempgoal + (tempswing/2) <= avg_oventemp) 
  {
    //digitalWrite(Relay1,HIGH);
    digitalWrite(Relay2,HIGH);
    //digitalWrite(Relay5,HIGH);  // keep fan on
    //digitalWrite(Relay6,HIGH);
  }
  else if (tempgoal - (tempswing/2) >= avg_oventemp)
  {
    //digitalWrite(Relay1,LOW);
    digitalWrite(Relay2,LOW); 
    //digitalWrite(Relay5,LOW);
    digitalWrite(Relay6,LOW);
    digitalWrite(Relay8,LOW);
  }

}
//============================================================================

//============================================================================
uint8_t Get_Deg(void)
  {
  uint8_t return_num;
  
  uint8_t deg;
  deg = 0b11011111;
  return_num=deg;
    
  return(return_num);
  } 

//============================================================================
//============================================================================
//============================================================================
 uint8_t Temperature()
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;


  
  cfPacket->writeText(0,0, "Set Temperature ", 16);
  cfPacket->writeText(0,1, "Input:         C", 16);
  
  char spec_string[1];
    
      spec_string[0]=Get_Deg();
      cfPacket->writeText(14,1, spec_string, 1); //Prints degree symbol to LCD  

  while(1)
    {
      
        
    //Ask the module what is happening with the keys
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    
    if (keys_down & KP_UP)
      {
        i++; 
        if (i>100)
        {
          i=100;
        } 
        delay(75);
      }
    if (keys_down & KP_DOWN)
      {
        i--;
        if (i<0)
        {
          i=0;
        }
        delay(75);
      }
    if (key_presses & key_releases & KP_UP)
      {
        i++;
        if (i>80)
        {
          i=80;
        }
      }
    if (key_presses & key_releases & KP_DOWN)
      {
        i--;
        if (i<0)
        {
          i=0;
        }
      }
    if (key_presses & KP_CANCEL)
      {
        digitalWrite(Relay1,HIGH);
        digitalWrite(Relay2,HIGH);
        digitalWrite(Relay5,HIGH);
        digitalWrite(Relay6,HIGH);
        tempgoal = 0;
      }
    if (key_presses & KP_ENTER)
      {
       tempgoal = i;
        SensorDisplay();
      }
     //Print Number
     if (i==0){cfPacket->writeText(11,1, "  0", 3);}if (i==1){cfPacket->writeText(11,1, "  1", 3);}if (i==2){cfPacket->writeText(11,1, "  2", 3);}if (i==3){cfPacket->writeText(11,1, "  3", 3);}if (i==4){cfPacket->writeText(11,1, "  4", 3);}if (i==5){cfPacket->writeText(11,1, "  5", 3);}if (i==6){cfPacket->writeText(11,1, "  6", 3);}if (i==7){cfPacket->writeText(11,1, "  7", 3);}if (i==8){cfPacket->writeText(11,1, "  8", 3);}if (i==9){cfPacket->writeText(11,1, "  9", 3);}
     if (i==10){cfPacket->writeText(11,1, " 10", 3);}if (i==11){cfPacket->writeText(11,1, " 11", 3);}if (i==12){cfPacket->writeText(11,1, " 12", 3);}if (i==13){cfPacket->writeText(11,1, " 13", 3);}if (i==14){cfPacket->writeText(11,1, " 14", 3);}if (i==15){cfPacket->writeText(11,1, " 15", 3);}if (i==16){cfPacket->writeText(11,1, " 16", 3);}if (i==17){cfPacket->writeText(11,1, " 17", 3);}if (i==18){cfPacket->writeText(11,1, " 18", 3);}if (i==19){cfPacket->writeText(11,1, " 19", 3);}
     if (i==20){cfPacket->writeText(11,1, " 20", 3);}if (i==21){cfPacket->writeText(11,1, " 21", 3);}if (i==22){cfPacket->writeText(11,1, " 22", 3);}if (i==23){cfPacket->writeText(11,1, " 23", 3);}if (i==24){cfPacket->writeText(11,1, " 24", 3);}if (i==25){cfPacket->writeText(11,1, " 25", 3);}if (i==26){cfPacket->writeText(11,1, " 26", 3);}if (i==27){cfPacket->writeText(11,1, " 27", 3);}if (i==28){cfPacket->writeText(11,1, " 28", 3);}if (i==29){cfPacket->writeText(11,1, " 29", 3);}
     if (i==30){cfPacket->writeText(11,1, " 30", 3);}if (i==31){cfPacket->writeText(11,1, " 31", 3);}if (i==32){cfPacket->writeText(11,1, " 32", 3);}if (i==33){cfPacket->writeText(11,1, " 33", 3);}if (i==34){cfPacket->writeText(11,1, " 34", 3);}if (i==35){cfPacket->writeText(11,1, " 35", 3);}if (i==36){cfPacket->writeText(11,1, " 36", 3);}if (i==37){cfPacket->writeText(11,1, " 37", 3);}if (i==38){cfPacket->writeText(11,1, " 38", 3);}if (i==39){cfPacket->writeText(11,1, " 39", 3);}
     if (i==40){cfPacket->writeText(11,1, " 40", 3);}if (i==41){cfPacket->writeText(11,1, " 41", 3);}if (i==42){cfPacket->writeText(11,1, " 42", 3);}if (i==43){cfPacket->writeText(11,1, " 43", 3);}if (i==44){cfPacket->writeText(11,1, " 44", 3);}if (i==45){cfPacket->writeText(11,1, " 45", 3);}if (i==46){cfPacket->writeText(11,1, " 46", 3);}if (i==47){cfPacket->writeText(11,1, " 47", 3);}if (i==48){cfPacket->writeText(11,1, " 48", 3);}if (i==49){cfPacket->writeText(11,1, " 49", 3);}
     if (i==50){cfPacket->writeText(11,1, " 50", 3);}if (i==51){cfPacket->writeText(11,1, " 51", 3);}if (i==52){cfPacket->writeText(11,1, " 52", 3);}if (i==53){cfPacket->writeText(11,1, " 53", 3);}if (i==54){cfPacket->writeText(11,1, " 54", 3);}if (i==55){cfPacket->writeText(11,1, " 55", 3);}if (i==56){cfPacket->writeText(11,1, " 56", 3);}if (i==57){cfPacket->writeText(11,1, " 57", 3);}if (i==58){cfPacket->writeText(11,1, " 58", 3);}if (i==59){cfPacket->writeText(11,1, " 59", 3);}
     if (i==60){cfPacket->writeText(11,1, " 60", 3);}if (i==61){cfPacket->writeText(11,1, " 61", 3);}if (i==62){cfPacket->writeText(11,1, " 62", 3);}if (i==63){cfPacket->writeText(11,1, " 63", 3);}if (i==64){cfPacket->writeText(11,1, " 64", 3);}if (i==65){cfPacket->writeText(11,1, " 65", 3);}if (i==66){cfPacket->writeText(11,1, " 66", 3);}if (i==67){cfPacket->writeText(11,1, " 67", 3);}if (i==68){cfPacket->writeText(11,1, " 68", 3);}if (i==69){cfPacket->writeText(11,1, " 69", 3);}
     if (i==70){cfPacket->writeText(11,1, " 70", 3);}if (i==71){cfPacket->writeText(11,1, " 71", 3);}if (i==72){cfPacket->writeText(11,1, " 72", 3);}if (i==73){cfPacket->writeText(11,1, " 73", 3);}if (i==74){cfPacket->writeText(11,1, " 74", 3);}if (i==75){cfPacket->writeText(11,1, " 75", 3);}if (i==76){cfPacket->writeText(11,1, " 76", 3);}if (i==77){cfPacket->writeText(11,1, " 77", 3);}if (i==78){cfPacket->writeText(11,1, " 78", 3);}if (i==79){cfPacket->writeText(11,1, " 79", 3);}
     if (i==80){cfPacket->writeText(11,1, " 80", 3);}
     
     //if (i==81){cfPacket->writeText(11,1, " 81", 3);}if (i==82){cfPacket->writeText(11,1, " 82", 3);}if (i==83){cfPacket->writeText(11,1, " 83", 3);}if (i==84){cfPacket->writeText(11,1, " 84", 3);}if (i==85){cfPacket->writeText(11,1, " 85", 3);}if (i==86){cfPacket->writeText(11,1, " 86", 3);}if (i==87){cfPacket->writeText(11,1, " 87", 3);}if (i==88){cfPacket->writeText(11,1, " 88", 3);}if (i==89){cfPacket->writeText(11,1, " 89", 3);}
     //if (i==90){cfPacket->writeText(11,1, " 90", 3);}if (i==91){cfPacket->writeText(11,1, " 91", 3);}if (i==92){cfPacket->writeText(11,1, " 92", 3);}if (i==93){cfPacket->writeText(11,1, " 93", 3);}if (i==94){cfPacket->writeText(11,1, " 94", 3);}if (i==95){cfPacket->writeText(11,1, " 95", 3);}if (i==96){cfPacket->writeText(11,1, " 96", 3);}if (i==97){cfPacket->writeText(11,1, " 97", 3);}if (i==98){cfPacket->writeText(11,1, " 98", 3);}if (i==99){cfPacket->writeText(11,1, " 99", 3);}
     //if (i==100){cfPacket->writeText(11,1, "100", 3);}


    
    //Change Menu 
    if (key_presses & KP_RIGHT)
        {
          Humidity();
          //return(tempgoal);
          
        }
    if (key_presses & KP_LEFT)
        {
          RPM();
          //return(tempgoal);
        }
      SensorDisplay();
    }
  
  }
//============================================================================
//============================================================================
 uint8_t RPM()
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;
  int relay7status;
  
  cfPacket->writeText(0,0, "Set Speed       ", 16);
  cfPacket->writeText(0,1, "Input:       RPM", 16);


  while(1)
    {
      
    SensorDisplay();
    //Ask the module what is happening with the keys
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    
    if (keys_down & KP_UP)
      {
        i2++; 
        if (i2>250)
        {
          i2=250;
        } 
        if(relay7status = 2018)
        {
          //Display Current RPM
            I2C_String_XY(0,3,F("Speed:           RPM"));
            I2C_show_dec3_XY(13,3,i2);
        }
        delay(75);
      }
    if (keys_down & KP_DOWN)
      {
        i2--;
        if (i2<0)
        {
          i2=0;
        }
        if(relay7status = 2018)
        {
          //Display Current RPM
            I2C_String_XY(0,3,F("Speed:           RPM"));
            I2C_show_dec3_XY(13,3,i2);
        }
        delay(75);
      }
    if (key_presses & key_releases & KP_UP)
      {
        i2++;
        if (i2>200)
        {
          i2=200;
        }
        if(relay7status = 2018)
        {
          //Display Current RPM
            I2C_String_XY(0,3,F("Speed:           RPM"));
            I2C_show_dec3_XY(13,3,i2);
        }
      }
    if (key_presses & key_releases & KP_DOWN)
      {
        i2--;
        if (i2<0)
        {
          i2=0;
        }
        if(relay7status = 2018)
        {
          //Display Current RPM
            I2C_String_XY(0,3,F("Speed:           RPM"));
            I2C_show_dec3_XY(13,3,i2);
        }
      }
   
        if(i2>0)
         {
          digitalWrite(Relay7,LOW); //Turn on relay 7 to power motor when enter key is pressed
          
          //RampRPM(); //Doesn't work right
          
          //relay7status = 2018; //doesnt work right
            //Display Current RPM
            I2C_String_XY(0,3,F("Speed:           RPM"));
            I2C_show_dec3_XY(13,3,i2);
         }
         
      
    if (key_presses & KP_CANCEL)
      {
        SlowRPM();
        
        //Display Current RPM
        I2C_String_XY(0,3,F("Speed:           RPM"));
        I2C_show_dec3_XY(13,3,i2);
      }
        
   if (i2 >= 0) 
         {
          analogWrite(pinPwm, i2);
          digitalWrite(pinDir, LOW);
         }
      
     //Print Number
     if (i2==0){cfPacket->writeText(9,1, "  0", 3);}if (i2==1){cfPacket->writeText(9,1, "  1", 3);}if (i2==2){cfPacket->writeText(9,1, "  2", 3);}if (i2==3){cfPacket->writeText(9,1, "  3", 3);}if (i2==4){cfPacket->writeText(9,1, "  4", 3);}if (i2==5){cfPacket->writeText(9,1, "  5", 3);}if (i2==6){cfPacket->writeText(9,1, "  6", 3);}if (i2==7){cfPacket->writeText(9,1, "  7", 3);}if (i2==8){cfPacket->writeText(9,1, "  8", 3);}if (i2==9){cfPacket->writeText(9,1, "  9", 3);}
     if (i2==10){cfPacket->writeText(9,1, " 10", 3);}if (i2==11){cfPacket->writeText(9,1, " 11", 3);}if (i2==12){cfPacket->writeText(9,1, " 12", 3);}if (i2==13){cfPacket->writeText(9,1, " 13", 3);}if (i2==14){cfPacket->writeText(9,1, " 14", 3);}if (i2==15){cfPacket->writeText(9,1, " 15", 3);}if (i2==16){cfPacket->writeText(9,1, " 16", 3);}if (i2==17){cfPacket->writeText(9,1, " 17", 3);}if (i2==18){cfPacket->writeText(9,1, " 18", 3);}if (i2==19){cfPacket->writeText(9,1, " 19", 3);}
     if (i2==20){cfPacket->writeText(9,1, " 20", 3);}if (i2==21){cfPacket->writeText(9,1, " 21", 3);}if (i2==22){cfPacket->writeText(9,1, " 22", 3);}if (i2==23){cfPacket->writeText(9,1, " 23", 3);}if (i2==24){cfPacket->writeText(9,1, " 24", 3);}if (i2==25){cfPacket->writeText(9,1, " 25", 3);}if (i2==26){cfPacket->writeText(9,1, " 26", 3);}if (i2==27){cfPacket->writeText(9,1, " 27", 3);}if (i2==28){cfPacket->writeText(9,1, " 28", 3);}if (i2==29){cfPacket->writeText(9,1, " 29", 3);}
     if (i2==30){cfPacket->writeText(9,1, " 30", 3);}if (i2==31){cfPacket->writeText(9,1, " 31", 3);}if (i2==32){cfPacket->writeText(9,1, " 32", 3);}if (i2==33){cfPacket->writeText(9,1, " 33", 3);}if (i2==34){cfPacket->writeText(9,1, " 34", 3);}if (i2==35){cfPacket->writeText(9,1, " 35", 3);}if (i2==36){cfPacket->writeText(9,1, " 36", 3);}if (i2==37){cfPacket->writeText(9,1, " 37", 3);}if (i2==38){cfPacket->writeText(9,1, " 38", 3);}if (i2==39){cfPacket->writeText(9,1, " 39", 3);}
     if (i2==40){cfPacket->writeText(9,1, " 40", 3);}if (i2==41){cfPacket->writeText(9,1, " 41", 3);}if (i2==42){cfPacket->writeText(9,1, " 42", 3);}if (i2==43){cfPacket->writeText(9,1, " 43", 3);}if (i2==44){cfPacket->writeText(9,1, " 44", 3);}if (i2==45){cfPacket->writeText(9,1, " 45", 3);}if (i2==46){cfPacket->writeText(9,1, " 46", 3);}if (i2==47){cfPacket->writeText(9,1, " 47", 3);}if (i2==48){cfPacket->writeText(9,1, " 48", 3);}if (i2==49){cfPacket->writeText(9,1, " 49", 3);}
     if (i2==50){cfPacket->writeText(9,1, " 50", 3);}if (i2==51){cfPacket->writeText(9,1, " 51", 3);}if (i2==52){cfPacket->writeText(9,1, " 52", 3);}if (i2==53){cfPacket->writeText(9,1, " 53", 3);}if (i2==54){cfPacket->writeText(9,1, " 54", 3);}if (i2==55){cfPacket->writeText(9,1, " 55", 3);}if (i2==56){cfPacket->writeText(9,1, " 56", 3);}if (i2==57){cfPacket->writeText(9,1, " 57", 3);}if (i2==58){cfPacket->writeText(9,1, " 58", 3);}if (i2==59){cfPacket->writeText(9,1, " 59", 3);}
     if (i2==60){cfPacket->writeText(9,1, " 60", 3);}if (i2==61){cfPacket->writeText(9,1, " 61", 3);}if (i2==62){cfPacket->writeText(9,1, " 62", 3);}if (i2==63){cfPacket->writeText(9,1, " 63", 3);}if (i2==64){cfPacket->writeText(9,1, " 64", 3);}if (i2==65){cfPacket->writeText(9,1, " 65", 3);}if (i2==66){cfPacket->writeText(9,1, " 66", 3);}if (i2==67){cfPacket->writeText(9,1, " 67", 3);}if (i2==68){cfPacket->writeText(9,1, " 68", 3);}if (i2==69){cfPacket->writeText(9,1, " 69", 3);}
     if (i2==70){cfPacket->writeText(9,1, " 70", 3);}if (i2==71){cfPacket->writeText(9,1, " 71", 3);}if (i2==72){cfPacket->writeText(9,1, " 72", 3);}if (i2==73){cfPacket->writeText(9,1, " 73", 3);}if (i2==74){cfPacket->writeText(9,1, " 74", 3);}if (i2==75){cfPacket->writeText(9,1, " 75", 3);}if (i2==76){cfPacket->writeText(9,1, " 76", 3);}if (i2==77){cfPacket->writeText(9,1, " 77", 3);}if (i2==78){cfPacket->writeText(9,1, " 78", 3);}if (i2==79){cfPacket->writeText(9,1, " 79", 3);}
     if (i2==80){cfPacket->writeText(9,1, " 80", 3);}if (i2==81){cfPacket->writeText(9,1, " 81", 3);}if (i2==82){cfPacket->writeText(9,1, " 82", 3);}if (i2==83){cfPacket->writeText(9,1, " 83", 3);}if (i2==84){cfPacket->writeText(9,1, " 84", 3);}if (i2==85){cfPacket->writeText(9,1, " 85", 3);}if (i2==86){cfPacket->writeText(9,1, " 86", 3);}if (i2==87){cfPacket->writeText(9,1, " 87", 3);}if (i2==88){cfPacket->writeText(9,1, " 88", 3);}if (i2==89){cfPacket->writeText(9,1, " 89", 3);}
     if (i2==90){cfPacket->writeText(9,1, " 90", 3);}if (i2==91){cfPacket->writeText(9,1, " 91", 3);}if (i2==92){cfPacket->writeText(9,1, " 92", 3);}if (i2==93){cfPacket->writeText(9,1, " 93", 3);}if (i2==94){cfPacket->writeText(9,1, " 94", 3);}if (i2==95){cfPacket->writeText(9,1, " 95", 3);}if (i2==96){cfPacket->writeText(9,1, " 96", 3);}if (i2==97){cfPacket->writeText(9,1, " 97", 3);}if (i2==98){cfPacket->writeText(9,1, " 98", 3);}if (i2==99){cfPacket->writeText(9,1, " 99", 3);}
     if (i2==100){cfPacket->writeText(9,1, "100", 3);}if (i2==101){cfPacket->writeText(9,1, "101", 3);}if (i2==102){cfPacket->writeText(9,1, "102", 3);}if (i2==103){cfPacket->writeText(9,1, "103", 3);}if (i2==104){cfPacket->writeText(9,1, "104", 3);}if (i2==105){cfPacket->writeText(9,1, "105", 3);}if (i2==106){cfPacket->writeText(9,1, "106", 3);}if (i2==107){cfPacket->writeText(9,1, "107", 3);}if (i2==108){cfPacket->writeText(9,1, "108", 3);}if (i2==109){cfPacket->writeText(9,1, "109", 3);}
     if (i2==110){cfPacket->writeText(9,1, "110", 3);}if (i2==111){cfPacket->writeText(9,1, "111", 3);}if (i2==112){cfPacket->writeText(9,1, "112", 3);}if (i2==113){cfPacket->writeText(9,1, "113", 3);}if (i2==114){cfPacket->writeText(9,1, "114", 3);}if (i2==115){cfPacket->writeText(9,1, "115", 3);}if (i2==116){cfPacket->writeText(9,1, "116", 3);}if (i2==117){cfPacket->writeText(9,1, "117", 3);}if (i2==118){cfPacket->writeText(9,1, "118", 3);}if (i2==119){cfPacket->writeText(9,1, "119", 3);}
     if (i2==120){cfPacket->writeText(9,1, "120", 3);}if (i2==121){cfPacket->writeText(9,1, "121", 3);}if (i2==122){cfPacket->writeText(9,1, "122", 3);}if (i2==123){cfPacket->writeText(9,1, "123", 3);}if (i2==124){cfPacket->writeText(9,1, "124", 3);}if (i2==125){cfPacket->writeText(9,1, "125", 3);}if (i2==126){cfPacket->writeText(9,1, "126", 3);}if (i2==127){cfPacket->writeText(9,1, "127", 3);}if (i2==128){cfPacket->writeText(9,1, "128", 3);}if (i2==129){cfPacket->writeText(9,1, "129", 3);}
     if (i2==130){cfPacket->writeText(9,1, "130", 3);}if (i2==131){cfPacket->writeText(9,1, "131", 3);}if (i2==132){cfPacket->writeText(9,1, "132", 3);}if (i2==133){cfPacket->writeText(9,1, "133", 3);}if (i2==134){cfPacket->writeText(9,1, "134", 3);}if (i2==135){cfPacket->writeText(9,1, "135", 3);}if (i2==136){cfPacket->writeText(9,1, "136", 3);}if (i2==137){cfPacket->writeText(9,1, "137", 3);}if (i2==138){cfPacket->writeText(9,1, "138", 3);}if (i2==139){cfPacket->writeText(9,1, "139", 3);}
     if (i2==140){cfPacket->writeText(9,1, "140", 3);}if (i2==141){cfPacket->writeText(9,1, "141", 3);}if (i2==142){cfPacket->writeText(9,1, "142", 3);}if (i2==143){cfPacket->writeText(9,1, "143", 3);}if (i2==144){cfPacket->writeText(9,1, "144", 3);}if (i2==145){cfPacket->writeText(9,1, "145", 3);}if (i2==146){cfPacket->writeText(9,1, "146", 3);}if (i2==147){cfPacket->writeText(9,1, "147", 3);}if (i2==148){cfPacket->writeText(9,1, "148", 3);}if (i2==149){cfPacket->writeText(9,1, "149", 3);}
     if (i2==150){cfPacket->writeText(9,1, "150", 3);}if (i2==151){cfPacket->writeText(9,1, "151", 3);}if (i2==152){cfPacket->writeText(9,1, "152", 3);}if (i2==153){cfPacket->writeText(9,1, "153", 3);}if (i2==154){cfPacket->writeText(9,1, "154", 3);}if (i2==155){cfPacket->writeText(9,1, "155", 3);}if (i2==156){cfPacket->writeText(9,1, "156", 3);}if (i2==157){cfPacket->writeText(9,1, "157", 3);}if (i2==158){cfPacket->writeText(9,1, "158", 3);}if (i2==159){cfPacket->writeText(9,1, "159", 3);}
     if (i2==160){cfPacket->writeText(9,1, "160", 3);}if (i2==161){cfPacket->writeText(9,1, "161", 3);}if (i2==162){cfPacket->writeText(9,1, "162", 3);}if (i2==163){cfPacket->writeText(9,1, "163", 3);}if (i2==164){cfPacket->writeText(9,1, "164", 3);}if (i2==165){cfPacket->writeText(9,1, "165", 3);}if (i2==166){cfPacket->writeText(9,1, "166", 3);}if (i2==167){cfPacket->writeText(9,1, "167", 3);}if (i2==168){cfPacket->writeText(9,1, "168", 3);}if (i2==169){cfPacket->writeText(9,1, "169", 3);}
     if (i2==170){cfPacket->writeText(9,1, "170", 3);}if (i2==171){cfPacket->writeText(9,1, "171", 3);}if (i2==172){cfPacket->writeText(9,1, "172", 3);}if (i2==173){cfPacket->writeText(9,1, "173", 3);}if (i2==174){cfPacket->writeText(9,1, "174", 3);}if (i2==175){cfPacket->writeText(9,1, "175", 3);}if (i2==176){cfPacket->writeText(9,1, "176", 3);}if (i2==177){cfPacket->writeText(9,1, "177", 3);}if (i2==178){cfPacket->writeText(9,1, "178", 3);}if (i2==179){cfPacket->writeText(9,1, "179", 3);}
     if (i2==180){cfPacket->writeText(9,1, "180", 3);}if (i2==181){cfPacket->writeText(9,1, "181", 3);}if (i2==182){cfPacket->writeText(9,1, "182", 3);}if (i2==183){cfPacket->writeText(9,1, "183", 3);}if (i2==184){cfPacket->writeText(9,1, "184", 3);}if (i2==185){cfPacket->writeText(9,1, "185", 3);}if (i2==186){cfPacket->writeText(9,1, "186", 3);}if (i2==187){cfPacket->writeText(9,1, "187", 3);}if (i2==188){cfPacket->writeText(9,1, "188", 3);}if (i2==189){cfPacket->writeText(9,1, "189", 3);}
     if (i2==190){cfPacket->writeText(9,1, "190", 3);}if (i2==191){cfPacket->writeText(9,1, "191", 3);}if (i2==192){cfPacket->writeText(9,1, "192", 3);}if (i2==193){cfPacket->writeText(9,1, "193", 3);}if (i2==194){cfPacket->writeText(9,1, "194", 3);}if (i2==195){cfPacket->writeText(9,1, "195", 3);}if (i2==196){cfPacket->writeText(9,1, "196", 3);}if (i2==197){cfPacket->writeText(9,1, "197", 3);}if (i2==198){cfPacket->writeText(9,1, "198", 3);}if (i2==199){cfPacket->writeText(9,1, "199", 3);}
     if (i2==200){cfPacket->writeText(9,1, "200", 3);}
     
     //if (i2==201){cfPacket->writeText(9,1, "201", 3);}if (i2==202){cfPacket->writeText(9,1, "202", 3);}if (i2==203){cfPacket->writeText(9,1, "203", 3);}if (i2==204){cfPacket->writeText(9,1, "204", 3);}if (i2==205){cfPacket->writeText(9,1, "205", 3);}if (i2==206){cfPacket->writeText(9,1, "206", 3);}if (i2==207){cfPacket->writeText(9,1, "207", 3);}if (i2==208){cfPacket->writeText(9,1, "208", 3);}if (i2==209){cfPacket->writeText(9,1, "209", 3);}
     //if (i2==210){cfPacket->writeText(9,1, "210", 3);}if (i2==211){cfPacket->writeText(9,1, "211", 3);}if (i2==212){cfPacket->writeText(9,1, "212", 3);}if (i2==213){cfPacket->writeText(9,1, "213", 3);}if (i2==214){cfPacket->writeText(9,1, "214", 3);}if (i2==215){cfPacket->writeText(9,1, "215", 3);}if (i2==216){cfPacket->writeText(9,1, "216", 3);}if (i2==217){cfPacket->writeText(9,1, "217", 3);}if (i2==218){cfPacket->writeText(9,1, "218", 3);}if (i2==219){cfPacket->writeText(9,1, "219", 3);}
     //if (i2==220){cfPacket->writeText(9,1, "220", 3);}if (i2==221){cfPacket->writeText(9,1, "221", 3);}if (i2==222){cfPacket->writeText(9,1, "222", 3);}if (i2==223){cfPacket->writeText(9,1, "223", 3);}if (i2==224){cfPacket->writeText(9,1, "224", 3);}if (i2==225){cfPacket->writeText(9,1, "225", 3);}if (i2==226){cfPacket->writeText(9,1, "226", 3);}if (i2==227){cfPacket->writeText(9,1, "227", 3);}if (i2==228){cfPacket->writeText(9,1, "228", 3);}if (i2==229){cfPacket->writeText(9,1, "229", 3);}
     //if (i2==230){cfPacket->writeText(9,1, "230", 3);}if (i2==231){cfPacket->writeText(9,1, "231", 3);}if (i2==232){cfPacket->writeText(9,1, "232", 3);}if (i2==233){cfPacket->writeText(9,1, "233", 3);}if (i2==234){cfPacket->writeText(9,1, "234", 3);}if (i2==235){cfPacket->writeText(9,1, "235", 3);}if (i2==236){cfPacket->writeText(9,1, "236", 3);}if (i2==237){cfPacket->writeText(9,1, "237", 3);}if (i2==238){cfPacket->writeText(9,1, "238", 3);}if (i2==239){cfPacket->writeText(9,1, "239", 3);}
     //if (i2==240){cfPacket->writeText(9,1, "240", 3);}if (i2==241){cfPacket->writeText(9,1, "241", 3);}if (i2==242){cfPacket->writeText(9,1, "242", 3);}if (i2==243){cfPacket->writeText(9,1, "243", 3);}if (i2==244){cfPacket->writeText(9,1, "244", 3);}if (i2==245){cfPacket->writeText(9,1, "245", 3);}if (i2==246){cfPacket->writeText(9,1, "246", 3);}if (i2==247){cfPacket->writeText(9,1, "247", 3);}if (i2==248){cfPacket->writeText(9,1, "248", 3);}if (i2==249){cfPacket->writeText(9,1, "249", 3);}
     //if (i2==250){cfPacket->writeText(9,1, "250", 3);}
    
    
    
    //Change Menu
    if (key_presses & KP_RIGHT)
        {
          
            
            Temperature();
            //return(i2);
          
        }
    if (key_presses & KP_LEFT)
        {
          
            
            Humidity();
            //return(i2);
         
        }
 
    }
   
  }
//============================================================================
 uint8_t Humidity()
  {
  uint8_t
    keys_down;
  uint8_t
    key_presses;
  uint8_t
    key_releases;

    

 
  cfPacket->writeText(0,0, "Set Humidity    ", 16);
  cfPacket->writeText(0,1, "Input:         %", 16); 

  while(1)
    {
      
     
     
    //Ask the module what is happening with the keys
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    
    if (keys_down & KP_UP)
      {
        i3++; 
        if (i3>100)
        {
          i3=100;
        } 
        delay(75);
      }
    if (keys_down & KP_DOWN)
      {
        i3--;
        if (i3<0)
        {
          i3=0;
        }
        delay(75);
      }
    if (key_presses & key_releases & KP_UP)
      {
        i3++;
        if (i3>100)
        {
          i3=100;
        }
      }
    if (key_presses & key_releases & KP_DOWN)
      {
        i3--;
        if (i3<0)
        {
          i3=0;
        }
      }
    if (key_presses & KP_CANCEL)
      {
        digitalWrite(Relay3,HIGH);
        digitalWrite(Relay4,HIGH);
        humidgoal = 0;
      }
    if (key_presses & KP_ENTER)
      {
       humidgoal = i3;
       SensorDisplay();      
      }
      
     //Print Number
     if (i3==0){cfPacket->writeText(12,1, "  0", 3);}if (i3==1){cfPacket->writeText(12,1, "  1", 3);}if (i3==2){cfPacket->writeText(12,1, "  2", 3);}if (i3==3){cfPacket->writeText(12,1, "  3", 3);}if (i3==4){cfPacket->writeText(12,1, "  4", 3);}if (i3==5){cfPacket->writeText(12,1, "  5", 3);}if (i3==6){cfPacket->writeText(12,1, "  6", 3);}if (i3==7){cfPacket->writeText(12,1, "  7", 3);}if (i3==8){cfPacket->writeText(12,1, "  8", 3);}if (i3==9){cfPacket->writeText(12,1, "  9", 3);}
     if (i3==10){cfPacket->writeText(12,1, " 10", 3);}if (i3==11){cfPacket->writeText(12,1, " 11", 3);}if (i3==12){cfPacket->writeText(12,1, " 12", 3);}if (i3==13){cfPacket->writeText(12,1, " 13", 3);}if (i3==14){cfPacket->writeText(12,1, " 14", 3);}if (i3==15){cfPacket->writeText(12,1, " 15", 3);}if (i3==16){cfPacket->writeText(12,1, " 16", 3);}if (i3==17){cfPacket->writeText(12,1, " 17", 3);}if (i3==18){cfPacket->writeText(12,1, " 18", 3);}if (i3==19){cfPacket->writeText(12,1, " 19", 3);}
     if (i3==20){cfPacket->writeText(12,1, " 20", 3);}if (i3==21){cfPacket->writeText(12,1, " 21", 3);}if (i3==22){cfPacket->writeText(12,1, " 22", 3);}if (i3==23){cfPacket->writeText(12,1, " 23", 3);}if (i3==24){cfPacket->writeText(12,1, " 24", 3);}if (i3==25){cfPacket->writeText(12,1, " 25", 3);}if (i3==26){cfPacket->writeText(12,1, " 26", 3);}if (i3==27){cfPacket->writeText(12,1, " 27", 3);}if (i3==28){cfPacket->writeText(12,1, " 28", 3);}if (i3==29){cfPacket->writeText(12,1, " 29", 3);}
     if (i3==30){cfPacket->writeText(12,1, " 30", 3);}if (i3==31){cfPacket->writeText(12,1, " 31", 3);}if (i3==32){cfPacket->writeText(12,1, " 32", 3);}if (i3==33){cfPacket->writeText(12,1, " 33", 3);}if (i3==34){cfPacket->writeText(12,1, " 34", 3);}if (i3==35){cfPacket->writeText(12,1, " 35", 3);}if (i3==36){cfPacket->writeText(12,1, " 36", 3);}if (i3==37){cfPacket->writeText(12,1, " 37", 3);}if (i3==38){cfPacket->writeText(12,1, " 38", 3);}if (i3==39){cfPacket->writeText(12,1, " 39", 3);}
     if (i3==40){cfPacket->writeText(12,1, " 40", 3);}if (i3==41){cfPacket->writeText(12,1, " 41", 3);}if (i3==42){cfPacket->writeText(12,1, " 42", 3);}if (i3==43){cfPacket->writeText(12,1, " 43", 3);}if (i3==44){cfPacket->writeText(12,1, " 44", 3);}if (i3==45){cfPacket->writeText(12,1, " 45", 3);}if (i3==46){cfPacket->writeText(12,1, " 46", 3);}if (i3==47){cfPacket->writeText(12,1, " 47", 3);}if (i3==48){cfPacket->writeText(12,1, " 48", 3);}if (i3==49){cfPacket->writeText(12,1, " 49", 3);}
     if (i3==50){cfPacket->writeText(12,1, " 50", 3);}if (i3==51){cfPacket->writeText(12,1, " 51", 3);}if (i3==52){cfPacket->writeText(12,1, " 52", 3);}if (i3==53){cfPacket->writeText(12,1, " 53", 3);}if (i3==54){cfPacket->writeText(12,1, " 54", 3);}if (i3==55){cfPacket->writeText(12,1, " 55", 3);}if (i3==56){cfPacket->writeText(12,1, " 56", 3);}if (i3==57){cfPacket->writeText(12,1, " 57", 3);}if (i3==58){cfPacket->writeText(12,1, " 58", 3);}if (i3==59){cfPacket->writeText(12,1, " 59", 3);}
     if (i3==60){cfPacket->writeText(12,1, " 60", 3);}if (i3==61){cfPacket->writeText(12,1, " 61", 3);}if (i3==62){cfPacket->writeText(12,1, " 62", 3);}if (i3==63){cfPacket->writeText(12,1, " 63", 3);}if (i3==64){cfPacket->writeText(12,1, " 64", 3);}if (i3==65){cfPacket->writeText(12,1, " 65", 3);}if (i3==66){cfPacket->writeText(12,1, " 66", 3);}if (i3==67){cfPacket->writeText(12,1, " 67", 3);}if (i3==68){cfPacket->writeText(12,1, " 68", 3);}if (i3==69){cfPacket->writeText(12,1, " 69", 3);}
     if (i3==70){cfPacket->writeText(12,1, " 70", 3);}if (i3==71){cfPacket->writeText(12,1, " 71", 3);}if (i3==72){cfPacket->writeText(12,1, " 72", 3);}if (i3==73){cfPacket->writeText(12,1, " 73", 3);}if (i3==74){cfPacket->writeText(12,1, " 74", 3);}if (i3==75){cfPacket->writeText(12,1, " 75", 3);}if (i3==76){cfPacket->writeText(12,1, " 76", 3);}if (i3==77){cfPacket->writeText(12,1, " 77", 3);}if (i3==78){cfPacket->writeText(12,1, " 78", 3);}if (i3==79){cfPacket->writeText(12,1, " 79", 3);}
     if (i3==80){cfPacket->writeText(12,1, " 80", 3);}if (i3==81){cfPacket->writeText(12,1, " 81", 3);}if (i3==82){cfPacket->writeText(12,1, " 82", 3);}if (i3==83){cfPacket->writeText(12,1, " 83", 3);}if (i3==84){cfPacket->writeText(12,1, " 84", 3);}if (i3==85){cfPacket->writeText(12,1, " 85", 3);}if (i3==86){cfPacket->writeText(12,1, " 86", 3);}if (i3==87){cfPacket->writeText(12,1, " 87", 3);}if (i3==88){cfPacket->writeText(12,1, " 88", 3);}if (i3==89){cfPacket->writeText(12,1, " 89", 3);}
     if (i3==90){cfPacket->writeText(12,1, " 90", 3);}if (i3==91){cfPacket->writeText(12,1, " 91", 3);}if (i3==92){cfPacket->writeText(12,1, " 92", 3);}if (i3==93){cfPacket->writeText(12,1, " 93", 3);}if (i3==94){cfPacket->writeText(12,1, " 94", 3);}if (i3==95){cfPacket->writeText(12,1, " 95", 3);}if (i3==96){cfPacket->writeText(12,1, " 96", 3);}if (i3==97){cfPacket->writeText(12,1, " 97", 3);}if (i3==98){cfPacket->writeText(12,1, " 98", 3);}if (i3==99){cfPacket->writeText(12,1, " 99", 3);}
     if (i3==100){cfPacket->writeText(12,1, "100", 3);}

        
    //Change Menu
    if (key_presses & KP_RIGHT)
        {
          RPM();
          //return(humidgoal);
        }
    if (key_presses & KP_LEFT)
        {
          Temperature();
          //return(humidgoal);
        }
      SensorDisplay();
    }
  
  }
//============================================================================
uint8_t CheckRPM()
{
         if (i2 >= 0) 
          {
            analogWrite(pinPwm, i2);
            digitalWrite(pinDir, LOW);
       
          }
        
}

//============================================================================
uint8_t SlowRPM()
{
  while(1)
    {
      i2-=iAcc;

       if (i2 >= 0) 
          {
            analogWrite(pinPwm, i2);
            digitalWrite(pinDir, LOW);
       
          }
            
      if (i2==0)
      {
        digitalWrite(Relay7,HIGH); //Turn off relay 7 to power off motor
        RPMgoal = 0;
        return(i2);
      }
      
      delay(50);
    }
    return(i2);
}
//============================================================================
uint8_t RampRPM()
{
  while(1)
    {
      i2+=iAcc;

       if (i2 >= 0) 
          {
            analogWrite(pinPwm, i2);
            digitalWrite(pinDir, LOW);
       
          }
            
      if (i2 = RPMgoal)
      {
        
      RPM();
      }
      
      delay(50);
    }
  return(i2);  
}
//============================================================================



void setup()
  {
  
  pinMode(led_pin, OUTPUT);
  pinMode(switch_pin, INPUT_PULLUP);
  Serial.begin(115200);  // start serial for output
  SerPrintFF(F("Free Memory: %d\n"),freeRam);

  Wire.begin(); // join i2c bus (address optional for master)

  //Give it time to boot, in case we are coming from power-up.
  delay(500);

  //Reboot, just in case the last time we ran the code we left the
  //display all messed up, and we were not power-cycled.
  //Start the I2C transaction
  Wire.beginTransmission(CFA_634_I2C_ADDRESS);
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(26);
  Wire.write(26);
  Wire.endTransmission();
  //Give it time to re-boot
  delay(80);
  
  // (Optional) Delay to see the logo screen
  delay(100);
  
  Wire.beginTransmission(CFA_634_I2C_ADDRESS);
  //Hide the cursor
  Wire.write(4);
  //Turn Scroll Off
  Wire.write(20);
  //Turn Wrap Off
  Wire.write(24);
  //Clear the LCD
  Wire.write(12);
  Wire.endTransmission();
  //Give it time to clear
  delay(5);

    Serial.begin(9600); //initialize serial communication
  pinMode(Relay1,OUTPUT);
  pinMode(Relay2,OUTPUT);
  pinMode(Relay3,OUTPUT);
  pinMode(Relay4,OUTPUT);
  pinMode(Relay5,OUTPUT);
  pinMode(Relay6,OUTPUT);
  pinMode(Relay7,OUTPUT);
  pinMode(Relay8,OUTPUT);
  
  digitalWrite(Relay1,HIGH); //HIGH sets all relays to "off" or "open" position
  digitalWrite(Relay2,HIGH);
  digitalWrite(Relay3,HIGH);
  digitalWrite(Relay4,HIGH);
  digitalWrite(Relay5,HIGH);
  digitalWrite(Relay6,HIGH);
  digitalWrite(Relay7,HIGH);
  digitalWrite(Relay8,HIGH);

  Serial.println("DHT22 Test");
  
  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();
  
    
  //If you know the address:
  //cfPacket = new CrystalfontzI2CPacketLCD(42);
  //If you do not know the address, this will search for the module.
  //cfPacket = new CrystalfontzI2CPacketLCD(0xFF);
  //cfPacket->Set_I2C_Adress(cfPacket->Search_I2C_Adresses());
  cfPacket = new CrystalfontzI2CPacketLCD(10);
  //Setup New Address
   // cfPacket->Change_Module_Address(10);

  
  pinMode(pinPwm, OUTPUT);  //motor shield
  pinMode(pinDir, OUTPUT);  //motor shield
  
  }
//============================================================================

void loop()
{
  //cfPacket->clearScreen();
          
 //Read Temperature from Sensor 1 and Add value to avg_temp
  oventemp = dht1.readTemperature();  //true for F, blank for C
  avg_oventemp = oventemp;
  //Read Humidity for Sensor 1
  humidity = dht1.readHumidity();
  avg_humidity = humidity;

//    Serial.print("dht1 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht1 Humidity \t");
//    Serial.print(humidity);
 
  //Read Temperature from Sensor 2 and add to average
    oventemp = dht2.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read Humidity from Sensor 2
    humidity = dht2.readHumidity();
    avg_humidity = avg_humidity + humidity;

//    Serial.print("\t dht2 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht2 Humidity \t");
//    Serial.print(humidity);
    
    // Read Temperature from Sensor 3 and add to average
    oventemp = dht3.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read humidity
    humidity = dht3.readHumidity();
    avg_humidity = avg_humidity + humidity;
   
//    Serial.print("\t dht3 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht3 Humidity \t");
//    Serial.print(humidity);

    // Read Temperature from Sensor 4 and add to average
    oventemp = dht4.readTemperature();  //true for F, blank for C
    avg_oventemp = avg_oventemp + oventemp;
  //Read Humidity from Sensor 4
    humidity = dht4.readHumidity();
    avg_humidity = avg_humidity + humidity;
    
//    Serial.print("\t dht4 Temp \t");
//    Serial.print(oventemp);
//    Serial.print("dht4 Humidity \t");
//    Serial.print(humidity);

  avg_oventemp = avg_oventemp/4;
  avg_humidity = avg_humidity/4;

  
  
  //                  "01234567890123456789"
  I2C_String_XY(5,0,F("LIVE FEED:"));
  I2C_String_XY(0,1,F("Temperature:       C"));
  I2C_show_dec3_XY(15,1,avg_oventemp);  
  I2C_String_XY(0,2,F("Humidity:          %"));  
  I2C_show_dec3_XY(16,2,avg_humidity);
  I2C_String_XY(0,3,F("Speed:         0 RPM"));
  //I2C_show_dec3_XY(13,3,i2);
  I2C_String_XY(18,1,F("\x1E\x01\x80"));
 

       Temperature();
       Humidity();
       RPM();
} 
