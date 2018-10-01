/*
   Rat Trap Sensor Receiver
    Lo-Ra enabled rat trap sensor.
    Designed for a Arduino Pro Mini (3.3V, 8Mhz) attached to
    Lo-Ra radio via SPI.
    Receives a signal from the sensor on the rat trap when the trap is
    triggered.

    Pin Assignments:
     3 -> LoRa DOI0
     4 -> SoftSerial TX (to pin D1 on Wemos D1)
     9 -> LoRa Reset
    10 -> LoRa Clock Select (NSS)
    11 -> LoRa MISO
    12 -> LoRa MOSI
    13 -> LoRa SCK



    Quentin McDonald
    July 2018
*/

#include <stdlib.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/* Pin definitions: */
#define RX 5
#define TX 4
SoftwareSerial soft_serial = SoftwareSerial(RX, TX);


// Send a string value via Serial to the D1
bool send_string( const String& str)
{

  // Send the length of the string first:
  int len = str.length();

  Serial.print("Sending data of length = ");
  Serial.println(len);
  soft_serial.write((char)len);

  // Now send the bytes individually:
  for ( int i = 0; i < len; i++ ) {
    soft_serial.write(str[i]);
  }

  soft_serial.flush();

}

// Configure the LoRa radio
void setupLoRa() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Initializing LoRa radio");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);


  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Set to slow speed for longer range
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
}

void setup()
{

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  soft_serial.begin(9600);


  while (!Serial);
  Serial.begin(9600);
  delay(500);

  setupLoRa();

}

void loop()
{

  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      delay(1000);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      float rssi = rf95.lastRssi();
      Serial.println(rssi, DEC);

      // If it's a test message then send a differnt reply as it's probably
      // from the range finder:
      if ( strncmp(buf, "Test:", 5) == 0 ) {
        Serial.println("Test message");
        // Send a reply
        char data[32];
        memcpy(data, "Got: ", 5 );
        memcpy(data + 5, buf + 5, sizeof(data) - 5);
        data[10] = ':';
        data[11] = ' ';
        dtostrf( rssi, 5, 1, data+12);
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent a reply to test:");
        Serial.println(data);

      } else {

        // Send a reply
        uint8_t data[] = "OK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");

        // Send the data to the Wemos D1 which will generate an Email
        send_string( buf );
      }


    } else {
      Serial.println("Receive failed");
    }



  }
}
