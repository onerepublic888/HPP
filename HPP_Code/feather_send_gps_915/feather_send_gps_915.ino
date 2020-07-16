#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

//Uart Serial3 (&sercom4, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0); //  GPS , SDA SCL

#define LED 13
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RF95_FREQ 915.0
TinyGPSPlus gps;

String uwb_str = "";
String gps_str = "";
String comdata = "";
String str0, str1, str2, str3, str4, str5;
volatile char chr5 ;

int m = 0;
long gps_data[10] = { 0 };
int16_t data[2];


void SERCOM3_Handler()
{
  Serial3.IrqHandler();
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  Serial1.begin(115200);  //rpi
  Serial3.begin(9600);  //GPS

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("FREQ FAIL");
    while (1);
  }
  Serial.print("Set Freq: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); // can't work ? Don't kwon why
  pinPeripheral(10, PIO_SERCOM); // Open Serial port
  pinPeripheral(11, PIO_SERCOM); // OPen Serial port
  pinPeripheral(20, PIO_SERCOM);
  pinPeripheral(21, PIO_SERCOM);
  Serial.println("Setup finish");

}

void loop() {
    handle_gps_str(); 

}

void handle_gps_str() {
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      displayInfo();
    
  delay(10);
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.println("============Get GPS data ============== ");
    digitalWrite(LED, HIGH);//    String GPShd = "GPS";long GPS_hd = strtol(GPShd.c_str(), NULL, 16);
    gps_data[0] = gps.speed.mps(); gps_data[1] = gps.location.lat() * 10000000;
    gps_data[2] = gps.location.lng() * 10000000; gps_data[3] = gps.altitude.meters() * 10; gps_data[4] = gps.satellites.value();
    Serial.print(gps_data[0]); Serial.print(";"); Serial.print(gps_data[1]); Serial.print(";");
    Serial.print(gps_data[2]); Serial.print(";"); Serial.print(gps_data[3]); Serial.print(";"); Serial.println(gps_data[4]);

    rf95.send((uint8_t*)gps_data, sizeof(gps_data));
    digitalWrite(LED, LOW);
  }
  else {
    Serial.println("Numbers of GPS statelite not enough!");
    
  }
}
