#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);  // D10 TX ,D11 RX UWB software serial
//Uart Serial3 (&sercom4, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0); //  GPS , SDA SCL

#define LED 13
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RF95_FREQ 921.0
TinyGPSPlus gps;

String uwb_str = "";
String gps_str = "";
String comdata = "";
String str0, str1, str2, str3, str4, str5;
volatile char chr5 ;

int m = 0;
long uwb_data[10] = { 0 };
long gps_data[10] = { 0 };

uint8_t pollingID;
int ID[6] = {1, 2, 3, 4};
int ID_ACK[6] = { 11, 22, 33, 44};
int16_t data[2];

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
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
  Serial2.begin(115200); // UWB Read
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
  //  Serial.println("===============Start Loop!==============="); Serial.println();
  handle_uwb_str(); //Serial.println();
//  handle_gps_str(); //Serial.println();
  get_mac_send_switch(); //Serial.println();
  //  polling_fun(); Serial.println();

}

void handle_uwb_str()
{
  if (Serial2.available())
  {
    Serial.println("=======Get UWB data=========");
    comdata = Serial2.readStringUntil('\n');
    if (comdata[0] == 'm')
    {
      digitalWrite(LED, HIGH);
      m = comdata.indexOf(' ') - 2; //找 m 是第几个字节
      str0 = comdata.substring(m + 6, m + 14);
      //Serial.print("T-A0:");Serial.println(str0 + " ");
      long A0 = strtol(str0.c_str(), NULL, 16); //HexString conver to 10's

      str1 = comdata.substring(m + 15, m + 23);
      // Serial.print("T-A1:");Serial.println(str1 + " ");
      long A1 = strtol(str1.c_str(), NULL, 16); //HexString convert to 10's

      str2 = comdata.substring(m + 24, m + 32);
      //Serial.print("T-A2:");Serial.println(str2 + " ");
      long A2 = strtol(str2.c_str(), NULL, 16);

      str3 = comdata.substring(m + 33, m + 41);
      //Serial.print("T-A3:");Serial.println(str3 + " ");
      long A3 = strtol(str3.c_str(), NULL, 16);

      str4 = comdata.substring(m + 60, m + 61); // tag_num
      long tag_num = strtol(str4.c_str(), NULL, 16);

      str5 = comdata.substring(m + 59, m + 60); // anchor or tag
      chr5 = str5[0];
      uwb_data[0] = (A0); uwb_data[1] = (A1); uwb_data[2] = (A2); uwb_data[3] = (A3);

      rf95.send((uint8_t*)uwb_data, sizeof(uwb_data));
      Serial.print(uwb_data[0]); Serial.print(';'); Serial.print(uwb_data[1]); Serial.print(';');
      Serial.print(uwb_data[2]); Serial.print(';'); Serial.println(uwb_data[3]);
      Serial.print("UWB type: "); Serial.println(chr5);

      Serial1.print(comdata); Serial1.print("; ");Serial.println("Send All UWB data ");
//      Serial1.print(uwb_data[0]); Serial1.print(';'); Serial1.print(uwb_data[1]); Serial1.print(';');
//      Serial1.print(uwb_data[2]); Serial1.print(';'); Serial1.print(uwb_data[3]);
      long uwb_data[10] = { 0 };
      digitalWrite(LED, LOW);
    }
    else {
      comdata = "";
    }
  }
  else {
    //    Serial.println("Get UWB data failed!");
    delay(5);
  }
  //  while (Serial2.read() >= 0)
  //  {} //清空串口缓存
}

void polling_fun() {
  if (rf95.available())
  {
    delay(10);
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    //Serial2.write('AT+SW=10010000\r\n')(a0)AT+SW=10010010\r\n(a1)('AT+SW=10000000')(t0)
    if (rf95.recv((uint8_t*)buf, &len))
    {
      delay(10);
      pollingID = buf[0];
      switch (pollingID) {
        case 1:
          Serial2.write("AT+SW=10010010\r\n");    //tag1
          Serial.println("Tag1 change to Anchor1 !");
          rf95.send((uint8_t*)ID_ACK[0], sizeof(ID_ACK[0]));
          rf95.waitPacketSent();
          break;

        case 2:
          Serial2.write("AT+SW=10010100\r\n");    //tag2
          Serial.println("Tag2 change to Anchor2 !");
          rf95.send((uint8_t*)ID_ACK[1], sizeof(ID_ACK[1]));
          rf95.waitPacketSent();
          break;

        case 3:
          Serial2.write("AT+SW=10010110\r\n");    //tag3
          Serial.println("Tag3 change to Anchor3 !");
          rf95.send((uint8_t*)ID_ACK[2], sizeof(ID_ACK[2]));
          rf95.waitPacketSent();
          break;

        case 11:
          Serial.println("Got ACK from Anchor1 !");
          break;

        case 22:
          Serial.println("Got ACK from Anchor2 !");
          break;

        case 33:
          Serial.println("Got ACK from Anchor3 !");
          break;

      }
    }
    else
    {
      //      Serial.println("Receive failed");
    }
  }
}

void get_mac_send_switch() {
  if (Serial1.available()) {
    digitalWrite(LED, HIGH);
    delay(20);
    String received = "";
    String value1, value2;

    while (Serial1.available()) {
      char tmp = (char)Serial1.read();
      received += tmp;
    }
    //    Serial.println();  Serial.println(received); Serial.println();//received.toCharArray(rec_char, 20);

    for (int i = 0; i < received.length(); i++) {
      if (received.substring(i, i + 1) == ",") {
        value1 = received.substring(0, i);
        value2 = received.substring(i + 1);
        break;
      }
    }
    Serial.println("=========received MAC address ============== "); Serial.println(value1); Serial.println();
    //    Serial.print("UWB type: "); Serial.println(chr5); Serial.println();
    if (value2[1] == 'o' ) {
      Serial.print("received value2 : "); Serial.println(value2); Serial.println();
//      anchor_tag_switch(value2[0], value2[1]);
    }
    else {
      //      Serial.println("value2 not 1o or is anchor now, can't switch to anchor!");
    }

    int _size = value1.length();
    uint8_t *mac_data = (uint8_t *) malloc((_size + 1) * sizeof(uint8_t));
    memset(mac_data, 0, sizeof(mac_data));
    for (int i = 0; i < _size; i++) {
      mac_data[i] = (uint8_t)value1[i];
    }
    mac_data[_size] = 0;

    rf95.send(mac_data, _size + 1);
    rf95.waitPacketSent(10);

    digitalWrite(LED, LOW);
  }
  else {
    //    Serial.println("Get mac failed!");
    delay(5);
  }
}

void anchor_tag_switch(uint8_t x, uint8_t y) {    //x,y => 1o or 2o or 3o or 4o
  int16_t data[2];    // [0] = ID, [1] = Response???
  Serial.println("Start switch Tag to Anchor!");
  switch (x) {
    case 1:
      Serial.println("LoRa send 1!");
      rf95.send(&x, sizeof(x));
      rf95.waitPacketSent();
      break;

    case 2:
      Serial.println("LoRa send 2!");
      rf95.send(&x, sizeof(x));
      rf95.waitPacketSent();
      break;

    case 3:
      Serial.println("LoRa send 3!");
      rf95.send(&x, sizeof(x));
      rf95.waitPacketSent();
      break;

    case 4:
      Serial.println("LoRa send 4!");
      rf95.send(&x, sizeof(x));
      rf95.waitPacketSent();
      break;

  }
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
