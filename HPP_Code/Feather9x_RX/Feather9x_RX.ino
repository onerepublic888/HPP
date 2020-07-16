#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define LED 13

long cnt = 0;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  Serial.println("Feather LoRa RX Test!");

  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  digitalWrite(LED, LOW);
  rf95.setTxPower(23, false);

}
float comdata;
void loop()
{
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    long* long_buf = (long*)buf;

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      Serial.print("CNT: "); Serial.println(cnt);
//      Serial.print("BUF Len: "); Serial.println(len);
      //      RH_RF95::printBuffer("Received: ", buf, len);
      //      Serial.print("Got: ");Serial.println(*((long*)buf));
      //      Serial.print("Got: ");Serial.println(*((long*)(buf+1)));
      //      Serial.print("Got: ");Serial.println(*(((long*)buf)+1));

      if (len > 20) {
        Serial.print("Got UWB: "); Serial.print(long_buf[0]);
        Serial.print(";"); Serial.print(long_buf[1]);
        Serial.print("mm;"); Serial.print(long_buf[2]);
        Serial.print("mm;"); Serial.print(long_buf[3]);
        Serial.print("mm;"); Serial.print(long_buf[4]);Serial.println("mm;");

        Serial.print("RSSI: "); Serial.print(rf95.lastRssi(), DEC);Serial.println(" dBm");
        delay(10);
      }
      else if (len < 20) {
        Serial.print("Got MAC: "); Serial.println((char*)buf);
        Serial.print("RSSI: "); Serial.print(rf95.lastRssi(), DEC);Serial.println(" dBm");
        delay(10);
      }
      else {
        Serial.print("Got: "); Serial.print(long_buf[0]);
        Serial.print(';'); Serial.print(long_buf[1]);
        Serial.print(';'); Serial.print(long_buf[2]);
        Serial.print(';'); Serial.print(long_buf[3]);
        Serial.print(';'); Serial.println(long_buf[4]);

        Serial.print("RSSI: "); Serial.print(rf95.lastRssi(), DEC);Serial.println(" dBm");
        delay(10);
      }
      digitalWrite(LED, LOW);
    }
  cnt += 1;
  }
  else
  {
    //      Serial.println("rf95 not available");
  }
}
