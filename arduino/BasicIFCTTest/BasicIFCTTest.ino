#include <IFCT.h>

CAN_message_t TX_msg, RX_msg;

void setup()
{
  pinMode(13, OUTPUT);
  Can0.setBaudRate(500000);
  // Can0.begin();
  Can0.enableFIFO();
  pinMode(13, OUTPUT);

  TX_msg.ext = 0;
  TX_msg.id = 0xF02;
  TX_msg.len = 8;
  TX_msg.buf[0] = 44;
  TX_msg.buf[1] = 53;
  TX_msg.buf[2] = 0;
  TX_msg.buf[3] = 0;
  TX_msg.buf[4] = 31;
  TX_msg.buf[5] = 65;
  TX_msg.buf[6] = 0;
  TX_msg.buf[7] = 0;
}

//Make the led blip
void blip(int mili = 20)
{
  digitalWrite(13, HIGH);
  delay(mili);
  digitalWrite(13, LOW);
  delay(mili);
}

void loop()
{
  Can0.write(TX_msg);
  if (Can0.read(RX_msg))
  {
    blip();
    canSniff(RX_msg);
  }
}

void canSniff(const CAN_message_t &RX_msg)
{
  Serial.print("MB ");
  Serial.print(RX_msg.mb);
  Serial.print("  LEN: ");
  Serial.print(RX_msg.len);
  Serial.print(" EXT: ");
  Serial.print(RX_msg.flags.extended);
  Serial.print(" REMOTE: ");
  Serial.print(RX_msg.rtr);
  Serial.print(" TS: ");
  Serial.print(RX_msg.timestamp);
  Serial.print(" ID: ");
  Serial.print(RX_msg.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < RX_msg.len; i++)
  {
    Serial.print(RX_msg.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
