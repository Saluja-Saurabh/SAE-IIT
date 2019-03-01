#include <IFCT.h>

//Global vairables 
CAN_message_t RX_msg, TX_msg;
int motor_speed = 0;


void setup() 
{
  pinMode(13, OUTPUT); // On board LED to know if code is running 
  digitalWrite(13,HIGH);
  Can1.setBaudRate(250000);
  Can1.enableFIFO();
}

void loop() 
{
  read_can();
  Serial.println(motor_speed);
}

void write_speed(int m_speed, bool m_direction) // Max torque speed is 120 NM || 0 = Clockwise  1 = CounterClockwise
{
  (m_speed >860) ? m_speed = 860 : 1; 
  int percent_speed = map(m_speed,0,860,0,1200); // Converts analog to motor values (NM) || 120NM = 1200 in Code
  if ((percent_speed < 1200) && (percent_speed > 0)) // Checks if negative or above 120 NM
  {
    //Calculations value = (high_byte x 256) + low_byte
    byte low_byte = percent_speed % 256;
    byte high_byte = percent_speed / 256;
    
      //Setting up sending data parameters
    TX_msg.ext = 0;
    TX_msg.id = 0x0C0; // Command message ID
    TX_msg.len = 8;
    TX_msg.buf[0] = low_byte; // NM
    TX_msg.buf[1] = high_byte;
    TX_msg.buf[2] = 0; // Speed
    TX_msg.buf[3] = 0;
    TX_msg.buf[4] = m_direction; // Direction
    TX_msg.buf[5] = 1; // Inverter enable byte
    TX_msg.buf[6] = 0; // Last two are the maximum torque values || if 0 then defualt values are set
    TX_msg.buf[7] = 0;
    
    Can1.write(TX_msg);
  }
  
 else
 {
  Serial.println("Exceeding max torque value within write speed function.");
 }
  
}

void read_can()
{
  //canSniff(RX_msg); // Check data
  if ( Can1.read(RX_msg)) 
  {
    //canSniff(RX_msg); // Check data 
    read_motor_speed();
  }
}

void read_motor_speed()
{
  if (RX_msg.id == 0xA5) // ID of motor array
    {
      
      if(RX_msg.buf[3] < 128)
      {
        motor_speed = (RX_msg.buf[3] * 255) + RX_msg.buf[2];
      }
      else if(RX_msg.buf[3] > 128)
      {
        motor_speed = map((RX_msg.buf[3] * 255) + RX_msg.buf[2],65280,32640,0,-32640);
      }
    }
}



void canSniff(const CAN_message_t &RX_msg) // Reads all data
{
  Serial.print("MB "); Serial.print(RX_msg.mb);
  Serial.print("  LEN: "); Serial.print(RX_msg.len);
  Serial.print(" EXT: "); Serial.print(RX_msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(RX_msg.rtr);
  Serial.print(" TS: "); Serial.print(RX_msg.timestamp);
  Serial.print(" ID: "); Serial.print(RX_msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < RX_msg.len; i++ ) {
    Serial.print(RX_msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
