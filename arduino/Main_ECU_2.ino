#include <IFCT.h>


CAN_message_t RX_msg, TX_msg;
// Pins data


// ID'S
byte ID_temp_1 = 0x0A0;
byte ID_temp_2 = 0x0A1;
byte ID_temp_3 = 0x0A2;
byte ID_motor_poition = 0x0A5;
byte ID_current = 0x0A6;
byte ID_voltage = 0x0A7;
byte ID_faults = 0x0AB;

struct Motor_controller_CAN_data
{
  int temp_phase_A;
  int temp_phase_B;
  int temp_phase_C;
  int temp_driver_board;

  int temp_control_board;

  int sensor_angle;
  int angular_velocity;
  int electrical_frequncy;

  int current_PA;
  int current_PB;
  int current_PC;
  int current_DC;

  int voltage_DC;
  int voltage_output;
  int voltage_AB;
  int voltage_BC;

  byte faults[8][8];
  
}motor_0,motor_1;

char *faults_decoder[8][8] = 
{
  {
    "Harawre Gate/Desaturation Fault",
    "HW over-current Fault",
    "Accelerator shorted",
    "Accelerator Open",
    "Current sensor Low",
    "Current sensor High",
    "Module Tempature Low",
    "Module Tempature High"
  },
  {
    "Control PCB Temperature Low",
    "Control PCB Temperature High",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
  {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    ""
  },
};


void setup() 
{
  pinMode(13, OUTPUT); // On board LED to know if code is running 
  digitalWrite(13,HIGH);
  Can1.setBaudRate(250000);
  Can1.enableFIFO();
  Can0.setBaudRate(250000);
  Can0.enableFIFO();
}

void loop() 
{
  read_can();
}

void write_speed(int m_speed, bool m_direction) // Max torque speed is 100 NM || 0 = Clockwise  1 = CounterClockwise
{
  (m_speed >860) ? m_speed = 860 : 1; 
  int percent_speed = map(m_speed,0,860,0,1000); // Converts analog to motor values (NM) || 100NM = 1000 in Code
  if ((percent_speed < 1000) && (percent_speed > 0)) // Checks if negative or above 100 NM
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
    read_signed_data();
  }
  else if(Can0.read(RX_msg))
  {
    //canSniff(RX_msg); // Check data 
    read_signed_data();
  }
}

void read_signed_data()
{
  if (RX_msg.id == ID_motor_poition) // ID of motor array
    {
      
      if(RX_msg.buf[3] < 128)
      {
        motor_1.angular_velocity = (RX_msg.buf[3] * 255) + RX_msg.buf[2];
      }
      else if(RX_msg.buf[3] > 128)
      {
        motor_1.angular_velocity = map((RX_msg.buf[3] * 255) + RX_msg.buf[2],65280,32640,0,-32640);
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
