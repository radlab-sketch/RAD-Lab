#define MCU_RX A0    // Yellow wire is wind sensor UART TX (Pin 8)
#define MCU_TX A1    // Green wire is wind sensor UART RX  (Pin 7)
#define LED_DEBUG 13
#define MIN_PACKET_SIZE 2

#include <SoftwareSerial.h>
#include <DRV8835MotorShield.h>

#include "RotaryEncoders.h"

extern "C" {
#include <pid.h>
}

DRV8835MotorShield motors;

static lrad_pid_t pid1;
static lrad_pid_t pid2;

//setting the pins on the arduino for encoder output
Encoder encoder1(2, 4);  
Encoder encoder2(3, 5);

// Software Serial to comm. with wind sensor
SoftwareSerial windSerial(MCU_RX, MCU_TX);

// LED DEBUG constants
const long interval = 500;
int LED_STATE = LOW;
unsigned long previousMillis = 0;

// Header for command from raspi
typedef enum {
  SET_MOTOR_RPS, 
} header_command_t;

// headers for packet sent from arduino to raspi
typedef enum {
  HEADER_RSP_RPS,   // encoder data
  HEADER_RSP_SPEED, // wind sensor data
} header_response_t;

// buffer for packet payload filled before calling handle_packet()
static uint8_t payload_buffer[UINT8_MAX];

static double ff1;
static double ff2;
static double ff3;
static double ff4;

void setup() {
  // setting the Kp, Ki, Kd terms for our PID
  // currently set as a P controller since only Kp is set
  lrad_pid_init(&pid1, 0.5, 0, 0, 45, 5, NULL, false);
  lrad_pid_init(&pid2, 0.5, 0, 0, 45, 5, NULL, false);
  // setting LED connected to pin 13 as Output used for debugging the packets
  pinMode(LED_DEBUG, OUTPUT);
  //Hardware serial port is used to communicate with the raspi Pin0 and Pin1
  Serial.begin(115200); 
  windSerial.begin(38400);
  delay(2000);
  attachInterrupt(0, updateEncoder1, CHANGE);
  attachInterrupt(1, updateEncoder2, CHANGE);
  lrad_pid_update_set_point(&pid1, 0);
  lrad_pid_update_set_point(&pid2, 0);
  windSerial.write("$01CIU*//\r\n");
  delay(1000);
}
void updateEncoder1() {
  encoder1.update();
}
void updateEncoder2() {
  encoder2.update();
}
unsigned long prev_time, p_time = 0;
char data[26];
char ch;
int i = 0;
void loop() {

  unsigned long now_time = millis();
  if ( (now_time - prev_time) >= 400) {
    send_encoderPacket();
    prev_time = now_time;
  }
  unsigned long n_time = millis();
  if ( (n_time - p_time) >= 500) {
    req_windspeed();
    while (windSerial.available() > 0)
    {
      ch = windSerial.read();
      data[i] = ch; //convert ascii to numeric value I is incremented in check to prevent incrementing i too early
      if (data[i++] == '\n') {
        send_windspeed(data);
        feed_forward();
        i = 0;
      }
    }
    p_time = n_time;
  }

  get_packet();
  pid_motor1();
  pid_motor2();
  
  delay(25);
}




void get_packet() {
  // all packets have the format: [header, payload_length, ...payload...]
  // once there are two bytes available, read the header and length
  // Set motor packet will contain a payload
  // request packet contain only header and payload length and no payload the packet is [0, 0]
  if (Serial.available() >= 2) {
    header_command_t header = (header_command_t)Serial.read();
    //    Serial.print(header);
    if ( header == SET_MOTOR_RPS) {
      uint8_t len = Serial.read();
      //    Serial.print("length of payload: ");
      //    Serial.print(len);
      if (len == 2) {
        packet_ok(); // blink led to show packet received ok
        while ( Serial.available() < len);

        for (uint8_t i = 0; i < len; i++) {
          payload_buffer[i] = Serial.read();
        }
        //      Serial.println("the payload is");
        //      for (uint8_t i = 0; i < len; i++) {
        //        Serial.print(payload_buffer[i]);
        //      }
      }
      // handle the received packet
      handle_packet(header, len, payload_buffer);
    }
  }
}
void packet_ok() {
  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( LED_STATE == LOW) {
      LED_STATE = HIGH;
    } else {
      LED_STATE = LOW;
    }
    digitalWrite(LED_DEBUG, LED_STATE);
  }
}
int set_point1, set_point2 = 0;
void handle_packet(header_command_t header, uint8_t len, uint8_t *payload) {
  int i = 0;

  int num = 0;
  //  Serial.print("motor number: ");
  //  Serial.println(payload[0]);
  switch (header) {
    case SET_MOTOR_RPS: {
        if ( len == sizeof(int16_t) ) {
          if ( payload[0] == 1) {
            memcpy(&set_point1, &payload[1], sizeof(int16_t));
            //            Serial.print("set point1: ");
            //            Serial.println(set_point1);
            lrad_pid_update_set_point(&pid1, set_point1);
          }

          if ( payload[0] == 2) {
            memcpy(&set_point2, &payload[1], sizeof(int16_t));
            //            Serial.print("set point2: ");
            //            Serial.println(set_point2);
            lrad_pid_update_set_point(&pid2, set_point2);
          }
        }
        break;
      }
  }
}
static unsigned long last_time1, last_time2;
static double last_revolutions1, last_revolutions2;
double pwm1, pwm2 = 0;

void pid_motor1()
{
  unsigned long now = millis();

  double ticks = encoder1.getPosition();
  double revolutions = ticks / 30.0; // why divided by 30 and not 60

  double dt = (now - last_time1) / 1000.0; // to compute change in time
  //    Serial.print("time I: ");
  //    Serial.println(dt);
  double revolutions_per_second = (revolutions - last_revolutions1) / dt;
  //    Serial.print("rps I: ");
  //    Serial.println(revolutions_per_second);

  double feedback = lrad_pid_compute(&pid1, revolutions_per_second, dt);
  //    Serial.print("set output I: ");
  //    Serial.println(output);
  pwm1 += feedback;

  if (pwm1 > 400) pwm1 = 400;
  if (pwm1 < -400) pwm1 = -400;

  double _pwm = pwm1 + ff1;

  motors.setM1Speed(_pwm);
  //  Serial.print("PWM I: ");
  //  Serial.println(pwm1);

  last_time1 = now;
  last_revolutions1 = revolutions;
  send_encoderData1( feedback, 1);

}


void pid_motor2()
{
  unsigned long now2 = millis();

  double ticks = encoder2.getPosition();
  double revolutions = ticks / 30.0; // why divided by 30 and not 60

  double dt = (now2 - last_time2) / 1000.0; // to compute change in time
  //  Serial.print("time II: ");
  //  Serial.println(dt);
  double revolutions_per_second = (revolutions - last_revolutions2) / dt;
  //  Serial.print("rps II: ");
  //  Serial.println(revolutions_per_second);


  double feedback = lrad_pid_compute(&pid2, revolutions_per_second, dt);
  //    Serial.print("set output I: ");
  //    Serial.println(output);
  pwm2 += feedback;

  if (pwm2 > 400) pwm2 = 400;
  if (pwm2 < -400) pwm2 = -400;

  double _pwm = pwm2 + ff3;

  motors.setM2Speed(_pwm);
  //  Serial.print("PWM II: ");
  //  Serial.println(pwm2);

  last_time2 = now2;
  last_revolutions2 = revolutions;

  send_encoderData2( feedback, 2);
}
uint8_t e_packet[3 + sizeof(float)];
void send_encoderData1( float outp, uint8_t motor_number) {
  //  for ( int k = 0 ; k < 7; k ++) {
  //    e_packet[k] = 0;
  //  }
  e_packet[0] = HEADER_RSP_RPS;
  e_packet[1] = 5;
  e_packet[2] = motor_number ;
  memcpy(&e_packet[3], &outp, sizeof(float));
}
uint8_t f_packet[3 + sizeof(float)];
void send_encoderData2( float outp, uint8_t motor_number) {
  //  for ( int k = 0 ; k < 7; k ++) {
  //    e_packet[k] = 0;
  //  }
  f_packet[0] = HEADER_RSP_RPS;
  f_packet[1] = 5;
  f_packet[2] = motor_number ;
  memcpy(&f_packet[3], &outp, sizeof(float));
}

void send_encoderPacket() {
  for ( int k = 0 ; k < 7; k ++) {
    Serial.write(e_packet[k]);
    //    Serial.print(e_pack[k]);
  }
    for (int k = 0 ; k < 7; k ++) {
    Serial.write(f_packet[k]);
    //    Serial.print(e_pack[k]);
  }
  
}
float get_speed = 0.0;
float direction_datum = 0.0;

uint8_t packet[2 + sizeof(float) + sizeof(float)];

void req_windspeed() {
  windSerial.write("$//WV?*//\r\n");
}

void send_windspeed(char *data) {
  for ( int j = 0 ; j <= i ; j ++) {
    data[j] = data[j] - 48;
  }
  get_speed = 0.0;
  direction_datum = 0.0;
  data[26];

  if ( data[18] == 0)   // Check error bit if 0 then no error
  {
    // extract speed from sensor data packet
    get_speed = data[9] * 10 + data[10] + data[12] * 0.1;
    direction_datum = data[14] * 100 + data[15] * 10 + data[16];
  }
  packet[0] = HEADER_RSP_SPEED;
  packet[1] = sizeof(float) + sizeof(float);
  memcpy(&packet[2], &get_speed, sizeof(float));
  memcpy(&packet[6], &direction_datum, sizeof(float));
  for (int  k = 0 ; k < 10 ; k ++) {
    Serial.write(packet[k]);
  }
}

//void feed_forward(){
//  
//  double theta = direction_datum * M_PI / 180.0 + M_PI;
//  double FF_GAIN = 150;
//  double mag = get_speed * FF_GAIN;
//
//  double x_component = mag * cos(theta) / 2;
//  double y_component = mag * sin(theta) / 2;
//
//  ff1 = -x_component;
//  ff2 = -y_component; 
//  ff3 = x_component;
//  ff4 = y_component;
//}
