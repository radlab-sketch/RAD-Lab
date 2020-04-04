#include <SoftwareSerial.h>

SoftwareSerial windSerial(A0, A1);

void setup() {
  Serial.begin(38400);
  windSerial.begin(38400);
  Serial.print("Wind sensor initialize");
  delay(3000);
  // SET communication interface of the sensor to UART
  windSerial.write("$//CIU*//\r\n");
  delay(2000);
  // QUERY to check if the communication protocol is set or not
  windSerial.write("$//CI?*//\r\n");
  delay(1000);
//  windSerial.write("$//BR0*//\r\n");
//  delay(500);
  windSerial.write("$//BR?*//\r\n");
  delay(400);
//  windSerial.write("$//RSU*//\r\n");
}

void loop() {
  get_windspeed();
  delay(2000);
}

unsigned long t_prev, t_now = 0;
void get_windspeed() {
  char data[26];
  char ch;
  int i = 0, index = 0;
  float get_speed = 0.0;
  float direction_datum = 0.0;
  t_now = millis();
  windSerial.write("$//WV?*//\r\n");
  //  delay(500);
//  if ( t_now - t_prev < 500) {
    while  ( windSerial.available())
    {
      ch = windSerial.read();
      data[i++] = ch - 48; //convert ascii to numeric value
    }
    if ( data[18] == 0)   // Check error bit if 0 then no error
    {
      // extract speed from sensor data packet
      get_speed = data[9] * 10 + data[10] + data[12] * 0.1;
      direction_datum = data[14] * 100 + data[15] * 10 + data[16];
      Serial.print("Speed :");
      Serial.println(get_speed, 1);
      Serial.print("Direction :");
      Serial.println(direction_datum);
            uint8_t packet[2 + sizeof(float) + sizeof(float)];
            // uint8_t packet[2 + sizeof(long)];
            int k = 0;
            packet[0] = 1;
            packet[1] = sizeof(float) + sizeof(float);
            memcpy(&packet[2], &get_speed, sizeof(float));
            memcpy(&packet[6], &direction_datum, sizeof(float));
//            for (int  k = 0 ; k < 10 ; k ++) {
//              mySerial.write(packet[k]);
//              Serial.print(packet[k]);
//            }
    }
    t_prev = t_now;
  }
//}
