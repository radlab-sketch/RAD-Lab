# Arduino Interface with FT205EV Ultrasonic Anemometer
​
The FT205EV is the first in a new generation of lightweight ultrasonic wind sensors. It is low weight(100g) and a wind speed range up to 75m/s. It has a graphite and nylon composite body. The light weight of the FT205EV together with the proven FT Acu-Res technology make it ideal for use on aerial drones and other weight critical applications. The FT205EV allows configurable RS422 (full-duplex), RS485 (half-duplex) or buffered UART (full-duplex) communication outputs.  
​
The following instructions guide the reader to setting up the circuit connections between a Raspberry Pi, Arduino, and a FT205EV ultrasonic anemometer. 
​
## Hardware Requirements
​
- FT205EV Ultrasonic Anemometer
- Ardunio Uno
- Raspberry Pi
- Power supply 6-30VDC
​
​
​
## Circuit Diagram
​
The circuit diagram to connect the Arduino with the ultrasonic anemometer is shown here
​
​
The circuit diagram to connect the RaspberryPi to the arduino and anemometer is as follows
​
​
​
​
## Getting Started
​
Setup the circuit according to the given circuit diagram. Compile and upload the Arduino file SoftwareSerial_windSensor.ino  under the folder UART_to_WindSensor, to the Arduino. This example is used to test that the ultrasonic anemometer is functioning and the Arduino is able to read the data from the sensor. 
​
The Arduino is used to send commands to the sensor using UART interface. The data sent by the sensor is displayed on the serial monitor on the computer. We make pins 10 and 11 software serial pins to communicate with the sensor since the Arduino serial pins 0 and 1 (hardware serial) are used to display data on the serial monitor on the computer. We give enough delay between the commands since the sensor takes minimum 400ms to process a command. The Arduino sends a command to the sensor to set the communication interface to UART and then we send a Query to check if the communication interface has been set. We then send a query to the sensor for the wind speed and direction every 1 second. The sensor responds with ASCII messages in the format as discussed in the Tutorial-WindSensor-Arduino.pdf.   

​
## Authors
​
* **Deepan Lobo** 
​
* **Pramod Abichandani**
​
## License
​
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
​
