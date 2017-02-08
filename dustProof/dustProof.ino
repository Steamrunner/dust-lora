/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  This sketch uses the SM-PWM-01A dust sensor to measure particles.
  The sensor has two outputs:
    P1 for particles > 1 micron
    P2 for particles > 3 micron
    PM2.5 = P1 - P2

  Note: Many examples use "duration = duration + pulseIn(pin, LOW);"
  But pulseIn() cannot be used at the same time on two pins,
  we'll have to write our own pulseIn() substitute.

  I2C HTU21D sensor is used for measuring temperature and humidity
  There is a link between temperature, humidity and particulate matter

  The circuit:
    Pins 2 & 3: SM-PWM-01A 5V Connector: NC-P1-5V-P2-GND
    P1 and P2 are pulled up to 5V in the SM-PWM-01A with a 10K resistor.
    For a 3.3V MCU a 18K pulldown resistor is used.
    The I2C sensors are connected to SDA - A4 and SCL - A5

  Created 8th of February 2016
  By Luc Janssens
  Modified 7th of February 2017
  By Michael Smith

  https://dustproof.be

*/

// Include necessary libraries
#include <Wire.h>                                   // for I2C
#include <SparkFunHTU21D.h>                         // for HTU21D

// Constants
const unsigned long sampleTime = 60000;             // Length of the particle sampling period (60000ms = 60s)
const int pinP1 = 3;                                // Dust sensor P1 connected to digital pin 8
const int pinP2 = 2;                                // Dust sensor P2 connected to digital pin 9

// Variables
unsigned long seconds;                              // Time the MCU is running (in seconds = millis()/1000 )
int stateP1 = HIGH;                                 // P1 dust detected? LOW = yes - HIGH = no
int stateP2 = HIGH;                                 // P2 dust detected? LOW = yes - HIGH = no
unsigned long startP1;                              // P1 pulse duration start
unsigned long startP2;                              // P2 pulse duration start
volatile unsigned long lpoP1;                       // Duration of time in sampling period during which
                                                    // P1 particles are detected,
                                                    // aka Low Pulse Occupancy (lpo)
volatile unsigned long lpoP2;                       // Low Pulse Occupancy for P2
volatile int nP1 = 0;                               // Number of P1 reads
volatile int nP2 = 0;                               // Number of P2 reads
float ratioP1;                                      // % of time a P1 particle is detected during the sampling period
float ratioP2;                                      // % of time a PM10 particle is detected during the sampling period
float ratioPM25;                                    // % of time a P2.5 particle is detected during the sampling period

// HTU21D
float humidity;
float temperature;

// LoRa
const char loraDevAddr[] = "SET_YOUR_OWN";
const char loraAppsAndNwksKey[] = "SET_YOUR_OWN";
unsigned int loraSeqId = 0;

// Objects
HTU21D myHumidity;                                  // for HTU21D

// Helper functions
/*
  The dec2Hex routine converts decimal values to a 2 bytes fixed width
  HEX representation.
*/
String dec2Hex(unsigned int decValue, byte desiredStringLength = 4) {
  String hexString = String(decValue, HEX);
  while (hexString.length() < desiredStringLength) hexString = "0" + hexString;

  return hexString;
}

/*
  The setup routine runs once when you press reset or turn the power on.
  It initializes the serial connection, configures the pins and records
  the start time.
*/
void setup() {
  // HTU21D
  myHumidity.begin();

  // initialize Serial communication at 57600 bits per second:
  Serial.begin(57600);
  Serial.setTimeout(5000);

  // Initialize the LoRa transmitter:
  configureLora();

  // Allow P1 and P2 to trigger interrupt
  attachInterrupt(digitalPinToInterrupt(pinP1), measureStateChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinP2), measureStateChange, CHANGE);

  delay(sampleTime);                                // wait for first sample
}

/*
  The loop routine runs over and over again forever. During the sampling period
  it measures the pulses sent by the dust sensor on P1 and P2.
  At the end of each sampling period it calculates the low pulse occupancy
  ratios (lpo) and sends the data depending on the user's dataHandler configuration.
*/
void loop() {
  // calculate P1 ratio
  ratioP1 = 0.1 * float(lpoP1) / float(sampleTime);

  // calculate P2 ratio (PM10)
  ratioP2 = 0.1 * float(lpoP2) / float(sampleTime);

  ratioPM25 = ratioP1 - ratioP2;                    // PM2.5 = P1- PM10

  // save the time:
  seconds = millis() / 1000;

  // HTU21D
  humidity = myHumidity.readHumidity();
  temperature = myHumidity.readTemperature();

  data2Lora();

  // reset P1 and P2 variables for the next sampling period:
  lpoP1 = 0;
  lpoP2 = 0;
  nP1 = 0;
  nP2 = 0;

  // start next sampling period:
  delay(sampleTime);
}

/*
  The measureStateChange routine is called by a hardware interrupt on pin 2
  or pin 3 whenever the state of P1 or P2 changes.
  It determines the type of change (low to high or high to low) and takes the
  appropriate action.
*/
void measureStateChange() {
  unsigned long us = micros();

  // determine which pins changed state and take appropriate action:
  if (stateP1 != digitalRead(pinP1)) {              // P1 has changed
    if ((stateP1 = digitalRead(pinP1)) == LOW) {    // a new pulse has started
      startP1 = us;                                 // start timing the pulse
      nP1++;                                        // increment number of P1 particles read
    } else {                                        // a pulse has ended
      lpoP1 += us - startP1;                        // calculate the duration
    }
  }

  if (stateP2 != digitalRead(pinP2)) {              // P2 has changed
    if ((stateP2 = digitalRead(pinP2)) == LOW) {    // a new pulse has started
      startP2 = us;                                 // start timing the pulse
      nP2++;                                        // increment number of P2 particles read
    } else {                                        // a pulse has ended
      lpoP2 += us - startP2;                        // calculate the duration
    }
  }
}

/*
  The data2Lora routine sends the measurements in hexadecimal format to the
  Lora network, through the serial link to the Lora transmitter.
*/
void data2Lora() {
  int ratioP1Int = (ratioP1 + 0.005) * 100;         // convert P1 ratio to an integer
  int ratioP2Int = (ratioP2 + 0.005) * 100;         // convert PM10 ratio to an integer
  int ratioPM25Int = (ratioPM25 + 0.005) * 100;     // convert PM2.5 ratio to an integer
  int temperatureInt = (temperature + 0.05) * 10;   // convert temperature to integer
  int humidytInt = (humidity + 0.05) * 10;          // convert humidity to integer

  Serial.print("mac tx uncnf 1 ");                  // Start Lora transmit command
  Serial.print(dec2Hex(loraSeqId));                 // Add a sequence id
  Serial.print(dec2Hex(ratioP1Int));                // print P1 integer in hex to serial
  Serial.print(dec2Hex(ratioP2Int));                // print P2 integer in hex to serial
  Serial.print(dec2Hex(ratioPM25Int));              // print PM25 integer in hex to serial
  Serial.print(dec2Hex(nP1));                       // print number of P1 reads in hex to serial
  Serial.print(dec2Hex(nP2));                       // print number of P2 reads in hex to serial
  Serial.print(dec2Hex(temperatureInt));            // print temperature integer in hex to serial
  Serial.println(dec2Hex(humidytInt));              // print humidity integer in hex to serial
  Serial.find("mac_tx_ok");

  loraSeqId++;                                       // Increment lora sequence id.
}

/*
  The configureLora routine configures and prepapres the Lora module for
  transmitting data to the Lora network.
*/
void configureLora() {
  Serial.print("mac set appskey ");
  Serial.println(loraAppsAndNwksKey);
  Serial.find("ok");

  Serial.print("mac set nwkskey ");
  Serial.println(loraAppsAndNwksKey);
  Serial.find("ok");

  Serial.println("mac set adr off");
  Serial.find("ok");

  Serial.println("mac set ch dcycle 0 9");
  Serial.find("ok");

  Serial.println("mac set ch dcycle 1 9");
  Serial.find("ok");

  Serial.println("mac set ch dcycle 2 9");
  Serial.find("ok");

  Serial.print("mac set devaddr ");
  Serial.println(loraDevAddr);
  Serial.find("ok");

  Serial.println("mac set dr 0");
  Serial.find("ok");

  Serial.println("mac set pwridx 1");
  Serial.find("ok");

  Serial.println("mac save");
  Serial.find("ok");

  Serial.println("mac join abp");
  Serial.find("ok");

  Serial.println("radio set pwr 15");
  Serial.find("ok");
}

