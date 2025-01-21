//
//    FILE: mcp4725_voltage.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: test mcp4725 lib
//     URL: https://github.com/RobTillaart/MCP4725

#include "Wire.h"
#include "MCP4725.h"
#include <string.h>
#include <SPI.h>
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8

#define GONDOLA_RADIUS 0.2

#define RADIUS 1.37

#define GONDOLA_ANGLE(a_n, a_r) asin((-a_n + sqrt(a_n*a_n - (a_r*a_r+a_n*a_n)*(1-(a_r*a_r))))/(a_r*a_r + a_n*a_n))
#define GONDOLA_DISPLACE(a_n, a_r) GONDOLA_RADIUS * (-a_n + sqrt(a_n*a_n - (a_r*a_r+a_n*a_n)*(1-(a_r*a_r))))/(a_r*a_r + a_n*a_n) 

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

MCP4725 MCP(0x60);

volatile int x;
uint32_t start, stop;

const byte inputPin = 4;
boolean inputState = false;
boolean lastInputState = false;
long count = 0L;

boolean radioConnected = false;

float avgRPM = 0;
float numPoints = 0;

float payload[3] = {0, 0, 0};
uint8_t address[][6] = { "1Node", "2Node" };
float accelMag;

unsigned long previousCountMillis = millis();
const long countMillis = 500L;

void setup()
{
  pinMode(2, OUTPUT);           // Initialize pin 2 for motor controller 
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);           // motor controller enable break

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("MCP4725_VERSION: ");
  Serial.println(MCP4725_VERSION);

  Wire.begin();


  MCP.begin();
  //  calibrate max voltage
  MCP.setMaxVoltage(4.87);  

  pinMode(inputPin, INPUT);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }else{
    radioConnected = true;
    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!0]);
    radio.startListening();
  }

  Serial.print("\nVoltage:\t");
  Serial.println(MCP.getVoltage());
  Serial.println();
  digitalWrite(2, LOW);

  delay(1000);
  digitalWrite(2, HIGH);

  Serial.println();
}

#define MAXLENGTH 128

char inputt = 0;
char breakOn = 0;
char cbreakOn = 0;
float i = 0;
String f = "";
void loop() {
  if(radioConnected){
    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      // Serial.print(F("Received "));
      // Serial.print(bytes);  // print the size of the payload
      // Serial.print(F(" bytes on pipe "));
      // Serial.print(pipe);  // print the pipe number
      // Serial.print(F(": "));
      // Serial.print(payload[0]);  // print the payload's value
      // Serial.print(" ");  // print the payload's value
      // Serial.print(payload[1]);  // print the payload's value
      // Serial.print(" ");  // print the payload's value
      // Serial.println(payload[2]);  // print the payload's value
      accelMag = sqrt(payload[0]*payload[0] + payload[1]*payload[1] + payload[2]*payload[2]);
    }
  }
  inputState = digitalRead(inputPin);
  // count every transision HIGH<->LOW
  if (inputState != lastInputState) {
    count++;  
    lastInputState = inputState;
  }

  inputt = Serial.read();
  // startTime = millis();
  // curTime = millis();
  // Serial.println(breakOn); 
  if (inputt == 'C') {       // controller enabled
      digitalWrite(5, HIGH);
      digitalWrite(3, HIGH);
  }
  else if (inputt == 'c'){        // controller disabled
      digitalWrite(5, LOW);
  }
  else if (inputt == 'm'){        // brake on
    digitalWrite(3, LOW);
    digitalWrite(5, LOW);
  }
  else if (inputt == 'M'){        // brake off
    digitalWrite(3, HIGH);
  }else if (inputt == 'R'){        // reset avg
    avgRPM = 0;
    numPoints = 0;
  }else if (inputt <= '9' && inputt >= '0') {       // controller break on
    i = (int)(inputt - 48) / 9.0 * 5;
  }else if (inputt == 'u'){        // break off
    i += 0.01;
  }else if (inputt == 'd'){        // break off
    i -= 0.01;
  } else if (inputt == 'U'){        // break off
    i += 0.1;
  }else if (inputt == 'D'){        // break off
    i -= 0.1;
  } 
  else if (inputt == ':'){        // break off
    inputt = Serial.read();
    f = "";
    while (inputt != ';'){
      f = f + inputt;
      Serial.print(f);
      inputt = Serial.read(); 
    }
    i = atof(f.c_str());
  }
  MCP.setVoltage(i);

  // ------- every half second, count is equal to Hz.---------------
  if (millis() - previousCountMillis >= countMillis) {
    previousCountMillis += countMillis;
    
    // show Hz on Serial too if available
    float rpm = (count*20)/4.0;
    float sum = avgRPM * numPoints;
    sum += rpm;
    numPoints ++;
    avgRPM = sum/numPoints;
    Serial.print("x[");
      Serial.print(payload[0]);
      Serial.print("] y[");
      Serial.print(payload[1]);
      Serial.print("] z[");
      Serial.print(payload[2]);
      Serial.print("] mag[");
      Serial.print(accelMag);
      Serial.print("] ");
    Serial.print(millis());
    Serial.print("ms, ");
    Serial.print(rpm); 
    Serial.print("RPM, ");
    Serial.print(avgRPM);
    Serial.print("avgRPM, ");
    Serial.print(i);
    Serial.print("V ");
    Serial.print(digitalRead(5) ? "MTR_ENA " : "MTR_DIS ");
    Serial.print(digitalRead(3) ? "NOT BRAKING" : "YES BRAKING");
    if(radioConnected){
      // Serial.print("x[");
      // Serial.print(payload[0]);
      // Serial.print("] y[");
      // Serial.print(payload[1]);
      // Serial.print("] z[");
      // Serial.print(payload[2]);
      // Serial.print("] mag[");
      // Serial.print(accelMag);
      // Serial.print("] ");
      // Serial.print(GONDOLA_ANGLE(payload[1],payload[2]) * 180 / PI);
      // Serial.print(" degrees");

      // Serial.print(GONDOLA_DISPLACE(payload[1],payload[2]) + RADIUS);
      // Serial.print(" meters");
    }
    Serial.println();
    

    // reset to zero for the next half second's sample
    count = 0L;
  }
}






//bisection: 
// double func(double x){

// }
// double bisection(double left, double right){
//   if (func(left) * func(right) > 0) {
//     printf("No roots in between range\n");
//     return -1;
//   }
//   if (func(left) * func(right) == 0) {
//     if (func(left) == 0) return left;
//     return right;
//   }
//   for (int i = 0; i < 10; i++){
//     double x = (left + right) / 2;
//     if (func(x) == 0) return x;
//     else if (func(left) * func(x) < 0){
//       right = x;
//       continue;
//     } else if (func(right) * func(x) < 0){
//       left = x;
//       continue;
//     } else {
//       return -1;
//     }
//   }
// }








