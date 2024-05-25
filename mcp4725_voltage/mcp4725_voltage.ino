//
//    FILE: mcp4725_voltage.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: test mcp4725 lib
//     URL: https://github.com/RobTillaart/MCP4725

#include "Wire.h"
#include "MCP4725.h"
#include <string.h>

MCP4725 MCP(0x60);

volatile int x;
uint32_t start, stop;

const byte inputPin = 4;
boolean inputState = false;
boolean lastInputState = false;
long count = 0L;

float avgRPM = 0;
float numPoints = 0;

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
  if (inputt == 'c') {       // controller enabled
      digitalWrite(5, HIGH);
      digitalWrite(3, HIGH);
  }
  else if (inputt == 'C'){        // controller disabled
      digitalWrite(5, LOW);
  }
  else if (inputt == 'm'){        // break on
    digitalWrite(3, LOW);
    digitalWrite(5, LOW);
  }
  else if (inputt == 'M'){        // break off
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
    Serial.print(rpm); 
    float sum = avgRPM * numPoints;
    sum += rpm;
    numPoints ++;
    avgRPM = sum/numPoints;
    Serial.print("RPM, ");
    Serial.print(avgRPM);
    Serial.print("avgRPM, ");
    // Serial.print(sum);
    // Serial.print(" sum, ");
    // Serial.print(numPoints);
    // Serial.print(" points, ");
    Serial.print(i);
    Serial.println("V");

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








