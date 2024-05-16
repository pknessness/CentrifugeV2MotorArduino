

#define MAXLENGTH 128
#define MAXACTUALLENGTH 10
#include <Wire.h>
#include <Arduino.h>
#include "MCP4725.h"
// #include <string.h>

MCP4725 MCP(0x60);

// outputData myData;
int myData = 0;

// char inByte;
String inst;
char runstatus = 'w';                                       //u for uploading, w for uploaded Waiting for start, r for running
int counter = 0;
int timeCounter = 0;
int dataCounter = 0;
bool start = false;
//mcp4725 variables
// char inputt = 0;
// char breakOn = 0;
// char cbreakOn = 0;
// float i = 0;
String f = "";
//PID variables
#define kP 1
#define kI 0
#define kD 1
// float kI = 0;
// float kD = 1;
float pid = 0;   
float preInput = 0;
float curInput = 0; 
int expectedCounter = 0; 
float expectedInstGravity = 0;
int expectedLength = 0;
unsigned long runStartTime = 0;
unsigned long curTime;
unsigned long preTime = 0;

unsigned long timee[MAXLENGTH] = {0};
int expectedGravity[MAXLENGTH] ={0};
float actualGravity[MAXACTUALLENGTH] = {0};
float actualTime[MAXACTUALLENGTH] ={0};
unsigned long loopTimer;
double actual_value;
unsigned int long timeStamp;

void setup() {
  pinMode(2, OUTPUT);           // Initialize pin 2 for motor controller 
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);           // motor controller enable break
  // start serial port :
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();


  MCP.begin();
  //  calibrate max voltage
  MCP.setMaxVoltage(5);
  MCP.setVoltage(0);
  digitalWrite(2, LOW);
  Serial.println("Finished setup, waiting to start");
}

void loop() {
  int refLoop = 0;
  unsigned long timeStart = millis();
  if((timeStart - loopTimer)> 200){
      //int randomValue = random(1000)/100.0;
      if(timeCounter >= MAXACTUALLENGTH) {
   
        timeCounter = 0;
      }
      if(timeCounter < MAXACTUALLENGTH){
      // actual_value = sqrt(myData.xData*myData.xData + myData.yData*myData.yData + myData.zData*myData.zData);
      // actualGravity[timeCounter] = actual_value;
      actualTime[timeCounter] = millis()-timeStamp;
      timeCounter += 1;
    }
    loopTimer = timeStart;
   }
//   int randomValue = random(1000)/100.0;
//   if(counter < MAXLENGTH){
//     expectedGravity[counter] = randomValue;
//   }
//   else if(counter >= MAXLENGTH){
//     for(int i = 0; i<MAXLENGTH-1; i++){
//       actualGravity[i] = actualGravity[i+1];
//     }
//     actualGravity[MAXLENGTH-1] = randomValue;
//   }
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    char inByte = Serial.read();
    if (inByte == '\n' || inByte == '\r' || inByte == ' '){                 //skip over white spaces
      
    }else if(inByte != ';'){                                                // ';' character stops uploading status
      inst += inByte;
      // runstatus = 'w';                                                      // runstatus waiting for START signal
    }else{
      if(inst.equals("START") && runstatus == 'w'){                         //START button pressed AND uploaded inputs
        // start = true;
        runstatus = 'r';
        digitalWrite(LED_BUILTIN,HIGH);
        timeStamp = millis();
        runStartTime = millis();
      }else if(inst.equals("STOP") && runstatus == 'r'){                    //STOP button pressed
        // start = false;
        runstatus = 0;
        digitalWrite(LED_BUILTIN, LOW);
      }else if(inst.equals("REQUEST_DATA")){
        for(int j = 0; j < MAXLENGTH; j ++){
          Serial.print(actualGravity[j]);
        }
      }else if(inst.equals("UPLOAD") && (runstatus == 0 || runstatus == 'w')){                                      //UPLOAD button pressed
        runstatus = 'u';
        Serial.println("UPLoading");

        digitalWrite(LED_BUILTIN,HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN,LOW);
        delay(500);
         digitalWrite(LED_BUILTIN,HIGH);
     }else{ 
        if(runstatus == 'u'){                                               // uploading 
          String s = "";
          for(int i = 0; i < inst.length(); i ++){
            char parsedByte = inst[i];
            if (57 >= parsedByte && 48 <= parsedByte) {
              s += parsedByte;
            }
            if(parsedByte == 'M'){
              timee[counter/2] = atoi(s.c_str());
              s = "";
              counter++;
            }
            if(parsedByte == 'G'){
              expectedGravity[counter/2] = atoi(s.c_str());
              s = "";
              counter++;
            }
          } 
          //In case last gravity entri is not 0
          timee[counter / 2] = timee[counter / 2 - 1] + 5000;
          counter ++;
          expectedGravity[counter/2 + 1] = 0;
          runstatus = 'w';
          expectedLength = counter / 2 + 1;
          for (int k = 0; k < expectedLength; k++){
            Serial.print(expectedGravity[k]);
            Serial.print(" ");
          }
          Serial.println();
          for (int k = 0; k < expectedLength; k++){
            Serial.print(timee[k]);
            Serial.print(" ");
          }
          Serial.println();
        }
      }
      inst = "";
    }
  }

  //PID 
  digitalWrite(5, HIGH);
  if (runstatus == 'r'){
    curTime = millis() - runStartTime;
    pid = kP * curInput + kI * (curTime - preTime) * (curInput + preInput) / 2 + kD * (curInput - preInput) / (curTime - preTime);
    
    while (curTime > timee[expectedCounter]) {
      expectedCounter ++;
    }
    if (expectedGravity[expectedCounter]  < expectedGravity[expectedCounter-1]){
      digitalWrite(5, LOW);
      digitalWrite(2, HIGH);      

    } else {
      digitalWrite(5, HIGH);
      digitalWrite(2, LOW);
    }
    if (expectedCounter == 0){
      expectedInstGravity =  (expectedGravity[expectedCounter] * 1.0 / timee[expectedCounter] * curTime );
    } else if (expectedCounter >= expectedLength) {
      runstatus = 0;
      Serial.print("Run finished ");
    } else {
      expectedInstGravity = ((expectedGravity[expectedCounter] * 1.0 - expectedGravity[expectedCounter-1]) / ((timee[expectedCounter ]*1.0 - timee[expectedCounter-1]))* (curTime*1.0 - timee[expectedCounter-1]) + expectedGravity[expectedCounter-1]);
    }


    

    Serial.print(curTime);
    Serial.print(" ");
    Serial.print(expectedCounter);
    Serial.print(" ");
    Serial.print(expectedInstGravity);
    Serial.print(" ");
    Serial.print(expectedGravity[expectedCounter]);
    if (expectedGravity[expectedCounter] > 5){
      expectedGravity[expectedCounter] = 5;
    }
    // MCP.setVoltage(expectedGravity[expectedCounter]);
    MCP.setVoltage(expectedInstGravity);
    
    Serial.print(" ");
    Serial.println( timee[expectedCounter]);
    digitalWrite(5, HIGH);
    preTime = curTime;
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}

