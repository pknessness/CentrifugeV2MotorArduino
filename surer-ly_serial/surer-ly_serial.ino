

#define MAXLENGTH 100
#define MAXACTUALLENGTH 10

#define VOLT_MAX 3  
#include <Wire.h>
#include <Arduino.h>
#include "MCP4725.h"
#include <SPI.h>
#include "RF24.h"

// #include <string.h>
#define CE_PIN 7
#define CSN_PIN 8

#define VOLT_SET_DIRECT 0

#define RADIANS_PER_SECOND_TO_VOLTAGE 0.298415518297
#define RPM_TO_VOLTAGE 0.03125
#define RADIUS  1.37

#define BISECTION_ITERATIONS 10

#define BIG_PRINT 1
#define LOG_PRINT 0

//PID variables
#define kP 0.02
#define kI 0.003
#define kD 0.001
#define integralCap 100

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

MCP4725 MCP(0x60);

// outputData myData;
int myData = 0;

// char inByte;
String inst;
char runstatus = 0;                                       //u for uploading, w for uploaded Waiting for start, r for running
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

// float kI = 0;
// float kD = 1;
float pid = 0;   
float lastError = 0;
float error = 0; 
int expectedCounter = 0; 
float expectedInstGravity = 0;
int expectedLength = 0;
unsigned long runStartTime = 0;
unsigned long curTime;
unsigned long preTime = 0;
float integral = 0;

unsigned long timee[MAXLENGTH] = {0};
float expectedGravity[MAXLENGTH] ={0};
float actualGravity[MAXACTUALLENGTH] = {0};
float actualTime[MAXACTUALLENGTH] ={0};
unsigned long loopTimer;
double actual_value;
unsigned int long timeStamp;

float FF;

float payload[3] = {0, 0, 0};
uint8_t address[][6] = { "1Node", "2Node" };
float acc;

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
  digitalWrite(5, LOW);
  digitalWrite(3, LOW);

  //radio 
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!0]);
  radio.startListening();
  Serial.print("Finished setup, waiting to start, PID:");
  Serial.print(" p:");
  Serial.print( kP);
  Serial.print(" i(x100):");
  Serial.print( kI*100);
  Serial.print(" d(x10):");
  Serial.print(kD*10);
  Serial.print(" iCap:");
  Serial.println(integralCap);
}

void loop() {

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
      acc = sqrt(payload[0]*payload[0] + payload[1]*payload[1] + payload[2]*payload[2]);
    }


  // string parsing
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
        // for (int i = 0; i < MAXLENGTH; i ++){
        //   timee[i] = 0;
        //   expectedGravity[i] = 0;
        // }

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
            if ((57 >= parsedByte && 48 <= parsedByte) || parsedByte == '.') {
              s += parsedByte;
            }
            if(parsedByte == 'M'){
              // Serial.print("csdt");
              // Serial.println(s.c_str());
              timee[counter/2] = atol(s.c_str());
              s = "";
              counter++;
            }
            if(parsedByte == 'G'){
              expectedGravity[counter/2] = atof(s.c_str());
              if(expectedGravity[counter/2] < 1) expectedGravity[counter/2] = 1;
              s = "";
              counter++;
            }
          } 
          //In case last gravity entri is not 0
          timee[counter / 2] = timee[counter / 2 - 1] + 5000;
          counter ++;
          expectedGravity[counter/2 + 1] = 1;
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
    int dt = curTime-preTime;
    
    while (curTime > timee[expectedCounter]) {
      expectedCounter ++;
    }
    if (expectedCounter == 0){
      if (timee[expectedCounter] == 0) {
        expectedInstGravity = 0;
      }else {
        expectedInstGravity =  (expectedGravity[expectedCounter] * 1.0 / timee[expectedCounter] * curTime );
      }
    } else if (expectedCounter >= expectedLength) {
      runstatus = 0;
      Serial.print("Run finished ");
      digitalWrite(3, HIGH);
      digitalWrite(5, LOW);
    } else {
      expectedInstGravity = ((expectedGravity[expectedCounter] * 1.0 - expectedGravity[expectedCounter-1]) / ((timee[expectedCounter ]*1.0 - timee[expectedCounter-1]))* (curTime*1.0 - timee[expectedCounter-1]) + expectedGravity[expectedCounter-1]);
    }
    if (expectedCounter < expectedLength){
      float interpolateGradient = (expectedGravity[expectedCounter] * 1.0 - expectedGravity[expectedCounter-1]) / (timee[expectedCounter ]*1.0 - timee[expectedCounter-1]);

      error = expectedInstGravity - acc;

      if (expectedGravity[expectedCounter]  < expectedGravity[expectedCounter-1]){
        digitalWrite(5, LOW);
        MCP.setVoltage(0);
        // digitalWrite(3, LOW);      
        if(error > 0){ //error positive, we are below setpoint, not braking
          digitalWrite(3, HIGH);
        }else{ //error negative, we are above setpoint, braking
          digitalWrite(3, LOW);
        }
        #if BIG_PRINT
          Serial.print("DOWNSLOPE");
          Serial.print(curTime);
          // Serial.print(" x:");
          // Serial.print(payload[0]);
          // Serial.print(" y:");
          // Serial.print(payload[1]);
          // Serial.print(" z:");
          // Serial.print(payload[2]);
          Serial.print(" acc:");
          Serial.print(acc);
          Serial.print(" inst:");
          Serial.print(expectedInstGravity);
          Serial.print(" dt:");
          Serial.println(dt);
          Serial.print(" BRK_EN:");
          Serial.print(digitalRead(3) ? "DIS" : "ENG"); //disengaged, engaged
        #endif
      } else {
        digitalWrite(5, HIGH);
        digitalWrite(3, HIGH);

        // FF = sqrt((-1/dt/dt+sqrt(1/dt/dt/dt/dt + (4*(expectedInstGravity * expectedInstGravity - 9.81))/RADIUS/RADIUS))/2)*RADIANS_PER_SECOND_TO_VOLTAGE;
        // FF = sqrt(sqrt((acc * acc - 9.81 - RADIUS * RADIUS * interpolateGradient)/(RADIUS * RADIUS * RADIUS * RADIUS)));
        float expectedAccel = expectedInstGravity * 9.81;
        float theValue = sqrt(sqrt((expectedAccel * expectedAccel - 9.81)/(RADIUS * RADIUS))) * 30 / PI;
        // Serial.print("pid but RPM:");
        // Serial.print(pid / RPM_TO_VOLTAGE);
        // Serial.print("theValue:");
        // Serial.println(theValue / RPM_TO_VOLTAGE);
        float bisectedFF = bisection( pid / RPM_TO_VOLTAGE, pid / RPM_TO_VOLTAGE+ 10, pid / RADIANS_PER_SECOND_TO_VOLTAGE, dt/1000.0, expectedAccel);
        if(bisectedFF < 0 || bisectedFF != bisectedFF){
          FF = RPM_TO_VOLTAGE * theValue;
          #if BIG_PRINT
            Serial.print("NO_BS[");
            Serial.print(FF);
            Serial.print("]");
          #endif
        }else{
          FF = RPM_TO_VOLTAGE * bisectedFF;
        }
        integral += (dt/1000.0) * (error + lastError) / 2;
        if (integral > integralCap){
          integral  = integralCap;
        }
        pid = kP * error + kI * integral + kD * (error - lastError) / (dt/1000.0) + FF;
      
        if (pid > 5){
          pid = 5;
        }
        if (pid < 0){
          pid = 0;
        }
        if (pid > VOLT_MAX){
          pid = VOLT_MAX;
        }
        #if VOLT_SET_DIRECT
          if(expectedInstGravity < VOLT_MAX)
            MCP.setVoltage(expectedInstGravity);
          else
            MCP.setVoltage(VOLT_MAX);
        #else
          if(pid >= 0){
            MCP.setVoltage(pid); 
          }
        #endif
        #if BIG_PRINT
          Serial.print(curTime);
          // Serial.print(" x:");
          // Serial.print(payload[0]);
          // Serial.print(" y:");
          // Serial.print(payload[1]);
          // Serial.print(" z:");
          // Serial.print(payload[2]);
          Serial.print(" acc:");
          Serial.print(acc);
          Serial.print(" inst:");
          Serial.print(expectedInstGravity);
          Serial.print(" dt:");
          // Serial.print(expectedGravity[expectedCounter]);
          Serial.print(dt);
          // Serial.print(" timee:");
          // Serial.print( timee[expectedCounter]);
          Serial.print(" pComp:");
          Serial.print( kP * error );
          Serial.print(" iComp:");
          Serial.print( kI * integral);
          Serial.print(" dComp:");
          Serial.print(kD * (error - lastError) / (dt/1000.0));
          Serial.print(" FF:");
          Serial.print(FF);
          Serial.print(" PID:");
          Serial.print(pid);
          Serial.print(" BRK_EN:");
          Serial.print(digitalRead(3) ? "DIS" : "ENG"); //disengaged, engaged
          Serial.print(" MTR_EN:");
          Serial.println(digitalRead(5) ? "YES" : "FLO"); //yes, floating
        #endif
      }
    }

    
    #if LOG_PRINT
      Serial.print(curTime);
      Serial.print("\t");
      Serial.print(acc);
      Serial.print("\t");
      Serial.print(expectedInstGravity);
      Serial.print("\t");
      Serial.print(pid);
      Serial.print("\t");
      Serial.print(digitalRead(3));
      Serial.print("\t");
      Serial.println(digitalRead(5));
    #endif
    

    lastError = error;
    preTime = curTime;
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}

float G_func(float x, float p, float dt, float a){
  float xrad = (x * PI / 30);
  float comp1 = xrad * xrad * xrad * xrad;
  float comp2 = ((xrad - p)/dt) * ((xrad - p)/dt);
  float comp3 = -(a * a - 9.81)/(RADIUS*RADIUS);
  // = left * left * left * left / 810000 * PI *PI*PI*PI + ((left * PI / 30 - p)/dt) * ((left * PI / 30 - p)/dt) - (a * a - 9.81) / RADIUS / RADIUS;
  // Serial.print(" x:");
  // Serial.print(x);
  // Serial.print(" p:");
  // Serial.print(p);
  // Serial.print(" dt:");
  // Serial.print(dt);
  // Serial.print(" a:");
  // Serial.print(a);
  // Serial.print(" c1:");
  // Serial.print(comp1);
  // Serial.print( " c2:");
  // Serial.print(comp2);
  // Serial.print(" c3:");
  // Serial.println(comp3);
  return comp1 + comp2 + comp3;
  // return comp3;
}

float bisection(float left, float right, float p, float dt, float a){
  if(dt == 0.0) dt = 0.001;
  float leftF = G_func(left, p, dt, a);
  float rightF = G_func(right, p, dt, a);
  if (leftF * rightF > 0) {
    // Serial.print("No roots in between range l["); 
    // Serial.print(left);
    // Serial.print("] r["); 
    // Serial.print(right);
    // Serial.print("] lF["); 
    // Serial.print(leftF);
    // Serial.print("] rF["); 
    // Serial.print(rightF);
    // Serial.print("] p["); 
    // Serial.print(p);
    // Serial.print("] dt["); 
    // Serial.print(dt);
    // Serial.print("] a["); 
    // Serial.print(a);
    // Serial.println("]"); 
    return -2;
  }
  if (leftF * rightF == 0) {
    if (leftF == 0) return left;
    return right;
  }
  float xF = 0;
  float x = 0;
  for (int i = 0; i < BISECTION_ITERATIONS; i++){
    x = (left + right) / 2;
    xF = G_func(x, p, dt, a);
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.print(right);
    // Serial.print(" ");
    // Serial.print(leftF);
    // Serial.print(" ");
    // Serial.println(rightF);   
    if (xF == 0) return x;
    else if (leftF * xF < 0){
      right = x;
    } else if (rightF * xF < 0){
      left = x;
    } else {
      return -1;
    }
    leftF = G_func(left, p, dt, a);
    rightF = G_func(right, p, dt, a);

  }
  // Serial.print("==================");
  // Serial.println(x);
  return x;
}

