#define MAXLENGTH 30
#define MAXACTUALLENGTH 10
#define MAXINSTRUCTIONLENGTH MAXLENGTH*15

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

// #define RADIANS_PER_SECOND_TO_VOLTAGE 0.298415518297
// #define RPM_TO_VOLTAGE 0.03125
#define VOLTS_TO_RAD(V) VOLTS_TO_RPM(V) * PI / 30
#define RAD_TO_VOLTS(R) RPM_TO_VOLTS(V) * 30 / PI

#define VOLTS_TO_RPM(V) (-3.17 + 59.6*V - 5.41*V*V)
#define RPM_TO_VOLTS(R) (59.6 - sqrt(3552.16 - (3.17 + R) * 21.64)) / 10.82

#define GONDOLA_RADIUS 0.2
// #define GONDOLA_DISPLACE(a_n, a_r) GONDOLA_RADIUS * (-a_n + sqrt(a_n*a_n - (a_r*a_r+a_n*a_n)*(1-(a_r*a_r))))/(a_r*a_r + a_n*a_n) 
#define GONDOLA_DISPLACE(a_n, a_r) GONDOLA_RADIUS *(-a_n + sqrt(a_n*a_n - (a_r*a_r+a_n*a_n)*(1-(a_r*a_r))))/(a_r*a_r + a_n*a_n)

#define RADIUS  1.5//1.37

#define Brake_threshold_G_per_s -2

#define BISECTION_ITERATIONS 10
#define BISECTION_RANGE 3

#define BIG_PRINT 0
#define LOG_PRINT 1

// #define G_natural_descent(R) 0.0409 + R* 0.000219 - 0.00000112 * R * R 
#define G_natural_descent(R) 0.0152 + 0.00000575*R + -0.000000493 * R * R
#define NGD 1

//PID variables
// #define kP 0.12
// #define kI 0.1
// #define kD 0.001
#define integralCap 300

// #define kP 1
#define kPUP 1.5
#define kPDN 0.3
#define kI 0.005
#define kD 0.00001

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

MCP4725 MCP(0x60);

// outputData myData;
int myData = 0;
int timerCounter = 0;
// char inByte;
//String inst;
char instruction[MAXINSTRUCTIONLENGTH] = {0};
int instructionPointer = 0;

char runstatus = 'i';                                       //u for uploading, w for uploaded Waiting for start, r for running
int counter = 0;
int timeCounter = 0;
int dataCounter = 0;
bool start = false;
//mcp4725 variables
// char inputt = 0;
// char breakOn = 0;
// char cbreakOn = 0;
// float i = 0;
//String f = "";

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
  //pinMode(LED_BUILTIN, OUTPUT);
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
  Serial.print( kPUP);
  Serial.print(" i(x100):");
  Serial.print( kI*100);
  Serial.print(" d(x1000):");
  Serial.print(kD*1000);
  Serial.print(" iCap:");
  Serial.println(integralCap);

  runstatus = 'i';
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
      //actual_value = sqrt(myData.xData*myData.xData + myData.yData*myData.yData + myData.zData*myData.zData);
      actualGravity[timeCounter] = acc;//actual_value;
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
    if (inByte == '\n' || inByte == '\r' || inByte == ' ' || inByte < ' ' || inByte > '~'){                 //skip over white spaces
      
    }else if(inByte != ';'){                                                // ';' character stops uploading status
      //inst += inByte;
      instruction[instructionPointer++] = inByte;
      //runstatus = 'w';                                                      // runstatus waiting for START signal
      //Serial.print("=");
      //Serial.print(instructionPointer);
      //Serial.print("+");
      //Serial.println(instruction);
    }else{
      //Serial.print("<");
      //Serial.print(instruction);
      //Serial.println(">");
      if(instruction[0] == 'S' && instruction[1] == 'T' && instruction[2] == 'A' && instruction[3] == 'R' && instruction[4] == 'T'){                         //START button pressed AND uploaded inputs
        if(runstatus == 'w'){
          // start = true;
          runstatus = 'r';
          digitalWrite(LED_BUILTIN,HIGH);
          timeStamp = millis();
          runStartTime = millis();
        }else{
          Serial.print("WRONG STATE TO START "); 
          Serial.println(runstatus); 
        }
      }else if(instruction[0] == 'S' && instruction[1] == 'T' && instruction[2] == 'O' && instruction[3] == 'P'){                    //STOP button pressed
        if(runstatus == 'r'){
          // start = false;
          runstatus = 'i';
          digitalWrite(LED_BUILTIN, LOW);
          Serial.println("STOPPED");
        }else{
          Serial.print("WRONG STATE TO STOP "); 
          Serial.println(runstatus); 
        }
      }else if(instruction[0] == 'R' && instruction[1] == 'E' && instruction[2] == 'Q' && instruction[3] == 'U' && instruction[4] == 'E' && instruction[5] == 'S' && instruction[6] == 'T'){
        for(int j = 0; j < MAXLENGTH; j ++){
          Serial.print(actualTime[j]);  
          Serial.print("ms");  
          Serial.print(actualGravity[j]);  
          Serial.println("g");  
        }
      }else if(instruction[0] == 'S' && instruction[1] == 'T' && instruction[2] == 'A' && instruction[3] == 'T' && instruction[4] == 'U' && instruction[5] == 'S'){
        if(runstatus == 'i'){
          Serial.println("IDLE");  
        }else if(runstatus == 'u'){
          Serial.println("UPLOADING");  
        }else if(runstatus == 'w'){
          Serial.println("READY TO GO");  
        }else if(runstatus == 'r'){
          Serial.println("RUNNING");  
        }
      }else if(instruction[0] == 'U' && instruction[1] == 'P' && instruction[2] == 'L' && instruction[3] == 'O' && instruction[4] == 'A' && instruction[5] == 'D'){                                      //UPLOAD button pressed
        if(runstatus == 'i' || runstatus == 'w'){
          runstatus = 'u';
          Serial.println("UPLoaded");
          for (int i = 0; i < MAXLENGTH; i ++){
            timee[i] = 0;
            expectedGravity[i] = 0;
          }
          counter = 0;
          // digitalWrite(LED_BUILTIN,HIGH);
          // delay(500);
          // digitalWrite(LED_BUILTIN,LOW);
          // delay(500);
          // digitalWrite(LED_BUILTIN,HIGH);
        }else{
          Serial.print("WRONG STATE TO UPLOAD "); 
          Serial.println(runstatus); 
        }
      }else{ 
        if(runstatus == 'u'){                                               // uploading 
          String s = "";
          for(int i = 0; i < instructionPointer; i ++){
            char parsedByte = instruction[i];
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
          expectedGravity[counter/2 ] = 1;
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
        }else{
          Serial.print("BAD INSTRUCTION <"); 
          //Serial.print(instruction); 
          for(int ins = 0; ins < 8; ins++){
            Serial.print('|');
            Serial.print(instruction[ins]);
          }
          Serial.print("|> STATE: "); 
          Serial.println(runstatus); 
        }
      }
      //inst = "";
      instructionPointer = 0;
      //memset(instruction, MAXINSTRUCTIONLENGTH); //shouldnt need to, as long as im doing my instruction pointer correctly 
      //i can also set the instruction end to '\0' on ;
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
    float slope = (expectedGravity[expectedCounter] - expectedGravity[expectedCounter-1]) / (timee[expectedCounter ] - timee[expectedCounter-1]);
    if (expectedCounter == 0){
      if (timee[expectedCounter] == 0) {
        expectedInstGravity = 0;
      }else {
        expectedInstGravity =  (expectedGravity[expectedCounter] * 1.0 / timee[expectedCounter] * curTime);
      }
    } else if (expectedCounter >= expectedLength) {
      runstatus = 'i';
      Serial.print("Run finished ");
      MCP.setVoltage(0);
      digitalWrite(3, HIGH);
      digitalWrite(5, LOW);
    } else {
      expectedInstGravity = (slope * (curTime*1.0 - timee[expectedCounter-1]) + expectedGravity[expectedCounter-1]);
    }
    if (expectedCounter < expectedLength){
      // float interpolateGradient = (expectedGravity[expectedCounter] * 1.0 - expectedGravity[expectedCounter-1]) / (timee[expectedCounter ]*1.0 - timee[expectedCounter-1]);

      error = expectedInstGravity - acc;
      // Serial.print("slope");
      // Serial.print(slope * 1000);
      // Serial.print("rpm");
      // Serial.print(VOLTS_TO_RPM(pid)*25);
      // Serial.print("natural G");
      // Serial.println(G_natural_descent(VOLTS_TO_RPM(pid)*25));
      #if NGD
      if (pid == pid && slope < 0 && (slope*1000) < G_natural_descent(VOLTS_TO_RPM(pid)*25)){
      #else
      if ((slope*1000) < Brake_threshold_G_per_s){
      #endif
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
        // float r = GONDOLA_DISPLACE(payload[1],payload[2]) + RADIUS;
        float expectedAccel = expectedInstGravity * 9.81;
        float theValue = sqrt(sqrt((expectedAccel * expectedAccel - 9.81)/(RADIUS * RADIUS))) * 30 / PI;
        // Serial.print("pid but RPM:");
        // Serial.print(pid / RPM_TO_VOLTAGE);
        // Serial.print("theValue:");
        // Serial.println(theValue / RPM_TO_VOLTAGE);
        float bisectedFF = bisection( VOLTS_TO_RPM(pid), VOLTS_TO_RPM(pid) + BISECTION_RANGE, VOLTS_TO_RAD(pid), dt/1000.0, expectedAccel);
        if(expectedInstGravity <= 1){
          FF = 0;
        }
        else if(bisectedFF < 0 || bisectedFF != bisectedFF){
          FF = RPM_TO_VOLTS(theValue);
          #if BIG_PRINT
            Serial.print("NO_BS[");
            Serial.print(bisectedFF);
            Serial.print("]");
          #endif
        }else{
          FF = RPM_TO_VOLTS(bisectedFF);
        }
        integral += (dt/1000.0) * (error + lastError) / 2;
        if (integral > integralCap){
          integral  = integralCap;
        }
        if (slope < 0){
          pid = kPDN * error + kI * integral + kD * (error - lastError) / (dt/1000.0) + FF;

        } else {
          pid = kPUP * error + kI * integral + kD * (error - lastError) / (dt/1000.0) + FF;          
        }
      
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
      timeCounter++;
      if (timeCounter > 10){
        timeCounter = 0; 
        Serial.print(curTime);
        Serial.print("\t");
        Serial.print(acc);
        Serial.print("\t");
        Serial.print(expectedInstGravity);
        Serial.print("\t");
        Serial.println(pid);
        // Serial.print("\t");
        // Serial.print(digitalRead(3));
        // Serial.print("\t");
        // Serial.println(digitalRead(5));
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    #endif
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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
  // float r = GONDOLA_DISPLACE(payload[1],payload[2]) + RADIUS;
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