//최종코드
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Stepper.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int stepsPerRevolution = 2048;
const int sensorIn = 34; 
int mVperAmp = 185; 
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

//바퀴 모터 핀번호
#define DIR1 32
#define STEP1 33
#define DIR2 14
#define STEP2 27 

//케이블 모터 핀번호
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 21

//로봇 이동 모터 제어
AccelStepper stepper1 = AccelStepper(1, STEP1, DIR1);
AccelStepper stepper2 = AccelStepper(1, STEP2, DIR2);
MultiStepper steppers;

//케이블 모터 제어
Stepper cableStepper(stepsPerRevolution,IN1,IN3,IN2,IN4);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("bluetoothTest BT"); //블루투스 기능
  
  //로봇 이동 모터 제어_전진을 위해 양쪽 모터 회전 방향 반대로 설정(50, -50)
  stepper1.setMaxSpeed(200); 
  stepper1.setCurrentPosition(0);
  stepper1.setSpeed(-50);
  stepper2.setMaxSpeed(200); 
  stepper2.setCurrentPosition(0);
  stepper2.setSpeed(50); 

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  cableStepper.setSpeed(14);
}

void loop() {
  if(SerialBT.available()){
   char controlKey = SerialBT.read();
   Serial.write(SerialBT.read());
   
   switch (controlKey) {
    //모형에 주차 자리 A,B,C만 설치
    case 'A': { 
      Serial.println(controlKey);
      long positions[] = {0,0};
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      Charging();
      controlKey = ' ';
      break;
    }
    case 'B': {
      Serial.println(controlKey);
      long positions[] = {1000,-1000};
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      Charging();
      controlKey = ' ';
      break;
    }
    case 'C': {
      Serial.println(controlKey);
      long positions[] = {2000,-2000};
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      Charging();
      controlKey = ' ';
      break;
    }
   /* case 'D': {
      Serial.println(controlKey);
      long positions[] = {-1500,1500};
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      Charging();
      controlKey = ' ';
      break;
    }
    case 'E': {
      Serial.println(controlKey);
      long positions[] = {-2000,2000};
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      Charging();
      controlKey = ' ';
      break;
    }  */
    default :
      break;
    }
  }
}

//케이블 내려 충전 진행 후 케이블 올리기
void Charging() {
  // 시계 방향으로 한바퀴 회전-> 케이블 내리기
  Serial.println("cable down");
  cableStepper.step(stepsPerRevolution*2);
  delay(1000);
  //케이블 어댑터와 연결

  Serial.println ("Charging Start"); 
  for (;;) {
    //전류 측정 시작
    Voltage = getVPP();
    VRMS = (Voltage/2.0) *0.707;
    AmpsRMS = ((VRMS * 1000)/mVperAmp);
    Serial.print(AmpsRMS);
    Serial.println(" A");
    
    //전류가 0.45이하이면 케이블 올리는 코드 실행 / 그 외엔 계속 전류 측정
    if(AmpsRMS <= 0.45){ 
      Serial.println("Charging Complete");
      Serial.println("cable up");
      // 시계 반대 방향으로 한바퀴 회전-> 케이블 올리기
      cableStepper.step(-stepsPerRevolution*2);
      delay(1000);
      break;
    }
  }
  delay(1000);
}

//완충신호를 위한 전류측정
float getVPP()
{
  float result;
  int readValue;                
  int maxValue = 0;             
  int minValue = 4096;          
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000)
   {
       readValue = analogRead(sensorIn);
       if (readValue > maxValue) 
       {
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           minValue = readValue;
       }
   }
   result = ((maxValue - minValue) * 5)/4096.0;
   return result;
 }

//좌석별 좌표값 : A{0,0} B{-500,500} c{-1000,1000} D{-1500,1500} E{-2000,2000}
//좌석사이 거리 500으로 설정