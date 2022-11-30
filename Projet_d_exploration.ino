#include <math.h>

#if ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)
#if !defined(RP2040_ISR_SERVO_USING_MBED)
  #define RP2040_ISR_SERVO_USING_MBED     false
#endif

#elif ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)

#if !defined(RP2040_ISR_SERVO_USING_MBED)
  #define RP2040_ISR_SERVO_USING_MBED     true
#endif

#else
#error This code is intended to run on the mbed / non-mbed RP2040 platform! Please check your Tools->Board setting.
#endif

#define ISR_SERVO_DEBUG             4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RP2040_ISR_Servo.h"

//#include <Servo.h>
#include <math.h>
#define MIN_MICROS        800
#define MAX_MICROS        2450
/*
const int servo_pin[4][3] = { 
  { 4, 2, 3 }, // Leg 1 - Front right (R2) - {H, F, T}
  { 7, 5, 6 }, // Leg 2 - Back right (R1) - {H, F, T}
  { 16, 14, 15 }, // Leg 3 - Front left (L1) - {H, F, T}
  { 19, 17, 18 } // Leg 4 - Back left (L2) - {H, F, T}
};*/


#define SERVO_PIN_1       4
#define SERVO_PIN_2       2
#define SERVO_PIN_3       3
#define SERVO_PIN_4       7
#define SERVO_PIN_5       5
#define SERVO_PIN_6       6
#define SERVO_PIN_7       16
#define SERVO_PIN_8       14
#define SERVO_PIN_9       15
#define SERVO_PIN_10      19
#define SERVO_PIN_11      17
#define SERVO_PIN_12      18

typedef struct
{
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;

#define NUMBER_LEG       4
#define NUMBER_SERVOS_BY_LEG       3

ISR_servo_t servo[NUMBER_LEG][NUMBER_SERVOS_BY_LEG] = {
  {{ -1, SERVO_PIN_1 },
  { -1, SERVO_PIN_2 },
  { -1, SERVO_PIN_3 }},
  {{ -1, SERVO_PIN_4 },
  { -1, SERVO_PIN_5 },
  { -1, SERVO_PIN_6 }},
  {{ -1, SERVO_PIN_7 },
  { -1, SERVO_PIN_8 },
  { -1, SERVO_PIN_9 }},
  {{ -1, SERVO_PIN_10 },
  { -1, SERVO_PIN_11 },
  { -1, SERVO_PIN_12 }}
};

class AngleCoo
{
  public:
    float Gamma;
    float Alpha;
    float Beta;
    AngleCoo(){
      Gamma = 0;
      Alpha = 0;
      Beta = 0;
    }
    AngleCoo(float gamma,float alpha,float beta){
      Gamma = gamma;
      Alpha = alpha;
      Beta = beta;
    }
};


/*const int offset[4][3] = {
  {6,-19,16},
  {0,-7,13},
  {5,9,-13},
  {0,-16,1}
};*/

int offset[4][3];

/*const int realPosition[4][3] = {
  {102,53,53},
  {92,67,42},
  {107,80,88}, 
  {101,58,60}
};*/

const int realPosition[4][3] = {
  {100,70,42},
  {100,70,42},
  {100,70,42}, 
  {100,70,42}
};

const int R2 = 0;
const int R1 = 1;
const int L1 = 2;
const int L2 = 3;

const float LH = 30.5;
const float LF = 53;
const float LT = 79.5;

void calculateOffset(){
  AngleCoo target = ConvertPointToAngle(100,70,42);

  Serial.println("TARGET");
  Serial.println(target.Gamma);
  Serial.println(target.Alpha);
  Serial.println(target.Beta);
  
  AngleCoo realR2 = ConvertPointToAngle(realPosition[0][0],realPosition[0][1],realPosition[0][2]);
  AngleCoo realR1 = ConvertPointToAngle(realPosition[1][0],realPosition[1][1],realPosition[1][2]);
  AngleCoo realL1 = ConvertPointToAngle(realPosition[2][0],realPosition[2][1],realPosition[2][2]);
  AngleCoo realL2 = ConvertPointToAngle(realPosition[3][0],realPosition[3][1],realPosition[3][2]);

  Serial.println("R2");
  Serial.println(realR2.Gamma);
  Serial.println(realR2.Alpha);
  Serial.println(realR2.Beta);

  offset[0][0] = target.Gamma-realR2.Gamma;
  offset[0][1] = target.Alpha-realR2.Alpha;
  offset[0][2] = target.Beta-realR2.Beta;

  Serial.println("R1");
  Serial.println(realR1.Gamma);
  Serial.println(realR1.Alpha);
  Serial.println(realR1.Beta);

  offset[1][0] = target.Gamma-realR1.Gamma;
  offset[1][1] = target.Alpha-realR1.Alpha;
  offset[1][2] = target.Beta-realR1.Beta;

  Serial.println("L1");
  Serial.println(realL1.Gamma);
  Serial.println(realL1.Alpha);
  Serial.println(realL1.Beta);
    
  offset[2][0] = target.Gamma-realL1.Gamma;
  offset[2][1] = target.Alpha-realL1.Alpha;
  offset[2][2] = target.Beta-realL1.Beta;

  Serial.println("L2");
  Serial.println(realL2.Gamma);
  Serial.println(realL2.Alpha);
  Serial.println(realL2.Beta);
    
  offset[3][0] = target.Gamma-realL2.Gamma;
  offset[3][1] = target.Alpha-realL2.Alpha;
  offset[3][2] = target.Beta-realL2.Beta;

  for (int i = 0; i < 4; i++)
  {
    Serial.println("");
    for (int y = 0; y < 3; y++)
    {
      Serial.println(offset[i][y]);
    }
  }  
}

void setPLS(int indexLeg, AngleCoo coo){
  Serial.println("enter in PLS");
  Serial.println("Gamma");
  Serial.println(coo.Gamma);
  Serial.println("Alpha");
  Serial.println(coo.Alpha);
  Serial.println("Beta");
  Serial.println(coo.Beta);
  Serial.println("Leg");
  Serial.println(indexLeg);

  
  RP2040_ISR_Servos.setPosition(servo[0][0].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[1][0].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[2][0].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[3][0].servoIndex,90);

  RP2040_ISR_Servos.setPosition(servo[0][1].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[1][1].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[2][1].servoIndex,90);
  RP2040_ISR_Servos.setPosition(servo[3][1].servoIndex,90);

  RP2040_ISR_Servos.setPosition(servo[0][2].servoIndex,160);
  RP2040_ISR_Servos.setPosition(servo[1][2].servoIndex,20);
  RP2040_ISR_Servos.setPosition(servo[2][2].servoIndex,20);
  RP2040_ISR_Servos.setPosition(servo[3][2].servoIndex,160);
}

AngleCoo ConvertPointToAngle(float x, float y, float z){
  z -= 27;
  float gamma = atan(y/x)*(180/M_PI);
  float hypo = sqrt(pow(x,2)+pow(y,2));
  float v = hypo - LH;
  float d = sqrt(pow(v,2)+pow(z,2));
  float alphaOne = atan(z / v) * (180/M_PI);
  float alphaTwo = acos((pow(d,2) + pow(LF,2) - pow(LT,2))/(2*LF*d)) * (180/M_PI);
  float alpha = alphaOne+alphaTwo;
  float beta = acos((pow(LF,2)+pow(LT,2)-pow(d,2))/(2*LT*LF)) * (180/M_PI);
  return AngleCoo(gamma, alpha, beta);
}

void move(int indexLeg, AngleCoo coo)
{
  coo.Gamma+offset[indexLeg][0];
  coo.Alpha+offset[indexLeg][1];
  coo.Beta+offset[indexLeg][2];
  switch (indexLeg)
  {
    case R2:
      coo.Gamma = 90 + coo.Gamma;
      coo.Alpha = 95 - coo.Alpha;
      break;
    case R1:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      coo.Beta = 180 - coo.Beta;
      break;
    case L1:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      coo.Beta = 190 - coo.Beta;
      break;
    case L2:
      coo.Gamma = 90 + coo.Gamma;
      coo.Alpha = 90 - coo.Alpha;
      break;
    default:
      break;
  }

  if(checkAngle(indexLeg, coo)){
    RP2040_ISR_Servos.setPosition(servo[indexLeg][0].servoIndex, coo.Gamma);
    RP2040_ISR_Servos.setPosition(servo[indexLeg][1].servoIndex, coo.Alpha);
    RP2040_ISR_Servos.setPosition(servo[indexLeg][2].servoIndex, coo.Beta);
  }else{
    setPLS(indexLeg, coo);
  }
}

bool checkAngle(int indexLeg, AngleCoo coo){
  if(indexLeg == R2 || indexLeg == L2){
      if(coo.Gamma <60){
        return false;
      }else if(coo.Gamma > 175){
        return false;
      }
      
      if(coo.Alpha < 5){
        return false;
      }else if(coo.Alpha > 120){
        return false;
      }
      
      if(coo.Beta < 40){
        return false;
      }else if(coo.Beta > 160){
        return false;
      }
  }else{
      if(coo.Gamma <5){
        return false;
      }else if(coo.Gamma > 120){
        return false;
      }
      
      if(coo.Alpha < 60){
        return false;
      }else if(coo.Alpha > 175){
        return false;
      }
      
      if(coo.Beta < 20){
        return false;
      }else if(coo.Beta > 140){
        return false;
      }
  }

  return true;
}

void moveTriangleR2(){
  AngleCoo firstStep = ConvertPointToAngle(90,10,0);
  AngleCoo calibration = ConvertPointToAngle(100,70,42);
  
  move(R2, firstStep);
  move(R1, calibration);
  move(L1, calibration);
  move(L2, calibration);

  delay(1000);
  AngleCoo secondStep = ConvertPointToAngle(140,10,0);
  move(R2, secondStep);
  
  delay(1000);
  AngleCoo thirdStep = ConvertPointToAngle(80,90,0);
  move(R2, thirdStep);
  delay(1000);
}

void sit(){
  AngleCoo assis = ConvertPointToAngle(70,50,0);
  move(R2, assis);
  move(R1, assis);
  move(L1, assis);
  move(L2, assis);
}

void stand(){
  AngleCoo debout = ConvertPointToAngle(70,50,-40);
  move(R2, debout);
  move(R1, debout);
  move(L1, debout);
  move(L2, debout);
}

void Walk(int nbr = 1){
  for (int i = 0; i < nbr; i++)
  {
    move(R2, ConvertPointToAngle(70, 100, -20));
    delay(300);
    move(R2, ConvertPointToAngle(70, 100, -40));
    delay(300);
    move(L1, ConvertPointToAngle(70, 0, -40));
    move(L2, ConvertPointToAngle(70, 100, -40));
    move(R1, ConvertPointToAngle(70, 100, -40));
    move(R2, ConvertPointToAngle(70, 50, -40));
    delay(500);
    move(R1, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(R1, ConvertPointToAngle(70, 50, -40));
    delay(500);
    move(L2, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(L2, ConvertPointToAngle(70, 50, -40));
    delay(500);
    move(L1, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(L1, ConvertPointToAngle(70, 50, -40));
    delay(500);
  }  
}


void TurnLeft(int nbr = 1){
  for (int i = 0; i < nbr; i++)
  {
    move(L1, ConvertPointToAngle(75, 25, -20));
    delay(300);
    move(L1, ConvertPointToAngle(75, 25, -40));
    delay(300);
    move(L2, ConvertPointToAngle(33, 70, -20));
    delay(300);
    move(L2, ConvertPointToAngle(33, 70, -40));
    delay(300);
    move(R1, ConvertPointToAngle(73, 25, -20));
    delay(300);
    move(R1, ConvertPointToAngle(73, 25, -40));
    delay(300);
    move(R2, ConvertPointToAngle(33, 70, -20));
    delay(300);
    move(R2, ConvertPointToAngle(33, 70, -40));
    delay(300);
    move(L1, ConvertPointToAngle(33, 70, -40));
    move(L2, ConvertPointToAngle(73, 25, -40));
    move(R1, ConvertPointToAngle(33, 70, -40));
    move(R2, ConvertPointToAngle(73, 25, -40));
    delay(500);
  }  
    move(L1, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(L1, ConvertPointToAngle(70, 50, -40));
    delay(300);
    move(L2, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(L2, ConvertPointToAngle(70, 50, -40));
    delay(300);
    move(R1, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(R1, ConvertPointToAngle(70, 50, -40));
    delay(300);
    move(R2, ConvertPointToAngle(70, 50, -20));
    delay(300);
    move(R2, ConvertPointToAngle(70, 50, -40));
    delay(300);
}

void Calibrage(){
  for (int i = 0; i < NUMBER_LEG; i++)
  {
    move(i, ConvertPointToAngle(100,70,42));
  }
  
}

void initServo(){
  for (int i = 0; i < NUMBER_LEG; i++)
  {
    for (int y = 0; y < NUMBER_SERVOS_BY_LEG; y++)
    {
      pinMode(servo[i][y].servoPin, OUTPUT);
      digitalWrite(servo[i][y].servoPin, LOW);

      delay(200);
  
      servo[i][y].servoIndex = RP2040_ISR_Servos.setupServo(servo[i][y].servoPin, MIN_MICROS, MAX_MICROS);

      if (servo[i][y].servoIndex != -1)
      {
        Serial.print(F("Setup OK Servo index = "));
        Serial.println(servo[i][y].servoIndex);

        RP2040_ISR_Servos.setPosition(servo[i][y].servoIndex, 0);
      }
      else
      {
        Serial.print(F("Setup Failed Servo index = "));
        Serial.println(servo[i][y].servoIndex);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  initServo();
  Serial.println("fin du setup");
  calculateOffset();
}

void loop() {
  Calibrage();
  delay(100);
  /*stand();
  delay(1000);
  TurnLeft(10);
  sit();
  delay(1000);*/
}
