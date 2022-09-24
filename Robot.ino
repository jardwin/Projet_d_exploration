#include <Servo.h>
#include <math.h>

class AngleCoo
{
  public:
    int Gamma;
    int Alpha;
    int Beta;
    AngleCoo(){
      Gamma = 0;
      Alpha = 0;
      Beta = 0;
    }
    AngleCoo(int gamma,int alpha,int beta){
      Gamma = gamma;
      Alpha = alpha;
      Beta = beta;
    }
};


/* Array storing the 12 servos */
Servo servo[4][3]; // 4 legs of 3 segments each

/* Array of pins connected to the servo */
const int servo_pin[4][3] = { 
  { 4, 2, 3 }, // Leg 1 - Front right (R2) - {H, F, T}
  { 7, 5, 6 }, // Leg 2 - Back right (R1) - {H, F, T}
  { 16, 14, 15 }, // Leg 3 - Front left (L1) - {H, F, T}
  { 19, 17, 18 } // Leg 4 - Back left (L2) - {H, F, T}
};
/*
34
63
85
*/

/*const int offset[4][3] = {
  {6,-19,16},
  {0,-7,13},
  {5,9,-13},
  {0,-16,1}
};*/

int offset[4][3];

const int realPosition[4][3] = {
  {96,52,43},
  {93,63,30},
  {111,64,61}, 
  {94,65,65}
};

const int R2 = 0;
const int R1 = 1;
const int L1 = 2;
const int L2 = 3;

const double LH = 30.5;
const double LF = 53;
const double LT = 79.5;

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

  offset[0][0] = realR2.Gamma < target.Gamma ? target.Gamma-realR2.Gamma : realR2.Gamma-target.Gamma;
  offset[0][1] = realR2.Alpha > target.Alpha ? target.Alpha-realR2.Alpha : realR2.Alpha-target.Alpha;
  offset[0][2] = realR2.Beta < target.Beta ? target.Beta-realR2.Beta : realR2.Beta-target.Beta;

  Serial.println("R1");
  Serial.println(realR1.Gamma);
  Serial.println(realR1.Alpha);
  Serial.println(realR1.Beta);

  offset[1][0] = realR1.Gamma < target.Gamma ? target.Gamma-realR1.Gamma : realR1.Gamma-target.Gamma;
  offset[1][1] = realR1.Alpha > target.Alpha ? target.Alpha-realR1.Alpha : realR1.Alpha-target.Alpha;
  offset[1][2] = realR1.Beta < target.Beta ? target.Beta-realR1.Beta : realR1.Beta-target.Beta;

  Serial.println("L1");
  Serial.println(realL1.Gamma);
  Serial.println(realL1.Alpha);
  Serial.println(realL1.Beta);
    
  offset[2][0] = realL1.Gamma < target.Gamma ? target.Gamma-realL1.Gamma : realL1.Gamma-target.Gamma;
  offset[2][1] = realL1.Alpha < target.Alpha ? target.Alpha-realL1.Alpha : realL1.Alpha-target.Alpha;
  offset[2][2] = realL1.Beta > target.Beta ? target.Beta-realL1.Beta : realL1.Beta-target.Beta;

  Serial.println("L2");
  Serial.println(realL2.Gamma);
  Serial.println(realL2.Alpha);
  Serial.println(realL2.Beta);
    
  offset[3][0] = realL2.Gamma < target.Gamma ? target.Gamma-realL2.Gamma : realL2.Gamma-target.Gamma;
  offset[3][1] = realL2.Alpha > target.Alpha ? target.Alpha-realL2.Alpha : realL2.Alpha-target.Alpha;
  offset[3][2] = realL2.Beta < target.Beta ? target.Beta-realL2.Beta : realL2.Beta-target.Beta;

  for (int i = 0; i < 4; i++)
  {
    Serial.println("");
    for (int y = 0; y < 3; y++)
    {
      Serial.println(offset[i][y]);
    }
  }  
}

void setPLS(){
  servo[0][0].write(90);
  servo[1][0].write(90);
  servo[2][0].write(90);
  servo[3][0].write(90);

  servo[0][1].write(90);
  servo[1][1].write(90);
  servo[2][1].write(90);
  servo[3][1].write(90);

  servo[0][2].write(160);
  servo[1][2].write(20);
  servo[2][2].write(20);
  servo[3][2].write(160);
}

AngleCoo ConvertPointToAngle(float x, float y, float z){
  z -= 27;
  int gamma = atan(y/x)*(180/M_PI);
  int hypo = sqrt(pow(x,2)+pow(y,2));
  int v = hypo - LH;
  int d = sqrt(pow(v,2)+pow(z,2));
  int alphaOne = atan(z / v) * (180/M_PI);
  int alphaTwo = acos((pow(LF,2)+ pow(d,2) - pow(LT,2))/(2*LF*LT)) * (180/M_PI);
  int alpha = alphaOne+alphaTwo;
  int beta = acos((pow(LT,2)+pow(LF,2)-pow(d,2))/(2*LT*LF)) * (180/M_PI);
  return AngleCoo(gamma, alpha, beta);
}

void move(int indexLeg, AngleCoo coo)
{
  switch (indexLeg)
  {
    case R2:
      coo.Gamma = 90 + coo.Gamma;
      coo.Alpha = 90 - coo.Alpha;
      break;
    case R1:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      coo.Beta = 180 - coo.Beta;
      break;
    case L1:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      coo.Beta = 180 - coo.Beta;
      break;
    case L2:
      coo.Gamma = 90 + coo.Gamma;
      coo.Alpha = 90 - coo.Alpha;
      break;
    default:
      break;
  }

  if(checkAngle(indexLeg, coo)){
    servo[indexLeg][0].write(coo.Gamma+offset[indexLeg][0]);
    servo[indexLeg][1].write(coo.Alpha+offset[indexLeg][1]);
    servo[indexLeg][2].write(coo.Beta+offset[indexLeg][2]);
  }else{
    setPLS();
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

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++)
  {
    for (int y = 0; y < 3; y++)
    {
      servo[i][y].attach(servo_pin[i][y]);
    }
  }
  Serial.println("fin du setup");
  calculateOffset();
}

void loop() {
  AngleCoo calibration = ConvertPointToAngle(100,70,42);
  //AngleCoo calibration = AngleCoo(34.99, 68.08, 86.46);
  move(R2, calibration);
  move(R1, calibration);
  move(L1, calibration);
  move(L2, calibration);
}