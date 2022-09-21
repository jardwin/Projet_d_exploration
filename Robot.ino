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

const int R2 = 0;
const int R1 = 1;
const int L1 = 2;
const int L2 = 3;

const double LH = 30.5;
const double LF = 53;
const double LT = 79.5;

AngleCoo ConvertPointToAngle(int x, int y, int z){
  int angleH = atan(y/x);
  int hypo = sqrt(pow(x,2)+pow(y,2));
  int v = hypo - LH;
  int d = sqrt(pow(v,2)+pow(z,2));
  int alphaOne = atan(z / v);
  int alphaTwo = acos((pow(LF,2)+ pow(d,2) - pow(LT,2))/(2*LF*LT));
  int alpha = alphaOne+alphaTwo;
  int beta = acos((pow(LT,2)+pow(LF,2)-pow(d,2))/(2*LT*LF));
  return AngleCoo(angleH, alpha, beta);
}

void move(int indexLeg, AngleCoo coo)
{
  switch (indexLeg)
  {
    case 0:
      coo.Gamma += 90;
      coo.Alpha = 90 - coo.Alpha;
      break;
    case 1:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      break;
    case 2:
      coo.Gamma = 90 - coo.Gamma;
      coo.Alpha = 90 + coo.Alpha;
      break;
    case 3:
      coo.Gamma += 90;
      coo.Alpha = 90 - coo.Alpha;
      break;
    default:
      break;
  }
  coo = checkAngle(indexLeg, coo);
  servo[indexLeg][0].write(coo.Gamma);
  servo[indexLeg][1].write(coo.Alpha);
  servo[indexLeg][2].write(coo.Beta);
}

AngleCoo checkAngle(int indexLeg, AngleCoo coo){
  AngleCoo newAngle;
  switch (indexLeg)
  {
    case 0:
      if(coo.Gamma <60){
        newAngle.Gamma = 60;
      }else if(coo.Gamma > 175){
        newAngle.Gamma = 175;
      }
      
      if(coo.Alpha < 5){
        newAngle.Alpha = 5;
      }else if(coo.Alpha > 120){
        newAngle.Alpha = 120;
      }
      
      if(coo.Beta < 40){
        newAngle.Beta = 40;
      }else if(coo.Beta > 160){
        newAngle.Beta = 160;
      }
      break;
    case 1:
      if(coo.Gamma <5){
        newAngle.Gamma = 5;
      }else if(coo.Gamma > 120){
        newAngle.Gamma = 120;
      }
      
      if(coo.Alpha < 60){
        newAngle.Alpha = 60;
      }else if(coo.Alpha > 175){
        newAngle.Alpha = 175;
      }
      
      if(coo.Beta < 20){
        newAngle.Beta = 20;
      }else if(coo.Beta > 140){
        newAngle.Beta = 140;
      }
      break;
    case 2:
      if(coo.Gamma <5){
        newAngle.Gamma = 5;
      }else if(coo.Gamma > 120){
        newAngle.Gamma = 120;
      }
      
      if(coo.Alpha < 60){
        newAngle.Alpha = 60;
      }else if(coo.Alpha > 175){
        newAngle.Alpha = 175;
      }
      
      if(coo.Beta < 20){
        newAngle.Beta = 20;
      }else if(coo.Beta > 140){
        newAngle.Beta = 140;
      }
      break;
    case 3:
      if(coo.Gamma <60){
        newAngle.Gamma = 60;
      }else if(coo.Gamma > 175){
        newAngle.Gamma = 175;
      }
      
      if(coo.Alpha < 5){
        newAngle.Alpha = 5;
      }else if(coo.Alpha > 120){
        newAngle.Alpha = 120;
      }
      
      if(coo.Beta < 40){
        newAngle.Beta = 40;
      }else if(coo.Beta > 160){
        newAngle.Beta = 160;
      }
      break;
    default:
    return coo;
  }
  return newAngle;
}

void setup() {
  for (int i = 0; i < 4; i++)
  {
    for (int y = 0; y < 3; y++)
    {
      servo[i][y].attach(servo_pin[i][y]);
    }
  }
}

void loop() {
  AngleCoo toto = ConvertPointToAngle(100,70,42);
  move(R2, toto);
  move(R1, toto);
  move(L1, toto);
  move(L2, toto);
}