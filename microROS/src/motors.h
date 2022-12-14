#pragma once

#include <Servo.h>
#include <Arduino.h>

#define MOTORS_NUMBER 12

const int servo_emergency[MOTORS_NUMBER] = {
  90, 90, 160,
  90, 90, 20,
  90, 90, 20,
  90, 90, 160
};

const int servo_limit[MOTORS_NUMBER][2] = {
  {60, 180}, {0, 120}, {20, 160}, //R2
  {5, 120}, {60, 181}, {20, 160}, //R1
  {5, 120}, {60, 181}, {20, 160}, //L1
  {60, 180}, {0, 120}, {20, 160}, //L2
};

template<int N>
class Motors{
    public:
        /*
         *  Create a N motors controller.
         *  motor will be called by index 0..N and associate with corresponding pin
         * Only need to work with generic 0..N pins no hardware pin
         */
        Motors() { }

        /*
         * Attach the provided pins to the corresponding servo instance
         */
        void attachServo(const int pins[N]) {
            for (int i = 0; i < N; i++) {
                servos[i].attach(pins[i]);
            }
        }

        /*
         * Get the current estimated angle position of the servo motor
         */
        float getAngle(int index) const {
            return servos[index].read();
        }

        /*
         * Set the servo motor angle
         * index - number of the servo motor from 0 to N
         * angle - angle in degree to apply
         * 
         * This function will check ranges and apply emergency position if outside
         */
        void setServo(int index, float angle) {

            if (angle < servo_limit[index][0] || servo_limit[index][1] < angle) {
              Serial.println("Servo set bad position");
              Serial.println(angle);
              Serial.println(index);
              
              emergency();
              
            }

            servos[index].write(angle);
            //Serial.println(angle);
        }

        /*
         * Emergency abort
         * Will place the spider in safety position and exit code
         */
        void emergency() {
          for (int j = 2; j >= 0; j--) {
            for (int i = 0; i < 4; i++) { //4 pattes
              servos[i*3 + j].write(servo_emergency[i*3 + j]);
            }
            delay(300);
          }
          exit(1);
        }

    private:
        Servo servos[N];

};
