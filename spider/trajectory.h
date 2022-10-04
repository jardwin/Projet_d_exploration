#pragma once
#include <Arduino.h>

#define TRAJ_MAX_LENGTH 20

struct Path {
enum:uint8_t{SIT, STAND, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT} type;
     uint8_t number; //Repeat number of order
};

class Trajectory {
    public:

        /*
         * Method to add high level orders to the array
         */
        void setup();

        /*
         * Reset the counter of the current processing order
         */
        void reset() {
            cur_index = 0;
        };

        void step_forward(uint8_t n) { addTraj({Path::FORWARD,    n}); }
        void step_back(uint8_t n)    { addTraj({Path::BACKWARD,   n}); }
        void turn_left(uint8_t n)    { addTraj({Path::TURN_LEFT,  n}); }
        void turn_right(uint8_t n)   { addTraj({Path::TURN_RIGHT, n}); }
        void sit()                   { addTraj({Path::SIT,        1}); }
        void stand()                 { addTraj({Path::STAND,      1}); }

        /*
         * Util function to add a node to the trajectory
         * next - path order to enqueue
         */
        void addTraj(Path next) {
            if (traj_length >= TRAJ_MAX_LENGTH)
                return;

            traj[traj_length].type = next.type;
            traj[traj_length].number = next.number;
            traj_length++;
        }

        /*
         * Get next order to execute to process the trajectory
         */
        bool getNext(Path& next) {
            if (cur_index < traj_length) {
                next.type = traj[cur_index].type;
                next.number = traj[cur_index].number;
                cur_index++;
                return true;
            }
            return false;
        }

    private:
        Path traj[TRAJ_MAX_LENGTH];
        short traj_length;
        short cur_index;
};
