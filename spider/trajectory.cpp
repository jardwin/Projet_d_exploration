#include "trajectory.h"

void Trajectory::setup()
{
    stand();
    step_forward(10);
    turn_left(3);
    step_back(10);
    turn_right(3);
    sit();
}
