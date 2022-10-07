#include <Arduino.h>
#include "motors.h"
#include "armcontroller.h"
#include "body.h"

//List of servo pins connected to the arduino
// Hanche / Femur / Tibia
int servo_pin[12] = { 4,  2,  3,
    7,  5,  6,
    16,  14, 15,
    19, 17, 18
};

//Controller motors
Motors<12> motors;

// On choisit 4 bras qui ont chacun 3 axes
ArmController<4, 3> armController{motors};
Vectorf corrections[] = {{103,53,33},{95,63,28},{105,66,75},{96,65,46}}; // Default
//Vectorf corrections[] = {{103,53,45},{95,63,40},{105,66,97},{96,65,58}}; // Default
//Vectorf corrections[] = {{102,53,26}, {92,67,15}, {107,80,61}, {101,58,33}}; // Default
//Vectorf corrections[] = {{114, 73, 59}, {90, 71, 58}, {103, 83, 50}, {115, 75, 54}}; // Bleue
//Vectorf corrections[] = {{106, 84, 62}, {109, 85, 45}, {92, 87, 40}, {104, 69, 30}}; // grise

// On choisit le modèle d'arraignée (toutes les dimensions)
SpiderJDMI model;
//On utilise un body à 4 bras
Body4 body(armController, model);

// L'object qui gère et stock la trajectoire
Trajectory trajectory;

void setup()
{
    Serial.begin(115200);
    motors.attachServo(servo_pin); //First things to do. Arms will go to neutral
    body.init(corrections);
    trajectory.setup();
}

//* // Marche
void loop()
{
    trajectory.reset(); //Reset counter to 0
    body.process(trajectory);
    armController.process_orders();
}
//*/

/* //Calibration
   void loop()
   {
   for (int i = 0; i < 4; i++) {
   armController.addPosition({Order::POS, i, 10000, {100, 70, 15}});
   armController.addPosition({Order::WAIT});
   }
   armController.process_orders();
   delay(1000);
   }
//*/

/* //Avancer
   void loop()
   {
   float speed = 200;
   speed = model.leg_move_speed;
   armController.addPosition({Order::POS, 0, speed, {KEEP, KEEP, model.z_up}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 0, speed, {model.x_default, model.y_default + (2.) * model.y_step, KEEP}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 0, speed, {KEEP, KEEP, model.z_default}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);

   speed = model.body_move_speed;
   armController.addPosition({Order::POS, 0, speed, {model.x_default, model.y_default + (1.) * model.y_step, model.z_default}});
   armController.addPosition({Order::POS, 1, speed, {model.x_default, model.y_default + (2.) * model.y_step, model.z_default}});
   armController.addPosition({Order::POS, 2, speed, {model.x_default, model.y_default + (0.) * model.y_step, model.z_default}});
   armController.addPosition({Order::POS, 3, speed, {model.x_default, model.y_default + (2.) * model.y_step, model.z_default}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);

   speed = model.leg_move_speed;
   armController.addPosition({Order::POS, 1, speed, {KEEP, KEEP, model.z_up}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 1, speed, {model.x_default, model.y_default + (1.) * model.y_step, KEEP}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 1, speed, {KEEP, KEEP, model.z_default}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);

   armController.addPosition({Order::POS, 3, speed, {KEEP, KEEP, model.z_up}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 3, speed, {model.x_default, model.y_default + (1.) * model.y_step, KEEP}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 3, speed, {KEEP, KEEP, model.z_default}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);

   armController.addPosition({Order::POS, 2, speed, {KEEP, KEEP, model.z_up}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 2, speed, {model.x_default, model.y_default + (1.) * model.y_step, KEEP}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 2, speed, {KEEP, KEEP, model.z_default}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);
   }
//*/

/* //Mouvement
   void loop()
   {

   for (int i = 0; i < 4; i++) {
   armController.addPosition({Order::POS, i, 1000, {100, 80, 15}});
   }

   float speed = 200;
   armController.addPosition({Order::POS, 0, speed, {140, 10, -27}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 0, speed, {90, 10, -27}});
   armController.addPosition({Order::WAIT});
   armController.addPosition({Order::POS, 0, speed, {90, 80, -27}});
   armController.addPosition({Order::WAIT});

   armController.process_orders();
   delay(1000);
   }
//*/
