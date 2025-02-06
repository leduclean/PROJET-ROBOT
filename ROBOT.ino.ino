#include "Robot.h"
#include <IRremote.hpp>


// TODO -> changer créer un état arriere 

Robot* robot;

void setup() {
  robot = new Robot(110);
  Serial.begin(9600);

}



void loop() {
  robot->decode_ir();
  robot->update(); // Ne surtout pas enlever ou erreur -> gere tout les timings des fonctions 

}