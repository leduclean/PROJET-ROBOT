#include "Robot.h"
#include <IRremote.hpp>


// TODO -> choisir les bons delay pour la fonction height et right et left

Robot* robot;

void setup() {
  robot = new Robot(170);
  Serial.begin(9600);

}



void loop() {
  robot->line_follower();
  robot->decode_ir();
  robot->update(); // Ne surtout pas enlever ou erreur -> gere tout les timings des fonctions 


}
