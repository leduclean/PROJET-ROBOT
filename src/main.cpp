#include <Arduino.h>   
#include "Robot.h"
#include <IRremote.hpp>


// TODO -> trouver les bonnes valeurs des coefficients PID
// TODO -> tester une implémentatino sans sharp turning ou on tourne juste a 90 degré dans la dernière direction ou alors ralentir la vitesse 

Robot* robot;

void setup() {
  robot = new Robot(70);
  Serial.begin(9600);

}



void loop() {
  robot->decode_ir();
  robot->update(); // Ne surtout pas enlever ou erreur -> gere tout les timings des fonctions 
}

// Pour tourner a droite franchement: stoper la detection jusqua ce que celui de gauche voit 