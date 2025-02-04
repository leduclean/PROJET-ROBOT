#ifndef MOTEURS_H
#define MOTEURS_H
#include "ROBOT_CONFIG.h"


// ---------------------- Constantes de vitesse ---------------------- //
const int MAX_SPEED = 255; // Vitesse maximale du moteur (rapport cyclique maximal du signal PWM)



// Déclarations des fonctions liées aux moteurs
class Moteur {
public:
    int speed; // attribut de vitesse associé à chaque moteur
    // Declaration des sorties associées au moteur
    int pin_forward;
    int pin_backward;
    int PWM_pin;
    float correction;

  Moteur(int speed, int pin_forward, int pin_backward, int PWM_pin, float correction); // Déclaration du constructeur

  void set_speed(int speed);
  void stop_engine(); // méthode qui permet d'arreter le moteur 
  void debug();

};


#endif