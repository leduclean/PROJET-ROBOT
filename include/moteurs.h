#ifndef MOTEURS_H
#define MOTEURS_H

#include "ROBOT_CONFIG.h"
#include <stdint.h>  // Pour les types uint8_t, int16_t, int8_t etc.

// ---------------------- Constante de vitesse ---------------------- //
const uint8_t MAX_SPEED = 255; // Vitesse maximale (rapport cyclique maximal du signal PWM)

// Déclaration de la classe Moteur
class Moteur {
   private:
    int16_t speed;         // Vitesse du moteur (-255 à 255)
    uint8_t pin_forward;   // Broche pour avancer
    uint8_t pin_backward;  // Broche pour reculer
    uint8_t PWM_pin;       // Broche PWM
    float correction;      // Correction (pour ajustement de la vitesse)
    int8_t last_direction; // 0 : non initialisé, 1 : forward, -1 : backward

   public:
    // Constructeur avec types optimisés
    Moteur(int16_t speed, uint8_t pin_forward, uint8_t pin_backward, uint8_t PWM_pin, float correction);

    void set_speed(int16_t speed);
    int16_t get_speed();
    void stop_engine(); // Permet d'arrêter le moteur
    void debug();
};

#endif
