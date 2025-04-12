#include "moteurs.h"

#include <Arduino.h>

// Constructeur avec initialisation de la speed
Moteur::Moteur(int16_t speed, uint8_t pin_forward, uint8_t pin_backward, uint8_t PWM_pin, float correction) :
    speed(speed), pin_forward(pin_forward), pin_backward(pin_backward), PWM_pin(PWM_pin), correction(correction) {
    // On déclare les pins associés au controle du moteur en mode sortie car on va envoyer une tension au controleur de
    // moteur
    pinMode(pin_forward, OUTPUT);
    pinMode(pin_backward, OUTPUT);
    pinMode(PWM_pin, OUTPUT);
};

// Gere la vitesse du robot et la direction: si positif: avant, si négatif: arriere
void Moteur::set_speed(int16_t speed) {
    // Calculer la vitesse corrigée (toujours positive)
    int correctedSpeed = abs(speed) * this->correction;
    this->speed = correctedSpeed;
    // Déterminer la direction : -1 pour reculer, 1 pour avancer
    int newDirection = (speed < 0) ? -1 : 1;

    // Mettre à jour les broches de direction uniquement si le signe a changé
    if (newDirection != this->last_direction) {
        digitalWrite(this->pin_forward, (newDirection == 1) ? HIGH : LOW);
        digitalWrite(this->pin_backward, (newDirection == 1) ? LOW : HIGH);
        this->last_direction = newDirection;
    }

    // Appliquer la vitesse via le signal PWM
    analogWrite(this->PWM_pin, correctedSpeed);
}

int16_t Moteur::get_speed() {
    return this->speed;
}

void Moteur::stop_engine() {
    digitalWrite(this->pin_forward, LOW);
    digitalWrite(this->pin_backward, LOW);
}

void Moteur::debug() {
    Serial.println("Moteur speed :" + String(this->speed));
    Serial.println("Moteur pin_forward : " + String(this->pin_forward));
    Serial.println("Moteur pin_backward : " + String(this->pin_backward));
    Serial.println("Moteur PWM_pin : " + String(this->PWM_pin));
}
