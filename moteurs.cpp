#include "moteurs.h"
#include <Arduino.h>


// Constructeur avec initialisation de la speed
Moteur::Moteur(int speed, int pin_forward, int pin_backward, int PWM_pin, float correction)
    : speed(speed), pin_forward(pin_forward), pin_backward(pin_backward), PWM_pin(PWM_pin), correction(correction) {
      // On déclare les pins associés au controle du moteur en mode sortie car on va envoyer une tension au controleur de moteur
      pinMode(pin_forward, OUTPUT);
      pinMode(pin_backward, OUTPUT);
      pinMode(PWM_pin, OUTPUT);
    };
#define LINE_FOLLOWER_LEFT_SENSOR_PIN   A0
#define LINE_FOLLOWER_MID_SENSOR_PIN    A1
#define LINE_FOLLOWER_RIGHT_SENSOR_PIN  A2
// Méthode pour définir une nouvelle vitesse

// Gere la vitesse du robot et la direction: si positif: avant, si négatif: arriere
void Moteur::set_speed(int speed) {
    this->speed = abs(speed) * this->correction; // Toujours travailler avec la valeur absolue pour PWM

    // Détermine la direction en fonction du signe de speed
    if (speed < 0) {
        digitalWrite(this->pin_forward, LOW);
        digitalWrite(this->pin_backward, HIGH);
        
        Serial.print("!!!!");

    } else {
        digitalWrite(this->pin_forward, HIGH);
        digitalWrite(this->pin_backward, LOW);

    }


    // Applique la vitesse corrigée
    analogWrite(this->PWM_pin, this->speed);
}

void Moteur::stop_engine(){
  digitalWrite(this->pin_forward, LOW);
  digitalWrite(this->pin_backward, LOW);

}


void Moteur::debug(){
  Serial.println("Moteur speed :" + String(this->speed));
  Serial.println("Moteur pin_forward : " +  String(this->pin_forward));
  Serial.println("Moteur pin_backward : " + String(this->pin_backward));
  Serial.println("Moteur PWM_pin : " + String(this->PWM_pin));

}
