#ifndef CONFIG_ROBOT_H
#define CONFIG_ROBOT_H

// ---------------------- Définition des broches des moteurs ---------------------- //

// Moteur gauche
#define MOTOR_LEFT_FORWARD_PIN  9   // Broche pour avancer (IN1)
#define MOTOR_LEFT_BACKWARD_PIN 8   // Broche pour reculer (IN2)
#define MOTOR_LEFT_SPEED_PIN    6   // Broche PWM pour régler la vitesse (ENA)

// Moteur droit
#define MOTOR_RIGHT_FORWARD_PIN 11  // Broche pour avancer (IN3)
#define MOTOR_RIGHT_BACKWARD_PIN 10  // Broche pour reculer (IN4)
#define MOTOR_RIGHT_SPEED_PIN   5   // Broche PWM pour régler la vitesse (ENB)

// ---------------------- Définition des capteurs ---------------------- //

// Récepteur infrarouge
#define IR_RECEIVER_PIN A4  

// Suiveur de Ligne 

#define LINE_FOLLOWER_LEFT_PIN 4
#define LINE_FOLLOWER_MID_PIN 2
#define LINE_FOLLOWER_RIGHT_PIN 3


// ---------------------- Constantes de vitesse ---------------------- //

#define ROBOT_ACCELERATION_INCREMENT 20   // Incrément de vitesse lors de l’accélération
#define ROBOT_MAX_SPEED    250  // Vitesse maximale du robot

#endif // CONFIG_ROBOT_H
