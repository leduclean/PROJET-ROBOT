#ifndef ROBOT_H
#define ROBOT_H
#include "moteurs.h"
#include "ROBOT_CONFIG.h"
#include <Arduino.h>





class Robot {

public:
  // Attribut pubic
  // Constructeur
  Robot(int base_speed);

  // Méthodes pour les moteurs
  void set_robot_speed(int speed);
  void stop();
  void accel();
  void deccel();
  void right(bool pivot = false);
  void left(bool pivot = false);
  void rotate(int angle, bool move = false, bool backward = false);
  void update();
  void move_eight();
  void debug();




  // Méthodes pour les capteurs
  void initialize_ir();
  void decode_ir();
  void line_follower();



private:
  enum RobotState {
    IDLE,
    ROTATING,
    TURNING,
};
    enum class CommandeIR : unsigned long {
    FORWARD = 0xB847FF00,   // CH +
    BACKWARD = 0xBA45FF00,  // CH -
    STOP = 0xBC43FF00,      // Play/ Pause
    ACCEL = 0xEA15FF00,     // +
    DECCEL = 0xF807FF00,    // -
    RIGHT = 0xBF40FF00,     // >>
    LEFT = 0xBB44FF00,      // <<
    DEMITOUR = 0xE916FF00,  // 0
    EIGHT = 0xBB44FF00,
  };

  Moteur moteurD;  // Moteur droit
  Moteur moteurG;  // Moteur gauche
  RobotState state;
  unsigned long rotationStartTime;  // Temps de début de la rotation
  unsigned long rotationDuration;   // Durée de rotation calculée en ms
  int robot_speed;                  // Vitesse de base
  uint8_t CurrentLineSensorState;



};

#endif