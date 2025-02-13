#ifndef ROBOT_H
#define ROBOT_H
#include "moteurs.h"
#include "line_follower.h"
#include "ROBOT_CONFIG.h"
#include <Arduino.h>





class Robot {

private:
enum TurnDirection {NONE, RIGHT, LEFT};
enum RobotMovementState { MOVEMENT_IDLE, FORWARD, BACKWARD, LINEFOLLOWING, SHARPTURNING};
enum RobotRotationState { ROTATION_IDLE, TURNING, ROTATING };

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
    LINEFOLLOWER = 0xF609FF00

  };

  Moteur moteurD;  // Moteur droit
  Moteur moteurG;  // Moteur gauche
  RobotMovementState movementState; // etat de mouvement
  RobotRotationState rotationState; // etat de rotation
  TurnDirection lastTurnDirection;
  unsigned long rotationStartTime;  // Temps de début de la rotation
  unsigned long rotationDuration;   // Durée de rotation calculée en ms
  int robot_speed;                  // Vitesse de base
  uint8_t CurrentLineSensorState;
  int base_speed;

  // pid value
  float pidKp = 72.0;
  float pidKi = 0.00015;
  float pidKd = 5.0;
  float pidIntegral = 0.0;
  float pidLastError = 0.0;
  unsigned long pidLastTime = 0;


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
  void changeMovementState(RobotMovementState newMovementState);
  void changeRotationState(RobotRotationState newRotationState);
  void update();
  void move_eight();
  void debug();
  void printState();
  
  



  // Méthodes pour les capteurs
  void initialize_ir();
  void decode_ir();
  void initialize_line_pin();
  uint8_t getSensorState();
  void line_follower();
  void sharpturn();
  void line_follower_pid();
  void resetPID();





};

#endif