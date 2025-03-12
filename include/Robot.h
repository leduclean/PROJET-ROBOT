#ifndef ROBOT_H
#define ROBOT_H
#include "moteurs.h"
#include "ROBOT_CONFIG.h"
#include <Arduino.h>
#include <PID_v1.h>





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
  int robot_speed;                  // Vitesse courante
  int base_speed;
  RobotMovementState movementState; // etat de mouvement
  RobotRotationState rotationState; // etat de rotation
  unsigned long rotationStartTime;  // Temps de début de la rotation
  unsigned long rotationDuration;   // Durée de rotation calculée en ms
  uint8_t CurrentLineSensorState;
  TurnDirection lastTurnDirection;



  // float pidKp = 37.0;
  // float pidKi = 0;
  // float pidKd = 0;

  double pid_input;    // la variable qui recevra l'erreur
  double pid_output;   // la correction calculée par le PID
  double pid_setpoint; // la consigne (souvent 0 pour une erreur nulle)
  PID pidController;   // l'objet PID

  // Les coefficients PID
  double pidKp;
  double pidKi;
  double pidKd;

  unsigned long lastComputeTime = 0;  // Variable globale

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
  // Line follower 
  uint8_t getSensorState();
  void line_follower();
  void sharpturn();
  void line_follower_pid();
  float errorestimation();
  void resetPID();
    // autotune
    void autoTunePID();





};

#endif