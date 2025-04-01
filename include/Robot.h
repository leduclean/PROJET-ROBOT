#ifndef ROBOT_H
#define ROBOT_H
#include "moteurs.h"
#include "ROBOT_CONFIG.h"
#include <Arduino.h>
#include <PID_v1.h>

// ---------------------- Constantes de vitesse ---------------------- //

#define ROBOT_ACCELERATION_INCREMENT 20   // Incrément de vitesse lors de l’accélération
#define ROBOT_MAX_SPEED    250 // Vitesse maximale du robot
#define ROBOT_MIN_SPEED    70 // vitesse minimale 





class Robot {

private:
enum TurnDirection {NONE, RIGHT, LEFT};
enum RobotMovementState { MOVEMENT_IDLE, FORWARD, BACKWARD, LINEFOLLOWING, SHARPTURNING, FIGURE_EIGHT};
enum RobotRotationState { ROTATION_IDLE, TURNING, ROTATING };
enum EightStep {
  EIGHT_NONE,
  EIGHT_FIRST,
  EIGHT_SECOND
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
    EIGHT = 0xAD52FF00,
    LINEFOLLOWER = 0xF609FF00,
    INCREASEKP = 0xE619FF00,

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
  EightStep currentEightStep = EIGHT_NONE;




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
  float Ku = 90;

  bool measurementDone = false;        // Flag indiquant que la mesure est terminée

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
  // line follower 
  void initialize_line_pin();
  void sharpturn();
  void line_follower_pid();
  float errorestimation();
  void resetPID();
    // autotune
    unsigned long measureOscillationPeriod();
    void updatetuning();
    void brake();


};

#endif