#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <PID_v1.h>
#include "ROBOT_CONFIG.h"
#include "moteurs.h"

// ---------------------- Constantes de vitesse ---------------------- //
#define ROBOT_ACCELERATION_INCREMENT 20 // Incrément de vitesse lors de l’accélération
#define ROBOT_MAX_SPEED 250             // Vitesse maximale du robot
#define ROBOT_MIN_SPEED 70              // Vitesse minimale

// ----- États de mouvement et rotation -----
enum class GlobalState {
    IDLE,
    MOVING,
    ROTATING
};

enum MovementState { 
    MOVEMENT_IDLE,   // pour éviter toute confusion avec GlobalState::IDLE
    FORWARD, 
    BACKWARD, 
    LINEFOLLOWING, 
    SHARPTURNING, 
    FIGURE_EIGHT 
};

enum RotationState { 
    ROTATION_IDLE,   // à utiliser au lieu de NONE pour plus de clarté
    TURNING, 
    ROTATING 
};

// ----- Types d'événements (issus par exemple du décodage IR) -----
enum RobotEvent {
    EVENT_NONE,
    EVENT_IR_FORWARD,
    EVENT_IR_BACKWARD,
    EVENT_IR_LEFT,
    EVENT_IR_RIGHT,
    EVENT_IR_STOP,
    EVENT_IR_ACCEL,
    EVENT_IR_DECCEL,
    EVENT_IR_DEMITOUR,
    EVENT_IR_EIGHT,
    EVENT_IR_LINEFOLLOWER,
    EVENT_IR_INCREASEKP,
    EVENT_TIMER_ROTATION_END // événement interne déclenché par le timer de rotation
};

class Robot {
   private:
    enum TurnDirection { NONE_DIR, RIGHT, LEFT };
    enum EightStep { EIGHT_NONE, EIGHT_FIRST, EIGHT_SECOND };

    enum class CommandeIR : unsigned long {
        FORWARD       = 0xB847FF00,  // CH +
        BACKWARD      = 0xBA45FF00,  // CH -
        STOP          = 0xBC43FF00,  // Play/Pause
        ACCEL         = 0xEA15FF00,  // +
        DECCEL        = 0xF807FF00,  // -
        RIGHT         = 0xBF40FF00,  // >>
        LEFT          = 0xBB44FF00,  // <<
        DEMITOUR      = 0xE916FF00,  // 0
        EIGHT         = 0xAD52FF00,
        LINEFOLLOWER  = 0xF609FF00,
        INCREASEKP    = 0xE619FF00
    };

    Moteur moteurD;  // Moteur droit
    Moteur moteurG;  // Moteur gauche
    int robot_speed; // Vitesse courante
    int base_speed;
    GlobalState globalState;
    MovementState movementState; // État de mouvement
    RotationState rotationState; // État de rotation
    MovementState previousMovementState; // Sauvegarde de l'état avant rotation

    unsigned long rotationStartTime;  // Temps de début de la rotation
    unsigned long rotationDuration;   // Durée de rotation calculée en ms
    uint8_t CurrentLineSensorState;
    TurnDirection lastTurnDirection;
    EightStep currentEightStep = EIGHT_NONE;

    // Variables et coefficients pour le PID
    double pid_input;    // L'erreur
    double pid_output;   // La correction calculée par le PID
    double pid_setpoint; // La consigne (souvent 0)
    PID pidController;   // L'objet PID
    double pidKp;
    double pidKi;
    double pidKd;
    float Ku = 90;

    bool measurementDone = false; // Flag indiquant que la mesure est terminée

   public:
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
    void brake();

    // Gestion des états
    void syncStates();
    void changeMovementState(MovementState newMovementState);
    void changeRotationState(RotationState newRotationState);
    void update();
    void updateMovement();
    void updateRotation();
    void handleEvent(RobotEvent event);
    void printState();
    
    // Mouvements complexes
    void move_eight();
    
    // Méthodes pour les capteurs
    void initialize_ir();
    void decode_ir();
    void initialize_line_pin();
    void sharpturn();
    void line_follower_pid();
    float errorestimation();
    void resetPID();

    // Autotune / Mesure
    unsigned long measureOscillationPeriod();
    void updatetuning();
};

#endif
