#include "Robot.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

// ---------------------- Constructeur et initialisation ---------------------- //

Robot::Robot(int base_speed) :
    moteurD(base_speed, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_BACKWARD_PIN, MOTOR_RIGHT_SPEED_PIN, 0.94),
    moteurG(base_speed, MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_BACKWARD_PIN, MOTOR_LEFT_SPEED_PIN, 1),
    robot_speed(base_speed),
    base_speed(base_speed),
    globalState(GlobalState::IDLE),
    movementState(MOVEMENT_IDLE),
    rotationState(ROTATION_IDLE),
    previousMovementState(MOVEMENT_IDLE),
    rotationStartTime(0),
    rotationDuration(0),
    CurrentLineSensorState(0),
    lastTurnDirection(NONE_DIR),
    currentEightStep(EIGHT_NONE),
    pid_input(0),
    pid_output(0),
    pid_setpoint(0),
    pidController(&pid_input, &pid_output, &pid_setpoint, 19, 85, 20, DIRECT), // Coefficients initiaux
    pidKp(19),
    pidKi(85),
    pidKd(20) {
    initialize_ir();
    initialize_line_pin();
    // Configuration du PID
    pid_setpoint = 0;                // On cherche à avoir une erreur nulle
    pidController.SetSampleTime(20); // 20 ms entre chaque calcul du PID
    pidController.SetOutputLimits(-250, 250);
    pidController.SetMode(AUTOMATIC);
}

// ---------------------- Initialisation des capteurs ---------------------- //

void Robot::initialize_ir() {
    Serial.println("Initialisation IR");
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
}

void Robot::initialize_line_pin() {
    pinMode(LINE_FOLLOWER_LEFT_PIN, INPUT);
    pinMode(LINE_FOLLOWER_MID_PIN, INPUT);
    pinMode(LINE_FOLLOWER_RIGHT_PIN, INPUT);
}

// ---------------------- Fonctions de déplacement linéaire ---------------------- //

void Robot::set_robot_speed(int speed) {
    robot_speed = abs(speed);
    moteurD.set_speed(speed);
    moteurG.set_speed(speed);
}

void Robot::stop() {
    set_robot_speed(0);
}

void Robot::accel() {
    int new_speed = robot_speed + ROBOT_ACCELERATION_INCREMENT;
    if (new_speed > ROBOT_MAX_SPEED) new_speed = ROBOT_MAX_SPEED;
    set_robot_speed((movementState == BACKWARD) ? -new_speed : new_speed);
}

void Robot::deccel() {
    int new_speed = robot_speed - ROBOT_ACCELERATION_INCREMENT;
    set_robot_speed((movementState == BACKWARD) ? -new_speed : new_speed);
}

// ---------------------- Fonctions de rotation / virage ---------------------- //

void Robot::right(bool pivot) {
    // Si pivot est true, rotation sur place, sinon rotation en mouvement
    rotate(90, pivot, (movementState == BACKWARD));
}

void Robot::left(bool pivot) {
    rotate(-90, pivot, (movementState == BACKWARD));
}

void Robot::rotate(int angle, bool move, bool backward) {
    // Calcul de la vitesse effective à partir des vitesses actuelles
    int effectiveSpeed = (abs(moteurG.get_speed()) + abs(moteurD.get_speed())) / 2;

    // Calcul du facteur de rotation dynamique
    float facteurRotation = 1.0 - (effectiveSpeed / (float)ROBOT_MAX_SPEED);
    if (facteurRotation < 0.3) facteurRotation = 0.3;

    int baseRotationSpeed = (int)(robot_speed * facteurRotation);
    baseRotationSpeed = (baseRotationSpeed < ROBOT_MIN_SPEED) ? ROBOT_MIN_SPEED : baseRotationSpeed;

    // Calcul de la durée de rotation (exemple de calcul, à ajuster)
    float timePerDegree = 800.0 / 90.0; // Exemple : 800 ms pour 90°, soit environ 8.9 ms par degré
    rotationDuration = abs(angle) * timePerDegree;
    rotationStartTime = millis();

    int speedG, speedD;
    if (!move) {
        // Rotation sur place
        speedG = (angle > 0) ? baseRotationSpeed : -baseRotationSpeed;
        speedD = (angle > 0) ? -baseRotationSpeed : baseRotationSpeed;
    } else {
        // Rotation en mouvement
        int moveSpeed = backward ? -robot_speed : robot_speed;
        speedG = (angle > 0) ? moveSpeed : moveSpeed / 2;
        speedD = (angle > 0) ? moveSpeed / 2 : moveSpeed;
    }

    previousMovementState = movementState;
    globalState = GlobalState::ROTATING;
    // On met à jour l'état de rotation selon le mode
    rotationState = (move ? TURNING : ROTATING);
    // Appliquer les vitesses calculées aux moteurs
    moteurG.set_speed(speedG);
    moteurD.set_speed(speedD);
}

// ---------------------- Gestion des états ---------------------- //

void Robot::printState() {
    String rotationStateStr = (rotationState == ROTATION_IDLE) ? "IDLE"
                              : (rotationState == TURNING)     ? "TURNING"
                              : (rotationState == ROTATING)    ? "ROTATING"
                                                               : "UNKNOWN";
    String movementStateStr = (movementState == MOVEMENT_IDLE)   ? "IDLE"
                              : (movementState == FORWARD)       ? "FORWARD"
                              : (movementState == BACKWARD)      ? "BACKWARD"
                              : (movementState == LINEFOLLOWING) ? "LINEFOLLOWING"
                              : (movementState == SHARPTURNING)  ? "SHARPTURNING"
                              : (movementState == FIGURE_EIGHT)  ? "FIGURE_EIGHT"
                                                                 : "UNKNOWN";

    Serial.print("Rotation State: ");
    Serial.print(rotationStateStr);
    Serial.print(" | Movement State: ");
    Serial.println(movementStateStr);
}

void Robot::changeMovementState(MovementState newMovementState) {
    if (newMovementState == LINEFOLLOWING) {
        resetPID();
    }
    movementState = newMovementState;
    printState();
}

void Robot::changeRotationState(RotationState newRotationState) {
    rotationState = newRotationState;
    printState();
}

void Robot::update() {
    switch (globalState) {
        case GlobalState::MOVING:
            updateMovement();
            break;
        case GlobalState::ROTATING:
            updateRotation();
            break;
        default:
            break;
    }
}

void Robot::updateMovement() {
    switch (movementState) {
        case FORWARD:
            set_robot_speed(base_speed);
            break;
        case BACKWARD:
            set_robot_speed(-base_speed);
            break;
        case LINEFOLLOWING:
            line_follower_pid();
            break;
        case SHARPTURNING:
            sharpturn();
            break;
        case FIGURE_EIGHT:
            move_eight();
            break;
        case MOVEMENT_IDLE:
        default:
            stop();
            break;
    }
}

void Robot::updateRotation() {
    if (rotationState != ROTATION_IDLE && (millis() - rotationStartTime >= rotationDuration)) {
        rotationState = ROTATION_IDLE;
        globalState = GlobalState::MOVING;
        movementState = previousMovementState;
    }
}

void Robot::handleEvent(RobotEvent event) {
    switch (event) {
        case EVENT_IR_FORWARD:
            changeMovementState(FORWARD);
            break;
        case EVENT_IR_BACKWARD:
            changeMovementState(BACKWARD);
            break;
        case EVENT_IR_LEFT:
            left(true);
            break;
        case EVENT_IR_RIGHT:
            right(true);
            break;
        case EVENT_IR_STOP:
            changeMovementState(MOVEMENT_IDLE);
            break;
        case EVENT_IR_ACCEL:
            accel();
            break;
        case EVENT_IR_DECCEL:
            deccel();
            break;
        case EVENT_IR_DEMITOUR:
            rotate(180, false, false);
            break;
        case EVENT_IR_EIGHT:
            changeMovementState(FIGURE_EIGHT);
            break;
        case EVENT_IR_LINEFOLLOWER:
            if (movementState == LINEFOLLOWING) {
                changeMovementState(MOVEMENT_IDLE);
                stop();
            } else {
                resetPID();
                changeMovementState(LINEFOLLOWING);
            }
            break;
        case EVENT_IR_INCREASEKP:
            pidKp += 10;
            pidController.SetTunings(pidKp, pidKi, pidKd);
            break;
        case EVENT_TIMER_ROTATION_END:
            changeRotationState(ROTATION_IDLE);
            if (movementState == FIGURE_EIGHT) {
                if (currentEightStep == EIGHT_FIRST) {
                    rotate(-390, true, false);
                    currentEightStep = EIGHT_SECOND;
                } else if (currentEightStep == EIGHT_SECOND) {
                    changeMovementState(FORWARD);
                    currentEightStep = EIGHT_NONE;
                }
            }
            break;
        case EVENT_NONE:
        default:
            break;
    }
    printState();
}

// ---------------------- Fonction de suivi de ligne / PID ---------------------- //

float Robot::errorestimation() {
    static constexpr float errorTable[8] = {
        5.0,  // 000 : aucun capteur actif
        0.0,  // 001 : centre seul
        1.7,  // 010 : droit seul
        1.2,  // 011 : centre et droit
        -1.7, // 100 : gauche seul
        -1.2, // 101 : gauche et centre
        0.0,  // 110 : gauche et droit
        0.0   // 111 : tous actifs
    };

    // Extraction rapide des bits des capteurs sur PIND
    uint8_t sensorState = (((PIND >> 4) & 0x01) << 2) | (((PIND >> 3) & 0x01) << 1) | ((PIND >> 2) & 0x01);

    float error = errorTable[sensorState];

    static unsigned int sharpturncount = 0;
    const unsigned int threshold = 130;
    static float lastValidError = error;

    if (error != 5.0) {
        lastValidError = error;
    }

    lastTurnDirection = (lastValidError > 0) ? RIGHT : LEFT;

    if (error == 5.0) {
        sharpturncount++;
    } else {
        sharpturncount = 0;
    }

    if (sharpturncount >= threshold) {
        changeMovementState(SHARPTURNING);
        sharpturncount = 0;
    }

    return error;
}

void Robot::line_follower_pid() {
    pid_input = errorestimation();
    pidController.Compute();
    float correction = pid_output;
    float errorMagnitude = fabs(pid_input);

    int baseSpeed = robot_speed;
    float speedFactor = 1.0;
    const float threshold = 1.3;
    const float maxError = 2.0;
    const float minSpeedFactor = 0.2;
    const int iterationsThreshold = 20;
    static int highErrorCounter = 0;

    if (errorMagnitude >= threshold) {
        highErrorCounter++;
    } else {
        highErrorCounter = 0;
    }

    if (highErrorCounter >= iterationsThreshold) {
        if (errorMagnitude >= maxError)
            speedFactor = minSpeedFactor;
        else
            speedFactor = 1.0 - ((errorMagnitude - threshold) / (maxError - threshold)) * (1.0 - minSpeedFactor);
        baseSpeed = (int)(robot_speed * speedFactor);
        correction *= speedFactor;
    }

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    leftSpeed = constrain(leftSpeed, 0, ROBOT_MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, ROBOT_MAX_SPEED);

    moteurG.set_speed(leftSpeed);
    moteurD.set_speed(rightSpeed);
}

void Robot::sharpturn() {
    // Utilisation de la lecture des capteurs via PIND
    uint8_t sensorValue = (((PIND >> 4) & 0x01) << 2) | (((PIND >> 3) & 0x01) << 1) | ((PIND >> 2) & 0x01);
    if (sensorValue > 0) {
        brake();
        resetPID();
        changeRotationState(ROTATION_IDLE);
        set_robot_speed(base_speed);
        changeMovementState(LINEFOLLOWING);
        return;
    } else {
        if (rotationState == ROTATION_IDLE) {
            if (lastTurnDirection == RIGHT)
                rotate(40);
            else if (lastTurnDirection == LEFT)
                rotate(-40);
        }
    }
}

void Robot::brake() {
    int currentSpeed = base_speed;
    const int decelerationStep = 25;
    // Attention : cette boucle est bloquante.
    while (currentSpeed > 0) {
        currentSpeed -= decelerationStep;
        if (currentSpeed < 0) currentSpeed = 0;
        moteurG.set_speed(currentSpeed);
        moteurD.set_speed(currentSpeed);
    }
    moteurG.set_speed(0);
    moteurD.set_speed(0);
}

void Robot::resetPID() {
    pidController.SetMode(MANUAL);
    pid_input = 0;
    pid_output = 0;
    pid_setpoint = 0;
    pidController.SetMode(AUTOMATIC);
}

// ---------------------- Fonctions d'autotuning ---------------------- //

unsigned long Robot::measureOscillationPeriod() {
    static bool firstPeakDetected = false;
    static unsigned long lastPeakTime = 0;
    static float previousError = 0.0;
    static bool rising = false;

    unsigned long currentTime = millis();
    float currentError = errorestimation();

    if (currentError > previousError) {
        rising = true;
    } else if (rising && currentError < previousError) {
        if (!firstPeakDetected) {
            firstPeakDetected = true;
            lastPeakTime = currentTime;
        } else {
            unsigned long period = currentTime - lastPeakTime;
            lastPeakTime = currentTime;
            rising = false;
            previousError = currentError;
            measurementDone = true;
            return period;
        }
        rising = false;
    }
    previousError = currentError;
    return 0;
}

void Robot::updatetuning() {
    moteurG.set_speed(base_speed);
    moteurD.set_speed(base_speed);
    line_follower_pid();
    unsigned long period = measureOscillationPeriod();
    if (measurementDone) {
        Serial.print("Période d'oscillation mesurée : ");
        Serial.print(period);
        Serial.println(" ms");
        measurementDone = false;
    }
}

// ---------------------- Décodage IR ---------------------- //

void Robot::decode_ir() {
    if (IrReceiver.decode()) {
        unsigned long code = IrReceiver.decodedIRData.decodedRawData;
        RobotEvent event = EVENT_NONE;
        switch (static_cast<CommandeIR>(code)) {
            case CommandeIR::FORWARD:
                event = EVENT_IR_FORWARD;
                break;
            case CommandeIR::BACKWARD:
                event = EVENT_IR_BACKWARD;
                break;
            case CommandeIR::STOP:
                event = EVENT_IR_STOP;
                break;
            case CommandeIR::ACCEL:
                event = EVENT_IR_ACCEL;
                break;
            case CommandeIR::DECCEL:
                event = EVENT_IR_DECCEL;
                break;
            case CommandeIR::RIGHT:
                event = EVENT_IR_RIGHT;
                break;
            case CommandeIR::LEFT:
                event = EVENT_IR_LEFT;
                break;
            case CommandeIR::DEMITOUR:
                event = EVENT_IR_DEMITOUR;
                break;
            case CommandeIR::EIGHT:
                event = EVENT_IR_EIGHT;
                break;
            case CommandeIR::LINEFOLLOWER:
                set_robot_speed(70);
                event = EVENT_IR_LINEFOLLOWER;
                break;
            case CommandeIR::INCREASEKP:
                event = EVENT_IR_INCREASEKP;
                break;
            default:
                event = EVENT_NONE;
                break;
        }
        handleEvent(event);
        IrReceiver.resume();
    }
}

// ---------------------- Mouvements complexes ---------------------- //

void Robot::move_eight() {
    if (currentEightStep == EIGHT_NONE) {
        rotate(360, true, false);
        currentEightStep = EIGHT_FIRST;
    }
}
