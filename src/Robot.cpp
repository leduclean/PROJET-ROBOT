/**
 * @file Robot.cpp
 * @brief Contrôle du robot : moteurs, PID, états, suivi de ligne et commandes IR.
 */

#include "Robot.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

// ---------------------- Constructeur et initialisation ---------------------- //

/**
 * @brief Initialise le robot (moteurs, PID, capteurs, états).
 * @param base_speed Vitesse de base du robot.
 */
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
    pidController(&pid_input, &pid_output, &pid_setpoint, 19, 85, 20, DIRECT),
    pidKp(19),
    pidKi(85),
    pidKd(20) {
    // Initialisation des capteurs IR et des broches de suivi de ligne
    initialize_ir();
    initialize_line_pin();

    // Configuration du PID :
    // On veut obtenir une erreur nulle et effectuer le calcul toutes les 20 ms.
    pid_setpoint = 0;
    pidController.SetSampleTime(20);
    pidController.SetOutputLimits(-250, 250);
    pidController.SetMode(AUTOMATIC);
}

// ---------------------- Initialisation des capteurs ---------------------- //

/**
 * @brief Initialise le récepteur IR.
 */
void Robot::initialize_ir() {
    Serial.println("Initialisation IR");
    // Démarre le récepteur IR avec retour LED activé pour le debug
    IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);
}

/**
 * @brief Configure les broches des capteurs de suivi de ligne.
 */
void Robot::initialize_line_pin() {
    pinMode(LINE_FOLLOWER_LEFT_PIN, INPUT);
    pinMode(LINE_FOLLOWER_MID_PIN, INPUT);
    pinMode(LINE_FOLLOWER_RIGHT_PIN, INPUT);
}

// ---------------------- Déplacement linéaire ---------------------- //

/**
 * @brief Définit la vitesse du robot et la transmet aux moteurs.
 * @param speed Vitesse souhaitée (négatif pour marche arrière).
 */
void Robot::set_robot_speed(int speed) {
    robot_speed = abs(speed); // On conserve la vitesse de base comme valeur absolue
    moteurD.set_speed(speed);
    moteurG.set_speed(speed);
}

/**
 * @brief Arrête le robot.
 */
void Robot::stop() {
    set_robot_speed(0);
}

/**
 * @brief Augmente la vitesse du robot.
 */
void Robot::accel() {
    int new_speed = robot_speed + ROBOT_ACCELERATION_INCREMENT;
    if (new_speed > ROBOT_MAX_SPEED) new_speed = ROBOT_MAX_SPEED;
    // Inverse la vitesse si le robot est en marche arrière
    set_robot_speed((movementState == BACKWARD) ? -new_speed : new_speed);
}

/**
 * @brief Diminue la vitesse du robot.
 */
void Robot::deccel() {
    int new_speed = robot_speed - ROBOT_ACCELERATION_INCREMENT;
    // Inverse la vitesse si le robot est en marche arrière
    set_robot_speed((movementState == BACKWARD) ? -new_speed : new_speed);
}

// ---------------------- Rotation / virage ---------------------- //

/**
 * @brief Effectue une rotation à droite.
 * @param pivot Si vrai, effectue une rotation sur place.
 */
void Robot::right(bool pivot) {
    // Appelle rotate() avec un angle positif (90°)
    rotate(90, pivot, (movementState == BACKWARD));
}

/**
 * @brief Effectue une rotation à gauche.
 * @param pivot Si vrai, effectue une rotation sur place.
 */
void Robot::left(bool pivot) {
    // Appelle rotate() avec un angle négatif (-90°)
    rotate(-90, pivot, (movementState == BACKWARD));
}

/**
 * @brief Effectue une rotation avec angle, mode et direction spécifiés.
 * @param angle Angle de rotation (positif = droite, négatif = gauche).
 * @param move Si vrai, rotation en mouvement ; sinon sur place.
 * @param backward Si vrai, robot en marche arrière.
 */
void Robot::rotate(int angle, bool move, bool backward) {
    // Calcule la vitesse moyenne des moteurs pour obtenir une vitesse effective
    int effectiveSpeed = (abs(moteurG.get_speed()) + abs(moteurD.get_speed())) / 2;

    // Calcul d'un facteur de rotation qui diminue avec la vitesse pour éviter des rotations trop brusques
    float facteurRotation = 1.0 - (effectiveSpeed / (float)ROBOT_MAX_SPEED);
    if (facteurRotation < 0.3) facteurRotation = 0.3;

    // Vitesse de base pour la rotation, ajustée par le facteur de rotation
    int baseRotationSpeed = (int)(robot_speed * facteurRotation);
    baseRotationSpeed = (baseRotationSpeed < ROBOT_MIN_SPEED) ? ROBOT_MIN_SPEED : baseRotationSpeed;

    // Calcul de la durée de rotation : temps par degré * nombre de degrés
    float timePerDegree = 800.0 / 90.0; // Exemple : ~8.9 ms par degré
    rotationDuration = abs(angle) * timePerDegree;
    rotationStartTime = millis();

    int speedG, speedD;
    if (!move) {
        // Rotation sur place : les moteurs tournent en sens opposé
        speedG = (angle > 0) ? baseRotationSpeed : -baseRotationSpeed;
        speedD = (angle > 0) ? -baseRotationSpeed : baseRotationSpeed;
    } else {
        // Rotation en mouvement : ajuste la vitesse selon l'angle
        int moveSpeed = backward ? -robot_speed : robot_speed;
        speedG = (angle > 0) ? moveSpeed : moveSpeed / 2;
        speedD = (angle > 0) ? moveSpeed / 2 : moveSpeed;
    }

    // Mémorise l'état de mouvement précédent et met à jour l'état de rotation
    previousMovementState = movementState;
    rotationState = (move ? TURNING : ROTATING);
    changeRotationState(ROTATING);

    // Applique les vitesses calculées aux moteurs
    moteurG.set_speed(speedG);
    moteurD.set_speed(speedD);
}

// ---------------------- Décodage IR ---------------------- //

/**
 * @brief Décode la commande IR et déclenche l'action correspondante.
 */
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
                // Fixe une vitesse spécifique pour le suivi de ligne
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
        // Transmet l'événement à la gestion des états du robot
        handleEvent(event);
        IrReceiver.resume();
    }
}

// ---------------------- Gestion des états ---------------------- //

/**
 * @brief Affiche l'état de rotation et de mouvement du robot.
 */
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

/**
 * @brief Met à jour l'état global en fonction des états de rotation et de mouvement.
 */
void Robot::syncStates() {
    if (rotationState != ROTATION_IDLE)
        globalState = GlobalState::ROTATING;
    else if (movementState != MOVEMENT_IDLE)
        globalState = GlobalState::MOVING;
    else
        globalState = GlobalState::IDLE;
}

/**
 * @brief Change l'état de mouvement et synchronise l'état global.
 * Réinitialise le PID si nécessaire (pour LINEFOLLOWING).
 *
 * @param newMovementState Nouvel état de mouvement.
 */
void Robot::changeMovementState(MovementState newMovementState) {
    if (newMovementState == LINEFOLLOWING) {
        resetPID();
    }
    movementState = newMovementState;
    syncStates();
    printState();
}

/**
 * @brief Change l'état de rotation et synchronise l'état global.
 *
 * @param newRotationState Nouvel état de rotation.
 */
void Robot::changeRotationState(RotationState newRotationState) {
    rotationState = newRotationState;
    syncStates();
    printState();
}

/**
 * @brief Met à jour l'état global du robot (mouvement ou rotation).
 */
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

/**
 * @brief Met à jour le comportement en fonction de l'état de déplacement.
 */
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

/**
 * @brief Vérifie si la rotation est terminée et rétablit l'état.
 */
void Robot::updateRotation() {
    if (rotationState != ROTATION_IDLE && (millis() - rotationStartTime >= rotationDuration)) {
        rotationState = ROTATION_IDLE;
        globalState = GlobalState::MOVING;
        movementState = previousMovementState;
    }
}

/**
 * @brief Traite l'événement reçu et effectue la transition d'état correspondante.
 *
 * Certains cas complexes, comme EVENT_TIMER_ROTATION_END ou FIGURE_EIGHT, gèrent
 * des séquences spécifiques de mouvement.
 *
 * @param event Événement à traiter.
 */
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

// ---------------------- Suivi de ligne / PID ---------------------- //

/**
 * @brief Estime l'erreur pour le suivi de ligne.
 *
 * Lit directement les bits de PIND pour obtenir l'état des capteurs,
 * puis retourne une valeur d'erreur en fonction d'un tableau prédéfini.
 * Si aucun capteur n'est actif pendant un certain nombre de cycles, déclenche un virage serré.
 *
 * @return Erreur estimée.
 */
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
    bool calibration = false;

    // Extraction des bits correspondant aux capteurs (D2: centre, D3: droit, D4: gauche)
    uint8_t sensorState = (((PIND >> 4) & 0x01) << 2) | (((PIND >> 3) & 0x01) << 1) | ((PIND >> 2) & 0x01);

    float error = errorTable[sensorState];

    // Comptage des cycles où aucun capteur ne détecte la ligne (erreur == 5.0)
    static unsigned int sharpturncount = 0;
    const unsigned int threshold = 130;
    static float lastValidError = error;

    // Mise à jour de la dernière erreur valide si un capteur détecte la ligne
    if (error != 5.0) {
        lastValidError = error;
    }

    // Détermine la dernière direction de virage basée sur la dernière erreur valide
    lastTurnDirection = (lastValidError > 0) ? RIGHT : LEFT;

    // Incrémente le compteur si aucun capteur ne détecte la ligne
    if (error == 5) {
        sharpturncount++;
    } else {
        sharpturncount = 0;
    }

    // Si le compteur dépasse le seuil, déclenche le virage serré
    if (!calibration && sharpturncount >= threshold) {
        changeMovementState(SHARPTURNING);
        sharpturncount = 0;
    }

    return error;
}

/**
 * @brief Exécute le suivi de ligne avec PID.
 *
 * Calcule la correction à appliquer aux moteurs en fonction de l'erreur mesurée.
 * Une modulation de la vitesse est appliquée si l'erreur est élevée de façon répétée.
 */
void Robot::line_follower_pid() {
    pid_input = errorestimation();
    pidController.Compute();
    float correction = pid_output;
    float errorMagnitude = fabs(pid_input);

    int baseSpeed = robot_speed;
    float speedFactor = 1.0;

    // Paramètres pour réduire la vitesse en cas d'erreur importante répétée
    const float threshold = 1.3;
    const float maxError = 2.0;
    const float minSpeedFactor = 1;
    const int iterationsThreshold = 10;
    static int highErrorCounter = 0;

    // Incrémente le compteur si l'erreur dépasse le seuil
    if (errorMagnitude >= threshold)
        highErrorCounter++;
    else
        highErrorCounter = 0;

    // Si l'erreur reste élevée pendant plusieurs cycles, applique une réduction de vitesse
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

    // Applique les vitesses corrigées aux moteurs
    moteurG.set_speed(leftSpeed);
    moteurD.set_speed(rightSpeed);
}

/**
 * @brief Gère le virage serré en fonction des capteurs.
 *
 * Si un capteur détecte la ligne, freine, réinitialise le PID et repasse en mode suivi.
 * Sinon, effectue une rotation pour retrouver la ligne.
 */
void Robot::sharpturn() {
    uint8_t sensorValue = (((PIND >> 4) & 0x01) << 2) | (((PIND >> 3) & 0x01) << 1) | ((PIND >> 2) & 0x01);
    if (sensorValue > 0) {
        // Freine pour réduire l'inertie et réinitialise le PID
        brake();
        resetPID();
        // Arrête la rotation en cours et repasse en mode suivi de ligne
        changeRotationState(ROTATION_IDLE);
        set_robot_speed(base_speed);
        changeMovementState(LINEFOLLOWING);
        return;
    } else {
        if (rotationState == ROTATION_IDLE) {
            // Relance une petite rotation pour retrouver la piste
            if (lastTurnDirection == RIGHT)
                rotate(40);
            else if (lastTurnDirection == LEFT)
                rotate(-40);
        }
    }
}

/**
 * @brief Freine le robot en réduisant progressivement la vitesse.
 */
void Robot::brake() {
    static int currentSpeed = base_speed;
    const int decelerationStep = 25;

    if (currentSpeed > 0) {
        currentSpeed -= decelerationStep;
        if (currentSpeed < 0) currentSpeed = 0;
        moteurG.set_speed(currentSpeed);
        moteurD.set_speed(currentSpeed);
    } else {
        currentSpeed = base_speed; // Réinitialise pour la prochaine utilisation
    }
}

/**
 * @brief Réinitialise le PID.
 */
void Robot::resetPID() {
    pidController.SetMode(MANUAL);
    pid_input = 0;
    pid_output = 0;
    pid_setpoint = 0;
    pidController.SetMode(AUTOMATIC);
}

// ---------------------- Tuning du PID ---------------------- //

/**
 * @brief Mesure la période d'oscillation pour le tuning du PID.
 *
 * Détecte un pic dans l'erreur (montée puis descente) pour calculer la période d'oscillation.
 *
 * @return Période d'oscillation en ms, ou 0 si non mesurée.
 */
unsigned long Robot::measureOscillationPeriod() {
    static bool firstPeakDetected = false; // Indique si le premier pic a été détecté
    static unsigned long lastPeakTime = 0; // Temps du dernier pic détecté
    static float previousError = 0.0;      // Erreur du cycle précédent
    static bool rising = false;            // Indique si le signal est en phase de montée

    unsigned long currentTime = millis();
    float currentError = errorestimation();

    // Si le signal augmente, on est en phase ascendante
    if (currentError > previousError) {
        rising = true;
    }
    // Si le signal passe de montée à descente, on détecte un pic
    else if (rising && currentError < previousError) {
        if (!firstPeakDetected) {
            firstPeakDetected = true;
            lastPeakTime = currentTime;
        } else {
            unsigned long period = currentTime - lastPeakTime;
            lastPeakTime = currentTime; // Met à jour pour la prochaine mesure
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

/**
 * @brief Met à jour le tuning du PID en affichant la période mesurée.
 */
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

// ---------------------- Mouvements complexes ---------------------- //

/**
 * @brief Exécute le mouvement en huit.
 */
void Robot::move_eight() {
    if (currentEightStep == EIGHT_NONE) {
        rotate(360, true, false);
        currentEightStep = EIGHT_FIRST;
    }
}
