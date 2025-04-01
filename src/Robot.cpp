#include "Robot.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>
#include <PID_v1.h>


#if !defined(LINE_FOLLOWER_LEFT_PIN)
#error The LineFollower program requires that LINE_FOLLOWER_[LEFT,MID,RIGHT]_PIN are defined
#endif

// ===========================================
// Constructeur et initialisation
// ===========================================

Robot::Robot(int base_speed)
    : moteurD(base_speed, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_BACKWARD_PIN, MOTOR_RIGHT_SPEED_PIN, 0.94),
      moteurG(base_speed, MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_BACKWARD_PIN, MOTOR_LEFT_SPEED_PIN, 1),
      robot_speed(base_speed),
      base_speed(base_speed),
      movementState(MOVEMENT_IDLE),
      rotationState(ROTATION_IDLE),
      rotationStartTime(0),
      rotationDuration(0),
      CurrentLineSensorState(0),
      lastTurnDirection(NONE),
      // Coefficients initiaux (à ajuster)
      pidKp(80),
      pidKi(0),
      pidKd(0), // 400 ms 
      // Initialisation des variables du PID
      pid_input(0),
      pid_output(0),
      pid_setpoint(0),
      // Construction de l'objet PID. Note : PID_v1 attend des pointeurs sur les variables.
      pidController(&pid_input, &pid_output, &pid_setpoint, pidKp, pidKi, pidKd, DIRECT)
{
  initialize_ir();
  initialize_line_pin();
    // Configuration du PID
    pid_setpoint = 0;         // On cherche à avoir une erreur nulle
    pidController.SetSampleTime(20); // 17 ms entre chaque calcul du PID
    pidController.SetOutputLimits(-250, 250);
    pidController.SetMode(AUTOMATIC);
}

// Fonction d'initialisation
void Robot::initialize_ir() {
  Serial.println("Initialisation IR");
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);  // Démarrer le récepteur IR
}

void Robot::initialize_line_pin() {
  pinMode(LINE_FOLLOWER_LEFT_PIN, INPUT);
  pinMode(LINE_FOLLOWER_MID_PIN, INPUT);
  pinMode(LINE_FOLLOWER_RIGHT_PIN, INPUT);
}

// ===========================================
// Fonctions de déplacement linéaire
// ===========================================

// Définit la vitesse du robot et met à jour l'état de déplacement
void Robot::set_robot_speed(int speed) {
  robot_speed = abs(speed);  // Met à jour la vitesse de base (toujours positive)
  changeMovementState((speed > 0) ? FORWARD : (speed < 0) ? BACKWARD
                                                          : MOVEMENT_IDLE);

  moteurD.set_speed(speed);
  moteurG.set_speed(speed);
}

// Arrête le robot
void Robot::stop() {
  set_robot_speed(0);
}

// Accélère le robot
void Robot::accel() {
  int new_speed = this->robot_speed + ROBOT_ACCELERATION_INCREMENT;  // Augmente la vitesse
  if (new_speed > ROBOT_MAX_SPEED) {
    new_speed = ROBOT_MAX_SPEED;  // Limite à la vitesse max
  }
  set_robot_speed((movementState == BACKWARD)? - new_speed: new_speed );
}

// Décélère le robot
void Robot::deccel() {
  int new_speed = this->robot_speed - ROBOT_ACCELERATION_INCREMENT;  // Augmente la vitesse
  if (new_speed > ROBOT_MAX_SPEED) {
    new_speed = ROBOT_MAX_SPEED;  // Limite à la vitesse max
  }
  set_robot_speed((movementState == BACKWARD)? - new_speed: new_speed );
}

// ===========================================
// Fonctions de rotation / virage
// ===========================================

// Tourne à droite (virage ou pivot)
void Robot::right(bool pivot) {
  changeRotationState(TURNING);

  int speedG = this->robot_speed*2;
  int speedD = (pivot) ? 0 : this->robot_speed / 2;

  // Si le robot est en marche arrière, inverser les vitesses
  if (movementState == BACKWARD) {
    speedG = -speedG;
    speedD = -speedD;
  }

  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  // Enregistrer le temps de début et la durée du virage
  rotationStartTime = millis();
  rotationDuration = 200;
}

// Tourne à gauche (virage ou pivot)
void Robot::left(bool pivot) {
  changeRotationState(TURNING);

  int speedG = (pivot) ? 0 : this->robot_speed / 2;
  int speedD = this->robot_speed*2;

  if (movementState == BACKWARD) {
    speedG = -speedG;
    speedD = -speedD;
  }

  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  rotationStartTime = millis();
  rotationDuration = 200;
}

// Effectue une rotation avec angle spécifié
// move indique si la rotation se fait en mouvement (avancer/reculer) et backward si c'est en marche arrière/ 
// angle: positif a droite, negatif gauche 
void Robot::rotate(int angle, bool move, bool backward) {
  // Calculer la vitesse effective à partir des vitesses actuelles
  // On suppose que dans le mode line follower, leftSpeed et rightSpeed
  // ont été calculés par le PID et sont accessibles (ou stockés dans des variables membres).
  int effectiveSpeed = (abs(moteurG.get_speed()) + abs(moteurD.get_speed())) / 2;

  // Calculer le facteur de rotation dynamique
  float facteurRotation = 1.0 - (effectiveSpeed / (float)ROBOT_MAX_SPEED);
  // Placer une borne minimale pour éviter que le facteur ne devienne trop faible
  if (facteurRotation < 0.3) {
      facteurRotation = 0.3;
  }

  // Calcul de la vitesse de rotation de base
  int baseRotationSpeed = (int)(robot_speed * facteurRotation);
    
  // Calculer la durée de rotation selon l'angle souhaité et une constante tempsParDegre
  float timePerDegree = 800/90;  // par exemple, 2 ms par degré (à ajuster)
  rotationDuration = abs(angle) * timePerDegree;
  rotationStartTime = millis();

  int speedG, speedD;
  baseRotationSpeed = (baseRotationSpeed < ROBOT_MIN_SPEED) ? ROBOT_MIN_SPEED : baseRotationSpeed;
  if (!move) {
    // Rotation sur place : les moteurs tournent en sens opposé
    speedG = (angle > 0) ? baseRotationSpeed : -baseRotationSpeed;
    speedD = (angle > 0) ? -baseRotationSpeed : baseRotationSpeed;
  } else {
    // Rotation en mouvement
    int moveSpeed = backward ? -robot_speed : robot_speed;
    // On peut ajuster différemment en fonction de l'angle
    speedG = (angle > 0) ? moveSpeed : moveSpeed / 2;
    speedD = (angle > 0) ? moveSpeed / 2 : moveSpeed;
  }
  
  // Appliquer les vitesses calculées aux moteurs
  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  changeRotationState(ROTATING);
}


// ===========================================
// Gestion des états
// ===========================================

// Affiche les états actuels
void Robot::printState() {
  String RotationstateStr = (rotationState == ROTATION_IDLE) ? "IDLE" : (rotationState == TURNING)  ? "TURNING"
                                                                      : (rotationState == ROTATING) ? "ROTATING"
                                                                                                    : "UNKNOWN";

  String movementStr = (movementState == MOVEMENT_IDLE)   ? "IDLE"
                       : (movementState == FORWARD)       ? "FORWARD"
                       : (movementState == BACKWARD)      ? "BACKWARD"
                       : (movementState == LINEFOLLOWING) ? "LINEFOLLOWING"
                       : (movementState == SHARPTURNING)  ? "SHARPTURNING"
                                                          : "UNKNOWN";  // Cas par défaut

  Serial.print("Rotation State: ");
  Serial.print(RotationstateStr);
  Serial.print(" | Movement State: ");
  Serial.println(movementStr);
}
// Change l'état de déplacement
void Robot::changeMovementState(RobotMovementState newMovementState) {
  if (newMovementState == LINEFOLLOWING) {
    resetPID();
  }
  movementState = newMovementState;
  printState();
}

// Change l'état de rotation
void Robot::changeRotationState(RobotRotationState newRotationState) {
  rotationState = newRotationState;
  printState();
}



// ===========================================
// Mise à jour (appelée dans la boucle principale)
// ===========================================

void Robot::update(){
  long int current_time = millis();

  // Vérification d'interruption : si l'état de mouvement n'est plus FIGURE_EIGHT, stopper la rotation
  if(movementState != FIGURE_EIGHT && rotationState != ROTATION_IDLE) {
      // Arrêter la rotation en cours
      int appliedSpeed = (movementState == BACKWARD) ? -robot_speed : robot_speed;
      moteurG.set_speed(appliedSpeed);
      moteurD.set_speed(appliedSpeed);
      changeRotationState(ROTATION_IDLE);
      currentEightStep = EIGHT_NONE; // Réinitialiser la séquence du 8
  }

  // Gestion de l'état de rotation
  switch (rotationState)
  {
      case ROTATING:
      case TURNING:
          if (current_time - rotationStartTime >= rotationDuration) {
              // Rotation terminée, arrêter la rotation
              int appliedSpeed = (movementState == BACKWARD) ? -robot_speed : robot_speed;
              moteurG.set_speed(appliedSpeed);
              moteurD.set_speed(appliedSpeed);
              changeRotationState(ROTATION_IDLE);

              // Si on est dans une séquence de figure en 8, passer à l'étape suivante
              if (movementState == FIGURE_EIGHT) {
                  if (currentEightStep == EIGHT_FIRST) {
                      // Passer à la deuxième boucle : rotation vers la droite
                      rotate(-390, true, false);
                      changeRotationState(ROTATING);
                      currentEightStep = EIGHT_SECOND;
                  } else if (currentEightStep == EIGHT_SECOND) {
                      // Fin de la séquence, on revient à FORWARD par exemple
                      changeMovementState(FORWARD);
                      currentEightStep = EIGHT_NONE;
                  }
              }
          }
          break;

      default:
          break;
  }

  // Gestion de l'état de mouvement
  switch (movementState)
  {
      case SHARPTURNING:
          sharpturn();
          break;
      case LINEFOLLOWING:
          line_follower_pid();
          break;
      case FIGURE_EIGHT:
          // Lancer l'initiation de la séquence si ce n'est pas déjà lancé
          if (currentEightStep == EIGHT_NONE) {
              move_eight();
          }
          break;
      case FORWARD:
          set_robot_speed(base_speed);
          break;
      case BACKWARD:
          set_robot_speed(-base_speed);
          break;
      default:
          break;
  }
}


// ===========================================
// Fonctions de debug
// ===========================================

void Robot::debug() {
  Serial.println("speed :" + String(this->robot_speed));
  this->moteurD.debug();
  this->moteurG.debug();
}

// ===========================================
// Fonctions de mouvement complexes
// ===========================================

// Exécute un mouvement en forme de "8"
void Robot::move_eight() {
  // Si aucune séquence n'est en cours, on démarre la première boucle
  if (currentEightStep == EIGHT_NONE) {
      rotate(360, true, false);
      changeRotationState(ROTATING);
      currentEightStep = EIGHT_FIRST;
  }
}


// ===========================================
// Fonctions liées aux capteurs et à l'IR
// ===========================================

// Décodage de l'IR et exécution de commandes
void Robot::decode_ir() {
  static unsigned long last = millis();
  static bool on = false;

  if (IrReceiver.decode()) {
    auto codeRecu = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(codeRecu, HEX);
    if (millis() - last > 250) {
      on = !on;
      digitalWrite(13, on ? HIGH : LOW);
      // Conversion du code reçu en type CommandeIR
      CommandeIR commande = static_cast<CommandeIR>(codeRecu);
      if (movementState == LINEFOLLOWING && 
        (commande != CommandeIR::LINEFOLLOWER && commande != CommandeIR::INCREASEKP)) {
        Serial.println("Commande ignorée : le robot est en mode LINE FOLLOWING.");
        IrReceiver.resume();  
        return;
      }

      // Exécuter la commande correspondante
      switch (commande) {
        case CommandeIR::FORWARD:
          changeMovementState(FORWARD);
          break;
        case CommandeIR::BACKWARD:
          changeMovementState(BACKWARD);
          break;
        case CommandeIR::LEFT:
          left(true);
          break;
        case CommandeIR::RIGHT:
          right(true);
          break;
        case CommandeIR::STOP:
          stop();
          break;
        case CommandeIR::ACCEL:
          accel();
          break;
        case CommandeIR::DECCEL:
          deccel();
          break;
        case CommandeIR::DEMITOUR:
          rotate(180, false, false);
          break;
        case CommandeIR::EIGHT:
          changeMovementState(FIGURE_EIGHT);
          break;
        case CommandeIR::LINEFOLLOWER:
          if (movementState == LINEFOLLOWING) {
            changeMovementState(MOVEMENT_IDLE);
            stop();
          } else {
            resetPID();
            set_robot_speed(base_speed);  // Réinitialise la vitesse à la valeur de base
            changeMovementState(LINEFOLLOWING);
          }
          break;
        case CommandeIR::INCREASEKP:
          pidKp = pidKd + 10;
          pidController.SetTunings(pidKp + 10, 0, 0);

          break;
        default:
          Serial.println("Commande inconnue !");
          break;
      }

      last = millis();
    }

    IrReceiver.resume();  // Préparer pour la prochaine réception
  }
}



void Robot::resetPID() {
  // Passe en mode MANUAL pour "vider" l'historique interne
  pidController.SetMode(MANUAL);
  // Réinitialise les variables d'entrée, de sortie et de consigne si nécessaire
  pid_input = 0;
  pid_output = 0;
  pid_setpoint = 0;
  // Repasse en mode AUTOMATIC pour reprendre le calcul PID
  pidController.SetMode(AUTOMATIC);
}


float Robot::errorestimation() {
  static constexpr float errorTable[8] = {
    5.0,   // 000 : aucun capteur actif
    0.0,   // 001 : centre seul
    1.7,   // 010 : droit seul
    1.2,   // 011 : centre et droit
    -1.7,  // 100 : gauche seul
    -1.2,  // 101 : gauche et centre
    0.0,   // 110 : gauche et droit
    0.0,  // 111 : tous actifs
  };
  bool calibration = true;  

  // Lire PIND et extraire les bits correspondant aux capteurs :
  // - capteur du milieu (D2) -> bit 0
  // - capteur droit   (D3) -> bit 1
  // - capteur gauche  (D4) -> bit 2
  uint8_t sensorState = (((PIND >> 4) & 0x01) << 2) | 
                        (((PIND >> 3) & 0x01) << 1) | 
                        ((PIND >> 2) & 0x01);

  float error = errorTable[sensorState];

  // Compteur statique pour vérifier les cycles consécutifs avec aucun capteur actif
  static unsigned int sharpturncount = 0;
  const unsigned int threshold = 130; // nombre de cycles à atteindre avant de déclencher le sharp turning

  if (error == 5) {
    sharpturncount++;
  } else {
    sharpturncount = 0; // Réinitialise si un capteur détecte la ligne
  }

  // Déclenchement du comportement SHARPTURNING si aucun capteur actif pendant plusieurs cycles
  if (!calibration && sharpturncount >= threshold) {
    changeMovementState(SHARPTURNING);
    sharpturncount = 0;  // Optionnel : réinitialiser le compteur après changement d'état
  }

  return error;
}





void Robot::line_follower_pid() {
  // Récupérer l'erreur estimée
  pid_input = errorestimation();
  // Calcul du PID (la sortie est stockée dans pid_output)
  pidController.Compute();
  float correction = pid_output;
  
  // Déterminer la magnitude de l'erreur
  float errorMagnitude = fabs(pid_input);
  
  // Vitesse de base par défaut
  int baseSpeed = robot_speed;
  float speedFactor = 1.0;
  
  // Paramètres pour la modulation de vitesse
  const float threshold = 3;      // Seuil à partir duquel on commence à envisager la réduction
  const float maxError = 2.0;       // Erreur maximale pour laquelle le facteur atteint son minimum
  const float minSpeedFactor = 1; // Facteur minimal : 20% de robot_speed en cas d'erreur très importante
  const int iterationsThreshold = 10; // Nombre d'itérations consécutives avec une erreur > threshold pour appliquer la modulation
  
  // Compteur statique pour les itérations consécutives avec erreur élevée
  static int highErrorCounter = 0;
  
  // Incrémenter ou réinitialiser le compteur en fonction de l'erreur
  if (errorMagnitude >= threshold) {
      highErrorCounter++;
  } else {
      highErrorCounter = 0;
  }
  
  // Appliquer la modulation seulement si l'erreur est élevée pendant plusieurs itérations
  if (highErrorCounter >= iterationsThreshold) {
    if (errorMagnitude >= maxError) {
      speedFactor = minSpeedFactor;
    } else {
      // Réduction linéaire entre threshold et maxError
      speedFactor = 1.0 - ((errorMagnitude - threshold) / (maxError - threshold)) * (1.0 - minSpeedFactor);
    }
    baseSpeed = (int)(robot_speed * speedFactor);
    correction = correction * speedFactor;
  }
  
  // Calcul des vitesses pour chaque moteur en soustrayant/ajoutant la correction
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  
  // Contraindre les vitesses pour ne pas dépasser les limites
  leftSpeed = constrain(leftSpeed, 0, ROBOT_MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, ROBOT_MAX_SPEED);
  
  // Appliquer les vitesses aux moteurs
  moteurG.set_speed(leftSpeed);
  moteurD.set_speed(rightSpeed);
}







void Robot::sharpturn() {
  // Vérifier si au moins un capteur détecte la ligne
  if (digitalRead(LINE_FOLLOWER_LEFT_PIN) == HIGH ||
      digitalRead(LINE_FOLLOWER_MID_PIN) == HIGH ||
      digitalRead(LINE_FOLLOWER_RIGHT_PIN) == HIGH) {
    
    // On applique d'abord une décélération ou un freinage progressif
    // (Remplace brake() par ta méthode de freinage, ou ajuste la vitesse)
    brake();                // Par exemple, arrêter brièvement le robot
    delay(100);             // Attendre un court instant pour laisser l'inertie se dissiper

    // Puis on réinitialise le PID et on reprend en mode LINEFOLLOWING avec une vitesse modérée
    resetPID();             // Réinitialise le PID
    set_robot_speed(base_speed);  // Démarrer à une vitesse réduite
    changeMovementState(LINEFOLLOWING);
    return;
  } else {
    // Sinon, continuer à tourner par petits incréments
    rotate((lastTurnDirection == RIGHT) ? 40 : -40);
  }
}

void Robot::brake() {
  int currentSpeed = base_speed;
  const int decelerationStep = 25;  // Incrément plus grand pour un freinage plus agressif
  const int brakeDelay = 5;        // Délai réduit

  while (currentSpeed > 0) {
    currentSpeed -= decelerationStep;
    if (currentSpeed < 0) {
      currentSpeed = 0;
    }
    moteurG.set_speed(currentSpeed);
    moteurD.set_speed(currentSpeed);
    delay(brakeDelay);
  }
  moteurG.set_speed(0);
  moteurD.set_speed(0);
}

unsigned long Robot::measureOscillationPeriod() {
  // Variables statiques pour conserver l'état entre les appels
  static bool firstPeakDetected = false;   // Indique si le premier pic a été détecté
  static unsigned long lastPeakTime = 0;     // Temps du dernier pic détecté
  static float previousError = 0.0;          // Valeur d'erreur du cycle précédent
  static bool rising = false;                // Indique si le signal est en phase ascendante

  unsigned long currentTime = millis();      // Temps actuel en ms
  float currentError = errorestimation();      // Lecture de l'erreur actuelle

  // Si le signal augmente, on est en phase de montée
  if (currentError > previousError) {
      rising = true;
  }
  // Si le signal passe de la montée à la descente, on détecte un pic
  else if (rising && currentError < previousError) {
      if (!firstPeakDetected) {
          // Premier pic détecté : initialisation du temps
          firstPeakDetected = true;
          lastPeakTime = currentTime;
      } else {
          // Calcul de la période entre deux pics successifs
          unsigned long period = currentTime - lastPeakTime;
          lastPeakTime = currentTime; // Met à jour pour la prochaine mesure
          rising = false;             // Réinitialise l'état de montée
          previousError = currentError;
          measurementDone = true;
          return period;              // Retourne la période en ms
      }
      rising = false; // Réinitialise l'état pour éviter plusieurs détections sur le même pic
  }

  previousError = currentError;
  return 0; // Retourne 0 si aucune nouvelle période n'est mesurée
}


void Robot::updatetuning() {
    // 1. Définir une vitesse de base pour démarrer le mouvement
    moteurG.set_speed(this->base_speed);
    moteurD.set_speed(this->base_speed);

    // 2. Exécuter le suivi de ligne avec le PID
    line_follower_pid();

    // 3. Mesurer l'oscillation pour le réglage PID
    unsigned long period = measureOscillationPeriod();
    if (measurementDone) {
      // Imprimer une seule fois le résultat
      Serial.print("Période d'oscillation mesurée : ");
      Serial.print(period);
      Serial.println(" ms");
      // Réinitialiser le flag pour éviter de réimprimer à chaque tour de boucle
      measurementDone = false;
  }
    // if (period > 0) {
    //      // Conversion de la période en secondes
    //      float Tu = period / 1000.0;

    //      // Calcul des coefficients PID avec la méthode de Ziegler-Nichols
    //      float Kp = 0.6 * Ku;             // Ku est le gain ultime déterminé empiriquement
    //      float Ki = 1.2 * Ku / Tu;
    //      float Kd = 0.075 * Ku * Tu;

    //      // Mise à jour des paramètres du contrôleur PID
    //      pidController.SetTunings(Kp, Ki, Kd);

    // }

}