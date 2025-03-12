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
      pidKp(30.0),
      pidKi(0),
      pidKd(0),
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
    pidController.SetSampleTime(10); // 45 ms entre chaque calcul du PID
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
  set_robot_speed(new_speed);
}

// Décélère le robot
void Robot::deccel() {
  int new_speed = this->robot_speed - ROBOT_ACCELERATION_INCREMENT;  // Réduit la vitesse
  if (new_speed < 0) {
    new_speed = 0;  // Évite une vitesse négative
  }
  set_robot_speed(new_speed);
}

// ===========================================
// Fonctions de rotation / virage
// ===========================================

// Tourne à droite (virage ou pivot)
void Robot::right(bool pivot) {
  changeRotationState(TURNING);

  int speedG = ROBOT_MAX_SPEED;
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
  int speedD = ROBOT_MAX_SPEED;

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
// move indique si la rotation se fait en mouvement (avancer/reculer) et backward si c'est en marche arrière

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
  float timePerDegree = 200/90;  // par exemple, 2 ms par degré (à ajuster)
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

void Robot::update() {
  long int current_time = millis();
  // On lit l'état des capteurs une seule fois

  if (rotationState == ROTATING || rotationState == TURNING) {
    if (current_time - rotationStartTime >= rotationDuration) {
      // Appliquer la vitesse en fonction de l'état de déplacement
      int appliedSpeed = (movementState == BACKWARD) ? -robot_speed : robot_speed;
      moteurG.set_speed(appliedSpeed);
      moteurD.set_speed(appliedSpeed);
      rotationState = ROTATION_IDLE;
    }
  }
  if (movementState == SHARPTURNING) {
    sharpturn();
  }
  if (movementState == LINEFOLLOWING) {
    line_follower_pid();
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
  // Premier virage à droite
  right();
  delay(500);   // Temps pour le premier quart du "8"
  delay(1000);  // Demi-boucle

  // Virage à gauche
  left();
  delay(500);   // Deuxième quart du "8"
  delay(1000);  // Deuxième demi-boucle

  // Dernier virage à droite pour terminer le "8"
  right();
  delay(500);  // Dernier virage
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

    if (millis() - last > 250) {
      on = !on;
      digitalWrite(13, on ? HIGH : LOW);

      // Conversion du code reçu en type CommandeIR
      CommandeIR commande = static_cast<CommandeIR>(codeRecu);

      if (movementState == LINEFOLLOWING && commande != CommandeIR::LINEFOLLOWER) {
        Serial.println("Commande ignorée : le robot est en mode LINE FOLLOWING.");
        IrReceiver.resume();  
        return;
      }

      // Exécuter la commande correspondante
      switch (commande) {
        case CommandeIR::FORWARD:
          set_robot_speed(this->base_speed);
          break;
        case CommandeIR::BACKWARD:
          set_robot_speed(-this->base_speed);
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
        default:
          Serial.println("Commande inconnue !");
          break;
      }

      last = millis();
    }

    IrReceiver.resume();  // Préparer pour la prochaine réception
  }
}

// LINE FOLLOWER via switch case : moins fluide (donc non recommandée)



// // Line follower via switch case est appelé
// void Robot::line_follower() {
//   uint8_t NewSensorState = getSensorState();
//   /*
//      * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
//      * 0 - All sensors are white or not connected -> stop 
//      * 1 - Only left sensor is black -> Sharp turn left
//      * 2 - Only mid sensor is black -> forward 
//      * 3 - Mid and left sensor are black -> left forward
//      * 4 - Only right sensor is black -> Sharp turn right
//      * 5 - Left and right sensor black -> panic stop
//      * 6 - right and mid sensor are black-> right forward
//      * 7 - All sensors are black  -> stop or go backward after turn
//      */
//   /*
//      * If sensor input does not change, we do not need to change movement!
//      */
//   if (this->CurrentLineSensorState != NewSensorState) {
//     printState();
//     Serial.print(F("SensorState="));
//     Serial.print(NewSensorState);
//     Serial.print(F(" -> "));
//     switch (NewSensorState) {
//       case 0:
//         stop();
//         Serial.println(F("Stop"));
//         break;
//       case 1:
//         changeMovementState(SHARPTURNING);
//         Serial.println(F("sharp turn left"));
//         break;
//       case 2:
//         set_robot_speed(this->base_speed);
//         Serial.println(F("Forward"));
//         break;
//       case 3:
//         lastTurnDirection = LEFT;
//         rotate(-180, true);
//         Serial.println(F("left forward"));
//         break;
//       case 4:
//         changeMovementState(SHARPTURNING);
//         Serial.println(F("sharp turn left"));
//         break;
//       case 5:
//         stop();
//         Serial.println(F("Panic stop"));
//         break;
//       case 6:
//         lastTurnDirection = RIGHT;
//         rotate(180, true);
//         Serial.println(F("right forward"));
//         break;
//       case 7:
//         sharpturn();
//         break;
//     }
//     this->CurrentLineSensorState = NewSensorState;
//   }
// }


// Line follower via PID


// reset des coefficients du PID

uint8_t Robot::getSensorState() {
  uint8_t sensorState = 0;
  if (digitalRead(LINE_FOLLOWER_LEFT_PIN) == HIGH) sensorState = 1;   // bit 0
  if (digitalRead(LINE_FOLLOWER_MID_PIN) == HIGH) sensorState |= 2;    // bit 1
  if (digitalRead(LINE_FOLLOWER_RIGHT_PIN) == HIGH) sensorState |= 4;  // bit 2
  return sensorState;
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
  static float previousError = 0.0;
  static constexpr float errorTable[8] = {0.0, -1.7, 0.0, -1.2, 1.7, -0.7, 1.2, 0.0};  
  bool calibration = true;  

  // Lire PIND et extraire les bits D4, D3, D2
  uint8_t sensorState = ((PIND & 0b00001100) >> 2) | ((PIND & 0b00010000) >> 2);

  float error = errorTable[sensorState];
  error = (previousError + error) / 2.0;
  previousError = error;
  // Gestion des cas spéciaux
  if (!calibration && sensorState == 0) {
      changeMovementState(SHARPTURNING);
  }

  return error;
}





    //Calcul de l'erreur via le mapping a priori moins efficace 

  // uint8_t sensorState = getSensorState();
  // Serial.println(sensorState);
  // float error = 0.0;  // Valeur par défaut

  // switch(sensorState) {
  //   case 0: // 000 : Aucun capteur ne détecte la ligne
  //     if(!calibration){
  //        changeMovementState(SHARPTURNING);
  //     } else {
  //        // En mode calibration, on peut par exemple assigner une valeur neutre
  //        error = 0;
  //     }
  //     break;
      
  //   case 1: // 001 : Seul le capteur gauche détecte la ligne
  //     error = -1.7;
  //     break;
      
  //   case 2: // 010 : Seul le capteur central détecte la ligne
  //     error = 0;
  //     break;
      
  //   case 3: // 011 : Gauche et centre
  //     error = -1.2;
  //     break;
      
  //   case 4: // 100 : Seul le capteur droit détecte la ligne
  //     error = 1.7;
  //     Serial.println("FFFFFFFFFFF");
  //     break;
      
  //   case 5: // 101 : Cas critique : pas censé arriver
  //     // On peut décider de déclencher une récupération ou assigner une valeur extrême
  //     error = -0.7; // Par exemple
  //     break;
      
  //   case 6: // 110 : Droite et centre 
  //     error = 1.2;
  //     break;
      
  //   case 7: // 111 : Tous les capteurs détectent la ligne, situation ambiguë (peut-être un carrefour)
  //     if(!calibration){
  //        changeMovementState(SHARPTURNING);
  //     } else {
  //        // En mode calibration, on peut par exemple assigner une valeur neutre
  //        error = 0;
  //     }
  //     break;
  //   default:
  //     error = 0;
  //     break;
  // }


void Robot::line_follower_pid() {
    // Récupérer l'erreur estimée à partir des capteurs
    pid_input = errorestimation();

    // Calcul du PID : le résultat sera stocké dans pid_output
    pidController.Compute();

    // Calcul des vitesses pour les moteurs en fonction de la correction
    float correction = pid_output;
    int leftSpeed = robot_speed + correction;
    int rightSpeed = robot_speed - correction;

    // Contraintes sur les vitesses
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
    // Un capteur détecte la ligne, on passe en mode LINEFOLLOWING
    resetPID();
    set_robot_speed(base_speed);  // Réinitialise la vitesse à la valeur de base
    changeMovementState(LINEFOLLOWING);
    return;
  } else {
    // Sinon, continuer à tourner par petits incréments
    rotate((lastTurnDirection == RIGHT) ? 10 : -10);
  }
}

// void Robot::autoTunePID() {
//   // Variables locales pour l'autotuning
//   double pidInput = 0;   // Valeur issue de l'estimation d'erreur
//   double pidOutput = 0;  // Correction calculée par l'auto-tuner
  
//   // Crée une instance locale du tuner, qui travaillera sur pidInput et pidOutput.
//   PID_ATune tuner(&pidInput, &pidOutput);
  
//   // Configuration du tuner PID
//   tuner.SetNoiseBand(0);     // Ajustez la bande de bruit en fonction de vos capteurs
//   tuner.SetOutputStep(10);      // Amplitude de perturbation appliquée aux moteurs
//   tuner.SetLookbackSec(2);     // Durée d'analyse (en secondes)
//   tuner.SetControlType(1);     // 1 = contrôle PID standard
  
//   bool tuning = true;
  
//   Serial.println("Démarrage de l'auto-tuning PID...");
  
//   // Boucle d'auto-tuning
//   while (tuning) {
//     // Met à jour l'entrée PID avec l'erreur actuelle calculée par votre méthode
//     pidInput = this->errorestimation();
//     // Exécute une étape d'auto-tuning
//     int tuningStatus = tuner.Runtime();
    
//     // Pendant le tuning, applique la sortie (pidOutput) aux moteurs.
//     // Ici, on ajuste temporairement la vitesse des moteurs en fonction du pidOutput.
//     int motorSpeed = 50 + (int)pidOutput;  // base_speed étant la vitesse de base de votre robot
//     moteurG.set_speed(constrain(motorSpeed, 0, ROBOT_MAX_SPEED));
//     moteurD.set_speed(constrain(motorSpeed, 0, ROBOT_MAX_SPEED));
    
//     // Si tuningStatus n'est pas zéro, le processus est terminé.
//     if (tuningStatus != 0) {
//       tuning = false;
//     }
    
//   }
  
//   // Une fois l'auto-tuning terminé, récupérez les valeurs optimales :
//   finalKp = tuner.GetKp();
//   finalKi = tuner.GetKi();
//   finalKd = tuner.GetKd();
  
  
//   Serial.println("Auto-tuning terminé !");
//   Serial.print("Kp = ");
//   Serial.println(finalKp);
//   Serial.print("Ki = ");
//   Serial.println(finalKi);
//   Serial.print("Kd = ");
//   Serial.println(finalKd);
// }