#include "Robot.h"
#include "moteurs.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

#if !defined(LINE_FOLLOWER_LEFT_PIN)
#error The LineFollower program requires that LINE_FOLLOWER_[LEFT,MID,RIGHT]_PIN are defined
#endif




// Définition du constructeur avec liste d'initialisation
Robot::Robot(int base_speed)
  : robot_speed(base_speed),
    state(IDLE),
    rotationStartTime(0),
    rotationDuration(0),
    CurrentLineSensorState(0),
    moteurD(base_speed, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_BACKWARD_PIN, MOTOR_RIGHT_SPEED_PIN, 0.94),
    moteurG(base_speed, MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_BACKWARD_PIN, MOTOR_LEFT_SPEED_PIN, 1) {
  initialize_ir();
}

// set_speed permet de démarrer les moteurs avec une vitesse spécifiée. Gère aussi la direction, si positif: avant, si négatif: arriere
void Robot::set_robot_speed(int speed) {
  moteurD.set_speed(speed);
  moteurG.set_speed(speed);
}

void Robot::debug() {
  Serial.println("speed :" + String(this->robot_speed));
  this->moteurD.debug();
  this->moteurG.debug();
}


void Robot::stop() {
  set_robot_speed(0);
}

void Robot::accel() {
  int new_speed = this->robot_speed + ROBOT_ACCELERATION_INCREMENT;  // Augmente la vitesse de 10 unités
  if (new_speed > ROBOT_MAX_SPEED) {
    new_speed = ROBOT_MAX_SPEED;  // Limite à la vitesse max
  }
  set_robot_speed(new_speed);  // Applique la nouvelle vitesse
}


void Robot::deccel() {
  int new_speed = this->robot_speed - ROBOT_ACCELERATION_INCREMENT;  // Réduit la vitesse de 10 unités
  if (new_speed < 0) {
    new_speed = 0;  // Empêche d’avoir une vitesse négative
  }
  set_robot_speed(new_speed);  // Applique la nouvelle vitesse
}

void Robot::right(bool pivot) {
  // Définir la vitesse selon le mode pivot ou virage progressif
  if (pivot) {
    moteurG.set_speed(ROBOT_MAX_SPEED);
    moteurD.set_speed(0);
  } else {
    moteurG.set_speed(ROBOT_MAX_SPEED);
    moteurD.set_speed(this->robot_speed / 2);
  }

  // Enregistrer le temps de début et activer l’état TURNING
  rotationStartTime = millis();
  rotationDuration = 200;  // On pourra adapter dynamiquement plus tard
  state = TURNING;
}

void Robot::left(bool pivot) {
  if (pivot) {
    moteurG.set_speed(0);
    moteurD.set_speed(ROBOT_MAX_SPEED);
  } else {
    moteurG.set_speed(this->robot_speed / 2);
    moteurD.set_speed(ROBOT_MAX_SPEED);
  }

  rotationStartTime = millis();
  rotationDuration = 200;
  state = TURNING;
}


void Robot::rotate(int angle, bool move, bool backward) {
  float timePerDegree = 200.0 / 90.0;  // Temps pour tourner de 90° (à ajuster)
  rotationDuration = abs(angle) * timePerDegree;
  rotationStartTime = millis();
  state = ROTATING;

  // Définition des vitesses selon les paramètres
  int speedG, speedD;

  if (!move) {
    // Rotation sur place
    speedG = (angle > 0) ? ROBOT_MAX_SPEED : -ROBOT_MAX_SPEED;
    speedD = (angle > 0) ? -ROBOT_MAX_SPEED : ROBOT_MAX_SPEED;
  } else {
    // Rotation en avançant ou en reculant
    int moveSpeed = backward ? -robot_speed : robot_speed;
    speedG = (angle > 0) ? moveSpeed : moveSpeed / 2;
    speedD = (angle > 0) ? moveSpeed / 2 : moveSpeed;
  }

  // Appliquer les vitesses aux moteurs
  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);
}

// Doit être appelée régulièrement dans la boucle principale (loop)
void Robot::update() {
  long int current_time = millis();
  if (state == ROTATING || state == TURNING) {
    if (current_time - rotationStartTime >= rotationDuration) {
      moteurG.set_speed(this->robot_speed);
      moteurD.set_speed(this->robot_speed);
      state = IDLE;  // Revenir à l'état normal
    }
  }
}


void Robot::move_eight() {
  // Premier tour à droite
  right();     // Le tourne à droite
  delay(500);  // Temps pour compléter le premier quart du "huit"

  delay(1000);  // Temps pour parcourir une demi-boucle

  // Deuxième virage à gauche
  left();      // Le tourne à gauche
  delay(500);  // Temps pour compléter la deuxième quart du "huit"

  delay(1000);  // Temps pour compléter la deuxième demi-boucle

  // Dernier virage à droite pour finir le "huit"
  right();     // Le tourne à droite
  delay(500);  // Temps pour faire un dernier virage
}



// CAPTEURS

// IR

void Robot::initialize_ir() {
  Serial.println("Initialisation IR");
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);  // Démarrer le récepteur IR
}


void Robot::decode_ir() {
  static unsigned long last = millis();
  static bool on = false;

  if (IrReceiver.decode()) {
    auto codeRecu = IrReceiver.decodedIRData.decodedRawData;

    if (millis() - last > 250) {
      Serial.print("Code reçu : ");
      Serial.println(codeRecu, HEX);

      on = !on;
      digitalWrite(13, on ? HIGH : LOW);

      // Conversion du code reçu en type CommandeIR
      CommandeIR commande = static_cast<CommandeIR>(codeRecu);

      // Exécuter la commande correspondante
      switch (commande) {
        case CommandeIR::FORWARD:
          set_robot_speed(this->robot_speed);
          break;
        case CommandeIR::BACKWARD:
          set_robot_speed(-this->robot_speed);
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
        default:
          Serial.println("Commande inconnue !");
          break;
      }
      Serial.print(state);

      last = millis();
    }

    IrReceiver.resume();  // Préparer le récepteur pour la prochaine réception
  }
}

// Suiveur de ligne

void Robot::line_follower() {
  uint8_t NewSensorState = 0;
  if (digitalRead(LINE_FOLLOWER_LEFT_PIN) == LOW) NewSensorState = 1;    // set bit 0 if input is HIGH
  if (digitalRead(LINE_FOLLOWER_MID_PIN) == LOW) NewSensorState |= 2;    // set bit 1 if input is HIGH
  if (digitalRead(LINE_FOLLOWER_RIGHT_PIN) == LOW) NewSensorState |= 4;  // set bit 2 if input is HIGH

  /*
     * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
     * 0 - All sensors are dark or not connected -> stop or go forward after sharp turn
     * 1 - Mid and right sensors are dark -> sharp right
     * 2 - Left and right sensors are dark -> panic stop, because this is unexpected
     * 3 - Only right sensor is dark -> right
     * 4 - Mid and left sensors are dark -> sharp left
     * 5 - Only mid sensor is dark -> forward
     * 6 - Only left sensor is dark -> left
     * 7 - All sensors are not dark -> stop or go backward after turn
     */

  /*
     * If sensor input does not change, we do not need to change movement!
     */
  if (this->CurrentLineSensorState != NewSensorState) {

    Serial.print(F("SensorState="));
    Serial.print(NewSensorState);
    Serial.print(F(" -> "));

    switch (NewSensorState) {
      case 0:
        // All sensors are dark or not connected -> stop or go forward after sharp turn
        if (this->CurrentLineSensorState == 1 || this->CurrentLineSensorState == 4) {  // test for sharp turns
          set_robot_speed(this->robot_speed);
          Serial.println(F("Forward during sharp turn"));
        } else {
          stop();
          Serial.println(F("Stop"));
        }
        break;
      case 1:
        // Mid and right sensors are dark -> sharp right
        rotate(-180);
        Serial.println(F("Turn sharp right"));
        break;
      case 2:
        // Left and right sensors are dark -> panic stop, because this is unexpected
        stop();
        Serial.println(F("panic stop"));
        break;
      case 3:
        // Only right sensor is dark -> right
        rotate(-180, true);
        Serial.println(F("Turn right"));
        break;
      case 4:
        // Mid and left sensors are dark -> sharp left
        rotate(180);
        Serial.println(F("Turn sharp left"));
        break;
      case 5:
        // Only mid sensor is dark -> forward
        set_robot_speed(this->robot_speed);
        Serial.println(F("Forward"));
        break;
      case 6:
        // Only left sensor is dark -> left
        rotate(180, true);
        Serial.println(F("Turn left"));
        break;
      case 7:
        // All sensors are not dark -> stop or go backward after turn
        if (this->CurrentLineSensorState == 6 || this->CurrentLineSensorState == 3) {
          set_robot_speed(-this->robot_speed);
          Serial.println(F("Forward after turn"));
        } else {
          stop();
          Serial.println(F("Stop"));
        }
        break;
    }
    this->CurrentLineSensorState = NewSensorState;
  }
}


