#include <Arduino.h>   
#include "Robot.h"
#include <IRremote.hpp>




Robot* robot;

void setup() {
  robot = new Robot(70);
  Serial.begin(9600);
}

void loop() {
  robot->decode_ir();
  robot->update();

}
