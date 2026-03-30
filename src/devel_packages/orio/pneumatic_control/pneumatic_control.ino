#include "ClearCore.h"

// Pin Definitions
#define PNP_CUP IO3
#define LBL_CUP IO4       


#define baudRateSerialPort 115200

void setup() {
  Serial.begin(baudRateSerialPort);
  while (!Serial) {
    continue;
  }

  pinMode(LBL_CUP, OUTPUT);
  pinMode(PNP_CUP, OUTPUT);

  // Set Initial State: OFF (HIGH)
  digitalWrite(LBL_CUP, HIGH);
  digitalWrite(PNP_CUP, HIGH);
  
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Labelling Cup
    if (command == "lbl_on") {
      digitalWrite(LBL_CUP, LOW);
    } 
    else if (command == "lbl_off") {
      digitalWrite(LBL_CUP, HIGH);
    }

    // PnP Cup
    else if (command == "pnp_on") {
      digitalWrite(PNP_CUP, LOW);
    } 
    else if (command == "pnp_off") {
      digitalWrite(PNP_CUP, HIGH);
    }

    // Global Disable
    else if (command == "disable") {
      digitalWrite(LBL_CUP, HIGH);
      digitalWrite(PNP_CUP, HIGH);
    }
  }
  delay(10);
}