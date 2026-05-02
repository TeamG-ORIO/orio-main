#include "ClearCore.h"

// --- Pin Definitions ---
#define PNP_CUP IO3
#define LBL_CUP IO4       
#define LBL_VAC_PIN A11
#define PNP_VAC_PIN A10   

#define baudRateSerialPort 115200

// --- Vacuum Thresholds (0-4095 scale) ---
// You will likely need to tune these values based on the "read_vac" command output!
#define VAC_THRESHOLD_HIGH 1000 // Raw ADC value that guarantees an item is firmly attached
#define VAC_THRESHOLD_LOW  800 // Raw ADC value that guarantees the item has detached/dropped

// --- State Tracking Variables ---
bool lbl_has_item = false;
bool pnp_has_item = false;

void setup() {
  Serial.begin(baudRateSerialPort);
  while (!Serial) {
    continue;
  }

  // Configure Output
  pinMode(LBL_CUP, OUTPUT);
  pinMode(PNP_CUP, OUTPUT);

  // Set Initial State: OFF (HIGH)
  digitalWrite(LBL_CUP, HIGH);
  digitalWrite(PNP_CUP, HIGH);

  // Configure Inputs
  pinMode(LBL_VAC_PIN, INPUT);
  pinMode(PNP_VAC_PIN, INPUT);
}

void loop() {
  // ==========================================
  // 1. HANDLE SERIAL COMMANDS
  // ==========================================
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "lbl_on") {
      digitalWrite(LBL_CUP, LOW);
      Serial.println("CMD: LBL Cup ON");
    } 
    else if (command == "lbl_off") {
      digitalWrite(LBL_CUP, HIGH);
      Serial.println("CMD: LBL Cup OFF");
    }
    else if (command == "pnp_on") {
      digitalWrite(PNP_CUP, LOW);
      Serial.println("CMD: PNP Cup ON");
    } 
    else if (command == "pnp_off") {
      digitalWrite(PNP_CUP, HIGH);
      Serial.println("CMD: PNP Cup OFF");
    }
    else if (command == "disable") {
      digitalWrite(LBL_CUP, HIGH);
      digitalWrite(PNP_CUP, HIGH);
      Serial.println("CMD: ALL Cups DISABLED");
    }
    else if (command == "read_vac") {
      // Diagnostic command to help you find your threshold numbers
      Serial.print("LBL_VAC: ");
      Serial.print(analogRead(LBL_VAC_PIN));
      Serial.print(" | PNP_VAC: ");
      Serial.println(analogRead(PNP_VAC_PIN));
    }
  }

  // ==========================================
  // 2. MONITOR VACUUM STATES (ITEM DETECTION)
  // ==========================================
  int current_lbl_vac = analogRead(LBL_VAC_PIN);
  int current_pnp_vac = analogRead(PNP_VAC_PIN);

  // --- Labeling Cup Detection ---
  if (!lbl_has_item && current_lbl_vac > VAC_THRESHOLD_HIGH) {
    // Item was just picked up
    lbl_has_item = true;
    Serial.println("EVENT: LBL Cup Picked Up Item");
  } 
  else if (lbl_has_item && current_lbl_vac < VAC_THRESHOLD_LOW) {
    // Item was just dropped (or vacuum was turned off)
    lbl_has_item = false;
    Serial.println("EVENT: LBL Cup Dropped Item");
  }

  // --- Pick and Place Cup Detection ---
  if (!pnp_has_item && current_pnp_vac > VAC_THRESHOLD_HIGH) {
    // Item was just picked up
    pnp_has_item = true;
    Serial.println("EVENT: PNP Cup Picked Up Item");
  } 
  else if (pnp_has_item && current_pnp_vac < VAC_THRESHOLD_LOW) {
    // Item was just dropped (or vacuum was turned off)
    pnp_has_item = false;
    Serial.println("EVENT: PNP Cup Dropped Item");
  }

  // Small delay for loop stability
  delay(10);
}