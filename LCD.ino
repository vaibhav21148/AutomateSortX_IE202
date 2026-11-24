#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- LCD Configuration ---
// Set the LCD address to 0x27 or 0x3F for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- Pins ---
const int irSensorPin = 12; 
const int enA = 11;         
const int in1 = 8;          
const int in2 = 9;          

// --- Settings ---
int conveyorSpeed = 180;       
const long runDuration = 1400; 
unsigned long conveyorStartTime = 0;
char targetColor = ' '; 

// --- Robotic Arm Setup ---
Servo servos[6];
int servoPins[6] = {2, 3, 4, 5, 6, 7}; 
int curpos[6]    = {90, 90, 90, 90, 90, 180}; 
int homepos[6]   = {90, 90, 90, 90, 90, 180}; 
int tgtpos[6]    = {90, 90, 90, 90, 90, 180}; 

unsigned long lastMoveTime = 0;
const int stepDelay = 15;

enum SystemState { 
  IDLE, CONVEYOR_RUNNING, WAITING_FOR_COLOR, 
  PICK_BASE_SHOULDER, PICK_WRIST, PICK_ELBOW, 
  PRE_GRAB_DELAY, GRABBING, LIFTING, MOVING_TO_BIN, DROPPING, RETURNING_HOME 
};

SystemState currentState = IDLE;
unsigned long stateWaitTimer = 0;

/**
 * Helper function to update the LCD display efficiently
 */
void updateDisplay(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

bool armReachedTarget() {
  for (int i = 0; i < 6; i++) { if (curpos[i] != tgtpos[i]) return false; }
  return true;
}

void setTarget(int b, int s, int e, int w, int r, int g) {
  tgtpos[0] = b; tgtpos[1] = s; tgtpos[2] = e;
  tgtpos[3] = w; tgtpos[4] = r; tgtpos[5] = g;
}

void controlConveyor(int state) {
  if (state == 1) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    analogWrite(enA, conveyorSpeed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

void updateServos() {
  if (millis() - lastMoveTime >= stepDelay) {
    lastMoveTime = millis();
    for (int i = 0; i < 6; i++) {
      if (curpos[i] < tgtpos[i]) { curpos[i]++; servos[i].write(curpos[i]); }
      else if (curpos[i] > tgtpos[i]) { curpos[i]--; servos[i].write(curpos[i]); }
    }
  }
}

void setup() {
  Serial.begin(115200); 
  
  // LCD Initialization
  lcd.init();
  lcd.backlight();
  updateDisplay("AutomateSortX", "Initializing...");
  
  pinMode(irSensorPin, INPUT);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(curpos[i]);
  }
  
  controlConveyor(0);
  delay(1500); 
  
  updateDisplay("AutomateSortX", "System Ready");
  Serial.println("READY");
}

void loop() {
  updateServos();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      char cmd = input.charAt(0);
      
      if (cmd == 'h' || cmd == 'H') {
        updateDisplay("MANUAL RESET", "Returning Home");
        setTarget(homepos[0], homepos[1], homepos[2], homepos[3], homepos[4], 180); 
        currentState = RETURNING_HOME;
        controlConveyor(0);
      } 
      else if (currentState == WAITING_FOR_COLOR) {
        if (cmd == 'r' || cmd == 'g' || cmd == 'b' || cmd == 'R' || cmd == 'G' || cmd == 'B') {
          targetColor = tolower(cmd);
          String colorName = (targetColor == 'r') ? "Red" : (targetColor == 'g') ? "Green" : "Blue";
          updateDisplay("Color Detected:", colorName);
          currentState = PICK_BASE_SHOULDER; 
        }
      }
    }
  }

  switch (currentState) {
    
    case IDLE:
      // Small logic to show "Waiting" once if we just finished
      static bool waitingMsgShown = false;
      if (!waitingMsgShown) {
        updateDisplay("AutomateSortX", "Waiting for next");
        waitingMsgShown = true;
      }
      
      if (digitalRead(irSensorPin) == LOW) {
        updateDisplay("Object Detected", "Moving Belt...");
        conveyorStartTime = millis();
        controlConveyor(1);
        currentState = CONVEYOR_RUNNING;
        waitingMsgShown = false;
      }
      break;

    case CONVEYOR_RUNNING:
      if (millis() - conveyorStartTime >= runDuration) {
        controlConveyor(0);
        updateDisplay("At Station", "Scanning Color");
        Serial.println("START_VISION"); 
        currentState = WAITING_FOR_COLOR;
      }
      break;

    case PICK_BASE_SHOULDER:
      // Base, Shoulder, Roll set, Gripper open
      setTarget(90, 65, curpos[2], curpos[3], 90, 180); 
      if (armReachedTarget()) currentState = PICK_WRIST;
      break;

    case PICK_WRIST:
      tgtpos[3] = 110;
      if (armReachedTarget()) currentState = PICK_ELBOW;
      break;

    case PICK_ELBOW:
      tgtpos[2] = 180;
      if (armReachedTarget()) {
        stateWaitTimer = millis();
        currentState = PRE_GRAB_DELAY;
      }
      break;

    case PRE_GRAB_DELAY:
      if (millis() - stateWaitTimer >= 1000) {
        updateDisplay("Picking Up", "Closing Gripper");
        currentState = GRABBING;
      }
      break;

    case GRABBING:
      tgtpos[5] = 98;
      if (armReachedTarget()) {
        delay(400); 
        tgtpos[2] = 90; // Lift Elbow
        currentState = LIFTING;
      }
      break;

    case LIFTING:
      if (armReachedTarget()) {
        updateDisplay("Sorting...", "Moving to Bin");
        if (targetColor == 'r')      setTarget(180, 80, 115, 45, 90, 98);
        else if (targetColor == 'g') setTarget(155, 80, 115, 45, 90, 98);
        else if (targetColor == 'b') setTarget(135, 70, 105, 45, 100, 98);
        currentState = MOVING_TO_BIN;
      }
      break;

    case MOVING_TO_BIN:
      if (armReachedTarget()) {
        updateDisplay("Bin Reached", "Placed in Box");
        tgtpos[5] = 180; 
        currentState = DROPPING;
      }
      break;

    case DROPPING:
      if (armReachedTarget()) {
        delay(600);
        updateDisplay("Task Done", "Back to Home");
        setTarget(homepos[0], homepos[1], homepos[2], homepos[3], homepos[4], 180);
        currentState = RETURNING_HOME;
      }
      break;

    case RETURNING_HOME:
      if (armReachedTarget()) {
        Serial.println("COMPLETE");
        currentState = IDLE;
      }
      break;
  }
}
