#include <Servo.h>

// --- Pins ---
const int irSensorPin = 12; 
const int enA = 11;         
const int in1 = 8;          
const int in2 = 9;          

// --- Conveyor Settings ---
int conveyorSpeed = 180;       
const long runDuration = 1400; 
unsigned long conveyorStartTime = 0;
char targetColor = ' '; // Store r, g, or b

// --- Robotic Arm Setup ---
Servo servos[6];
int servoPins[6] = {2, 3, 4, 5, 6, 7}; 
int curpos[6]    = {90, 90, 90, 90, 90, 180}; 
int homepos[6]   = {90, 90, 90, 90, 90, 180}; 
int tgtpos[6]    = {90, 90, 90, 90, 90, 180}; 
char labels[6]   = {'B', 'S', 'E', 'W', 'R', 'G'};

unsigned long lastMoveTime = 0;
const int stepDelay = 15;

// --- Automation States ---
enum SystemState { 
  IDLE, 
  CONVEYOR_RUNNING, 
  WAITING_FOR_COLOR,  // Pause here for Python detection
  PICK_BASE_SHOULDER, // Step 1
  PICK_WRIST,         // Step 2
  PICK_ELBOW,         // Step 3
  PRE_GRAB_DELAY,     // Step 4
  GRABBING,           // Step 5
  LIFTING, 
  MOVING_TO_BIN, 
  DROPPING, 
  RETURNING_HOME 
};

SystemState currentState = IDLE;
unsigned long stateWaitTimer = 0;

/**
 * Checks if the arm has reached its target position
 */
bool armReachedTarget() {
  for (int i = 0; i < 6; i++) {
    if (curpos[i] != tgtpos[i]) return false;
  }
  return true;
}

/**
 * Sequential Target Setter
 */
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
  Serial.begin(115200); // 115200 for fast communication with Python
  pinMode(irSensorPin, INPUT);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(curpos[i]);
  }
  
  controlConveyor(0);
  delay(1500); // Stabilization
  Serial.println("READY");
}

void loop() {
  updateServos();

  // --- Serial Command Logic ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      char cmd = input.charAt(0);
      
      // Reset/Home command
      if (cmd == 'h' || cmd == 'H') {
        setTarget(homepos[0], homepos[1], homepos[2], homepos[3], homepos[4], 180); 
        currentState = RETURNING_HOME;
        controlConveyor(0);
      } 
      // Color Input from Python
      else if (currentState == WAITING_FOR_COLOR) {
        if (cmd == 'r' || cmd == 'g' || cmd == 'b' || cmd == 'R' || cmd == 'G' || cmd == 'B') {
          targetColor = tolower(cmd);
          Serial.print("Target Color Locked: "); Serial.println(targetColor);
          currentState = PICK_BASE_SHOULDER; // Start strict pickup sequence
        }
      }
    }
  }

  // --- Automation Logic ---
  switch (currentState) {
    
    case IDLE:
      if (digitalRead(irSensorPin) == LOW) {
        Serial.println("STARTING_BELT");
        conveyorStartTime = millis();
        controlConveyor(1);
        currentState = CONVEYOR_RUNNING;
      }
      break;

    case CONVEYOR_RUNNING:
      if (millis() - conveyorStartTime >= runDuration) {
        controlConveyor(0);
        delay(500); // Wait for vibration to stop
        Serial.println("START_VISION"); // Signal Python to scan
        currentState = WAITING_FOR_COLOR;
      }
      break;

    case PICK_BASE_SHOULDER:
      // Step 1: Base:90, Shoulder:65, Roll:90, Gripper:180 (Open)
      setTarget(90, 65, curpos[2], curpos[3], 90, 180); 
      if (armReachedTarget()) currentState = PICK_WRIST;
      break;

    case PICK_WRIST:
      // Step 2: Wrist: 110 (Elbow still tucked)
      tgtpos[3] = 110;
      if (armReachedTarget()) currentState = PICK_ELBOW;
      break;

    case PICK_ELBOW:
      // Step 3: Elbow: 180 (Final reach)
      tgtpos[2] = 180;
      if (armReachedTarget()) {
        stateWaitTimer = millis();
        currentState = PRE_GRAB_DELAY;
      }
      break;

    case PRE_GRAB_DELAY:
      // Step 4: Stabilize for 1 second
      if (millis() - stateWaitTimer >= 1000) {
        currentState = GRABBING;
      }
      break;

    case GRABBING:
      // Step 5: Close Gripper: 98
      tgtpos[5] = 98;
      if (armReachedTarget()) {
        delay(400); 
        tgtpos[2] = 90; // Lift Elbow immediately
        currentState = LIFTING;
      }
      break;

    case LIFTING:
      if (armReachedTarget()) {
        // Now choose bin based on the stored targetColor
        if (targetColor == 'r')      setTarget(180, 80, 115, 45, 90, 98);
        else if (targetColor == 'g') setTarget(155, 80, 115, 45, 90, 98);
        else if (targetColor == 'b') setTarget(135, 70, 105, 45, 100, 98);
        currentState = MOVING_TO_BIN;
      }
      break;

    case MOVING_TO_BIN:
      if (armReachedTarget()) {
        tgtpos[5] = 180; // Release object
        currentState = DROPPING;
      }
      break;

    case DROPPING:
      if (armReachedTarget()) {
        delay(600);
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
