 //===============================
// 3pi+ Line Maze Runner V2
// Improved Maze Runner.
// By: aldrick-t, DaniDRG04, ArlesMolina
// Version: 1.1 (JUN 2024)
//
//===============================

#include <Arduino.h>
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <string.h>

using namespace Pololu3piPlus32U4;
 
OLED display;
Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
LineSensors lineSensors;
BumpSensors bumpSensors;
Motors motors;
Encoders encoders;

//======================== Global Variables ========================
//Motor Speed Placeholders
int motorSpeed = 60;
int minMotorSpeed = 35;

//Encoder Variables
signed long encCountsL = encoders.getCountsAndResetLeft();
signed long encCountsR = encoders.getCountsAndResetRight();
int encCountsAvg = 0;

//Bump Sensor Variables
bool bumpLeft = false;
bool bumpRight = false;

//Line Sensor Variables
uint16_t lineSensVals[5];
uint16_t lineSensCalib[5];
uint16_t predict = 0;
uint16_t sensVals[4];

//Maze Runner Decision Memory Variables
const int MAX_DECISIONS = 100; // Maximum size of the decision history
char decisionHistory[MAX_DECISIONS]; // Stores decisions made during the first run
char optimizedPath[MAX_DECISIONS]; // optimized path
int decisionCount = -1; // Initialize at -1 because we increment before storing
int optCount = -1;
char decision = 0;
char decisionMem = 0;
bool deadEnd = false;
char intersection = 0;
int printCount = 0;

//Time Variables
float prevTime = 0;
float period = 0.05;

//Maze Runner Mode Variables
bool whiteLine = false;
bool rightHand = true;

// Left, Center, Right Line Sensor Isolations
int leftInt;
int centerInt;
int rightInt;

bool left;
bool center;
bool right;

bool leftMem;
int centerLMem;
bool centerMem;
int centerRMem;
bool rightMem;

//PID Control Variables
int motorSpeedAdj;
int motorSpeedL = 0;
int motorSpeedR = 0;
uint16_t Kp = 64; //0.5 //10:68s, 8:86s
uint16_t Kd = 256; //3.0 //10:68s, 8:86s
const int midPoint = 2000;
int deviation = 0;
int lastDeviation = 0;
long integral = 0;

//Angle Variables
int angleTotal = 0;

//================= Function Declarations ==================
//Menu display declarations
char mainMenu(char);
char opMenu(char);
int settings(int);

//Settings function declarations
void speed();
void lineSensorsSet(int);

//Operation modes declarations
void mazeRunner();

//About function declaration
void about();

//Conversion functions
float tick2deg(int);

//utility functions
void optimizePath(char[], int&);
char rightHandDecision();
char leftHandDecision();
void updateSensors();
void verifyIntersection_crawlFwd(int ticks, bool leftRef, bool centerRef, bool rightRef);
void crawlFwd_alignToWheel();

//Maze Solver Dedicated Functions
void straightSegment();
char leftHandRule();
char rightHandRule();
void turnControl();
//================= Special Character Definitions ==================
//Forward arrows
const char forwardArrows[] PROGMEM = {
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
};

//Fully lit up character matrix.
const char fullBlock[] PROGMEM = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};

//Back arrow. (return arrow)
const char backArrow[] PROGMEM = {
  0b00000,
  0b00010,
  0b00001,
  0b00101,
  0b01001,
  0b11110,
  0b01000,
  0b00100,
};

//Two chevrons pointing down.
const char reverseArrows[] PROGMEM = {
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b10001,
  0b01010,
  0b00100,
  0b00000,
};

//Per second character
const char perSec[] PROGMEM = {
  0B00000,
  0B00010,
  0B00100,
  0B01011,
  0B10100,
  0B00011,
  0B00001,
  0B00110
};

//Reload character (rounded arrow)
const char reload[] PROGMEM = {
  0B00000,
  0B10110,
  0B11001,
  0B11101,
  0B00001,
  0B10001,
  0B01110,
  0B00000
};

//============= Principal Behavioral Structure =====================

void setup() {
  //loads custom characters to memory
  display.loadCustomCharacter(forwardArrows, 1);
  display.loadCustomCharacter(reverseArrows, 2);
  display.loadCustomCharacter(backArrow, 7);
  display.loadCustomCharacter(fullBlock, 3);
  display.loadCustomCharacter(perSec, 4);
  display.loadCustomCharacter(reload, 5);

  display.setLayout21x8();
  display.noAutoDisplay();
  display.clear();

  bumpSensors.calibrate();
  Serial.begin(9600);
}

void loop() {
  char mode = 0;
  //int vel = motorSpeed;
  while (true) {
    //update settings variables
    //vel = motorSpeed;
    //menu switch
    switch (mode) {
    case 0:
      mode = mainMenu(mode);
      break;
    case 1:
      //Operation menu
      mode = opMenu(mode);
      break;
    case 2:
      //Settings menu
      mode = settings(mode);
      break;
    case 3:
      //About menu
      about();
      mode = 0;
      break;
    case 11:
      //Line Follow mode
      mazeRunner();
      mode = 1;
      break;
    case 21:
      //Motor Speed
      speed();
      mode = 2;
      break;
    case 22:
      //Line Sensors
      lineSensorsSet(mode);
      mode = 2;
      break;

    default:
      break;
    }
  }
}

//==================== Menu Displays ===============================

char mainMenu(char mode) {
  //main menu
  display.clear();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("3pi+ Test Platform   ");
  display.gotoXY(0,5);
  display.print("Start              :A");
  display.gotoXY(0,6);
  display.print("Settings           :B");
  display.gotoXY(0,7);
  display.print("About              :C");
  display.display();

  while(true) {
    if(buttonA.getSingleDebouncedPress()) {
      mode = 1;
      break;
    }
    else if(buttonB.getSingleDebouncedPress()) {
      mode = 2;
      break;
    }
    else if(buttonC.getSingleDebouncedPress()) {
      mode = 3;
      break;
    }
  }

  return mode;
}

char opMenu(char mode) {
  display.clear();
  display.noInvert();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("Operation Modes:     ");
  display.gotoXY(0,2);
  display.print("                   >>");
  display.gotoXY(0,5);
  display.print("Next               :A");
  display.gotoXY(0,6);
  display.print("Select             :B");
  display.gotoXY(0,7);
  display.print("Back\7              :C");
  //display.display();

  int setting = 0;
  const char* settings[] = {"Maze Runner"};
  while(true) {
    //display.gotoXY(0,2);
    //display.print("                   ");
    display.gotoXY(0,2);
    //display.print();
    display.print(settings[setting]);
    display.display();
    if (buttonA.getSingleDebouncedPress()){
      setting++;
      if (setting == 1) setting = 0;
    }
    else if (buttonB.getSingleDebouncedPress()){
      mode = setting + 11;
      break;
    }
    if(buttonC.getSingleDebouncedPress()) {
      mode = 0;
      break;
    }
  }
  return mode;
}

int settings(int mode) {
  display.clear();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("Settings:            ");
  display.gotoXY(0,2);
  display.print("                   >>");
  display.gotoXY(0,5);
  display.print("Next               :A");
  display.gotoXY(0,6);
  display.print("Select             :B");
  display.gotoXY(0,7);
  display.print("Back\7              :C");
  display.display();

  int setting = 0;
  String settings[] = {"Motor Speed ", "Line Sensors"};
  while(true) {
    display.gotoXY(0,2);
    display.print(settings[setting]);
    display.gotoXY(0,2);
    display.displayPartial(2, 0, 23);
    if (buttonA.getSingleDebouncedPress()){
      setting++;
      if (setting == 2) setting = 0;
    }
    else if (buttonB.getSingleDebouncedPress()){
      mode = setting + 21;
      break;
    }
    if(buttonC.getSingleDebouncedPress()) {
      mode = 0;
      break;
    }
  }
  return mode;
}

void about() {
  display.clear();
  display.gotoXY(0,0);
  display.print("3pi+ Pest Platform   ");
  display.gotoXY(0,1);
  display.print("Version: 1.2.1       ");
  display.gotoXY(0,2);
  display.print("All in one functiona-");
  display.gotoXY(0,3);
  display.print("lity test platform.  ");
  display.gotoXY(0,7);
  display.print("Back\7              :C");
  display.display();

  while(true){
    if(buttonC.getSingleDebouncedPress()) {
      break;
    }
  }
}

void speed(){
  int vel = motorSpeed;
  //velocity
  //display velocity menu
  display.clear();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("Motor Speed:         ");
  display.gotoXY(0,6);
  display.print(" A        B        C ");
  display.gotoXY(0,7);
  display.print(" -        +        \7 ");
  display.display();
  while(true){
    //vel edit
    //reduce by 20 until 0
    if (buttonA.getSingleDebouncedPress() && vel > 0){
      vel = vel - 20;  
    //increase by 20 to limit
    } 
    else if (buttonB.getSingleDebouncedPress() && vel < 400){
      vel = vel + 20;
    }
    //print vel value
    display.gotoXY(0,2);
    display.print("Min");
    display.gotoXY(18,2);
    display.print("Max");
    display.gotoXY(0,3);
    display.print(" 0 ");
    display.gotoXY(18,3);
    display.print("400");
    display.display();
    display.gotoXY(9,3);
    display.print(vel);
    display.print(" ");
    display.gotoXY(0,2);
    display.displayPartial(2, 0, 23);
    
    motors.setSpeeds(vel, vel);
    //option exit
    if (buttonC.getSingleDebouncedPress()){
      motors.setSpeeds(0, 0);
      motorSpeed = vel;
      break;
    }
  }
  motors.setSpeeds(0, 0);
}

void lineSensorsSet(int sens) {
  bool emitterToggle = false;
  //lineSensors.calibrate();
  //lineSensors.emittersOn();

  display.clear();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("Line Sens:");
  //display.gotoXY(0,1);
  display.print(" Emitters:         ");
  display.gotoXY(0,2);
  display.print("    2    3    4      ");
  display.gotoXY(0,3);
  display.print("1                   5");
  display.gotoXY(0,5);
  display.print("Calibrate          :A");
  display.gotoXY(0,6);
  display.print("Toggle Emitters    :B");
  display.gotoXY(0,7);
  display.print("Back\7              :C");

  while(true) {
    display.gotoXY(0,0);
    display.print("Line Sens:           ");
    lineSensors.readCalibrated(lineSensVals);

    display.gotoXY(10,0);
    //display.print(" Calibrated");
    display.gotoXY(0,4);
    display.print(lineSensVals[0]);
    display.print("    ");
    display.gotoXY(4,3);
    display.print(lineSensVals[1]);
    display.print("    ");
    display.gotoXY(9,3);
    display.print(lineSensVals[2]);
    display.print("    ");
    display.gotoXY(14,3);
    display.print(lineSensVals[3]);
    display.print("    ");
    display.gotoXY(17,4);
    display.print(lineSensVals[4]);
    display.print("    ");
    display.display();


    if(emitterToggle) {
      lineSensors.emittersOn();
      display.gotoXY(13,1);
      display.print("On ");
    } 
    else if(!emitterToggle) {
      lineSensors.emittersOff();
      display.gotoXY(13,1);
      display.print("Off");
    }
    if (buttonA.getSingleDebouncedPress()) {
      for (int i = 0; i<100; i++){
        lineSensors.calibrate();
        delay(100);
        display.gotoXY(0,0);
        display.print("Calibrating...");
      }
    }
    else if(buttonB.getSingleDebouncedPress()) {
      emitterToggle = !emitterToggle;
      delay(100);
    }
    if(buttonC.getSingleDebouncedPress()) {
      lineSensors.emittersOff();
      break;
    }
  }
}

//==================== Maze Runner ================================

void mazeRunner() {
  int8_t modeLoc = 1;

  //Initial Menu Screen
  display.clear();
  display.noInvert();
  display.setLayout21x8();
  display.gotoXY(0,0);
  display.print("Maze Runner:         ");
  lineSensors.readCalibrated(lineSensVals);

  //Line type setting screen
  display.gotoXY(0,1);
  display.print("  Select Line Type:  ");
  display.gotoXY(0,3);
  display.print("     Black Line      ");
  display.gotoXY(0,4);
  display.print("     White Line      ");
  display.gotoXY(0,6);
  display.print(" A        B          ");
  display.gotoXY(0,7);
  display.print("\1/\2      SEL       ");

  //line type setting loop
  while(modeLoc == 1) {
    if(buttonA.getSingleDebouncedPress()) {
      whiteLine = !whiteLine;
      if (whiteLine) {
        //select white on screen
        display.gotoXY(3,4);
        display.print("->");
        display.print("White Line");
        display.print("<-");
        //deselect black on screen
        display.gotoXY(3,3);
        display.print("  ");
        display.print("Black Line");
        display.print("  ");
      }
      else {
        display.gotoXY(3,3);
        display.print("->");
        display.print("Black Line");
        display.print("<-");
        display.gotoXY(3,4);
        display.print("  ");
        display.print("White Line");
        display.print("  ");
      }
    }
    else if(buttonB.getSingleDebouncedPress()) {
      modeLoc = 2;
      break;
    }
    else if(buttonC.getSingleDebouncedPress()) {
      modeLoc = 3;
      break;
    }
  }

  //search rule setting
  display.clear();
  display.gotoXY(0,1);
  display.print(" Select Search Rule: ");
  display.gotoXY(0,3);
  display.print("     Right Hand      ");
  display.gotoXY(0,4);
  display.print("     Left Hand       ");
  display.gotoXY(0,6);
  display.print(" A        B          ");
  display.gotoXY(0,7);
  display.print("\1/\2      SEL       ");

  //search rule setting loop
  while(modeLoc == 2) {
    if(buttonA.getSingleDebouncedPress()) {
      rightHand = !rightHand;
      if (rightHand) {
        //select right on screen
        display.gotoXY(3,3);
        display.print("->");
        display.print("Right Hand");
        display.print("<-");
        //deselect left on screen
        display.gotoXY(3,4);
        display.print("  ");
        display.print("Left Hand");
        display.print("  ");
      }
      else {
        display.gotoXY(3,3);
        display.print("  ");
        display.print("Right Hand");
        display.print("  ");
        display.gotoXY(3,4);
        display.print("->");
        display.print("Left Hand");
        display.print("<-");
      }
    }
    else if(buttonB.getSingleDebouncedPress()) {
      modeLoc = 2;
      break;
    }
    else if(buttonC.getSingleDebouncedPress()) {
      modeLoc = 3;
      break;
    }
  }  

  //Startup Delay
  display.clear();
  display.gotoXY(0,0);
  display.print("Line Follow:         ");
  display.gotoXY(0,1);
  display.print("Line Type: ");
  if (whiteLine) {
    display.print("White Line");
  }
  else {
    display.print("Black Line");
  }
  display.gotoXY(0,2);
  display.print("Search Rule: ");
  if (rightHand) {
    display.print("Right Hand");
  }
  else {
    display.print("Left  Hand");
  }
  display.gotoXY(0,3);
  display.print("Calibration in: ");
  display.print("3 ");
  delay(1000);
  display.print("2 ");
  delay(1000);
  display.print("1 ");
  delay(1000);

  //Calibration Loop
  display.clear();
  display.gotoXY(0,0);
  display.print("Line Follow:         ");
  display.gotoXY(0,1);
  display.print("Calibrating...");
  display.gotoXY(0,6);
  display.print("Press A to skip");
  for (int i = 0; i<40; i++){
    lineSensors.calibrate();
    delay(100);
    display.gotoXY(0,1);
    display.print("Calibrating.. ");
    display.gotoXY(16,1);
    display.print(i*2);
    display.print("%");
    motors.setSpeeds(100, -100);
  }
  //Wait for button press to start
  motors.setSpeeds(0,0);
  display.gotoXY(0,6);
  display.print("Press B to start");
  while(true) {
    if(buttonB.getSingleDebouncedPress()) {
      modeLoc = 20;
      break;
    }
  }

  //Startup Delay
  display.clear();
  display.gotoXY(0,0);
  display.print("Starting in: ");
  display.print("3 ");
  delay(1000);
  display.print("2 ");
  delay(1000);
  display.print("1 ");
  delay(1000);

  //Line Follow Loop
  while (true) {
    //Standard Straight Segment Functionality
    straightSegment();  

    //Crawl Forward to verify intersection detection
    motors.setSpeeds(61,61);
    delay(38); //Essential timing delay

    leftMem = 0;
    centerMem = 0;
    rightMem = 0;

    //Update Sensors after intersection detection and crawl
    updateSensors();
    leftMem = left;
    rightMem = right;
    display.gotoXY(0,1);
    display.print(leftMem);
    display.gotoXY(4,1);
    display.print(rightMem);

    
    //Align to wheel
    crawlFwd_alignToWheel();
    motors.setSpeeds(0,0);
    delay(100); //Non essential delay

    //Update Sensors again
    updateSensors();
    if (center == 1 || sensVals[1] > 700 || sensVals[3] > 700) {
      centerMem = 1;
      display.gotoXY(2,1);
      display.print(centerMem);
    }
    else {
      centerMem = 0;
      display.gotoXY(2,1);
      display.print(centerMem);
    }

    if(leftMem && centerMem && rightMem && left && center && right) {
      break;
    }
    //decision upon Search Rule
    if (rightHand) {
      display.gotoXY(0,3);
      display.print("Right Hand Rule");
      rightHandRule();
    }
    else if (!rightHand){ 
      display.gotoXY(0,3);
      display.print("Left Hand Rule");
      leftHandRule(); 
    }
    
    if(leftMem == 0 && centerMem == 0 && rightMem == 1) {
      decision = 'R';
    }
    if (decision == 'U' && rightMem) {
      decision = 'R';
    }

    display.gotoXY(19,0);
    display.print(decision);
    display.gotoXY(printCount,2);
    display.print(decision);

    turnControl();
    motors.setSpeeds(0,0);
    delay(100); //Non essential delay

    //Store decision in memory if intersection, not adding basic turns
    if (decision == 'U') { //Record Uturn
      decisionMem = decision;
      decisionCount++;
      decisionHistory[decisionCount] = decision;
    }
    else if ((centerMem && (leftMem || rightMem))) { //Record T Intersections (SR and SL)
      decisionMem = decision;
      decisionCount++;
      decisionHistory[decisionCount] = decision;
    }
    else if (leftMem && rightMem && (decision == 'L' || decision == 'R')) { 
      decisionMem = decision;
      decisionCount++;
      decisionHistory[decisionCount] = decision;
    }
    display.gotoXY(printCount,7);
    display.print(decisionMem);



    printCount++;


    // decisionCount++;
    // decisionHistory[decisionCount] = decision;
  }

  display.clear();
  while(true) { //Maze Solved Screen
    display.gotoXY(0,0);
    display.print("Maze Solved!     ");
    display.gotoXY(0,1);
    display.print("Recorded Path:   ");
    display.gotoXY(0,2);
    for(int i = 0; i <= decisionCount; i++) {
      display.print(decisionHistory[i]);
      if(i == 20) {
        display.gotoXY(0,3);
      }
    }
    optimizePath(decisionHistory, decisionCount);
    display.gotoXY(0,4);
    display.print("Optimized Path:  ");
    display.gotoXY(0,5);
    for(int i = 0; i <= decisionCount; i++) {
      display.print(optimizedPath[i]);
    }
    display.gotoXY(0,4);
    if (rightHand) {display.print("Right Hand Rule");}
    else {display.print("Left Hand Rule");}

    display.gotoXY(0,6);
    display.print("SER-OUT RUN-OPT  QUIT");
    display.gotoXY(0,7);
    display.print(" A        B        C ");
    if(buttonA.getSingleDebouncedPress()) {
      for(int i = 0; i <= decisionCount; i++){
        Serial.print(optimizedPath[i]);
      }
    }
    else if(buttonB.getSingleDebouncedPress()) {
      modeLoc = 21;
      break;
    }
    else if(buttonC.getSingleDebouncedPress()) {
      modeLoc = 101;
      break;
    }
  }

  display.gotoXY(0,0);
  display.print("Running In: ");
  display.print("3 ");
  delay(1000);
  display.print("2 ");
  delay(1000);
  display.print("1 ");
  delay(1000);

  while(modeLoc == 21) {// run optimized maze
      display.gotoXY(0,0);
      display.print("Running Opt. Path...");
        

      //Standard Straight Segment Functionality
      straightSegment();  

      //Crawl Forward to verify intersection detection
      motors.setSpeeds(61,61);
      delay(38); //Essential timing delay

      leftMem = 0;
      centerMem = 0;
      rightMem = 0;

      //Update Sensors after intersection detection and crawl
      updateSensors();
      leftMem = left;
      rightMem = right;
      display.gotoXY(0,1);
      display.print(leftMem);
      display.gotoXY(4,1);
      display.print(rightMem);

      
      //Align to wheel
      crawlFwd_alignToWheel();
      motors.setSpeeds(0,0);
      delay(100); //Non essential delay

      //Update Sensors again
      updateSensors();
      if (center == 1 || sensVals[1] > 700 || sensVals[3] > 700) {
        centerMem = 1;
        display.gotoXY(2,1);
        display.print(centerMem);
      }
      else {
        centerMem = 0;
        display.gotoXY(2,1);
        display.print(centerMem);
      }
      //End of maze detection
      if(leftMem && centerMem && rightMem && left && center && right || (optCount == decisionCount)) {
        display.clear();
        motors.setSpeeds(0,0);
        modeLoc = 22;
        break;
      }


      if(!leftMem && !centerMem && rightMem) {
        decision = 'R';
      }
      else if (leftMem && !centerMem && !rightMem) {
        decision = 'L';
      }
      else {
        optCount++;
        decision = optimizedPath[optCount];
      }
      if (decision == 'U' && rightMem) {
        decision = 'R';
      }

      display.gotoXY(0,1);
      display.print(decision);

      //Run decision
      turnControl();
      motors.setSpeeds(0,0);
      delay(100);
  }
  
  while(modeLoc == 22) {//Post run opt maze menu (final screen)
    display.gotoXY(0,0);
    display.print("Opt. Path Completed!");

  }

}

//==================== Utility Functions ==========================

// Helper function to shift the array elements to the left
void shiftArrayLeft(char path[], int start, int shiftBy, int &len) {
    for (int i = start; i < len - shiftBy; i++) {
        path[i] = path[i + shiftBy];
    }
    len -= shiftBy;
}

// Path optimizer function
void optimizePath(char path[], int &decisionCount) {
    bool simplified = true;
    int optIndex = 0;
    int u = 0;

    while (simplified) {
        simplified = false;
        optIndex = 0;

        for (int i = 0; i <= decisionCount; ++i) {
            // Check for patterns of 3 decisions to apply simplification rules
            if (i < decisionCount - 1) {
                if (path[i] == 'S' && path[i + 1] == 'U' && path[i + 2] == 'L') {
                    optimizedPath[optIndex++] = 'R';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'S' && path[i + 1] == 'U' && path[i + 2] == 'R') {
                    optimizedPath[optIndex++] = 'L';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'L' && path[i + 1] == 'U' && path[i + 2] == 'S') {
                    optimizedPath[optIndex++] = 'R';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'R' && path[i + 1] == 'U' && path[i + 2] == 'S') {
                    optimizedPath[optIndex++] = 'L';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'L' && path[i + 1] == 'U' && path[i + 2] == 'L') {
                    optimizedPath[optIndex++] = 'S';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'R' && path[i + 1] == 'U' && path[i + 2] == 'R') {
                    optimizedPath[optIndex++] = 'S';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'R' && path[i + 1] == 'U' && path[i + 2] == 'L') {
                    optimizedPath[optIndex++] = 'U';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'L' && path[i + 1] == 'U' && path[i + 2] == 'R') {
                    optimizedPath[optIndex++] = 'U';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                } else if (path[i] == 'S' && path[i + 1] == 'U' && path[i + 2] == 'S') {
                    optimizedPath[optIndex++] = 'U';
                    i += 2; // Skip two additional positions
                    simplified = true;
                    continue;
                }
            }
            // If no pattern is found, add the current decision
            optimizedPath[optIndex++] = path[i];
        }

        // Copy the optimized path back to the original array
        for (int i = 0; i < optIndex; i++) {
            path[i] = optimizedPath[i];
        }

        // Verify if the optimized path still contains any U-turns
        u = 0;
        for (int i = 0; i < optIndex; ++i) {
            if (optimizedPath[i] == 'U') {
                u++;
            }
        }
        if (u == 0) {
            decisionCount = optIndex - 1;
            break;
        }
    }
}

//Raw encoder to degree conversion
float tick2deg(int ticks) {
  float deg;
  deg = ticks * (1.0/12.0) * (1.0/29.86) * (360);
  return deg;
}

//Reads sensors and updates isolations
void updateSensors() {
  lineSensors.readCalibrated(lineSensVals);

  sensVals[0] = lineSensVals[0]; //left
  sensVals[1] = lineSensVals[1];
  sensVals[2] = lineSensVals[2]; //center
  sensVals[3] = lineSensVals[3];
  sensVals[4] = lineSensVals[4]; //right

  if(whiteLine) {
    predict = lineSensors.readLineWhite(lineSensVals);
    sensVals[0] = 1000 - sensVals[0];
    sensVals[1] = 1000 - sensVals[1];
    sensVals[2] = 1000 - sensVals[2];
    sensVals[3] = 1000 - sensVals[3];
    sensVals[4] = 1000 - sensVals[4];
  }
  else {
    predict = lineSensors.readLineBlack(lineSensVals);
  }

  leftInt = sensVals[0];
  centerInt = sensVals[2];
  rightInt = sensVals[4];

  if(leftInt > 700){left = 1;} 
  else {left = 0;}

  if(centerInt > 700){center = 1;} 
  else {center = 0;}

  if(rightInt > 700){right = 1;} 
  else {right = 0;}

  display.gotoXY(0,0);
  display.print(left);
  display.print(" ");
  display.print(center);
  display.print(" ");
  display.print(right);
  display.print("              ");
  display.gotoXY(0,3);
}

void crawlFwd_alignToWheel() {
  motors.setSpeeds(40, 40);
  delay(140);
}

char leftHandRule() {
  delay(50);
  if(leftMem){
    delay(50);
    decision = 'L';
    return 'L';
  }
  else if(centerMem){
    delay(50);
    decision = 'S';
    return 'S';
  }
  else if(rightMem){
    delay(50);
    decision = 'R';
    return 'R';
  }
  else {
    delay(50);
    decision = 'U';
    return 'U';
  }
}

char rightHandRule() {
  delay(50);
  if(rightMem){
    delay(50);
    decision = 'R';
    return 'R';
  } 
  else if(centerMem){
    delay(50);
    decision = 'S';
    return 'S';
  }
  else if(leftMem){
    delay(50);
    decision = 'L';
    return 'L';
  }
  else {
    delay(50);
    decision = 'U';
    return 'U';
  }
}

void straightSegment() {
  display.gotoXY(0,4);
  display.print("Straight          ");
  lastDeviation = 0;
  while(true) {
    updateSensors();
    //Simple Line Follower Control
    deviation = predict - midPoint;
    motorSpeedAdj = deviation * (int32_t)Kp / 256  + (deviation - lastDeviation) * (int32_t)Kd / 256;
    lastDeviation = deviation;

    motorSpeedL = (int16_t)motorSpeed + motorSpeedAdj;
    motorSpeedR = (int16_t)motorSpeed - motorSpeedAdj;

    motorSpeedL = constrain(motorSpeedL, motorSpeed*(0.7), (int16_t)motorSpeed);
    motorSpeedR = constrain(motorSpeedR, motorSpeed*(0.7), (int16_t)motorSpeed);

    motors.setSpeeds(motorSpeedL, motorSpeedR);
    //Print Motor Speeds
    display.gotoXY(0,5); 
    display.print(motorSpeedL);
    display.print(" ");
    display.print(motorSpeedR);
    display.print("    ");

    //Condition to check for intersection
    if (center == 0 && sensVals[1] < 600 && sensVals[3] < 600) {

      return;
    }
    else if (left == 1 || right == 1) {

      return;
    }
  }
}

void turnControl() {
  delay(50);
  switch (decision) {
      case 'R': //RIGHT TURN
        display.gotoXY(0,4);
        display.print("Right Turn        ");
        motors.setSpeeds(96, -96);
        delay(200);
        break;
      case 'L': //LEFT TURN
        display.gotoXY(0,4);
        display.print("Left Turn         ");
        motors.setSpeeds(-96, 96);
        delay(200);
        break;
      case 'U': //U-TURN
        display.gotoXY(0,4);
        display.print("U-Turn            ");
        motors.setSpeeds(96,-96);
		    delay(400);
        break;
       // END U-TURN
      case 'S': //STRAIGHT PATH
        display.gotoXY(0,4);
        display.print("Straight          ");
        break; //END STRAIGHT
      }
}